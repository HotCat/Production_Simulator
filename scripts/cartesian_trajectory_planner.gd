class_name CartesianTrajectoryPlanner
extends RefCounted

## LinuxCNC-inspired linear Cartesian trajectory planner.
##
## The planner accepts world-space waypoints, applies junction-deviation
## lookahead, performs forward/backward acceleration constraint passes, and
## builds an exact trapezoidal/triangular speed profile for every line segment.
## Units at the public configuration boundary are millimetres and seconds;
## sampled positions use Godot metres.

signal trajectory_started
signal trajectory_paused
signal trajectory_resumed
signal trajectory_completed
signal trajectory_triggered(key: String)

enum MotionState {
	IDLE,
	RUNNING,
	PAUSED,
	COMPLETED,
}

const EPSILON := 0.0000001

var _waypoints: Array[Vector3] = []
var _yaw_waypoints: PackedFloat32Array = PackedFloat32Array()
var _orientation_waypoints: Array[Vector3] = []
var _joint_waypoints: Array[PackedFloat32Array] = []
var _waypoint_events: Array[Array] = []
var _trigger_events: Array[Dictionary] = []
var _next_trigger_index := 0
var _segment_lengths: PackedFloat32Array = PackedFloat32Array()
var _segment_directions: Array[Vector3] = []
var _boundary_speeds: PackedFloat32Array = PackedFloat32Array()
var _phases: Array[Dictionary] = []

var _feed_speed_mps := 0.100
var _acceleration_mps2 := 0.500
var _junction_deviation_m := 0.001
var _elapsed_seconds := 0.0
var _total_seconds := 0.0
var _phase_index := 0
var _state := MotionState.IDLE
var _current_position := Vector3.ZERO
var _current_yaw := 0.0
var _current_orientation := Vector3.ZERO
var _current_joints := PackedFloat32Array()
var _current_speed_mps := 0.0


## Plans a list of Godot world-space coordinates in metres.
##
## `yaw_radians` is optional. When provided, it must contain one yaw for every
## coordinate and is interpolated along each linear segment using the shortest
## angular path.
## `orientation_euler` and `joint_waypoints` are optional full-pose channels;
## joint values are radians and are sampled from the same time/acceleration
## profile as Cartesian position.
func plan_world_path(
	coordinates: Array[Vector3],
	desired_speed_mm_s: float,
	acceleration_mm_s2: float = 500.0,
	junction_deviation_mm: float = 1.0,
	yaw_radians: PackedFloat32Array = PackedFloat32Array(),
	waypoint_events: Array = [],
	orientation_euler: Array[Vector3] = [],
	joint_waypoints: Array[PackedFloat32Array] = []
) -> bool:
	_clear_plan()
	if coordinates.size() < 2:
		push_error("Cartesian trajectory requires at least two coordinates")
		return false
	if desired_speed_mm_s <= 0.0 or acceleration_mm_s2 <= 0.0:
		push_error("Trajectory speed and acceleration must be greater than zero")
		return false
	if not yaw_radians.is_empty() and yaw_radians.size() != coordinates.size():
		push_error("Yaw waypoint count must match coordinate count")
		return false
	if not waypoint_events.is_empty() and waypoint_events.size() != coordinates.size():
		push_error("Waypoint event count must match coordinate count")
		return false
	if not orientation_euler.is_empty() and orientation_euler.size() != coordinates.size():
		push_error("Orientation waypoint count must match coordinate count")
		return false
	if not joint_waypoints.is_empty() and joint_waypoints.size() != coordinates.size():
		push_error("Joint waypoint count must match coordinate count")
		return false

	_feed_speed_mps = desired_speed_mm_s / 1000.0
	_acceleration_mps2 = acceleration_mm_s2 / 1000.0
	_junction_deviation_m = maxf(junction_deviation_mm, 0.0) / 1000.0

	# Remove zero-length moves while retaining the yaw attached to the surviving
	# coordinate. LinuxCNC likewise treats coincident linear endpoints as no-op
	# geometry rather than motion blocks.
	_waypoints.append(coordinates[0])
	_yaw_waypoints.append(yaw_radians[0] if not yaw_radians.is_empty() else 0.0)
	_orientation_waypoints.append(orientation_euler[0] if not orientation_euler.is_empty() else Vector3(0.0, 0.0, _yaw_waypoints[0]))
	_joint_waypoints.append(joint_waypoints[0].duplicate() if not joint_waypoints.is_empty() else PackedFloat32Array())
	_waypoint_events.append(waypoint_events[0].duplicate(true) if not waypoint_events.is_empty() else [])
	for index in range(1, coordinates.size()):
		if coordinates[index].distance_to(_waypoints[-1]) <= EPSILON:
			if not yaw_radians.is_empty():
				_yaw_waypoints[_yaw_waypoints.size() - 1] = yaw_radians[index]
			if not orientation_euler.is_empty():
				_orientation_waypoints[_orientation_waypoints.size() - 1] = orientation_euler[index]
			if not joint_waypoints.is_empty():
				_joint_waypoints[_joint_waypoints.size() - 1] = joint_waypoints[index].duplicate()
			if not waypoint_events.is_empty():
				_waypoint_events[_waypoint_events.size() - 1].append_array(waypoint_events[index].duplicate(true))
			continue
		_waypoints.append(coordinates[index])
		_yaw_waypoints.append(yaw_radians[index] if not yaw_radians.is_empty() else 0.0)
		_orientation_waypoints.append(orientation_euler[index] if not orientation_euler.is_empty() else Vector3(0.0, 0.0, _yaw_waypoints[-1]))
		_joint_waypoints.append(joint_waypoints[index].duplicate() if not joint_waypoints.is_empty() else PackedFloat32Array())
		_waypoint_events.append(waypoint_events[index].duplicate(true) if not waypoint_events.is_empty() else [])

	if _waypoints.size() < 2:
		push_error("Cartesian trajectory contains no non-zero moves")
		_clear_plan()
		return false

	_build_segments()
	_compute_lookahead_speeds()
	_build_motion_phases()
	_current_position = _waypoints[0]
	_current_yaw = _yaw_waypoints[0]
	_current_orientation = _orientation_waypoints[0]
	_current_joints = _joint_waypoints[0].duplicate()
	_state = MotionState.IDLE
	return not _phases.is_empty()


func start() -> void:
	if _phases.is_empty():
		return
	_elapsed_seconds = 0.0
	_phase_index = 0
	_next_trigger_index = 0
	_state = MotionState.RUNNING
	_sample_at_elapsed_time()
	_emit_due_triggers()
	trajectory_started.emit()


func pause() -> void:
	if _state != MotionState.RUNNING:
		return
	_state = MotionState.PAUSED
	trajectory_paused.emit()


func resume() -> void:
	if _state != MotionState.PAUSED:
		return
	_state = MotionState.RUNNING
	trajectory_resumed.emit()


func stop() -> void:
	_state = MotionState.IDLE
	_current_speed_mps = 0.0


## Advances the deterministic time profile and returns the sampled world pose.
func advance(delta: float) -> Dictionary:
	if _state != MotionState.RUNNING or delta <= 0.0:
		return get_current_pose()
	_elapsed_seconds = minf(_elapsed_seconds + delta, _total_seconds)
	_sample_at_elapsed_time()
	_emit_due_triggers()
	if _elapsed_seconds >= _total_seconds - EPSILON:
		_current_position = _waypoints[-1]
		_current_yaw = _yaw_waypoints[-1]
		_current_orientation = _orientation_waypoints[-1]
		_current_joints = _joint_waypoints[-1].duplicate()
		_current_speed_mps = 0.0
		_state = MotionState.COMPLETED
		trajectory_completed.emit()
	return get_current_pose()


func seek_time(seconds: float) -> Dictionary:
	_elapsed_seconds = clampf(seconds, 0.0, _total_seconds)
	_sample_at_elapsed_time()
	return get_current_pose()


func get_current_pose() -> Dictionary:
	return {
		"position": _current_position,
		"yaw": _current_yaw,
		"orientation": _current_orientation,
		"joints": _current_joints,
		"speed_mps": _current_speed_mps,
		"elapsed_seconds": _elapsed_seconds,
		"total_seconds": _total_seconds,
	}


func get_current_position() -> Vector3:
	return _current_position


func get_current_yaw() -> float:
	return _current_yaw


func get_current_speed_mm_s() -> float:
	return _current_speed_mps * 1000.0


func get_feed_speed_mm_s() -> float:
	return _feed_speed_mps * 1000.0


func get_acceleration_mm_s2() -> float:
	return _acceleration_mps2 * 1000.0


func get_junction_deviation_mm() -> float:
	return _junction_deviation_m * 1000.0


func get_elapsed_seconds() -> float:
	return _elapsed_seconds


func get_total_seconds() -> float:
	return _total_seconds


func get_progress_normalized() -> float:
	if _total_seconds <= 0.0:
		return 0.0
	return _elapsed_seconds / _total_seconds


func get_state() -> MotionState:
	return _state


func is_running() -> bool:
	return _state == MotionState.RUNNING


func is_completed() -> bool:
	return _state == MotionState.COMPLETED


func get_segment_count() -> int:
	return _segment_lengths.size()


func get_boundary_speed_mm_s(boundary_index: int) -> float:
	if boundary_index < 0 or boundary_index >= _boundary_speeds.size():
		return 0.0
	return _boundary_speeds[boundary_index] * 1000.0


func _clear_plan() -> void:
	_waypoints.clear()
	_yaw_waypoints = PackedFloat32Array()
	_orientation_waypoints.clear()
	_joint_waypoints.clear()
	_segment_lengths = PackedFloat32Array()
	_segment_directions.clear()
	_boundary_speeds = PackedFloat32Array()
	_phases.clear()
	_elapsed_seconds = 0.0
	_total_seconds = 0.0
	_phase_index = 0
	_state = MotionState.IDLE
	_current_speed_mps = 0.0
	_waypoint_events.clear()
	_trigger_events.clear()
	_next_trigger_index = 0


func _build_segments() -> void:
	for index in range(_waypoints.size() - 1):
		var displacement := _waypoints[index + 1] - _waypoints[index]
		var length := displacement.length()
		_segment_lengths.append(length)
		_segment_directions.append(displacement / length)


func _compute_lookahead_speeds() -> void:
	var segment_count := _segment_lengths.size()
	_boundary_speeds.resize(segment_count + 1)
	_boundary_speeds.fill(_feed_speed_mps)
	_boundary_speeds[0] = 0.0
	_boundary_speeds[segment_count] = 0.0

	for boundary in range(1, segment_count):
		_boundary_speeds[boundary] = _junction_speed_limit(
			_segment_directions[boundary - 1],
			_segment_directions[boundary]
		)

	# Reverse pass guarantees every segment can decelerate to its requested exit
	# speed. The forward pass then guarantees it can accelerate from its entry.
	for segment in range(segment_count - 1, -1, -1):
		var reachable_entry := sqrt(
			_boundary_speeds[segment + 1] * _boundary_speeds[segment + 1]
			+ 2.0 * _acceleration_mps2 * _segment_lengths[segment]
		)
		_boundary_speeds[segment] = minf(_boundary_speeds[segment], reachable_entry)
	for segment in range(segment_count):
		var reachable_exit := sqrt(
			_boundary_speeds[segment] * _boundary_speeds[segment]
			+ 2.0 * _acceleration_mps2 * _segment_lengths[segment]
		)
		_boundary_speeds[segment + 1] = minf(_boundary_speeds[segment + 1], reachable_exit)


func _junction_speed_limit(incoming: Vector3, outgoing: Vector3) -> float:
	var direction_dot := clampf(incoming.dot(outgoing), -1.0, 1.0)
	if direction_dot >= 0.999999:
		return _feed_speed_mps
	if direction_dot <= -0.999999 or _junction_deviation_m <= 0.0:
		return 0.0
	# GRBL/LinuxCNC-style junction-deviation approximation: straight motion has
	# no speed penalty, a reversal stops, and sharper corners receive lower speed.
	var sin_half_angle := sqrt(0.5 * (1.0 + direction_dot))
	var denominator := maxf(1.0 - sin_half_angle, EPSILON)
	var junction_speed := sqrt(
		_acceleration_mps2 * _junction_deviation_m * sin_half_angle / denominator
	)
	return minf(junction_speed, _feed_speed_mps)


func _build_motion_phases() -> void:
	_append_waypoint_events(0)
	for segment in range(_segment_lengths.size()):
		var length := _segment_lengths[segment]
		var entry_speed := _boundary_speeds[segment]
		var exit_speed := _boundary_speeds[segment + 1]
		var unconstrained_peak := sqrt(
			_acceleration_mps2 * length
			+ 0.5 * (entry_speed * entry_speed + exit_speed * exit_speed)
		)
		var peak_speed := minf(_feed_speed_mps, unconstrained_peak)
		var acceleration_distance := maxf(
			(peak_speed * peak_speed - entry_speed * entry_speed) / (2.0 * _acceleration_mps2),
			0.0
		)
		var deceleration_distance := maxf(
			(peak_speed * peak_speed - exit_speed * exit_speed) / (2.0 * _acceleration_mps2),
			0.0
		)
		var cruise_distance := maxf(length - acceleration_distance - deceleration_distance, 0.0)

		var segment_distance := 0.0
		if acceleration_distance > EPSILON:
			var duration := (peak_speed - entry_speed) / _acceleration_mps2
			_append_phase(segment, segment_distance, entry_speed, _acceleration_mps2, duration)
			segment_distance += acceleration_distance
		if cruise_distance > EPSILON:
			var duration := cruise_distance / peak_speed
			_append_phase(segment, segment_distance, peak_speed, 0.0, duration)
			segment_distance += cruise_distance
		if deceleration_distance > EPSILON:
			var duration := (peak_speed - exit_speed) / _acceleration_mps2
			_append_phase(segment, segment_distance, peak_speed, -_acceleration_mps2, duration)
		_append_waypoint_events(segment + 1)


func _append_waypoint_events(waypoint_index: int) -> void:
	if waypoint_index < 0 or waypoint_index >= _waypoint_events.size():
		return
	for event in _waypoint_events[waypoint_index]:
		var event_type: String = str(event.get("type", "")).to_lower()
		if event_type == "trigger":
			_trigger_events.append({
				"time": _total_seconds,
				"key": str(event.get("key", "")).to_lower(),
			})
		elif event_type == "delay":
			_append_hold_phase(waypoint_index, float(event.get("seconds", 0.0)))


func _append_hold_phase(waypoint_index: int, duration: float) -> void:
	if duration <= EPSILON:
		return
	_phases.append({
		"start_time": _total_seconds,
		"duration": duration,
		"type": "hold",
		"waypoint": waypoint_index,
	})
	_total_seconds += duration


func _append_phase(
	segment_index: int,
	segment_distance_start: float,
	start_speed: float,
	acceleration: float,
	duration: float
) -> void:
	if duration <= EPSILON:
		return
	_phases.append({
		"start_time": _total_seconds,
		"duration": duration,
		"segment": segment_index,
		"segment_distance_start": segment_distance_start,
		"start_speed": start_speed,
		"acceleration": acceleration,
	})
	_total_seconds += duration


func _sample_at_elapsed_time() -> void:
	if _phases.is_empty():
		return
	while _phase_index < _phases.size() - 1:
		var current_phase := _phases[_phase_index]
		if _elapsed_seconds <= current_phase.start_time + current_phase.duration + EPSILON:
			break
		_phase_index += 1
	while _phase_index > 0 and _elapsed_seconds < float(_phases[_phase_index].start_time):
		_phase_index -= 1

	var phase := _phases[_phase_index]
	if str(phase.get("type", "motion")) == "hold":
		var hold_waypoint: int = int(phase.waypoint)
		_current_position = _waypoints[hold_waypoint]
		_current_yaw = _yaw_waypoints[hold_waypoint]
		_current_orientation = _orientation_waypoints[hold_waypoint]
		_current_joints = _joint_waypoints[hold_waypoint].duplicate()
		_current_speed_mps = 0.0
		return
	var local_time: float = clampf(
		_elapsed_seconds - float(phase.start_time),
		0.0,
		float(phase.duration)
	)
	var start_speed: float = phase.start_speed
	var acceleration: float = phase.acceleration
	var distance: float = (
		float(phase.segment_distance_start)
		+ start_speed * local_time
		+ 0.5 * acceleration * local_time * local_time
	)
	var segment: int = phase.segment
	var interpolation := clampf(distance / _segment_lengths[segment], 0.0, 1.0)
	_current_position = _waypoints[segment].lerp(_waypoints[segment + 1], interpolation)
	_current_yaw = lerp_angle(_yaw_waypoints[segment], _yaw_waypoints[segment + 1], interpolation)
	var from_orientation := _orientation_waypoints[segment]
	var to_orientation := _orientation_waypoints[segment + 1]
	_current_orientation = Vector3(
		lerp_angle(from_orientation.x, to_orientation.x, interpolation),
		lerp_angle(from_orientation.y, to_orientation.y, interpolation),
		lerp_angle(from_orientation.z, to_orientation.z, interpolation)
	)
	if not _joint_waypoints[segment].is_empty() and _joint_waypoints[segment].size() == _joint_waypoints[segment + 1].size():
		_current_joints.resize(_joint_waypoints[segment].size())
		for joint_index in _current_joints.size():
			_current_joints[joint_index] = lerp_angle(_joint_waypoints[segment][joint_index], _joint_waypoints[segment + 1][joint_index], interpolation)
	else:
		_current_joints = PackedFloat32Array()
	_current_speed_mps = maxf(start_speed + acceleration * local_time, 0.0)


func _emit_due_triggers() -> void:
	while _next_trigger_index < _trigger_events.size():
		var event: Dictionary = _trigger_events[_next_trigger_index]
		if float(event.get("time", 0.0)) > _elapsed_seconds + EPSILON:
			break
		trajectory_triggered.emit(str(event.get("key", "")))
		_next_trigger_index += 1

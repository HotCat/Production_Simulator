extends SceneTree

const PlannerScript = preload("res://scripts/cartesian_trajectory_planner.gd")
const FEED_MM_S := 120.0
const ACCELERATION_MM_S2 := 500.0
const JUNCTION_DEVIATION_MM := 1.0
const POSITION_TOLERANCE := 0.000002


func point_is_on_path(point: Vector3, coordinates: Array[Vector3]) -> bool:
	for index in range(coordinates.size() - 1):
		var start := coordinates[index]
		var end := coordinates[index + 1]
		var segment := end - start
		var fraction := clampf((point - start).dot(segment) / segment.length_squared(), 0.0, 1.0)
		if point.distance_to(start.lerp(end, fraction)) <= POSITION_TOLERANCE:
			return true
	return false


func _initialize() -> void:
	var coordinates: Array[Vector3] = [
		Vector3(0.20, 0.20, 0.00),
		Vector3(0.40, 0.20, 0.00),
		Vector3(0.40, 0.20, -0.20),
		Vector3(0.60, 0.20, -0.20),
	]
	var yaws := PackedFloat32Array([0.0, 0.0, PI * 0.5, PI])
	var planner := PlannerScript.new()
	var planned := planner.plan_world_path(
		coordinates,
		FEED_MM_S,
		ACCELERATION_MM_S2,
		JUNCTION_DEVIATION_MM,
		yaws
	)
	if not planned:
		push_error("Trajectory planner rejected valid coordinates")
		quit(1)
		return

	var corner_one_speed := planner.get_boundary_speed_mm_s(1)
	var corner_two_speed := planner.get_boundary_speed_mm_s(2)
	var corners_blended := (
		corner_one_speed > 0.0 and corner_one_speed < FEED_MM_S
		and corner_two_speed > 0.0 and corner_two_speed < FEED_MM_S
	)
	var endpoints_stop := (
		planner.get_boundary_speed_mm_s(0) == 0.0
		and planner.get_boundary_speed_mm_s(3) == 0.0
	)

	planner.start()
	var path_ok := true
	var speed_ok := true
	var previous_speed := planner.get_current_speed_mm_s()
	var step_seconds := 0.001
	while planner.is_running():
		var pose := planner.advance(step_seconds)
		path_ok = path_ok and point_is_on_path(pose.position, coordinates)
		speed_ok = speed_ok and pose.speed_mps * 1000.0 <= FEED_MM_S + 0.001
		var observed_acceleration := absf(pose.speed_mps * 1000.0 - previous_speed) / step_seconds
		speed_ok = speed_ok and observed_acceleration <= ACCELERATION_MM_S2 + 0.1
		previous_speed = pose.speed_mps * 1000.0

	var exact_endpoint := planner.get_current_position().distance_to(coordinates[-1]) <= POSITION_TOLERANCE
	var exact_yaw := absf(angle_difference(planner.get_current_yaw(), yaws[-1])) <= 0.00001
	var completed := planner.is_completed() and planner.get_current_speed_mm_s() == 0.0

	var event_planner := PlannerScript.new()
	var event_coordinates: Array[Vector3] = [Vector3.ZERO, Vector3(0.1, 0.0, 0.0)]
	var event_waypoints: Array = [
		[{"type": "delay", "seconds": 0.5}],
		[
			{"type": "trigger", "key": "q"},
			{"type": "trigger", "key": "w"},
			{"type": "trigger", "key": "e"},
		],
	]
	var triggered_keys: Array[String] = []
	event_planner.trajectory_triggered.connect(func(key: String) -> void: triggered_keys.append(key))
	var events_planned: bool = event_planner.plan_world_path(event_coordinates, 100.0, 500.0, 1.0, PackedFloat32Array(), event_waypoints)
	event_planner.start()
	var hold_pose := event_planner.advance(0.25)
	var delay_hold_ok: bool = hold_pose.position == event_coordinates[0] and hold_pose.speed_mps == 0.0
	while event_planner.is_running():
		event_planner.advance(0.01)
	var events_ok: bool = (
		event_planner.get_total_seconds() >= 0.5
		and triggered_keys == ["q", "w", "e"]
		and event_planner.get_current_position() == event_coordinates[-1]
	)

	print("Trajectory segments: ", planner.get_segment_count())
	print("Corner speeds (mm/s): ", corner_one_speed, ", ", corner_two_speed)
	print("Trajectory duration (s): ", planner.get_total_seconds())
	print("Final position: ", planner.get_current_position())
	if not corners_blended or not endpoints_stop or not path_ok or not speed_ok or not exact_endpoint or not exact_yaw or not completed or not events_planned or not delay_hold_ok or not events_ok:
		push_error("Cartesian lookahead trajectory verification failed")
		quit(1)
	else:
		print("Cartesian lookahead, feed, acceleration, interpolation, and endpoint verification passed")
		quit(0)

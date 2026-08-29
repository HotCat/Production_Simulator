extends Node3D

const ConveyorProductMotionScript = preload("res://scripts/conveyor_product_motion.gd")
const CartesianTrajectoryPlannerScript = preload("res://scripts/cartesian_trajectory_planner.gd")
const TrajectoryFileParserScript = preload("res://scripts/trajectory_file_parser.gd")

@onready var robot: MG400Robot = $MG400Robot
@onready var product_flow: ConveyorProductMotionScript = $ProductFlow
@onready var target_marker: MeshInstance3D = $TargetMarker
@onready var camera: EditorOrbitCamera = $Camera3D
@onready var mode_label: Label = $UI/Margin/VBox/Mode
@onready var pose_label: Label = $UI/Margin/VBox/Pose
@onready var state_label: Label = $UI/Margin/VBox/State
@onready var trajectory_label: Label = $UI/Margin/VBox/Trajectory
@onready var conveyor_label: Label = $UI/Margin/VBox/ConveyorStatus
@onready var camera_label: Label = $UI/Margin/VBox/CameraPose
@onready var camera_orbit_label: Label = $UI/Margin/VBox/CameraOrbit
@onready var overlay_panel: PanelContainer = $UI/Margin
@onready var overlay_toggle: CheckBox = $UI/OverlayToggle
@onready var ik_indicator: Label = $UI/IKIndicator
@onready var interval_spin: SpinBox = $UI/FlowControls/VBox/IntervalRow/IntervalSpin
@onready var speed_spin: SpinBox = $UI/FlowControls/VBox/SpeedRow/SpeedSpin
@onready var stop_y_spin: SpinBox = $UI/FlowControls/VBox/StopRow/StopYSpin
@onready var product_count_label: Label = $UI/FlowControls/VBox/ProductCount
@onready var flow_state_label: Label = $UI/FlowControls/VBox/FlowState

var automatic := true
var elapsed := 0.0
var target_position := Vector3.ZERO
var target_yaw := 0.0
var reachable := true
var suppress_q_yaw_until_release := false
var trajectory := CartesianTrajectoryPlannerScript.new()
var loop_trajectory := true


## Replace the active trajectory with Godot world-space waypoints in metres.
## Feed, acceleration, and junction deviation are expressed in mm-based units.
func set_cartesian_trajectory_world(
	coordinates: Array[Vector3],
	speed_mm_s: float = 120.0,
	acceleration_mm_s2: float = 500.0,
	junction_deviation_mm: float = 1.0,
	yaw_radians: PackedFloat32Array = PackedFloat32Array()
) -> bool:
	var planned := trajectory.plan_world_path(coordinates, speed_mm_s, acceleration_mm_s2, junction_deviation_mm, yaw_radians)
	if planned:
		trajectory.start()
		automatic = true
		_update_trajectory_ui()
	return planned


## Replace the active trajectory with MG400/ROS Cartesian coordinates in mm.
func set_cartesian_trajectory_urdf(
	coordinates_mm: Array[Vector3],
	speed_mm_s: float = 120.0,
	acceleration_mm_s2: float = 500.0,
	junction_deviation_mm: float = 1.0,
	yaw_radians: PackedFloat32Array = PackedFloat32Array()
) -> bool:
	var world_coordinates: Array[Vector3] = []
	for coordinate in coordinates_mm:
		world_coordinates.append(robot.urdf_position_to_world(coordinate / 1000.0))
	return set_cartesian_trajectory_world(world_coordinates, speed_mm_s, acceleration_mm_s2, junction_deviation_mm, yaw_radians)


func _ready() -> void:
	target_position = robot.urdf_position_to_world(Vector3(0.30, 0.0, 0.25))
	reachable = robot.set_tcp_target_world(target_position, target_yaw, true)
	var default_trajectory: Array[Vector3] = [
		Vector3(0.240, 0.000, 0.245),
		Vector3(0.335, 0.000, 0.245),
		Vector3(0.335, 0.080, 0.245),
		Vector3(0.240, 0.080, 0.245),
		Vector3(0.240, 0.000, 0.245),
	]
	var default_world_path: Array[Vector3] = []
	for coordinate in default_trajectory:
		default_world_path.append(robot.urdf_position_to_world(coordinate))
	trajectory.plan_world_path(default_world_path, 120.0, 500.0, 1.0)
	trajectory.start()
	_load_command_line_trajectory()
	robot.target_reachability_changed.connect(_on_reachability_changed)
	overlay_toggle.toggled.connect(_on_overlay_toggled)
	interval_spin.value = product_flow.product_interval_mm
	speed_spin.value = product_flow.product_speed_mps * 1000.0
	stop_y_spin.value = product_flow.stop_position_y_mm
	interval_spin.value_changed.connect(_on_interval_changed)
	speed_spin.value_changed.connect(_on_speed_changed)
	stop_y_spin.value_changed.connect(_on_stop_y_changed)
	product_flow.configuration_changed.connect(_update_flow_controls)
	_on_overlay_toggled(overlay_toggle.button_pressed)
	_update_ik_indicator()
	_update_flow_controls()
	_update_ui()


func _process(delta: float) -> void:
	if Input.is_key_pressed(KEY_ESCAPE):
		get_tree().quit()

	if automatic:
		var pose := trajectory.advance(delta)
		target_position = pose.position
		target_yaw = pose.yaw
		if trajectory.is_completed() and loop_trajectory:
			trajectory.start()
	else:
		_update_manual_target(delta)

	reachable = robot.set_tcp_target_world(target_position, target_yaw)
	target_marker.global_position = target_position
	target_marker.rotation.y = target_yaw
	_update_ik_indicator()
	_update_ui()
	_update_trajectory_ui()


func _load_command_line_trajectory() -> void:
	var args := OS.get_cmdline_user_args()
	var trajectory_path := ""
	for index in range(args.size()):
		if args[index].begins_with("--trajectory="):
			trajectory_path = args[index].substr("--trajectory=".length())
		elif args[index] == "--trajectory" and index + 1 < args.size():
			trajectory_path = args[index + 1]
	if trajectory_path.is_empty():
		return
	var parsed: Dictionary = TrajectoryFileParserScript.parse_file(trajectory_path)
	if not parsed.get("ok", false):
		push_error("Could not load .traj file: %s" % parsed.get("error", "unknown error"))
		return
	var coordinates: Array[Vector3] = parsed["coordinates_mm"]
	var yaw_degrees: PackedFloat32Array = parsed["yaw_degrees"]
	var yaw_radians := PackedFloat32Array()
	for yaw in yaw_degrees:
		yaw_radians.append(deg_to_rad(yaw))
	var planned := set_cartesian_trajectory_urdf(
		coordinates,
		parsed["feed_mm_s"],
		parsed["acceleration_mm_s2"],
		parsed["junction_deviation_mm"],
		yaw_radians
	)
	if planned:
		loop_trajectory = parsed["loop"]
		trajectory_label.text = "TRAJECTORY FILE  %s" % trajectory_path.get_file()


func _input(event: InputEvent) -> void:
	if not event is InputEventKey or event.keycode != KEY_Q:
		return
	if not event.pressed:
		suppress_q_yaw_until_release = false
		return
	if event.echo or not product_flow.is_stopped_at_trigger():
		return
	product_flow.resume_after_stop()
	suppress_q_yaw_until_release = true
	get_viewport().set_input_as_handled()


func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	if event.keycode == KEY_SPACE:
		automatic = not automatic
		if not automatic:
			trajectory.pause()
			target_position = robot.get_tcp_world_position()
			target_yaw = robot.get_joint_angles().x + robot.get_joint_angles().w
		elif trajectory.get_segment_count() > 0:
			if trajectory.get_state() == CartesianTrajectoryPlannerScript.MotionState.PAUSED:
				trajectory.resume()
			else:
				trajectory.start()
	elif event.keycode in [KEY_0, KEY_KP_0, KEY_BACKSPACE]:
		automatic = false
		trajectory.pause()
		target_position = robot.urdf_position_to_world(Vector3(0.30, 0.0, 0.25))
		target_yaw = 0.0


func _update_manual_target(delta: float) -> void:
	var direction := Vector3.ZERO
	if Input.is_key_pressed(KEY_LEFT):
		direction.x -= 1.0
	if Input.is_key_pressed(KEY_RIGHT):
		direction.x += 1.0
	if Input.is_key_pressed(KEY_UP):
		direction.z -= 1.0
	if Input.is_key_pressed(KEY_DOWN):
		direction.z += 1.0
	if Input.is_key_pressed(KEY_R) or Input.is_key_pressed(KEY_PAGEUP):
		direction.y += 1.0
	if Input.is_key_pressed(KEY_F) or Input.is_key_pressed(KEY_PAGEDOWN):
		direction.y -= 1.0
	if direction.length_squared() > 0.0:
		target_position += direction.normalized() * 0.12 * delta
	if Input.is_key_pressed(KEY_Q) and not suppress_q_yaw_until_release:
		target_yaw += 1.5 * delta
	if Input.is_key_pressed(KEY_E):
		target_yaw -= 1.5 * delta
	target_yaw = wrapf(target_yaw, -PI, PI)


func _on_reachability_changed(is_reachable: bool) -> void:
	reachable = is_reachable
	_update_ik_indicator()


func _update_ik_indicator() -> void:
	if ik_indicator == null:
		return
	ik_indicator.text = "● IK reachable" if reachable else "● IK clamped"
	ik_indicator.modulate = Color("70e58a") if reachable else Color("ff796d")


func _on_overlay_toggled(show_overlay: bool) -> void:
	overlay_panel.visible = show_overlay


func _on_interval_changed(value: float) -> void:
	product_flow.set_product_interval_mm(value)


func _on_speed_changed(value: float) -> void:
	product_flow.set_product_speed_mm_s(value)


func _on_stop_y_changed(value: float) -> void:
	product_flow.set_stop_position_y_mm(value)


func _update_flow_controls() -> void:
	product_count_label.text = "Products on conveyor: %d" % product_flow.get_product_count()
	if product_flow.is_stopped_at_trigger():
		flow_state_label.text = "STOPPED at Y %.1f mm  ·  Press Q" % product_flow.stop_position_y_mm
		flow_state_label.modulate = Color("ffb45c")
	else:
		flow_state_label.text = "RUNNING  ·  Stop at Y %.1f mm" % product_flow.stop_position_y_mm
		flow_state_label.modulate = Color("70e58a")


func _update_trajectory_ui() -> void:
	if trajectory_label == null or trajectory.get_segment_count() <= 0:
		return
	var state_text := "RUNNING" if trajectory.is_running() else ("PAUSED" if trajectory.get_state() == CartesianTrajectoryPlannerScript.MotionState.PAUSED else ("COMPLETE" if trajectory.is_completed() else "IDLE"))
	trajectory_label.text = "TRAJECTORY  %s  %d segments  %.1f / %.1f mm/s  %.0f%%" % [
		state_text,
		trajectory.get_segment_count(),
		trajectory.get_current_speed_mm_s(),
		trajectory.get_feed_speed_mm_s(),
		trajectory.get_progress_normalized() * 100.0,
	]


func _update_ui() -> void:
	mode_label.text = "MODE  %s" % ("AUTO CARTESIAN PATH" if automatic else "MANUAL CARTESIAN JOG")
	var urdf_position := Vector3(target_position.x, -target_position.z, target_position.y)
	pose_label.text = "TARGET  X %6.1f   Y %6.1f   Z %6.1f mm   R %5.1f°" % [
		urdf_position.x * 1000.0,
		urdf_position.y * 1000.0,
		urdf_position.z * 1000.0,
		rad_to_deg(target_yaw),
	]
	state_label.text = "IK  %s" % ("REACHABLE" if reachable else "CLAMPED TO WORKSPACE")
	state_label.modulate = Color("70e58a") if reachable else Color("ff796d")
	conveyor_label.text = "CONVEYOR %s  %5.1f mm/s   GAP %5.1f mm   PRODUCTS %d   STOP Y %.1f mm" % [
		"STOPPED · Q resume" if product_flow.is_stopped_at_trigger() else "RUNNING",
		product_flow.product_speed_mps * 1000.0,
		product_flow.product_interval_mm,
		product_flow.get_product_count(),
		product_flow.stop_position_y_mm,
	]
	var camera_position := camera.global_position
	var focus := camera.get_focus_point()
	var orbit := camera.get_orbit_degrees()
	camera_label.text = "CAMERA  X %7.4f   Y %7.4f   Z %7.4f m" % [
		camera_position.x,
		camera_position.y,
		camera_position.z,
	]
	camera_orbit_label.text = "VIEW  Yaw %6.2f°   Pitch %6.2f°   Distance %.4f m   Focus (%+.3f, %+.3f, %+.3f)" % [
		orbit.x,
		orbit.y,
		camera.get_orbit_distance(),
		focus.x,
		focus.y,
		focus.z,
	]

extends Node3D
## Standalone Fairino3 demonstration scene. It uses the same LinuxCNC-style
## CartesianTrajectoryPlanner and .traj parser as the MG400 scene.

const Planner = preload("res://scripts/cartesian_trajectory_planner.gd")
const Parser = preload("res://scripts/trajectory_file_parser.gd")

@onready var robot: Fairino3Robot = $Fairino3Robot
@onready var target_marker: MeshInstance3D = $TargetMarker
@onready var camera: EditorOrbitCamera = $Camera3D
@onready var status: Label = $UI/Panel/Margin/VBox/Status
@onready var pose_label: Label = $UI/Panel/Margin/VBox/Pose
@onready var axis_panel: PanelContainer = $UI/AxisPanel
@onready var tcp_gizmo: TcpGizmo = $TcpGizmo
@onready var orientation_lock: CheckBox = $UI/OrientationPanel/Margin/VBox/LockOrientation
@onready var pose_reachability: Label = $UI/OrientationPanel/Margin/VBox/PoseReachability

var planner := Planner.new()
var automatic := true
var loop_trajectory := true
var target_position := Vector3.ZERO
var target_yaw := 0.0
var reachable := true
var axis_override := false
var updating_axis_ui := false
var axis_spins: Array[SpinBox] = []
var orientation_spins: Array[SpinBox] = []
var target_world_euler := Vector3.ZERO
var updating_orientation_ui := false

const MANUAL_JOG_SPEED_MPS := 0.12
const MANUAL_YAW_SPEED_RAD_S := 1.5
const RESET_URDF_POSITION := Vector3(0.22, 0.0, 0.30)


func _ready() -> void:
	target_position = robot.urdf_position_to_world(Vector3(0.22, 0.0, 0.30))
	robot.set_tcp_target_world(target_position, target_yaw, true)
	target_world_euler = robot.get_tcp_world_basis().get_euler()
	var points: Array[Vector3] = [
		Vector3(0.22, 0.0, 0.30), Vector3(0.34, 0.0, 0.30),
		Vector3(0.34, 0.12, 0.24), Vector3(0.22, 0.12, 0.24),
		Vector3(0.22, 0.0, 0.30)
	]
	var world_points: Array[Vector3] = []
	for p in points:
		world_points.append(robot.urdf_position_to_world(p))
	planner.plan_world_path(world_points, 100.0, 500.0, 1.0)
	planner.start()
	_load_command_line_trajectory()
	robot.target_reachability_changed.connect(_on_reachability_changed)
	for i in 6:
		var spin := get_node("UI/AxisPanel/Margin/VBox/Axis%dRow/SpinBox" % (i + 1)) as SpinBox
		axis_spins.append(spin)
		spin.value_changed.connect(_on_axis_value_changed.bind(i))
	_sync_axis_controls()
	for index in 3:
		var spin := get_node("UI/OrientationPanel/Margin/VBox/Orientation%dRow/SpinBox" % index) as SpinBox
		orientation_spins.append(spin)
		spin.value_changed.connect(_on_orientation_value_changed.bind(index))
	orientation_lock.toggled.connect(_on_orientation_lock_toggled)
	$UI/OrientationPanel/Margin/VBox/Buttons/AlignWorld.pressed.connect(_on_align_world_pressed)
	$UI/OrientationPanel/Margin/VBox/Buttons/CaptureCurrent.pressed.connect(_on_capture_current_pressed)
	_sync_orientation_controls()
	tcp_gizmo.target_dragged.connect(_on_tcp_gizmo_dragged)
	tcp_gizmo.global_position = robot.get_tcp_world_position()
	_update_ui()


func _process(delta: float) -> void:
	if automatic:
		var pose: Dictionary = planner.advance(delta)
		target_position = pose.position
		target_yaw = pose.yaw
		if orientation_lock.button_pressed:
			target_world_euler.y = target_yaw
		if planner.is_completed() and loop_trajectory:
			planner.start()
	else:
		if not axis_override:
			_update_manual_target(delta)
	if not axis_override:
		if orientation_lock.button_pressed:
			reachable = robot.set_tcp_target_pose_world(
				target_position, Basis.from_euler(target_world_euler)
			)
		else:
			reachable = robot.set_tcp_target_world(target_position, target_yaw)
	else:
		target_position = robot.get_tcp_world_position()
	target_marker.global_position = target_position
	tcp_gizmo.global_position = robot.get_tcp_world_position()
	_sync_axis_controls()
	_sync_orientation_controls()
	_update_ui()


func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		var key := event as InputEventKey
		if key.keycode == KEY_SPACE:
			automatic = not automatic
			axis_override = false
			if automatic:
				if planner.get_state() == Planner.MotionState.PAUSED:
					planner.resume()
				else:
					planner.start()
			else:
				planner.pause()
				target_position = robot.get_tcp_world_position()
			get_viewport().set_input_as_handled()
		elif key.keycode == KEY_L:
			loop_trajectory = not loop_trajectory
			get_viewport().set_input_as_handled()
		elif key.keycode in [KEY_0, KEY_KP_0, KEY_BACKSPACE]:
			automatic = false
			axis_override = false
			planner.pause()
			target_position = robot.urdf_position_to_world(RESET_URDF_POSITION)
			target_yaw = 0.0
			get_viewport().set_input_as_handled()


func _update_manual_target(delta: float) -> void:
	var direction := Vector3.ZERO
	# Match the MG400 scene: arrows jog X/Z in the Godot scene coordinates.
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
		axis_override = false
		target_position += direction.normalized() * MANUAL_JOG_SPEED_MPS * delta
	var yaw_jogging := Input.is_key_pressed(KEY_Q) or Input.is_key_pressed(KEY_E)
	if yaw_jogging:
		axis_override = false
	if Input.is_key_pressed(KEY_Q):
		target_world_euler.y += MANUAL_YAW_SPEED_RAD_S * delta
	if Input.is_key_pressed(KEY_E):
		target_world_euler.y -= MANUAL_YAW_SPEED_RAD_S * delta
	if yaw_jogging:
		orientation_lock.button_pressed = true
		target_world_euler.y = wrapf(target_world_euler.y, -PI, PI)
		target_yaw = target_world_euler.y


func _on_axis_value_changed(value: float, axis_index: int) -> void:
	if updating_axis_ui:
		return
	automatic = false
	axis_override = true
	planner.pause()
	var angles := robot.get_joint_angles()
	angles[axis_index] = deg_to_rad(value)
	robot.set_joint_angles_target(angles, true)
	target_position = robot.get_tcp_world_position()
	target_world_euler = robot.get_tcp_world_basis().get_euler()
	orientation_lock.button_pressed = false
	reachable = true


func _sync_axis_controls() -> void:
	if axis_spins.is_empty() or robot == null:
		return
	var angles := robot.get_joint_angles()
	updating_axis_ui = true
	for i in 6:
		var degrees := rad_to_deg(angles[i])
		if absf(axis_spins[i].value - degrees) > 0.01:
			axis_spins[i].value = degrees
	updating_axis_ui = false


func _on_orientation_value_changed(value: float, index: int) -> void:
	if updating_orientation_ui:
		return
	automatic = false
	axis_override = false
	planner.pause()
	target_world_euler[index] = deg_to_rad(value)
	target_yaw = target_world_euler.y
	orientation_lock.button_pressed = true
	reachable = robot.set_tcp_target_pose_world(
		target_position, Basis.from_euler(target_world_euler), true
	)


func _on_orientation_lock_toggled(enabled: bool) -> void:
	if enabled and not updating_orientation_ui:
		automatic = false
		axis_override = false
		planner.pause()
		reachable = robot.set_tcp_target_pose_world(
			target_position, Basis.from_euler(target_world_euler), true
		)


func _on_align_world_pressed() -> void:
	automatic = false
	axis_override = false
	planner.pause()
	target_world_euler = Vector3.ZERO
	target_yaw = 0.0
	orientation_lock.button_pressed = true
	# The flange face is perpendicular to TCP local +Z. Align that normal with
	# world -Y so the flange surface is parallel to the base plane and faces down.
	var aligned_basis := Fairino3Robot.FLANGE_PARALLEL_TO_BASE_BASIS
	target_world_euler = aligned_basis.get_euler()
	reachable = robot.set_tcp_target_pose_world(target_position, aligned_basis, true)
	_sync_orientation_controls()


func _on_capture_current_pressed() -> void:
	automatic = false
	axis_override = false
	planner.pause()
	target_position = robot.get_tcp_world_position()
	target_world_euler = robot.get_tcp_world_basis().get_euler()
	target_yaw = target_world_euler.y
	orientation_lock.button_pressed = true
	_sync_orientation_controls()


func _sync_orientation_controls() -> void:
	if orientation_spins.is_empty():
		return
	updating_orientation_ui = true
	for index in 3:
		var degrees := rad_to_deg(target_world_euler[index])
		if absf(orientation_spins[index].value - degrees) > 0.01:
			orientation_spins[index].value = degrees
	updating_orientation_ui = false


func _load_command_line_trajectory() -> void:
	var args := OS.get_cmdline_user_args()
	var path := ""
	for i in args.size():
		if args[i].begins_with("--trajectory="):
			path = args[i].substr("--trajectory=".length())
		elif args[i] == "--trajectory" and i + 1 < args.size():
			path = args[i + 1]
	if path.is_empty():
		return
	var parsed: Dictionary = Parser.parse_file(path)
	if not parsed.get("ok", false):
		status.text = "TRAJECTORY ERROR\n%s" % parsed.get("error", "unknown error")
		return
	var world_points: Array[Vector3] = []
	for p in parsed.coordinates_mm:
		world_points.append(robot.urdf_position_to_world(p / 1000.0))
	var yaws := PackedFloat32Array()
	for yaw in parsed.yaw_degrees:
		yaws.append(deg_to_rad(yaw))
	if planner.plan_world_path(world_points, parsed.feed_mm_s, parsed.acceleration_mm_s2, parsed.junction_deviation_mm, yaws, parsed.get("waypoint_events", [])):
		planner.start()
		loop_trajectory = parsed.loop


func _on_reachability_changed(value: bool) -> void:
	reachable = value


func _on_tcp_gizmo_dragged(world_position: Vector3) -> void:
	# A gizmo drag is a direct Cartesian command. Stop automatic playback and
	# send the requested position through the same IK/reachability path used by
	# keyboard jogging.
	automatic = false
	axis_override = false
	planner.pause()
	target_position = world_position
	if orientation_lock.button_pressed:
		reachable = robot.set_tcp_target_pose_world(
			target_position, Basis.from_euler(target_world_euler), true
		)
	else:
		reachable = robot.set_tcp_target_world(target_position, target_yaw, true)
	tcp_gizmo.global_position = robot.get_tcp_world_position()
	target_marker.global_position = target_position
	_update_ui()


func _update_ui() -> void:
	if status == null:
		return
	var state := "RUNNING" if planner.is_running() else ("PAUSED" if planner.get_state() == Planner.MotionState.PAUSED else "COMPLETE")
	var mode := "AUTO" if automatic else "MANUAL"
	status.text = "FAIRINO3 V6  ·  6-AXIS CARTESIAN IK\nIK %s  ·  %s / %s  ·  drag TCP gizmo X/Y/Z  ·  arrows X/Z  R/F height  Q/E yaw  ·  Space auto/manual" % [("REACHABLE" if reachable else "CLAMPED"), mode, state]
	pose_label.text = "TCP  X %.1f  Y %.1f  Z %.1f mm\nWORLD R/P/Y  %.1f°  %.1f°  %.1f°\nTRAJECTORY  %.1f / %.1f mm/s  %.0f%%" % [
		target_position.x * 1000.0, target_position.y * 1000.0, target_position.z * 1000.0,
		rad_to_deg(target_world_euler.x), rad_to_deg(target_world_euler.y), rad_to_deg(target_world_euler.z),
		planner.get_current_speed_mm_s(), planner.get_feed_speed_mm_s(), planner.get_progress_normalized() * 100.0
	]
	pose_reachability.text = "● POSE REACHABLE" if reachable else "● POSE CLAMPED / UNREACHABLE"
	pose_reachability.modulate = Color("70e58a") if reachable else Color("ff796d")

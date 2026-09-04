extends Node3D
## Standalone Fairino3 demonstration scene. It uses the same LinuxCNC-style
## CartesianTrajectoryPlanner and .traj parser as the MG400 scene.

const Planner = preload("res://scripts/cartesian_trajectory_planner.gd")
const Parser = preload("res://scripts/trajectory_file_parser.gd")
const Parser2 = preload("res://scripts/trajectory2_file_parser.gd")

@onready var robot: Fairino3Robot = $Fairino3Robot
@onready var target_marker: MeshInstance3D = $TargetMarker
@onready var camera: EditorOrbitCamera = $Camera3D
@onready var status: Label = $UI/Panel/Margin/VBox/Status
@onready var pose_label: Label = $UI/Panel/Margin/VBox/Pose
@onready var header_panel: PanelContainer = $UI/Panel
@onready var axis_panel: PanelContainer = $UI/AxisPanel
@onready var orientation_panel: PanelContainer = $UI/OrientationPanel
@onready var translation_panel: PanelContainer = $UI/TranslationPanel
@onready var pickup_panel: PanelContainer = $UI/PickupPanel
@onready var pendant_panel: PanelContainer = $UI/PendantPanel
@onready var calibration_product: Node3D = $H89CalibrationProduct
@onready var overlay_toolbar: HBoxContainer = $UI/OverlayToolbar
@onready var overlay_toggles: Array[CheckBox] = [
	$UI/OverlayToolbar/StatusToggle,
	$UI/OverlayToolbar/JointToggle,
	$UI/OverlayToolbar/OrientationToggle,
	$UI/OverlayToolbar/TranslationToggle,
	$UI/OverlayToolbar/PickupToggle,
	$UI/OverlayToolbar/RecordingToggle,
	$UI/OverlayToolbar/PendantToggle,
]
@onready var tcp_gizmo: TcpGizmo = $TcpGizmo
@onready var orientation_lock: CheckBox = $UI/OrientationPanel/Margin/VBox/LockOrientation
@onready var pose_reachability: Label = $UI/OrientationPanel/Margin/VBox/PoseReachability
@onready var recording_indicator: Label = $UI/RecordingIndicator

var planner := Planner.new()
var pickup_planner := Planner.new()
var return_planner := Planner.new()
var automatic := true
var loop_trajectory := true
var target_position := Vector3.ZERO
var base_target_position := Vector3.ZERO
# User-defined TCP calibration in the J6 flange frame (metres).
var tcp_translation_offset := Vector3.ZERO
var target_yaw := 0.0
var reachable := true
var axis_override := false
var updating_axis_ui := false
var axis_spins: Array[SpinBox] = []
var orientation_spins: Array[SpinBox] = []
var translation_spins: Array[SpinBox] = []
var target_world_euler := Vector3.ZERO
var updating_orientation_ui := false
var updating_translation_ui := false
var runtime_recorder: Node
var _pointer_over_ui := false
var pickup_active := false
var pickup_step := -1
var pickup_elapsed := 0.0
var pickup_strategy := "thin side wall"
var product_picked := false
var product_initial_transform := Transform3D.IDENTITY
var product_initial_basis := Basis.IDENTITY
var product_initial_parent: Node
var gripper: Node3D
var pickup_flange_basis := Fairino3Robot.FLANGE_PARALLEL_TO_BASE_BASIS
var pickup_grasp_position := Vector3.ZERO
var pickup_grasp_progress := 0.0
var pickup_points_total_length := 0.0
var pickup_grasp_commanded := false
var pickup_waiting_for_grasp := false
var pickup_jaw_closing := false
var pickup_jaw_close_elapsed := 0.0
var pickup_start_basis := Basis.IDENTITY
var pickup_dock_position := Vector3.ZERO
var pickup_dock_basis := Basis.IDENTITY
var pickup_dock_configured := false
var pendant_jog_frame := "Base"
var pendant_jog_position := Vector3.ZERO
var pendant_base_angles := Vector3.ZERO
var updating_pendant_ui := false
var pendant_base_basis := Basis.IDENTITY
var trajectory2_active := false
var trajectory2_path := ""
var trajectory2_mtime := 0
var trajectory2_command_signature := 0
var trajectory2_goto_signature := 0
var trajectory2_poll_elapsed := 0.0
var pending_trajectory2_path := ""
var runtime_pose_publish_elapsed := 0.0
var resume_trajectory2_after_pickup := false
var return_active := false
var return_release_elapsed := 0.0
var return_approach_position := Vector3.ZERO
var return_grasp_position := Vector3.ZERO
var return_target_basis := Basis.IDENTITY
var last_pickup_grasp_position := Vector3.ZERO
var last_pickup_basis := Basis.IDENTITY

const PICKUP_JAW_CLOSE_TIME_S := 0.22
const PICKUP_POSITION_TOLERANCE_M := 0.002
const PICKUP_ORIENTATION_TOLERANCE_RAD := 0.035

const MANUAL_JOG_SPEED_MPS := 0.12
const MANUAL_YAW_SPEED_RAD_S := 1.5
const RESET_URDF_POSITION := Vector3(0.22, 0.0, 0.30)
const LABEL_FIXTURE_TCP_URDF_MM := Vector3(346.407, -213.059, 529.029)
const LABEL_FIXTURE_TCP_RPY_DEG := Vector3(86.167, 32.259, -93.233) # pitch, roll, yaw


func _ready() -> void:
	product_initial_transform = calibration_product.transform
	product_initial_basis = calibration_product.global_basis
	product_initial_parent = calibration_product.get_parent()
	target_position = robot.urdf_position_to_world(Vector3(0.22, 0.0, 0.30))
	base_target_position = target_position
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
	planner.trajectory_triggered.connect(_on_trajectory2_triggered)
	_load_command_line_trajectory()
	_load_command_line_trajectory2()
	robot.target_reachability_changed.connect(_on_reachability_changed)
	for i in 6:
		var spin := get_node("UI/AxisPanel/Margin/VBox/Axis%dRow/SpinBox" % (i + 1)) as SpinBox
		axis_spins.append(spin)
		_configure_pose_spin(spin)
		spin.value_changed.connect(_on_axis_value_changed.bind(i))
	_sync_axis_controls()
	for index in 3:
		var spin := get_node("UI/OrientationPanel/Margin/VBox/Orientation%dRow/SpinBox" % index) as SpinBox
		orientation_spins.append(spin)
		_configure_pose_spin(spin)
		spin.value_changed.connect(_on_orientation_value_changed.bind(index))
	for index in 3:
		var spin := get_node("UI/TranslationPanel/Margin/VBox/Translation%dRow/SpinBox" % index) as SpinBox
		translation_spins.append(spin)
		_configure_pose_spin(spin)
		spin.value_changed.connect(_on_translation_value_changed.bind(index))
	orientation_lock.toggled.connect(_on_orientation_lock_toggled)
	$UI/OrientationPanel/Margin/VBox/Buttons/AlignWorld.pressed.connect(_on_align_world_pressed)
	$UI/OrientationPanel/Margin/VBox/Buttons/CaptureCurrent.pressed.connect(_on_capture_current_pressed)
	_sync_orientation_controls()
	_sync_translation_controls()
	gripper = robot.find_child("ParallelJawGripper", true, false) as Node3D
	var label_fixture := get_node_or_null("LabelApplicationFixture") as LabelApplicationFixture
	if label_fixture != null:
		# Match the uploaded flat-placement TCP pose. Traj2 stores Pitch/Roll/Yaw;
		# Basis.from_euler receives Godot's XYZ order, i.e. Roll/Pitch/Yaw here.
		var fixture_ros_basis := Basis.from_euler(Vector3(
			deg_to_rad(LABEL_FIXTURE_TCP_RPY_DEG.y),
			deg_to_rad(LABEL_FIXTURE_TCP_RPY_DEG.x),
			deg_to_rad(LABEL_FIXTURE_TCP_RPY_DEG.z)))
		label_fixture.set_flat_tcp_reference(
			robot.urdf_position_to_world(LABEL_FIXTURE_TCP_URDF_MM / 1000.0),
			robot.urdf_basis_to_world(fixture_ros_basis))
	pickup_planner.trajectory_triggered.connect(_on_pickup_triggered)
	return_planner.trajectory_completed.connect(_on_return_completed)
	$UI/PickupPanel/Margin/VBox/Buttons/ThinSide.pressed.connect(_start_thin_side_pickup)
	$UI/PickupPanel/Margin/VBox/Buttons/LongSide.pressed.connect(_start_long_side_pickup)
	$UI/PickupPanel/Margin/VBox/ResetPickup.pressed.connect(reset_pickup)
	_on_pendant_frame_selected("Base")
	for row_name in ["RzRow", "RyRow", "RxRow"]:
		var row := get_node("UI/PendantPanel/Margin/VBox/%s" % row_name)
		var axis_index := 2 if row_name == "RzRow" else (1 if row_name == "RyRow" else 0)
		var spin := row.get_node("SpinBox") as SpinBox
		_configure_pose_spin(spin)
		spin.value_changed.connect(_on_pendant_axis_value_changed.bind(axis_index))
	_capture_pendant_base_pose()
	overlay_toggles[0].toggled.connect(_on_status_overlay_toggled)
	overlay_toggles[1].toggled.connect(_on_joint_overlay_toggled)
	overlay_toggles[2].toggled.connect(_on_orientation_overlay_toggled)
	overlay_toggles[3].toggled.connect(_on_translation_overlay_toggled)
	overlay_toggles[4].toggled.connect(_on_pickup_overlay_toggled)
	overlay_toggles[5].toggled.connect(_on_recording_overlay_toggled)
	overlay_toggles[6].toggled.connect(_on_pendant_overlay_toggled)
	reset_pickup()
	tcp_gizmo.target_dragged.connect(_on_tcp_gizmo_dragged)
	# RuntimeRecorder is an autoload shared by both the MG400 and Fairino3
	# scenes.  Connecting here keeps the Fairino3 standalone scene's status
	# overlay in sync with the global F9/Ctrl+R recorder.
	runtime_recorder = get_node_or_null("/root/RuntimeRecorder")
	if runtime_recorder != null:
		runtime_recorder.recording_started.connect(_on_recording_started)
		runtime_recorder.recording_finalizing.connect(_on_recording_finalizing)
		runtime_recorder.recording_stopped.connect(_on_recording_stopped)
		runtime_recorder.recording_failed.connect(_on_recording_failed)
	_set_recording_indicator("● REC OFF  ·  F9 to start", Color("9aa8b8"))
	tcp_gizmo.global_position = robot.get_tcp_world_position()
	_update_ui()


func _process(delta: float) -> void:
	_poll_runtime_trajectory2(delta)
	# A physical jog command takes control immediately, even if an uploaded
	# trajectory is currently playing. This prevents the target marker from
	# moving without the arm when the operator starts pressing an arrow/R/F key.
	if automatic and not pickup_active and _manual_jog_requested():
		automatic = false
		axis_override = false
		planner.pause()
	if pickup_active:
		_process_pickup(delta)
		if not pickup_active and not pending_trajectory2_path.is_empty():
			var queued_path := pending_trajectory2_path
			pending_trajectory2_path = ""
			_apply_trajectory2(queued_path)
		elif not pickup_active and resume_trajectory2_after_pickup:
			resume_trajectory2_after_pickup = false
			automatic = true
			axis_override = false
			if planner.get_state() == Planner.MotionState.PAUSED:
				planner.resume()
	elif return_active:
		_process_return_product(delta)
	elif automatic:
		var pose: Dictionary = planner.advance(delta)
		target_position = pose.position
		base_target_position = target_position
		target_yaw = pose.yaw
		if trajectory2_active and automatic:
			var planned_joints: PackedFloat32Array = pose.get("joints", PackedFloat32Array()) as PackedFloat32Array
			if not planned_joints.is_empty():
				# A traj2 waypoint carries the desired FR3 branch explicitly. Apply
				# the interpolated J1-J6 pose directly; do not run a second IK solve
				# that could jump to another branch.
				var requested_position: Vector3 = target_position
				robot.set_joint_angles_target(planned_joints, true)
				target_position = robot.get_tcp_world_position()
				target_world_euler = robot.get_tcp_world_basis().get_euler()
				# The explicit J1-J6 values are authoritative for the physical
				# branch. FR3 pendant Euler reporting has a different wrist
				# convention than Godot's Basis Euler decomposition, so do not mark
				# a valid joint pose unreachable merely because those representations
				# differ by a wrist-frame convention. Cartesian XYZ still verifies
				# that the supplied pose and joint configuration agree.
				reachable = robot.get_tcp_world_position().distance_to(requested_position) < 0.003
			else:
				target_world_euler = pose.get("orientation", target_world_euler) as Vector3
			orientation_lock.button_pressed = true
		elif orientation_lock.button_pressed:
			target_world_euler.y = target_yaw
		if planner.is_completed() and loop_trajectory:
			planner.start()
	else:
		if not axis_override:
			_update_manual_target(delta)
		base_target_position = target_position
	if not axis_override and not pickup_active:
		if trajectory2_active and automatic:
			# Joint-bearing traj2 poses were already applied above. Keeping this
			# branch free of Cartesian IK preserves the commanded J1-J6 trajectory.
			pass
		elif orientation_lock.button_pressed:
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
	_update_pickup_ui()
	_publish_runtime_pose(delta)


func _manual_jog_requested() -> bool:
	return Input.is_key_pressed(KEY_LEFT) or Input.is_key_pressed(KEY_RIGHT) \
		or Input.is_key_pressed(KEY_UP) or Input.is_key_pressed(KEY_DOWN) \
		or Input.is_key_pressed(KEY_R) or Input.is_key_pressed(KEY_F) \
		or Input.is_key_pressed(KEY_PAGEUP) or Input.is_key_pressed(KEY_PAGEDOWN)


func _start_thin_side_pickup() -> void:
	_start_pickup("thin side wall", Vector3.ZERO)


func _start_long_side_pickup() -> void:
	_start_pickup("long side wall", Vector3(0.0, 0.0, PI * 0.5))


## Starts the inverse of a pickup: keep the current jaw rotation/span, move the
## carried product back to its saved conveyor pose, open the jaws, and release
## it without snapping it to a new production location.
func _start_return_product() -> void:
	if return_active or not product_picked or calibration_product == null or gripper == null:
		return
	return_active = true
	return_release_elapsed = 0.0
	resume_trajectory2_after_pickup = trajectory2_active
	automatic = false
	axis_override = true
	planner.pause()
	# Reuse the exact TCP grasp pose captured by the pickup routine. This is the
	# geometric inverse of the approach and avoids deriving a new IK target from
	# the robot's production-time wrist pose.
	return_target_basis = last_pickup_basis
	var current_tcp := robot.get_tcp_world_position()
	return_grasp_position = last_pickup_grasp_position
	return_approach_position = return_grasp_position + Vector3(0.0, 0.050, 0.0)
	var points: Array[Vector3] = [current_tcp, return_approach_position, return_grasp_position]
	return_planner.plan_world_path(points, 80.0, 300.0, 1.0)
	return_planner.start()
	_update_pickup_ui()


func _process_return_product(delta: float) -> void:
	var pose: Dictionary = return_planner.advance(delta)
	var commanded_position: Vector3 = pose.get("position", robot.get_tcp_world_position()) as Vector3
	# Keep the pickup flange orientation and local jaw rotation unchanged.
	robot.set_tcp_target_pose_world(commanded_position, return_target_basis)
	if return_planner.is_completed():
		if return_release_elapsed <= 0.0:
			if gripper != null:
				gripper.call("set_jaws_closed", false)
			return_release_elapsed = PICKUP_JAW_CLOSE_TIME_S
		else:
			return_release_elapsed -= delta
			if return_release_elapsed <= 0.0:
				_release_product_at_original_pose()
				return_active = false
				axis_override = true
				if resume_trajectory2_after_pickup:
					resume_trajectory2_after_pickup = false
					automatic = true
					axis_override = false
					if planner.get_state() == Planner.MotionState.PAUSED:
						planner.resume()
				_update_pickup_ui()
	target_position = commanded_position
	base_target_position = target_position


func _release_product_at_original_pose() -> void:
	if calibration_product == null or product_initial_parent == null:
		return
	calibration_product.reparent(product_initial_parent, false)
	calibration_product.transform = product_initial_transform
	product_picked = false
	pickup_step = -1


func _on_return_completed() -> void:
	# Release is intentionally handled in _process_return_product after a short
	# dwell at the exact grasp pose, so the jaws visibly open before reparenting.
	pass


func _start_pickup(strategy: String, jaw_rotation: Vector3) -> void:
	reset_pickup()
	pickup_strategy = strategy
	pickup_active = true
	pickup_step = 0
	pickup_elapsed = 0.0
	pickup_grasp_commanded = false
	pickup_waiting_for_grasp = false
	pickup_jaw_closing = false
	pickup_jaw_close_elapsed = 0.0
	resume_trajectory2_after_pickup = false
	automatic = false
	axis_override = true
	planner.pause()
	if gripper != null:
		gripper.rotation = jaw_rotation
		# Thin-side closes across the 37.95 mm width. Long-side rotates the jaw
		# axis onto the product depth and closes across its 4 mm thickness.
		gripper.call("set_product_span", 0.0045 if strategy == "long side wall" else 0.03795)
		gripper.call("set_jaws_closed", false)
	pickup_start_basis = robot.get_tcp_world_basis()
	# The gripper extends roughly 130 mm below the flange. Derive the grasp TCP
	# from the product's current origin and the exact seating offset used by
	# _attach_product_to_gripper(). This makes the jaws arrive at the product,
	# rather than moving to a hard-coded point and then snapping the product on
	# reparent. The approach point is a short vertical clearance above grasp.
	var tool_basis := pickup_flange_basis * Basis.from_euler(jaw_rotation)
	var product_origin := calibration_product.global_position
	var grasp := product_origin - tool_basis * Vector3(0.0, 0.0, 0.130) + Vector3(0.0, 0.023, 0.0)
	var approach := grasp + Vector3(0.0, 0.050, 0.0)
	pickup_grasp_position = grasp
	last_pickup_grasp_position = grasp
	last_pickup_basis = pickup_flange_basis
	var place := Vector3(0.40, 0.30, 0.16)
	var points: Array[Vector3] = [robot.get_tcp_world_position(), approach, grasp, approach, place]
	if pickup_dock_configured and points[0].distance_to(pickup_dock_position) > 0.001:
		points.insert(1, pickup_dock_position)
	pickup_points_total_length = 0.0
	for index in range(points.size() - 1):
		pickup_points_total_length += points[index].distance_to(points[index + 1])
	var grasp_index := points.find(grasp)
	pickup_grasp_progress = 0.0
	for point_index in range(grasp_index):
		pickup_grasp_progress += points[point_index].distance_to(points[point_index + 1])
	pickup_grasp_progress /= maxf(pickup_points_total_length, 0.000001)
	var events: Array = []
	for point_index in points.size():
		events.append([])
	# Keep the existing grasp trigger attached to the grasp waypoint after an
	# optional dock positioning point is inserted.
	if grasp_index >= 0:
		events[grasp_index].append({"type": "trigger", "key": "grip_close"})
	var pickup_orientations: Array[Vector3] = []
	for point_index in points.size():
		pickup_orientations.append(pickup_start_basis.get_euler().lerp(pickup_flange_basis.get_euler(), clampf(float(point_index) / maxf(float(points.size() - 1), 1.0), 0.0, 1.0)))
	pickup_orientations[grasp_index] = pickup_flange_basis.get_euler()
	pickup_planner.plan_world_path(points, 80.0, 300.0, 1.0, PackedFloat32Array(), events, pickup_orientations)
	pickup_planner.start()
	_update_pickup_ui()


func _process_pickup(delta: float) -> void:
	var pose: Dictionary = pickup_planner.advance(delta)
	# Keep this explicitly typed: the planner returns a Dictionary/Variant and
	# Godot cannot infer the ternary's Vector3 type in headless builds.
	var commanded_position: Vector3 = pickup_grasp_position if (pickup_waiting_for_grasp or pickup_jaw_closing) else (pose.get("position", robot.get_tcp_world_position()) as Vector3)
	robot.set_tcp_target_pose_world(commanded_position, pickup_flange_basis)
	var grasp_position_error := robot.get_tcp_world_position().distance_to(pickup_grasp_position)
	var grasp_orientation_error := (
		(robot.get_tcp_world_basis() * pickup_flange_basis.inverse()).get_rotation_quaternion().get_angle()
	)
	if pickup_waiting_for_grasp and grasp_position_error < PICKUP_POSITION_TOLERANCE_M and grasp_orientation_error < PICKUP_ORIENTATION_TOLERANCE_RAD:
		pickup_waiting_for_grasp = false
		pickup_jaw_closing = true
		pickup_jaw_close_elapsed = 0.0
		# The robot has physically arrived at the overlap pose. Only now do the
		# jaws close; the product remains stationary until the close completes.
		if gripper != null:
			gripper.call("set_jaws_closed", true)
	if pickup_jaw_closing:
		pickup_jaw_close_elapsed += delta
		if pickup_jaw_close_elapsed >= PICKUP_JAW_CLOSE_TIME_S:
			pickup_jaw_closing = false
			_attach_product_to_gripper()
			pickup_planner.resume()
	if product_picked:
		# Keep the product's original upright world orientation while it follows
		# the gripper translation; pickup must not twist the CAD part.
		calibration_product.global_basis = product_initial_basis
	# The planner emits grip_close at this waypoint. Keep a geometric fallback
	# for frames that advance across the trigger in one large simulation tick.
	if not product_picked and not pickup_grasp_commanded and pickup_planner.get_progress_normalized() >= pickup_grasp_progress:
		_on_pickup_triggered("grip_close")
	target_position = commanded_position
	base_target_position = target_position
	if pickup_planner.is_completed():
		pickup_step = 4
		pickup_active = false
		axis_override = true
	_update_pickup_ui()


func _on_pickup_triggered(key: String) -> void:
	if key != "grip_close" or not pickup_active or pickup_grasp_commanded:
		return
	pickup_step = 2
	pickup_grasp_commanded = true
	pickup_waiting_for_grasp = true
	pickup_jaw_closing = false
	pickup_jaw_close_elapsed = 0.0
	pickup_planner.pause()


func _on_trajectory2_triggered(key: String) -> void:
	# .traj2 trigger commands share the simulator's physical-control keys.  W/E
	# expose or hide the carried label; Q is intentionally accepted as a
	# synchronization trigger for programs shared with the MG400 conveyor.
	match key.to_lower():
		"w": robot.set_product_label_visible(true)
		"e": robot.set_product_label_visible(false)
		"pickup_thin_side":
			_start_thin_side_pickup()
			resume_trajectory2_after_pickup = true
		"pickup_long_side":
			_start_long_side_pickup()
			resume_trajectory2_after_pickup = true
		"return_product":
			_start_return_product()
		"q": pass


func _attach_product_to_gripper() -> void:
	if product_picked or calibration_product == null or gripper == null:
		return
	product_picked = true
	calibration_product.reparent(gripper, true)
	# Keep the CAD envelope upright in the world while seating its thin edge
	# between the pads. Product local X/Y/Z map to gripper Y/Z/X respectively:
	# width spans the jaw height, the 46 mm side follows the finger length, and
	# the 4 mm thickness is exactly along the jaw closing axis.
	# Seat the product past the jaw center so only the upper/end quarter of the
	# full-length fingers overlaps it. The rest of the product hangs below the
	# gripper, matching the reference pickup pose and leaving placement clearance.
	var grasp_center_world := gripper.to_global(Vector3(0.0, 0.0, 0.130))
	calibration_product.global_basis = product_initial_basis
	calibration_product.global_position = grasp_center_world + Vector3(0.0, -0.023, 0.0)


func reset_pickup() -> void:
	pickup_active = false
	return_active = false
	return_planner.stop()
	resume_trajectory2_after_pickup = false
	pickup_planner.stop()
	pickup_grasp_commanded = false
	pickup_waiting_for_grasp = false
	pickup_jaw_closing = false
	pickup_jaw_close_elapsed = 0.0
	pickup_step = -1
	pickup_elapsed = 0.0
	product_picked = false
	if calibration_product != null and product_initial_parent != null:
		calibration_product.reparent(product_initial_parent, false)
		calibration_product.transform = product_initial_transform
	if gripper != null:
		gripper.rotation = Vector3.ZERO
		gripper.call("set_product_span", 0.03795)
		gripper.call("set_jaws_closed", false)
	_update_pickup_ui()


func _update_pickup_ui() -> void:
	if pickup_panel == null:
		return
	var state := "RETURNING TO PICKUP" if return_active else ("READY" if pickup_step < 0 else ("PLACED" if pickup_step >= 4 else ("AT GRASP · CLOSING" if pickup_jaw_closing else ("WAITING FOR GRASP" if pickup_waiting_for_grasp else "PICKING"))))
	$UI/PickupPanel/Margin/VBox/State.text = "State: %s  ·  %s" % [state, pickup_strategy]


func _on_pendant_frame_selected(frame_name: String) -> void:
	pendant_jog_frame = frame_name
	for name in ["Base", "Tool", "Wobj"]:
		var button := get_node_or_null("UI/PendantPanel/Margin/VBox/FrameRow/%s" % name) as Button
		if button != null:
			button.modulate = Color("6ec8ff") if name == pendant_jog_frame else Color.WHITE
			button.disabled = name != "Base"
	pendant_jog_frame = "Base"


func _capture_pendant_base_pose() -> void:
	pendant_base_basis = robot.get_tcp_world_basis()
	pendant_base_angles = Vector3.ZERO
	updating_pendant_ui = true
	for row_name in ["RxRow", "RyRow", "RzRow"]:
		var spin := get_node_or_null("UI/PendantPanel/Margin/VBox/%s/SpinBox" % row_name) as SpinBox
		if spin != null:
			spin.value = 0.0
	updating_pendant_ui = false


func _on_pendant_axis_value_changed(value: float, axis_index: int) -> void:
	if updating_pendant_ui or pickup_active:
		return
	var previous_value := pendant_base_angles[axis_index]
	pendant_base_angles[axis_index] = value
	var delta := deg_to_rad(value - previous_value)
	# Pendant Base RX/RY/RZ always use the fixed FR3 base axes, never the
	# current TCP axes: RX=Base-X, RY=Base-Y, RZ=Base-Z.
	var world_axis := robot.pendant_base_axis_to_world(axis_index)
	var target_basis := Basis(Quaternion(world_axis, delta)) * robot.get_tcp_world_basis()
	pendant_jog_position = robot.get_tcp_world_position()
	target_position = pendant_jog_position
	base_target_position = pendant_jog_position
	automatic = false
	axis_override = true
	planner.pause()
	# Base pendant flange jog is deliberately separate from TCP orientation and
	# keeps the current Cartesian position as the IK target.
	reachable = robot.set_base_jog_pose_world(pendant_jog_position, target_basis, true)
	pendant_base_basis = target_basis
	target_world_euler = target_basis.get_euler()
	target_yaw = target_world_euler.y


func _input(event: InputEvent) -> void:
	# Move focus away from an edited SpinBox as soon as the pointer enters a UI
	# panel. This makes the following click on a Button/CheckBox a normal first
	# click instead of requiring one click to defocus and a second to activate.
	if event is InputEventMouseMotion:
		var motion := event as InputEventMouseMotion
		var over_ui := _is_ui_bounding_box_hit(motion.position)
		if over_ui and not _pointer_over_ui:
			call_deferred("_release_pose_focus")
		_pointer_over_ui = over_ui
		return
	if event is InputEventMouseButton:
		var mouse := event as InputEventMouseButton
		# Do not consume clicks inside any UI bounding box. The Control itself
		# must receive the click so Align/Capture and the fields activate at once.
		if mouse.pressed and mouse.button_index == MOUSE_BUTTON_LEFT and not _is_ui_bounding_box_hit(mouse.position):
			call_deferred("_release_pose_focus")
		return
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
				base_target_position = target_position
			get_viewport().set_input_as_handled()
		elif key.keycode == KEY_L:
			loop_trajectory = not loop_trajectory
			get_viewport().set_input_as_handled()
		elif key.keycode in [KEY_0, KEY_KP_0, KEY_BACKSPACE]:
			automatic = false
			axis_override = false
			planner.pause()
			tcp_translation_offset = Vector3.ZERO
			robot.set_tcp_translation_offset_flange(Vector3.ZERO)
			base_target_position = robot.urdf_position_to_world(RESET_URDF_POSITION)
			target_position = base_target_position
			target_yaw = 0.0
			_sync_translation_controls()
			get_viewport().set_input_as_handled()


func _configure_pose_spin(spin: SpinBox) -> void:
	if spin == null:
		return
	# Clicking a field still enables text entry, but arrows no longer move the
	# focus rectangle between sibling controls after focus has been released.
	spin.focus_mode = Control.FOCUS_CLICK
	var line_edit := spin.get_line_edit()
	if line_edit != null:
		line_edit.focus_mode = Control.FOCUS_CLICK


func _is_ui_bounding_box_hit(position: Vector2) -> bool:
	# The panels are disjoint, so their rectangles provide an unambiguous
	# pointer collision test without stealing input from the 3D viewport.
	for panel in [overlay_toolbar, header_panel, axis_panel, orientation_panel, translation_panel, pickup_panel, pendant_panel]:
		if is_instance_valid(panel) and panel.get_global_rect().has_point(position):
			return true
	return false


func _on_status_overlay_toggled(visible: bool) -> void:
	header_panel.visible = visible


func _on_joint_overlay_toggled(visible: bool) -> void:
	axis_panel.visible = visible


func _on_orientation_overlay_toggled(visible: bool) -> void:
	orientation_panel.visible = visible


func _on_translation_overlay_toggled(visible: bool) -> void:
	translation_panel.visible = visible


func _on_pickup_overlay_toggled(visible: bool) -> void:
	pickup_panel.visible = visible


func _on_recording_overlay_toggled(visible: bool) -> void:
	recording_indicator.visible = visible


func _on_pendant_overlay_toggled(visible: bool) -> void:
	pendant_panel.visible = visible


func _release_pose_focus() -> void:
	var focus_owner := get_viewport().gui_get_focus_owner()
	if focus_owner != null:
		focus_owner.release_focus()
	# Explicitly clear the viewport focus as well on Godot versions that expose
	# this helper; the method guard keeps the project compatible across 4.x.
	var viewport := get_viewport()
	if viewport.has_method("gui_release_focus"):
		viewport.call("gui_release_focus")


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
	base_target_position = target_position
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


func _on_translation_value_changed(value: float, index: int) -> void:
	if updating_translation_ui:
		return
	automatic = false
	axis_override = true
	planner.pause()
	tcp_translation_offset[index] = value / 1000.0
	# This is TCP calibration, not a Cartesian move command: keep all joints
	# fixed and move only the TCP marker/gizmo away from the J6 flange center.
	robot.set_tcp_translation_offset_flange(tcp_translation_offset)
	target_position = robot.get_tcp_world_position()
	base_target_position = target_position
	reachable = true


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
	base_target_position = target_position
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


func _sync_translation_controls() -> void:
	if translation_spins.is_empty():
		return
	updating_translation_ui = true
	for index in 3:
		var millimetres := tcp_translation_offset[index] * 1000.0
		if absf(translation_spins[index].value - millimetres) > 0.01:
			translation_spins[index].value = millimetres
	updating_translation_ui = false


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
		trajectory2_active = false


func _load_command_line_trajectory2() -> void:
	var args := OS.get_cmdline_user_args()
	var path := ""
	for i in args.size():
		if args[i].begins_with("--trajectory2="):
			path = args[i].substr("--trajectory2=".length())
		elif args[i] == "--trajectory2" and i + 1 < args.size():
			path = args[i + 1]
	if not path.is_empty():
		_apply_trajectory2(path)


func _poll_runtime_trajectory2(delta: float) -> void:
	trajectory2_poll_elapsed += delta
	if trajectory2_poll_elapsed < 0.25:
		return
	trajectory2_poll_elapsed = 0.0
	var goto_path := ProjectSettings.globalize_path("res://.runtime/trajectory2.goto")
	if FileAccess.file_exists(goto_path):
		var goto_file := FileAccess.open(goto_path, FileAccess.READ)
		if goto_file != null:
			var goto_text := goto_file.get_as_text()
			var goto_signature := hash(goto_text)
			if goto_signature != trajectory2_goto_signature:
				_apply_runtime_goto(goto_text)
				trajectory2_goto_signature = goto_signature
	var command_path := ProjectSettings.globalize_path("res://.runtime/trajectory2.command")
	if not FileAccess.file_exists(command_path):
		return
	var command_file := FileAccess.open(command_path, FileAccess.READ)
	if command_file == null:
		return
	var command_text := command_file.get_as_text()
	var requested := command_text.get_slice("\n", 0).strip_edges()
	if requested.is_empty():
		return
	var modified := FileAccess.get_modified_time(requested) if FileAccess.file_exists(requested) else 0
	var command_signature := hash(command_text)
	if requested == trajectory2_path and modified == trajectory2_mtime and command_signature == trajectory2_command_signature:
		return
	if pickup_active:
		pending_trajectory2_path = requested
		trajectory2_command_signature = command_signature
		return
	_apply_trajectory2(requested)
	trajectory2_command_signature = command_signature


func _apply_runtime_goto(command_text: String) -> bool:
	var line := command_text.get_slice("\n", 0).strip_edges()
	var fields := line.split(" ", false)
	var values: Array[float] = []
	for field in fields:
		if not field.is_empty():
			if not field.is_valid_float():
				status.text = "GOTO ERROR\nExpected 12 numeric fields"
				return false
			values.append(field.to_float())
	if values.size() != 12:
		status.text = "GOTO ERROR\nExpected 12 numeric fields, got %d" % values.size()
		return false
	var urdf_position := Vector3(values[0], values[1], values[2]) / 1000.0
	var world_position := robot.urdf_position_to_world(urdf_position)
	var joint_radians := PackedFloat32Array()
	for index in 6:
		joint_radians.append(deg_to_rad(values[6 + index]))
	# A goto is an operator jog, not a new looping program. Apply the explicit
	# J1-J6 branch directly so the robot reaches the requested pose without a
	# second IK solve selecting a different elbow/wrist configuration.
	automatic = false
	axis_override = true
	trajectory2_active = false
	planner.pause()
	robot.set_joint_angles_target(joint_radians, true)
	target_position = robot.get_tcp_world_position()
	base_target_position = target_position
	target_world_euler = robot.get_tcp_world_basis().get_euler()
	target_yaw = target_world_euler.y
	var cartesian_error := target_position.distance_to(world_position)
	reachable = cartesian_error < 0.003
	status.text = "GOTO POSE APPLIED  ·  TCP %.1f %.1f %.1f mm  ·  %s" % [
		target_position.x * 1000.0, target_position.y * 1000.0, target_position.z * 1000.0,
		("REACHABLE" if reachable else "CLAMPED / XYZ MISMATCH")]
	return reachable


func _apply_trajectory2(path: String) -> bool:
	var parsed: Dictionary = Parser2.parse_file(path)
	if not parsed.get("ok", false):
		status.text = "TRAJECTORY2 ERROR\n%s" % parsed.get("error", "unknown error")
		return false
	var world_points: Array[Vector3] = []
	var world_orientations: Array[Vector3] = []
	var joint_waypoints: Array[PackedFloat32Array] = []
	var yaw_radians := PackedFloat32Array()
	var pitches: PackedFloat32Array = parsed["pitch_degrees"]
	var rolls: PackedFloat32Array = parsed["roll_degrees"]
	var yaws: PackedFloat32Array = parsed["yaw_degrees"]
	pickup_dock_configured = false
	var parsed_dock: PackedFloat32Array = parsed.get("pickup_dock", PackedFloat32Array()) as PackedFloat32Array
	if parsed_dock.size() == 12:
		pickup_dock_position = robot.urdf_position_to_world(Vector3(parsed_dock[0], parsed_dock[1], parsed_dock[2]) / 1000.0)
		var dock_ros_basis := Basis.from_euler(Vector3(deg_to_rad(parsed_dock[4]), deg_to_rad(parsed_dock[3]), deg_to_rad(parsed_dock[5])))
		pickup_dock_basis = robot.urdf_basis_to_world(dock_ros_basis)
		pickup_dock_configured = true
	for i in parsed.coordinates_mm.size():
		world_points.append(robot.urdf_position_to_world(parsed.coordinates_mm[i] / 1000.0))
		var ros_basis := Basis.from_euler(Vector3(deg_to_rad(rolls[i]), deg_to_rad(pitches[i]), deg_to_rad(yaws[i])))
		world_orientations.append(robot.urdf_basis_to_world(ros_basis).get_euler())
		yaw_radians.append(deg_to_rad(yaws[i]))
		var joint_radians := PackedFloat32Array()
		for joint_degrees in parsed.joint_degrees[i]:
			joint_radians.append(deg_to_rad(joint_degrees))
		joint_waypoints.append(joint_radians)
	if not planner.plan_world_path(world_points, parsed.feed_mm_s, parsed.acceleration_mm_s2,
		parsed.junction_deviation_mm, yaw_radians, parsed.get("waypoint_events", []), world_orientations, joint_waypoints):
		return false
	planner.start()
	loop_trajectory = parsed.loop
	trajectory2_active = true
	automatic = true
	axis_override = false
	trajectory2_path = path
	trajectory2_mtime = FileAccess.get_modified_time(path) if FileAccess.file_exists(path) else 0
	_apply_overlay_settings(parsed.get("overlays", {}))
	status.text = "TRAJECTORY2 LOADED  ·  %s" % path.get_file()
	return true


func _apply_overlay_settings(settings: Dictionary) -> void:
	var values := [settings.get("status", true), settings.get("joints", true),
		settings.get("orientation", settings.get("pose", true)),
		settings.get("translation", settings.get("tcp_offset", true)),
		settings.get("pickup", true), settings.get("recording", true), settings.get("pendant", true)]
	for i in values.size():
		var toggle := overlay_toggles[i]
		if toggle.button_pressed != bool(values[i]):
			toggle.set_pressed_no_signal(bool(values[i]))
		match i:
			0: _on_status_overlay_toggled(bool(values[i]))
			1: _on_joint_overlay_toggled(bool(values[i]))
			2: _on_orientation_overlay_toggled(bool(values[i]))
			3: _on_translation_overlay_toggled(bool(values[i]))
			4: _on_pickup_overlay_toggled(bool(values[i]))
			5: _on_recording_overlay_toggled(bool(values[i]))
			6: _on_pendant_overlay_toggled(bool(values[i]))


func _publish_runtime_pose(delta: float) -> void:
	# Publish at a modest rate: this file is an editor handoff snapshot, not a
	# high-frequency telemetry stream. Emacs C-c C-p reads the latest complete
	# line and appends it as a waypoint.
	runtime_pose_publish_elapsed += delta
	if runtime_pose_publish_elapsed < 0.20 or robot == null:
		return
	runtime_pose_publish_elapsed = 0.0
	var directory := ProjectSettings.globalize_path("res://.runtime")
	DirAccess.make_dir_recursive_absolute(directory)
	var path := directory.path_join("trajectory2.pose")
	var position := robot.get_tcp_urdf_position() * 1000.0
	var ros_euler := robot.world_basis_to_urdf(robot.get_tcp_world_basis()).get_euler()
	var joints := robot.get_joint_angles()
	var fields := [position.x, position.y, position.z, rad_to_deg(ros_euler.y), rad_to_deg(ros_euler.x), rad_to_deg(ros_euler.z)]
	for joint in joints:
		fields.append(rad_to_deg(joint))
	var values := ""
	for index in fields.size():
		values += (" " if index > 0 else "") + ("%.4f" % fields[index])
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file != null:
		file.store_string("# Live FR3 pose: X Y Z Pitch Roll Yaw J1 J2 J3 J4 J5 J6\n" + values + "\n")


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
	base_target_position = target_position
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


func _set_recording_indicator(text: String, color: Color) -> void:
	if recording_indicator == null:
		return
	recording_indicator.text = text
	recording_indicator.modulate = color


func _on_recording_started(output_path: String) -> void:
	_set_recording_indicator(
		"● RECORDING  ·  F9 to stop\n%s" % output_path.get_file(),
		Color("ff5f5f")
	)


func _on_recording_finalizing(output_path: String) -> void:
	_set_recording_indicator(
		"◐ FINALIZING VIDEO\n%s" % output_path.get_file(),
		Color("ffc266")
	)


func _on_recording_stopped(output_path: String) -> void:
	_set_recording_indicator(
		"✓ VIDEO SAVED\nrecordings/%s" % output_path.get_file(),
		Color("70e58a")
	)
	get_tree().create_timer(4.0).timeout.connect(func() -> void:
		if runtime_recorder == null or (not runtime_recorder.is_recording and not runtime_recorder.is_finalizing):
			_set_recording_indicator("● REC OFF  ·  F9 to start", Color("9aa8b8"))
	)


func _on_recording_failed(message: String) -> void:
	_set_recording_indicator("✕ RECORDING ERROR\n%s" % message, Color("ff796d"))

extends Node3D

@onready var robot: MG400Robot = $MG400Robot
@onready var target_marker: MeshInstance3D = $TargetMarker
@onready var camera: EditorOrbitCamera = $Camera3D
@onready var mode_label: Label = $UI/Margin/VBox/Mode
@onready var pose_label: Label = $UI/Margin/VBox/Pose
@onready var state_label: Label = $UI/Margin/VBox/State
@onready var camera_label: Label = $UI/Margin/VBox/CameraPose
@onready var camera_orbit_label: Label = $UI/Margin/VBox/CameraOrbit
@onready var overlay_panel: PanelContainer = $UI/Margin
@onready var overlay_toggle: CheckBox = $UI/OverlayToggle
@onready var ik_indicator: Label = $UI/IKIndicator

var automatic := true
var elapsed := 0.0
var target_position := Vector3.ZERO
var target_yaw := 0.0
var reachable := true


func _ready() -> void:
	target_position = robot.urdf_position_to_world(Vector3(0.30, 0.0, 0.25))
	reachable = robot.set_tcp_target_world(target_position, target_yaw, true)
	robot.target_reachability_changed.connect(_on_reachability_changed)
	overlay_toggle.toggled.connect(_on_overlay_toggled)
	_on_overlay_toggled(overlay_toggle.button_pressed)
	_update_ik_indicator()
	_update_ui()


func _process(delta: float) -> void:
	if Input.is_key_pressed(KEY_ESCAPE):
		get_tree().quit()

	if automatic:
		elapsed += delta
		# Cartesian figure-eight path expressed directly in Godot world space.
		var phase := elapsed * 0.85
		var urdf_target := Vector3(
			0.300 + 0.045 * sin(phase),
			0.070 * sin(phase * 0.5),
			0.245 + 0.045 * cos(phase)
		)
		target_position = robot.urdf_position_to_world(urdf_target)
		target_yaw = 0.65 * sin(phase * 0.7)
	else:
		_update_manual_target(delta)

	reachable = robot.set_tcp_target_world(target_position, target_yaw)
	target_marker.global_position = target_position
	target_marker.rotation.y = target_yaw
	_update_ik_indicator()
	_update_ui()


func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	if event.keycode == KEY_SPACE:
		automatic = not automatic
		if not automatic:
			target_position = robot.get_tcp_world_position()
			target_yaw = robot.get_joint_angles().x + robot.get_joint_angles().w
	elif event.keycode in [KEY_0, KEY_KP_0, KEY_BACKSPACE]:
		automatic = false
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
	if Input.is_key_pressed(KEY_Q):
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

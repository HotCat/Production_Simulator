class_name EditorOrbitCamera
extends Camera3D
## Godot-editor-style orbit camera with macOS trackpad gesture support.

@export_category("Initial View")
@export var initial_focus := Vector3(0.22, 0.20, 0.0)
@export_range(-180.0, 180.0, 0.1) var initial_yaw_degrees := 34.8
@export_range(-85.0, 85.0, 0.1) var initial_pitch_degrees := 18.3
@export_range(0.1, 10.0, 0.01) var initial_distance := 0.925

@export_category("Limits")
@export_range(0.01, 10.0, 0.01) var minimum_distance := 0.16
@export_range(0.1, 20.0, 0.1) var maximum_distance := 4.0

@export_category("Sensitivity")
@export_range(0.001, 0.2, 0.001) var trackpad_orbit_sensitivity := 0.035
@export_range(0.001, 0.5, 0.001) var trackpad_zoom_sensitivity := 0.12
@export_range(0.001, 0.2, 0.001) var trackpad_pan_sensitivity := 0.022
@export_range(0.001, 0.05, 0.001) var mouse_orbit_sensitivity := 0.005
@export_range(0.001, 0.05, 0.001) var mouse_zoom_sensitivity := 0.010
@export_range(0.0001, 0.02, 0.0001) var mouse_pan_sensitivity := 0.002

var _focus := Vector3.ZERO
var _yaw := 0.0
var _pitch := 0.0
var _distance := 1.0


func _ready() -> void:
	reset_view()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventPanGesture:
		_handle_trackpad_pan(event as InputEventPanGesture)
		get_viewport().set_input_as_handled()
	elif event is InputEventMagnifyGesture:
		var magnify := event as InputEventMagnifyGesture
		if magnify.factor > 0.0:
			_distance = clampf(
				_distance / magnify.factor, minimum_distance, maximum_distance
			)
			_update_transform()
		get_viewport().set_input_as_handled()
	elif event is InputEventMouseMotion:
		var motion := event as InputEventMouseMotion
		if motion.button_mask & MOUSE_BUTTON_MASK_MIDDLE:
			_handle_middle_mouse(motion)
			get_viewport().set_input_as_handled()
	elif event is InputEventMouseButton:
		var button := event as InputEventMouseButton
		if button.pressed and button.button_index == MOUSE_BUTTON_WHEEL_UP:
			_zoom(-0.12)
			get_viewport().set_input_as_handled()
		elif button.pressed and button.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_zoom(0.12)
			get_viewport().set_input_as_handled()


func _unhandled_key_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == KEY_C:
			reset_view()
			get_viewport().set_input_as_handled()


func reset_view() -> void:
	_focus = initial_focus
	_yaw = deg_to_rad(initial_yaw_degrees)
	_pitch = deg_to_rad(initial_pitch_degrees)
	_distance = initial_distance
	_update_transform()


func get_focus_point() -> Vector3:
	return _focus


func get_orbit_degrees() -> Vector2:
	return Vector2(rad_to_deg(_yaw), rad_to_deg(_pitch))


func get_orbit_distance() -> float:
	return _distance


func _handle_trackpad_pan(event: InputEventPanGesture) -> void:
	# Modifier priority mirrors the requested macOS viewport controls.
	if event.ctrl_pressed:
		_zoom(event.delta.y * trackpad_zoom_sensitivity)
	elif event.shift_pressed:
		_pan(event.delta, trackpad_pan_sensitivity)
	else:
		_orbit(event.delta, trackpad_orbit_sensitivity)


func _handle_middle_mouse(event: InputEventMouseMotion) -> void:
	# Mouse fallback: middle drag orbits, Ctrl-middle dollies, Shift-middle pans.
	if event.ctrl_pressed:
		_zoom(event.relative.y * mouse_zoom_sensitivity)
	elif event.shift_pressed:
		_pan(event.relative, mouse_pan_sensitivity)
	else:
		_orbit(event.relative, mouse_orbit_sensitivity)


func _orbit(delta: Vector2, sensitivity: float) -> void:
	_yaw -= delta.x * sensitivity
	_pitch = clampf(
		_pitch - delta.y * sensitivity,
		deg_to_rad(-85.0),
		deg_to_rad(85.0)
	)
	_update_transform()


func _zoom(amount: float) -> void:
	_distance = clampf(
		_distance * exp(amount), minimum_distance, maximum_distance
	)
	_update_transform()


func _pan(delta: Vector2, sensitivity: float) -> void:
	var camera_right := global_transform.basis.x.normalized()
	var camera_up := global_transform.basis.y.normalized()
	_focus += (
		-camera_right * delta.x + camera_up * delta.y
	) * _distance * sensitivity
	_update_transform()


func _update_transform() -> void:
	var horizontal_distance := cos(_pitch) * _distance
	var offset := Vector3(
		sin(_yaw) * horizontal_distance,
		sin(_pitch) * _distance,
		cos(_yaw) * horizontal_distance
	)
	global_position = _focus + offset
	look_at(_focus, Vector3.UP)

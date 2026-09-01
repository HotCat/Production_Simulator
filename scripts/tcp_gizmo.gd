class_name TcpGizmo
extends Node3D
## Small runtime Cartesian gizmo for dragging a TCP along world X/Y/Z.
## It uses the active Camera3D and a screen-facing drag plane, so the handles
## remain usable from any editor-style orbit view without physics colliders.

signal target_dragged(world_position: Vector3)

@export_range(0.02, 0.30, 0.01) var axis_length := 0.10
@export_range(2.0, 30.0, 1.0) var pick_radius_pixels := 14.0

var _axes: Array[Vector3] = [Vector3.RIGHT, Vector3.UP, Vector3.BACK]
var _axis_colors: Array[Color] = [Color("f04444"), Color("55d878"), Color("4c8dff")]
var _drag_axis := -1
var _drag_start_position := Vector3.ZERO
var _drag_start_scalar := 0.0
var _drag_plane_normal := Vector3.ZERO


func _ready() -> void:
	_build_handles()


func _unhandled_input(event: InputEvent) -> void:
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return
	if event is InputEventMouseButton:
		var button := event as InputEventMouseButton
		if button.button_index != MOUSE_BUTTON_LEFT:
			return
		if button.pressed and _drag_axis < 0:
			var picked := _pick_axis(camera, button.position)
			if picked >= 0:
				var hit: Variant = _screen_plane_hit(camera, button.position, global_position, camera.global_basis.z.normalized())
				if hit != null:
					_drag_axis = picked
					_drag_start_position = global_position
					_drag_plane_normal = camera.global_basis.z.normalized()
					_drag_start_scalar = (hit as Vector3).dot(_axes[_drag_axis])
					get_viewport().set_input_as_handled()
		elif not button.pressed and _drag_axis >= 0:
			_drag_axis = -1
			get_viewport().set_input_as_handled()
	elif event is InputEventMouseMotion and _drag_axis >= 0:
		var motion := event as InputEventMouseMotion
		var hit: Variant = _screen_plane_hit(camera, motion.position, _drag_start_position, _drag_plane_normal)
		if hit != null:
			var scalar := (hit as Vector3).dot(_axes[_drag_axis])
			var candidate := _drag_start_position + _axes[_drag_axis] * (scalar - _drag_start_scalar)
			target_dragged.emit(candidate)
			get_viewport().set_input_as_handled()


func _build_handles() -> void:
	var center_mesh := SphereMesh.new()
	center_mesh.radius = 0.009
	center_mesh.height = 0.018
	var center := MeshInstance3D.new()
	center.name = "Center"
	center.mesh = center_mesh
	center.material_override = _make_material(Color("ffd447"))
	add_child(center)
	for i in 3:
		var axis := _axes[i]
		var shaft_mesh := BoxMesh.new()
		shaft_mesh.size = Vector3(0.004, 0.004, 0.004)
		if i == 0:
			shaft_mesh.size = Vector3(axis_length, 0.005, 0.005)
		elif i == 1:
			shaft_mesh.size = Vector3(0.005, axis_length, 0.005)
		else:
			shaft_mesh.size = Vector3(0.005, 0.005, axis_length)
		var shaft := MeshInstance3D.new()
		shaft.name = "Axis%d" % (i + 1)
		shaft.position = axis * axis_length * 0.5
		shaft.mesh = shaft_mesh
		shaft.material_override = _make_material(_axis_colors[i])
		add_child(shaft)
		var tip_mesh := SphereMesh.new()
		tip_mesh.radius = 0.009
		tip_mesh.height = 0.018
		var tip := MeshInstance3D.new()
		tip.name = "Axis%dTip" % (i + 1)
		tip.position = axis * axis_length
		tip.mesh = tip_mesh
		tip.material_override = _make_material(_axis_colors[i])
		add_child(tip)


func _make_material(color: Color) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.emission_enabled = true
	material.emission = color
	material.emission_energy_multiplier = 1.2
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	return material


func _pick_axis(camera: Camera3D, screen_position: Vector2) -> int:
	var center := camera.unproject_position(global_position)
	var best_axis := -1
	var best_distance := pick_radius_pixels
	for i in 3:
		var end := camera.unproject_position(global_position + _axes[i] * axis_length)
		var distance := _point_segment_distance(screen_position, center, end)
		if distance < best_distance:
			best_distance = distance
			best_axis = i
	return best_axis


func _point_segment_distance(point: Vector2, start: Vector2, finish: Vector2) -> float:
	var segment := finish - start
	var length_squared := segment.length_squared()
	if length_squared < 0.000001:
		return point.distance_to(start)
	var fraction := clampf((point - start).dot(segment) / length_squared, 0.0, 1.0)
	return point.distance_to(start + segment * fraction)


func _screen_plane_hit(camera: Camera3D, screen_position: Vector2, plane_point: Vector3, plane_normal: Vector3) -> Variant:
	var ray_origin := camera.project_ray_origin(screen_position)
	var ray_direction := camera.project_ray_normal(screen_position)
	var denominator := ray_direction.dot(plane_normal)
	if absf(denominator) < 0.000001:
		return null
	var distance := (plane_point - ray_origin).dot(plane_normal) / denominator
	if distance < 0.0:
		return null
	return ray_origin + ray_direction * distance

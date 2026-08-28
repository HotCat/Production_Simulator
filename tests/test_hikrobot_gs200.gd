extends SceneTree

const BODY_SIZE_GODOT := Vector3(0.029, 0.029, 0.042)
const ASSEMBLY_SIZE_GODOT := Vector3(0.033, 0.033, 0.09445)
const EXPECTED_MG400_MM := Vector3(262.9, 117.0, 291.3)
const EXPECTED_GODOT_POSITION := Vector3(0.2629, 0.302346, -0.009776026)
const EXPECTED_GODOT_BASIS := Basis(
	Vector3(-0.9995292, -0.019541625, -0.02365536),
	Vector3(0.0, 0.7709579, -0.63688606),
	Vector3(0.03068308, -0.6365862, -0.77059495)
)
const TOLERANCE := 0.00006


func aggregate_bounds(root_node: Node) -> AABB:
	var minimum := Vector3(INF, INF, INF)
	var maximum := Vector3(-INF, -INF, -INF)
	for child in root_node.find_children("*", "MeshInstance3D", true, false):
		var mesh_instance := child as MeshInstance3D
		var bounds := mesh_instance.get_aabb()
		for corner_index in range(8):
			var corner := Vector3(
				bounds.position.x + bounds.size.x * float(corner_index & 1),
				bounds.position.y + bounds.size.y * float((corner_index >> 1) & 1),
				bounds.position.z + bounds.size.z * float((corner_index >> 2) & 1)
			)
			var transformed := mesh_instance.global_transform * corner
			minimum = minimum.min(transformed)
			maximum = maximum.max(transformed)
	return AABB(minimum, maximum - minimum)


func _initialize() -> void:
	var packed: PackedScene = load("res://assets/hikrobot_gs200/hikrobot_gs200.glb")
	var camera_model := packed.instantiate()
	root.add_child(camera_model)
	await process_frame

	var body := camera_model.find_child("BodyHousing_EXACT_29x42x29mm", true, false) as MeshInstance3D
	if body == null:
		push_error("Could not find the dimension-authoritative GS200 body mesh")
		quit(1)
		return

	var body_bounds := body.get_aabb()
	var assembly_bounds := aggregate_bounds(camera_model)
	print("GS200 imported body dimensions: ", body_bounds.size)
	print("GS200 camera + lens dimensions: ", assembly_bounds.size)

	var body_ok := body_bounds.size.distance_to(BODY_SIZE_GODOT) <= TOLERANCE
	var assembly_ok := assembly_bounds.size.distance_to(ASSEMBLY_SIZE_GODOT) <= TOLERANCE

	var main_packed: PackedScene = load("res://main.tscn")
	var main_scene := main_packed.instantiate()
	root.add_child(main_scene)
	await process_frame
	var installed_camera := main_scene.get_node("HIKROBOTGS200") as Node3D
	var placement_ok := installed_camera.global_position.distance_to(EXPECTED_GODOT_POSITION) <= TOLERANCE
	var orientation_ok := installed_camera.global_basis.is_equal_approx(EXPECTED_GODOT_BASIS)
	print("GS200 requested MG400 position (mm): ", EXPECTED_MG400_MM)
	print("GS200 visually adjusted Godot position (m): ", installed_camera.global_position)
	print("GS200 visually adjusted Godot basis: ", installed_camera.global_basis)

	if not body_ok or not assembly_ok or not placement_ok or not orientation_ok:
		push_error("GS200 imported dimensions verification failed")
		quit(1)
	else:
		print("GS200 dimensions, placement, and lens orientation verification passed")
		quit(0)

extends SceneTree

const PRODUCT_SIZE := Vector3(0.03795, 0.00400, 0.04600)
const CONVEYOR_MODEL_HEIGHT := 0.070
const TOLERANCE := 0.00002


func aggregate_world_bounds(root_node: Node) -> AABB:
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
			var world_corner := mesh_instance.to_global(corner)
			minimum = minimum.min(world_corner)
			maximum = maximum.max(world_corner)
	return AABB(minimum, maximum - minimum)


func _initialize() -> void:
	var packed: PackedScene = load("res://main.tscn")
	var scene: Node3D = packed.instantiate()
	root.add_child(scene)
	await process_frame

	var conveyor := scene.get_node("Conveyor") as Node3D
	var product := scene.get_node("Product") as Node3D
	var bounds := aggregate_world_bounds(product)
	var product_center := bounds.get_center()
	var expected_center := Vector3(
		conveyor.global_position.x,
		conveyor.global_position.y + CONVEYOR_MODEL_HEIGHT + PRODUCT_SIZE.y * 0.5,
		conveyor.global_position.z
	)
	var expected_bottom := conveyor.global_position.y + CONVEYOR_MODEL_HEIGHT

	print("Product world center: ", product_center)
	print("Product world dimensions: ", bounds.size)
	print("Product bottom / conveyor belt top: ", bounds.position.y, " / ", expected_bottom)

	var dimensions_ok := bounds.size.distance_to(PRODUCT_SIZE) <= TOLERANCE
	var center_ok := product_center.distance_to(expected_center) <= TOLERANCE
	var contact_ok := absf(bounds.position.y - expected_bottom) <= TOLERANCE
	if not dimensions_ok or not center_ok or not contact_ok:
		push_error("Product size or conveyor-center placement verification failed")
		quit(1)
	else:
		print("Product size and conveyor-center placement verification passed")
		quit(0)

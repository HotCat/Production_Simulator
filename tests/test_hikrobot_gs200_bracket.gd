extends SceneTree

const BASE_POSITION := Vector3(0.440000, 0.0, -0.009776026)
const CAMERA_BODY_HEIGHT := 0.029
const MOUNT_PLATE_THICKNESS := 0.004
const MINIMUM_CONVEYOR_CLEARANCE := 0.015
const TOLERANCE := 0.00020


func world_bounds(root_node: Node) -> AABB:
	var minimum := Vector3(INF, INF, INF)
	var maximum := Vector3(-INF, -INF, -INF)
	var mesh_nodes := root_node.find_children("*", "MeshInstance3D", true, false)
	if root_node is MeshInstance3D:
		mesh_nodes.push_back(root_node)
	for child in mesh_nodes:
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


func mesh_world_center(mesh_instance: MeshInstance3D) -> Vector3:
	return mesh_instance.to_global(mesh_instance.get_aabb().get_center())


func _initialize() -> void:
	var packed: PackedScene = load("res://main.tscn")
	var scene := packed.instantiate() as Node3D
	root.add_child(scene)
	await process_frame

	var bracket := scene.get_node("HIKROBOTGS200Bracket") as Node3D
	var camera := scene.get_node("HIKROBOTGS200") as Node3D
	var conveyor := scene.get_node("Conveyor") as Node3D
	var base_plate := bracket.find_child("FloorBasePlate_100x120x12mm", true, false) as MeshInstance3D
	var mount_plate := bracket.find_child("CameraMountPlate_28x32x4mm", true, false) as MeshInstance3D
	if base_plate == null or mount_plate == null:
		push_error("Could not find the bracket's dimension-authoritative mounting meshes")
		quit(1)
		return

	var bracket_bounds := world_bounds(bracket)
	var base_bounds := world_bounds(base_plate)
	var conveyor_bounds := world_bounds(conveyor)
	var mount_center := mesh_world_center(mount_plate)
	var expected_mount_center := camera.global_position + camera.global_basis.y * (
		CAMERA_BODY_HEIGHT * 0.5 + MOUNT_PLATE_THICKNESS * 0.5
	)
	var conveyor_clearance := base_bounds.position.x - conveyor_bounds.end.x

	print("GS200 bracket base position: ", bracket.global_position)
	print("GS200 bracket floor contact Y: ", bracket_bounds.position.y)
	print("GS200 bracket / conveyor clearance (mm): ", conveyor_clearance * 1000.0)
	print("GS200 mount plate center: ", mount_center)
	print("GS200 expected camera-interface center: ", expected_mount_center)

	var position_ok := bracket.global_position.distance_to(BASE_POSITION) <= TOLERANCE
	var floor_ok := absf(bracket_bounds.position.y) <= TOLERANCE
	var clearance_ok := conveyor_clearance >= MINIMUM_CONVEYOR_CLEARANCE
	var attachment_ok := mount_center.distance_to(expected_mount_center) <= TOLERANCE
	if not position_ok or not floor_ok or not clearance_ok or not attachment_ok:
		push_error("GS200 bracket placement or attachment verification failed")
		quit(1)
	else:
		print("GS200 bracket placement, clearance, and camera attachment verification passed")
		quit(0)

extends SceneTree

const POSITION_TOLERANCE := 0.000001
const NOZZLE_TIP_OFFSET := 0.0261


func _initialize() -> void:
	var packed: PackedScene = load("res://main.tscn")
	var scene := packed.instantiate() as Node3D
	root.add_child(scene)
	await process_frame
	var robot: MG400Robot = scene.get_node("MG400Robot")
	var tcp: Marker3D = robot.find_child("TCP", true, false) as Marker3D
	var nozzle: Node3D = tcp.get_node("SMTNozzle")
	var label: Node3D = nozzle.get_node("ProductLabel")
	var attached := nozzle != null and nozzle.get_parent() == tcp
	var label_attached := label != null and label.get_parent() == nozzle
	var origin_ok := attached and nozzle.global_position.distance_to(tcp.global_position) <= POSITION_TOLERANCE
	var mesh_count := 0
	var maximum_diameter := 0.0
	var mount_y := INF
	var tip_y := -INF
	var label_min_y := INF
	var label_max_y := -INF
	var label_min_x := INF
	var label_max_x := -INF
	var label_min_z := INF
	var label_max_z := -INF
	for child in nozzle.find_children("*", "MeshInstance3D", true, false):
		var mesh_instance := child as MeshInstance3D
		if mesh_instance == null or mesh_instance.mesh == null:
			continue
		if label_attached and label.is_ancestor_of(mesh_instance):
			continue
		mesh_count += 1
		var size := mesh_instance.mesh.get_aabb().size
		maximum_diameter = maxf(maximum_diameter, maxf(size.x, size.y))
		var center := mesh_instance.to_global(mesh_instance.get_aabb().get_center())
		if child.name == "MountingBlock_8x8x6mm":
			mount_y = center.y
		if child.name == "VacuumTip_4mm":
			tip_y = center.y
	var dimensions_ok := mesh_count >= 10 and maximum_diameter <= 0.013 and nozzle.has_meta("overall_dimensions_mm")
	var orientation_ok := tip_y < mount_y - 0.02
	if label_attached:
		for child in label.find_children("*", "MeshInstance3D", true, false):
			var label_mesh := child as MeshInstance3D
			var label_aabb := label_mesh.get_aabb()
			for corner_index in range(8):
				var corner := Vector3(
					label_aabb.position.x + label_aabb.size.x * float(corner_index & 1),
					label_aabb.position.y + label_aabb.size.y * float((corner_index >> 1) & 1),
					label_aabb.position.z + label_aabb.size.z * float((corner_index >> 2) & 1)
				)
				var world_corner := label_mesh.to_global(corner)
				label_min_y = minf(label_min_y, world_corner.y)
				label_max_y = maxf(label_max_y, world_corner.y)
				label_min_x = minf(label_min_x, world_corner.x)
				label_max_x = maxf(label_max_x, world_corner.x)
				label_min_z = minf(label_min_z, world_corner.z)
				label_max_z = maxf(label_max_z, world_corner.z)
	var label_dimensions_ok := label_attached and label.has_meta("dimensions_mm") \
		and (label_max_y - label_min_y) <= 0.00025 \
		and (label_max_x - label_min_x) <= 0.0271 \
		and (label_max_z - label_min_z) <= 0.0095
	var label_placement_ok := label_attached \
		and label.global_position.distance_to(tcp.global_position + Vector3(0.0, -NOZZLE_TIP_OFFSET, 0.0)) <= POSITION_TOLERANCE
	print("SMT nozzle mesh parts: ", mesh_count)
	print("SMT nozzle maximum component diameter (m): ", maximum_diameter)
	print("SMT nozzle mount/tip Y (m): ", mount_y, " / ", tip_y)
	print("Product label thickness along Y (m): ", label_max_y - label_min_y)
	if not attached or not origin_ok or not dimensions_ok or not orientation_ok \
		or not label_dimensions_ok or not label_placement_ok:
		push_error("SMT nozzle attachment and dimensions verification failed")
		quit(1)
	else:
		print("SMT nozzle attachment and dimensions verification passed")
		quit(0)

extends SceneTree

const EXPECTED_GODOT_POSITION := Vector3(0.0530, 0.1504, -0.2314)


func _initialize() -> void:
	var packed: PackedScene = load("res://main.tscn")
	var scene: Node3D = packed.instantiate()
	root.add_child(scene)
	await process_frame
	var marker: Marker3D = scene.get_node("LabelStripper/EmittedLabelCenter")
	var origin_error := marker.global_position.distance_to(EXPECTED_GODOT_POSITION)
	var emitted := scene.get_node("LabelStripper/Model").find_child(
		"EmittedLabel_CENTER_ORIGIN", true, false
	) as MeshInstance3D
	var emitted_center := emitted.to_global(emitted.get_aabb().get_center())
	var mesh_origin_error := emitted_center.distance_to(marker.global_position)
	var minimum_y := INF
	for child in scene.get_node("LabelStripper/Model").find_children("*", "MeshInstance3D", true, false):
		var mesh_instance := child as MeshInstance3D
		var bounds := mesh_instance.get_aabb()
		for corner_index in range(8):
			var corner := Vector3(
				bounds.position.x + bounds.size.x * float(corner_index & 1),
				bounds.position.y + bounds.size.y * float((corner_index >> 1) & 1),
				bounds.position.z + bounds.size.z * float((corner_index >> 2) & 1)
			)
			minimum_y = minf(minimum_y, mesh_instance.to_global(corner).y)
	print("Label center Godot world: ", marker.global_position)
	print("Imported emitted-label mesh center: ", emitted_center)
	print("Label stripper minimum world Y: ", minimum_y)
	if origin_error > 0.000001 or mesh_origin_error > 0.0001 or absf(minimum_y) > 0.0002:
		push_error("Label stripper placement verification failed")
		quit(1)
	else:
		print("Label stripper placement verification passed")
		quit(0)

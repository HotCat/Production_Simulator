extends SceneTree


func _initialize() -> void:
	var packed_scene: PackedScene = load("res://main.tscn")
	var scene: Node = packed_scene.instantiate()
	root.add_child(scene)
	await process_frame
	await process_frame
	await process_frame
	var image := root.get_texture().get_image()
	var output := ProjectSettings.globalize_path("res://tests/mg400_preview.png")
	var error := image.save_png(output)
	if error != OK:
		push_error("Could not save preview: %s" % error)
		quit(1)
	else:
		print("Saved preview to ", output)
		quit(0)

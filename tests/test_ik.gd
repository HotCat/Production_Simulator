extends SceneTree

const TEST_POINTS := [
	Vector3(0.20, 0.00, 0.20),
	Vector3(0.25, 0.05, 0.25),
	Vector3(0.30, -0.06, 0.30),
	Vector3(0.35, 0.04, 0.20),
]


func _initialize() -> void:
	var robot := MG400Robot.new()
	root.add_child(robot)
	await process_frame
	var failed := false
	for urdf_point in TEST_POINTS:
		var world_point := robot.urdf_position_to_world(urdf_point)
		var reachable := robot.set_tcp_target_world(world_point, 0.25, true)
		var error := robot.get_tcp_world_position().distance_to(world_point)
		print("IK ", urdf_point, " error=", error)
		if not reachable or error > 0.0001:
			failed = true
	if failed:
		push_error("MG400 IK verification failed")
		quit(1)
	else:
		print("MG400 IK verification passed")
		quit(0)

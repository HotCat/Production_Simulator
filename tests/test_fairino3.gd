extends SceneTree

func _initialize() -> void:
	var scene: Node = load("res://fairino3_demo.tscn").instantiate()
	root.add_child(scene)
	await process_frame
	var robot: Fairino3Robot = scene.get_node("Fairino3Robot")
	var passed := true
	var tcp := robot.find_child("TCP", true, false) as Marker3D
	passed = passed and tcp != null and tcp.get_parent().name == "j6"
	passed = passed and tcp.position.distance_to(Vector3(0.0, 0.0, 0.1)) < 0.000001
	passed = passed and scene.get_node_or_null("TcpGizmo") != null
	passed = passed and scene.get_node_or_null("UI/OrientationPanel/Margin/VBox/PoseReachability") != null
	for i in 3:
		passed = passed and scene.get_node_or_null("UI/OrientationPanel/Margin/VBox/Orientation%dRow/SpinBox" % i) != null
	for i in 6:
		passed = passed and scene.get_node_or_null("UI/AxisPanel/Margin/VBox/Axis%dRow/SpinBox" % (i + 1)) != null
	scene.call("_on_axis_value_changed", 25.0, 0)
	passed = passed and not scene.automatic and scene.axis_override
	passed = passed and absf(rad_to_deg(robot.get_joint_angles()[0]) - 25.0) < 0.01
	for point in [Vector3(0.22, 0.0, 0.30), Vector3(0.30, 0.08, 0.24), Vector3(0.25, -0.08, 0.30)]:
		var target := robot.urdf_position_to_world(point)
		var reachable := robot.set_tcp_target_world(target, 0.0, true)
		var error := robot.get_tcp_world_position().distance_to(target)
		print("Fairino3 IK ", point, " error=", error, " reachable=", reachable, " q=", robot.get_joint_angles())
		passed = passed and reachable and error < 0.002
	var pose_position := robot.get_tcp_world_position()
	var pose_basis := robot.get_tcp_world_basis()
	robot.apply_joint_angles(PackedFloat32Array([0.2, -2.8, 2.1, 1.0, 0.4, -0.5]))
	var pose_reachable := robot.set_tcp_target_pose_world(pose_position, pose_basis, true)
	var pose_position_error := robot.get_tcp_world_position().distance_to(pose_position)
	var pose_delta := (pose_basis * robot.get_tcp_world_basis().inverse()).get_rotation_quaternion()
	var pose_orientation_error := rad_to_deg(pose_delta.get_angle())
	print("Fairino3 pose IK position error=", pose_position_error, " orientation error deg=", pose_orientation_error, " reachable=", pose_reachable, " q=", robot.get_joint_angles())
	passed = passed and pose_reachable and pose_position_error < 0.002 and pose_orientation_error < 1.0
	var flange_basis := Fairino3Robot.FLANGE_PARALLEL_TO_BASE_BASIS
	robot.apply_joint_angles(PackedFloat32Array([deg_to_rad(3.4), deg_to_rad(-115.1), deg_to_rad(-64.9), deg_to_rad(-88.0), deg_to_rad(91.0), deg_to_rad(86.6)]))
	scene.target_position = robot.get_tcp_world_position()
	scene.call("_on_align_world_pressed")
	var flange_normal := robot.get_tcp_world_basis().z.normalized()
	var aligned_normal_error := rad_to_deg(acos(clampf(flange_normal.dot(Vector3.DOWN), -1.0, 1.0)))
	print("Fairino3 flange normal-to-world-minus-Y error deg=", aligned_normal_error, " normal=", flange_normal, " reachable=", scene.reachable, " q=", robot.get_joint_angles())
	passed = passed and absf(flange_basis.z.dot(Vector3.DOWN) - 1.0) < 0.000001 and aligned_normal_error < 2.0
	if not passed:
		push_error("Fairino3 numerical IK verification failed")
		quit(1)
	else:
		print("Fairino3 six-axis IK verification passed")
		quit(0)

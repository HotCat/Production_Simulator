extends SceneTree

const PlannerScript = preload("res://scripts/cartesian_trajectory_planner.gd")
const POSITION_TOLERANCE := 0.0002


func _initialize() -> void:
	var packed: PackedScene = load("res://main.tscn")
	var scene := packed.instantiate() as Node3D
	root.add_child(scene)
	await process_frame

	var points_mm: Array[Vector3] = [
		Vector3(240.0, 0.0, 245.0),
		Vector3(300.0, 0.0, 245.0),
		Vector3(300.0, 40.0, 245.0),
	]
	var planned: bool = scene.call("set_cartesian_trajectory_urdf", points_mm, 100.0, 500.0, 1.0)
	var planner: CartesianTrajectoryPlanner = scene.get("trajectory")
	var robot: MG400Robot = scene.get_node("MG400Robot")
	var segments_ok: bool = planned and planner.get_segment_count() == 2
	var feed_ok: bool = absf(planner.get_feed_speed_mm_s() - 100.0) <= 0.001
	var points_reachable := true
	for point_mm in points_mm:
		points_reachable = points_reachable and robot.set_tcp_target_world(
			robot.urdf_position_to_world(point_mm / 1000.0),
			0.0,
			true
		)

	planner.start()
	while planner.is_running():
		planner.advance(0.002)
	var final_world: Vector3 = robot.urdf_position_to_world(points_mm[-1] / 1000.0)
	var final_ok := planner.get_current_position().distance_to(final_world) <= POSITION_TOLERANCE
	var complete_ok := planner.is_completed() and planner.get_current_speed_mm_s() == 0.0

	print("Main trajectory segments: ", planner.get_segment_count())
	print("Main trajectory duration (s): ", planner.get_total_seconds())
	if not segments_ok or not feed_ok or not points_reachable or not final_ok or not complete_ok:
		push_error("Main MG400 trajectory integration verification failed")
		quit(1)
	else:
		print("Main MG400 trajectory integration verification passed")
		quit(0)

extends SceneTree

## Regression coverage for return-product completion. A command on the final
## trajectory waypoint must leave the robot at return_dock instead of replaying
## the main planner's cached final pose on the following frame.

func _initialize() -> void:
	var scene: Node = load("res://fairino3_demo.tscn").instantiate()
	root.add_child(scene)
	await process_frame

	var points: Array[Vector3] = [Vector3.ZERO, Vector3(0.1, 0.0, 0.0)]
	var planned: bool = scene.planner.plan_world_path(points, 100.0, 500.0)
	scene.planner.start()
	scene.planner.advance(scene.planner.get_total_seconds() + 0.1)
	scene.resume_trajectory2_after_pickup = true
	scene.return_active = true
	scene.automatic = false
	scene.axis_override = true
	scene.call("_finish_return_product")

	var completed_stays_stopped: bool = (
		scene.planner.get_state() == scene.Planner.MotionState.COMPLETED
		and not scene.automatic
		and scene.axis_override
	)

	# A command fired before the final waypoint still pauses the main program and
	# must resume it after the internal return sequence completes.
	scene.planner.plan_world_path(points, 100.0, 500.0)
	scene.planner.start()
	scene.planner.pause()
	scene.resume_trajectory2_after_pickup = true
	scene.return_active = true
	scene.automatic = false
	scene.axis_override = true
	scene.call("_finish_return_product")
	var paused_program_resumes: bool = (
		scene.planner.get_state() == scene.Planner.MotionState.RUNNING
		and scene.automatic
		and not scene.axis_override
	)

	if planned and completed_stays_stopped and paused_program_resumes:
		print("Return-product completion state verification passed")
		quit(0)
	else:
		push_error("Return-product completion state verification failed")
		quit(1)

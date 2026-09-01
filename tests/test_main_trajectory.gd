extends SceneTree

const PlannerScript = preload("res://scripts/cartesian_trajectory_planner.gd")
const POSITION_TOLERANCE := 0.0002


func _initialize() -> void:
	var packed: PackedScene = load("res://main.tscn")
	var scene := packed.instantiate() as Node3D
	root.add_child(scene)
	await process_frame
	var recording_indicator := scene.get_node("UI/RecordingIndicator") as Label
	var recording_idle_ok: bool = recording_indicator.text.contains("REC OFF")
	scene.call("_on_recording_started", "/tmp/mg400-test.mp4")
	var recording_active_ok: bool = recording_indicator.text.contains("RECORDING")
	scene.call("_on_recording_finalizing", "/tmp/mg400-test.mp4")
	var recording_finalizing_ok: bool = recording_indicator.text.contains("FINALIZING")
	scene.call("_set_recording_indicator", "● REC OFF  ·  F9 to start", Color("9aa8b8"))
	var conveyor_toggle := scene.get_node("UI/ConveyorInfoToggle") as CheckBox
	var conveyor_panel := scene.get_node("UI/FlowControls") as PanelContainer
	var conveyor_overlay_initial := conveyor_toggle.button_pressed and conveyor_panel.visible
	scene.call("_on_conveyor_overlay_toggled", false)
	var conveyor_overlay_hidden := not conveyor_panel.visible
	scene.call("_on_conveyor_overlay_toggled", true)
	var conveyor_overlay_shown := conveyor_panel.visible

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

	# Trigger-driven production pace is measured from successive `trigger q`
	# events, independent of the conveyor's configured belt speed.
	scene.set("production_clock_seconds", 1.0)
	scene.call("_on_trajectory_triggered", "q")
	scene.set("production_clock_seconds", 3.0)
	scene.call("_on_trajectory_triggered", "q")
	var q_event_count: int = scene.get("q_event_count")
	var q_event_rate: float = scene.get("q_event_rate_per_minute")
	var q_stats_ok: bool = q_event_count == 2 and is_equal_approx(q_event_rate, 30.0)

	# W/E are the interactive label visibility bindings and are also accepted
	# through the same trigger dispatch used by trajectory files.
	var label_visible_initial: bool = robot.is_product_label_visible()
	var hide_event := InputEventKey.new()
	hide_event.keycode = KEY_E
	hide_event.pressed = true
	scene.call("_input", hide_event)
	var label_hidden: bool = not robot.is_product_label_visible()
	var show_event := InputEventKey.new()
	show_event.keycode = KEY_W
	show_event.pressed = true
	scene.call("_input", show_event)
	var label_shown: bool = robot.is_product_label_visible()
	scene.call("_on_trajectory_triggered", "e")
	var trigger_hidden: bool = not robot.is_product_label_visible()
	scene.call("_on_trajectory_triggered", "w")
	var trigger_shown: bool = robot.is_product_label_visible()
	var label_bindings_ok: bool = label_visible_initial and label_hidden and label_shown and trigger_hidden and trigger_shown

	print("Main trajectory segments: ", planner.get_segment_count())
	print("Main trajectory duration (s): ", planner.get_total_seconds())
	var conveyor_overlay_ok := conveyor_overlay_initial and conveyor_overlay_hidden and conveyor_overlay_shown
	var recording_indicator_ok := recording_idle_ok and recording_active_ok and recording_finalizing_ok
	if not segments_ok or not feed_ok or not points_reachable or not final_ok or not complete_ok or not q_stats_ok or not label_bindings_ok or not conveyor_overlay_ok or not recording_indicator_ok:
		push_error("Main MG400 trajectory integration verification failed")
		quit(1)
	else:
		print("Main MG400 trajectory integration verification passed")
		quit(0)

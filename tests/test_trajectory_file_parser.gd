extends SceneTree

const ParserScript = preload("res://scripts/trajectory_file_parser.gd")


func _initialize() -> void:
	var text := """
# editable trajectory
[trajectory]
units = mg400-mm
feed_mm_s = 150
acceleration_mm_s2 = 600
junction_deviation_mm = 1.5
loop = yes
	[waypoints]
	240 0 245 0
	delay 0.5
	300\t40\t245\t90
	trigger q
	340 40 245 -90
	"""
	var parsed: Dictionary = ParserScript.parse_text(text, "inline.traj")
	var coordinates: Array[Vector3] = parsed.get("coordinates_mm", [])
	var yaws: PackedFloat32Array = parsed.get("yaw_degrees", PackedFloat32Array())
	var waypoint_events: Array = parsed.get("waypoint_events", [])
	var valid_ok: bool = (
		parsed.get("ok", false)
		and is_equal_approx(parsed["feed_mm_s"], 150.0)
		and is_equal_approx(parsed["acceleration_mm_s2"], 600.0)
		and is_equal_approx(parsed["junction_deviation_mm"], 1.5)
		and parsed["loop"]
		and coordinates.size() == 3
		and coordinates[1] == Vector3(300.0, 40.0, 245.0)
		and yaws.size() == 3
		and is_equal_approx(yaws[1], 90.0)
		and waypoint_events.size() == 3
		and waypoint_events[0].size() == 1
		and waypoint_events[0][0]["type"] == "delay"
		and is_equal_approx(waypoint_events[0][0]["seconds"], 0.5)
		and waypoint_events[1][0]["type"] == "trigger"
		and waypoint_events[1][0]["key"] == "q"
		and waypoint_events[2].is_empty()
	)
	var bad := ParserScript.parse_text("[waypoints]\n1 2\n", "bad.traj")
	var invalid_rejected: bool = not bad.get("ok", true) and str(bad.get("error", "")).contains("waypoint")
	var bad_delay := ParserScript.parse_text("[waypoints]\n1 2 3\ndelay nope\n4 5 6\n", "bad-delay.traj")
	var invalid_delay_rejected: bool = not bad_delay.get("ok", true) and str(bad_delay.get("error", "")).contains("delay")
	var label_trigger_text := "[waypoints]\n1 2 3\ntrigger w\n4 5 6\ntrigger e\n"
	var label_triggers := ParserScript.parse_text(label_trigger_text, "label-triggers.traj")
	var label_triggers_ok: bool = (
		label_triggers.get("ok", false)
		and label_triggers["waypoint_events"][0][0]["key"] == "w"
		and label_triggers["waypoint_events"][1][0]["key"] == "e"
	)
	print("Parsed trajectory waypoints: ", coordinates.size())
	print("Parsed feed / acceleration: ", parsed.get("feed_mm_s"), " / ", parsed.get("acceleration_mm_s2"))
	if not valid_ok or not invalid_rejected or not invalid_delay_rejected or not label_triggers_ok:
		push_error(".traj parser verification failed")
		quit(1)
	else:
		print(".traj parser format and validation verification passed")
		quit(0)

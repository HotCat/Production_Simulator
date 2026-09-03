extends SceneTree

const Parser = preload("res://scripts/trajectory2_file_parser.gd")
const Planner = preload("res://scripts/cartesian_trajectory_planner.gd")

func _initialize() -> void:
	var text := """
[trajectory]
units = fr3-mm
feed_mm_s = 3520
acceleration_mm_s2 = 3500
junction_deviation_mm = 1
loop = true
[overlays]
status = false
pose = true
tcp_offset = false
pendant = true
[waypoints]
80.6 221.0 174.0 0.0 90.0 0.0 2.5 -104.7 -81.3 -98.2 38.0 102.8
delay 0.25
80.6 221.0 211.0 10.0 80.0 15.0 3.5 -100.0 -80.0 -95.0 40.0 100.0
trigger q
120.0 200.0 190.0 20.0 70.0 30.0 4.5 -95.0 -75.0 -90.0 42.0 98.0
thin side
return product
"""
	var parsed := Parser.parse_text(text, "inline.traj2")
	var orientations: Array[Vector3] = []
	for i in parsed.get("coordinates_mm", []).size():
		orientations.append(Vector3(deg_to_rad(parsed.pitch_degrees[i]), deg_to_rad(parsed.roll_degrees[i]), deg_to_rad(parsed.yaw_degrees[i])))
	var planner := Planner.new()
	var points: Array[Vector3] = [Vector3.ZERO, Vector3(0.1, 0, 0)]
	var joint_waypoints: Array[PackedFloat32Array] = [PackedFloat32Array([0.0, 0.0]), PackedFloat32Array([1.0, 1.0])]
	var planned := planner.plan_world_path(points, 100.0, 500.0, 1.0, PackedFloat32Array(), [[], []], [orientations[0], orientations[1]], joint_waypoints) if parsed.ok else false
	var valid: bool = parsed.get("ok", false) and parsed.coordinates_mm.size() == 3 and is_equal_approx(parsed.pitch_degrees[1], 10.0) and is_equal_approx(parsed.joint_degrees[0][1], -104.7) and not parsed["overlays"]["status"] and not parsed["overlays"]["translation"] and parsed.waypoint_events[0].size() == 1 and parsed.waypoint_events[2][0]["key"] == "pickup_thin_side" and parsed.waypoint_events[2][1]["key"] == "return_product" and planned
	var bad := Parser.parse_text("[waypoints]\n1 2 3 0 0 0 0 0 0 0 0\n4 5 6 0 0 0 0 0 0 0 0 0\n", "bad.traj2")
	print("Trajectory2 parser waypoints: ", parsed.get("coordinates_mm", []).size())
	if not valid or bad.ok:
		push_error(".traj2 parser/planner verification failed")
		quit(1)
	else:
		print(".traj2 parser and full orientation planner verification passed")
		quit(0)

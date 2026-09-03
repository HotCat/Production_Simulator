class_name Trajectory2FileParser
extends RefCounted

## Parser for FR3 Cartesian trajectory v1 (.traj2).
## Positions are ROS/FR3 millimetres (Z-up). Orientation columns are
## Pitch, Roll, Yaw and J1-J6 are degrees; callers convert them to their
## world basis and joint radians.

const DEFAULT_FEED_MM_S := 120.0
const DEFAULT_ACCELERATION_MM_S2 := 500.0
const DEFAULT_JUNCTION_DEVIATION_MM := 1.0
const DEFAULT_OVERLAYS := {
	"status": true, "joints": true, "orientation": true,
	"translation": true, "pickup": true, "recording": true, "pendant": true
}
const OVERLAY_ALIASES := {"pose": "orientation", "tcp_offset": "translation"}

static func parse_file(path: String) -> Dictionary:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		return _error("Could not open trajectory2 file: %s" % path)
	return parse_text(file.get_as_text(), path)

static func parse_text(text: String, source_name: String = "trajectory2") -> Dictionary:
	var section := ""
	var feed := DEFAULT_FEED_MM_S
	var acceleration := DEFAULT_ACCELERATION_MM_S2
	var junction := DEFAULT_JUNCTION_DEVIATION_MM
	var loop := true
	var coordinates: Array[Vector3] = []
	var pitch: PackedFloat32Array = PackedFloat32Array()
	var roll: PackedFloat32Array = PackedFloat32Array()
	var yaw: PackedFloat32Array = PackedFloat32Array()
	var joint_degrees: Array[PackedFloat32Array] = []
	var events: Array[Array] = []
	var overlays: Dictionary = DEFAULT_OVERLAYS.duplicate(true)
	var line_number := 0

	for raw_line in text.split("\n"):
		line_number += 1
		var line := raw_line.strip_edges()
		var comment_index := line.find("#")
		if comment_index >= 0:
			line = line.substr(0, comment_index).strip_edges()
		if line.is_empty():
			continue
		if line.begins_with("[") and line.ends_with("]"):
			section = line.substr(1, line.length() - 2).strip_edges().to_lower()
			if section not in ["trajectory", "overlays", "waypoints"]:
				return _error("%s:%d: unknown section [%s]" % [source_name, line_number, section])
			continue
		if section in ["trajectory", "overlays"] and line.contains("="):
			var assignment := line.split("=", false, 1)
			var key := assignment[0].strip_edges().to_lower()
			var value := assignment[1].strip_edges()
			if section == "overlays":
				var canonical: String = str(OVERLAY_ALIASES.get(key, key))
				if not overlays.has(canonical):
					return _error("%s:%d: unknown overlay %s" % [source_name, line_number, key])
				var parsed_bool: Variant = _parse_bool(value)
				if parsed_bool == null:
					return _error("%s:%d: overlay value must be true/false" % [source_name, line_number])
				overlays[canonical] = parsed_bool
				continue
			match key:
				"feed_mm_s":
					feed = _parse_number(value)
					if is_nan(feed) or feed <= 0.0: return _error("%s:%d: feed_mm_s must be positive" % [source_name, line_number])
				"acceleration_mm_s2":
					acceleration = _parse_number(value)
					if is_nan(acceleration) or acceleration <= 0.0: return _error("%s:%d: acceleration_mm_s2 must be positive" % [source_name, line_number])
				"junction_deviation_mm":
					junction = _parse_number(value)
					if is_nan(junction) or junction < 0.0: return _error("%s:%d: junction_deviation_mm must be non-negative" % [source_name, line_number])
				"loop":
					var parsed_loop: Variant = _parse_bool(value)
					if parsed_loop == null:
						return _error("%s:%d: loop must be true/false" % [source_name, line_number])
					loop = parsed_loop
				"units":
					if value.to_lower() != "fr3-mm":
						return _error("%s:%d: units must be fr3-mm" % [source_name, line_number])
				_:
					return _error("%s:%d: unknown trajectory parameter %s" % [source_name, line_number, key])
			continue
		if section == "waypoints":
			var fields := line.replace("\t", " ").split(" ", false)
			var values: Array[String] = []
			for field in fields:
				if not field.is_empty(): values.append(field)
			if values[0].to_lower() == "delay":
				if events.is_empty() or values.size() != 2:
					return _error("%s:%d: delay needs seconds after a waypoint" % [source_name, line_number])
				var seconds := _parse_number(values[1])
				if is_nan(seconds) or seconds < 0.0:
					return _error("%s:%d: delay must be zero or positive" % [source_name, line_number])
				events[-1].append({"type": "delay", "seconds": seconds})
				continue
			if values[0].to_lower() == "trigger":
				if events.is_empty() or values.size() != 2:
					return _error("%s:%d: trigger needs a key after a waypoint" % [source_name, line_number])
				events[-1].append({"type": "trigger", "key": values[1].to_lower()})
				continue
			var pickup_command := ""
			if values.size() == 2 and values[0].to_lower() == "thin" and values[1].to_lower() == "side":
				pickup_command = "pickup_thin_side"
			elif values.size() == 2 and values[0].to_lower() == "long" and values[1].to_lower() == "side":
				pickup_command = "pickup_long_side"
			if not pickup_command.is_empty():
				if events.is_empty():
					return _error("%s:%d: pickup command needs a preceding waypoint" % [source_name, line_number])
				events[-1].append({"type": "trigger", "key": pickup_command})
				continue
			if values.size() != 12:
				return _error("%s:%d: waypoint needs exactly X Y Z Pitch Roll Yaw J1 J2 J3 J4 J5 J6" % [source_name, line_number])
			var nums: Array[float] = []
			for value in values:
				var number := _parse_number(value)
				if is_nan(number): return _error("%s:%d: waypoint contains a non-numeric value" % [source_name, line_number])
				nums.append(number)
			coordinates.append(Vector3(nums[0], nums[1], nums[2]))
			pitch.append(nums[3]); roll.append(nums[4]); yaw.append(nums[5]); events.append([])
			var joints := PackedFloat32Array()
			for joint_index in 6: joints.append(nums[6 + joint_index])
			joint_degrees.append(joints)
			continue
		return _error("%s:%d: expected a section, assignment, or waypoint" % [source_name, line_number])

	if coordinates.size() < 2:
		return _error("%s: trajectory needs at least two waypoints" % source_name)
	return {"ok": true, "feed_mm_s": feed, "acceleration_mm_s2": acceleration,
		"junction_deviation_mm": junction, "loop": loop, "coordinates_mm": coordinates,
		"pitch_degrees": pitch, "roll_degrees": roll, "yaw_degrees": yaw,
		"joint_degrees": joint_degrees,
		"waypoint_events": events, "overlays": overlays}

static func _parse_number(value: String) -> float:
	return value.to_float() if value.is_valid_float() else NAN

static func _parse_bool(value: String):
	var v := value.to_lower()
	if v in ["true", "yes", "1"]: return true
	if v in ["false", "no", "0"]: return false
	return null

static func _error(message: String) -> Dictionary:
	return {"ok": false, "error": message}

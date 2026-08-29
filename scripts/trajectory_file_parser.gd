class_name TrajectoryFileParser
extends RefCounted

## Parser for the human-editable .traj format used by the Emacs major mode.
## Coordinates are MG400/ROS millimetres (X, Y, Z) with optional yaw degrees.

const DEFAULT_FEED_MM_S := 120.0
const DEFAULT_ACCELERATION_MM_S2 := 500.0
const DEFAULT_JUNCTION_DEVIATION_MM := 1.0


static func parse_file(path: String) -> Dictionary:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		return _error("Could not open trajectory file: %s" % path)
	return parse_text(file.get_as_text(), path)


static func parse_text(text: String, source_name: String = "trajectory") -> Dictionary:
	var section := ""
	var feed_mm_s := DEFAULT_FEED_MM_S
	var acceleration_mm_s2 := DEFAULT_ACCELERATION_MM_S2
	var junction_deviation_mm := DEFAULT_JUNCTION_DEVIATION_MM
	var loop := true
	var coordinates_mm: Array[Vector3] = []
	var yaw_degrees: PackedFloat32Array = PackedFloat32Array()
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
			if section not in ["trajectory", "waypoints"]:
				return _error("%s:%d: unknown section [%s]" % [source_name, line_number, section])
			continue

		if section == "trajectory" and line.contains("="):
			var assignment := line.split("=", false, 1)
			var key := assignment[0].strip_edges().to_lower()
			var value := assignment[1].strip_edges()
			match key:
				"feed_mm_s", "velocity_mm_s", "speed_mm_s":
					var parsed_feed := _parse_float(value)
					if is_nan(parsed_feed) or parsed_feed <= 0.0:
						return _error("%s:%d: feed must be a positive number" % [source_name, line_number])
					feed_mm_s = parsed_feed
				"acceleration_mm_s2", "accel_mm_s2":
					var parsed_acceleration := _parse_float(value)
					if is_nan(parsed_acceleration) or parsed_acceleration <= 0.0:
						return _error("%s:%d: acceleration must be a positive number" % [source_name, line_number])
					acceleration_mm_s2 = parsed_acceleration
				"junction_deviation_mm", "junction_mm":
					var parsed_junction := _parse_float(value)
					if is_nan(parsed_junction) or parsed_junction < 0.0:
						return _error("%s:%d: junction deviation must be zero or positive" % [source_name, line_number])
					junction_deviation_mm = parsed_junction
				"loop", "loop_trajectory":
					var normalized := value.to_lower()
					if normalized not in ["true", "false", "yes", "no", "1", "0"]:
						return _error("%s:%d: loop must be true/false" % [source_name, line_number])
					loop = normalized in ["true", "yes", "1"]
				"units":
					if value.to_lower() not in ["mm", "mg400-mm", "mg400"]:
						return _error("%s:%d: units must be mm or mg400-mm" % [source_name, line_number])
				_:
					return _error("%s:%d: unknown trajectory parameter %s" % [source_name, line_number, key])
			continue

		if section == "waypoints":
			if line.begins_with("waypoint") and line.contains("="):
				line = line.split("=", false, 1)[1].strip_edges()
			var fields := line.replace("\t", " ").split(" ", false)
			# Permit tabs or repeated whitespace by filtering empty fields.
			var values: Array[String] = []
			for field in fields:
				if not field.is_empty():
					values.append(field)
			if values.size() not in [3, 4]:
				return _error("%s:%d: waypoint needs X Y Z and optional yaw degrees" % [source_name, line_number])
			var numbers: Array[float] = []
			for value in values:
				var number := _parse_float(value)
				if is_nan(number):
					return _error("%s:%d: waypoint contains a non-numeric value" % [source_name, line_number])
				numbers.append(number)
			coordinates_mm.append(Vector3(numbers[0], numbers[1], numbers[2]))
			yaw_degrees.append(numbers[3] if values.size() == 4 else 0.0)
			continue

		return _error("%s:%d: expected a trajectory parameter or waypoint" % [source_name, line_number])

	if coordinates_mm.size() < 2:
		return _error("%s: trajectory needs at least two waypoints" % source_name)
	return {
		"ok": true,
		"feed_mm_s": feed_mm_s,
		"acceleration_mm_s2": acceleration_mm_s2,
		"junction_deviation_mm": junction_deviation_mm,
		"loop": loop,
		"coordinates_mm": coordinates_mm,
		"yaw_degrees": yaw_degrees,
	}


static func _parse_float(value: String) -> float:
	if value.is_empty():
		return NAN
	var number := value.to_float()
	# to_float() returns 0 for malformed text, so explicitly accept only a
	# numeric prefix that consumes the whole token (apart from a sign/decimal).
	if not value.is_valid_float():
		return NAN
	return number


static func _error(message: String) -> Dictionary:
	return {"ok": false, "error": message}

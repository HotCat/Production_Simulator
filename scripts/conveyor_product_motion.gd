class_name ConveyorProductMotion
extends Node3D

## Maintains a uniformly spaced stream of products on a moving conveyor.
##
## `product_interval_mm` is the clear edge-to-edge gap, not center pitch. The
## controller derives the product count from the usable belt length, rebuilds
## the pool when the interval changes, and advances every product with one
## wrapped phase so spacing remains exact through every loop.

signal configuration_changed
signal stopped_at_trigger(product_index: int, position_y_mm: float)
signal resumed_from_trigger

@export var conveyor_path: NodePath = NodePath("../Conveyor")
@export var product_scene: PackedScene
@export_range(0.0, 1000.0, 1.0, "or_greater") var product_interval_mm := 20.0
@export_range(0.001, 1.0, 0.001, "or_greater") var product_speed_mps := 0.100
@export_range(0.001, 10.0, 0.001, "or_greater") var conveyor_length_m := 0.800
@export_range(0.001, 1.0, 0.001, "or_greater") var product_length_m := 0.046
@export_range(0.001, 1.0, 0.001, "or_greater") var product_height_m := 0.004
@export_range(0.001, 1.0, 0.001, "or_greater") var conveyor_model_height_m := 0.070
@export var loop_motion := true
@export var stop_at_position_enabled := true
@export_range(-1000.0, 1000.0, 1.0) var stop_position_y_mm := 228.0

var _products: Array[Node3D] = []
var _path_infeed_world := Vector3.ZERO
var _travel_axis_world := Vector3.FORWARD
var _flow_distance_m := 0.0
var _loop_span_m := 0.0
var _initialized := false
var _stopped_at_trigger := false
var _stopped_product_index := -1


func _ready() -> void:
	var conveyor := get_node_or_null(conveyor_path) as Node3D
	if conveyor == null:
		push_error("ConveyorProductMotion could not find conveyor at %s" % conveyor_path)
		set_physics_process(false)
		return
	if product_scene == null:
		push_error("ConveyorProductMotion requires a product_scene")
		set_physics_process(false)
		return

	_travel_axis_world = conveyor.global_basis.x.normalized()
	var belt_center_world := Vector3(
		conveyor.global_position.x,
		conveyor.global_position.y + conveyor_model_height_m + product_height_m * 0.5,
		conveyor.global_position.z
	)
	_path_infeed_world = belt_center_world - _travel_axis_world * get_travel_distance_m() * 0.5
	_initialized = true
	rebuild_products()


func _physics_process(delta: float) -> void:
	advance_motion(delta)


func advance_motion(delta: float) -> void:
	if not _initialized or _stopped_at_trigger or delta <= 0.0 or product_speed_mps <= 0.0 or _loop_span_m <= 0.0:
		return
	var requested_distance := product_speed_mps * delta
	var crossing := _find_next_trigger_crossing()
	if crossing.x <= requested_distance + 0.0000001:
		_flow_distance_m = fposmod(_flow_distance_m + crossing.x, _loop_span_m)
		_apply_product_positions()
		_stopped_at_trigger = true
		_stopped_product_index = int(crossing.y)
		stopped_at_trigger.emit(_stopped_product_index, stop_position_y_mm)
		configuration_changed.emit()
		return
	var next_distance := _flow_distance_m + requested_distance
	_flow_distance_m = fposmod(next_distance, _loop_span_m) if loop_motion else minf(next_distance, _loop_span_m)
	_apply_product_positions()


func rebuild_products() -> void:
	if not _initialized:
		return
	var previous_progress := get_progress_normalized()
	for product in _products:
		if is_instance_valid(product):
			product.free()
	_products.clear()
	_stopped_at_trigger = false
	_stopped_product_index = -1

	var pitch := get_product_pitch_m()
	var travel_distance := get_travel_distance_m()
	var product_count := maxi(1, floori(travel_distance / pitch))
	# With two or more products, an integer number of exact pitches forms the
	# wrapped stream. A lone product uses the full belt from end to end.
	_loop_span_m = travel_distance if product_count == 1 else product_count * pitch
	_flow_distance_m = previous_progress * _loop_span_m

	for index in range(product_count):
		var product := product_scene.instantiate() as Node3D
		if product == null:
			push_error("The configured product scene root must inherit Node3D")
			continue
		product.name = "Product_%02d" % (index + 1)
		add_child(product)
		_products.append(product)
	_apply_product_positions()
	configuration_changed.emit()


func set_product_interval_mm(value: float) -> void:
	var sanitized := maxf(value, 0.0)
	if is_equal_approx(sanitized, product_interval_mm):
		return
	product_interval_mm = sanitized
	rebuild_products()


func set_product_speed_mm_s(value: float) -> void:
	product_speed_mps = maxf(value, 0.0) / 1000.0
	configuration_changed.emit()


func set_stop_position_y_mm(value: float) -> void:
	stop_position_y_mm = value
	if _stopped_at_trigger:
		resume_after_stop()
	configuration_changed.emit()


func resume_after_stop() -> void:
	if not _stopped_at_trigger:
		return
	_stopped_at_trigger = false
	_stopped_product_index = -1
	resumed_from_trigger.emit()
	configuration_changed.emit()


func reset_flow() -> void:
	_flow_distance_m = 0.0
	_stopped_at_trigger = false
	_stopped_product_index = -1
	if _initialized:
		_apply_product_positions()


func set_progress_normalized(value: float) -> void:
	_flow_distance_m = clampf(value, 0.0, 1.0) * _loop_span_m
	if _initialized:
		_apply_product_positions()


func get_progress_normalized() -> float:
	if _loop_span_m <= 0.0:
		return 0.0
	return _flow_distance_m / _loop_span_m


func get_product_count() -> int:
	return _products.size()


func get_product(index: int) -> Node3D:
	if index < 0 or index >= _products.size():
		return null
	return _products[index]


func is_stopped_at_trigger() -> bool:
	return _stopped_at_trigger


func get_stopped_product_index() -> int:
	return _stopped_product_index


func get_product_pitch_m() -> float:
	return product_length_m + product_interval_mm / 1000.0


func get_travel_distance_m() -> float:
	return maxf(conveyor_length_m - product_length_m, 0.0)


func get_loop_span_m() -> float:
	return _loop_span_m


func get_loop_seconds() -> float:
	if product_speed_mps <= 0.0:
		return INF
	return _loop_span_m / product_speed_mps


func get_products_per_minute() -> float:
	var pitch := get_product_pitch_m()
	if pitch <= 0.0:
		return 0.0
	return product_speed_mps / pitch * 60.0


func _apply_product_positions() -> void:
	if _products.is_empty() or _loop_span_m <= 0.0:
		return
	var pitch := get_product_pitch_m()
	for index in range(_products.size()):
		var distance := fposmod(_flow_distance_m + index * pitch, _loop_span_m)
		_products[index].global_position = _path_infeed_world + _travel_axis_world * distance


func _find_next_trigger_crossing() -> Vector2:
	# MG400/ROS uses Z-up coordinates. In this Godot Y-up project its Cartesian
	# Y coordinate maps to negative world Z, matching the conversion used by the
	# robot controller and the position overlay.
	if not stop_at_position_enabled or absf(_travel_axis_world.z) < 0.000001:
		return Vector2(INF, -1.0)
	var trigger_world_z := -stop_position_y_mm / 1000.0
	var trigger_distance := (trigger_world_z - _path_infeed_world.z) / _travel_axis_world.z
	if trigger_distance < 0.0 or trigger_distance >= _loop_span_m:
		return Vector2(INF, -1.0)

	var nearest_distance := INF
	var nearest_index := -1
	var pitch := get_product_pitch_m()
	for index in range(_products.size()):
		var product_distance := fposmod(_flow_distance_m + index * pitch, _loop_span_m)
		var forward_distance := fposmod(trigger_distance - product_distance, _loop_span_m)
		# The product already held at the station must be allowed to depart after
		# Q resumes the flow; the following product becomes the next trigger.
		if forward_distance <= 0.000001:
			continue
		if forward_distance < nearest_distance:
			nearest_distance = forward_distance
			nearest_index = index
	return Vector2(nearest_distance, float(nearest_index))

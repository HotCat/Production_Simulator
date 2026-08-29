extends SceneTree

const ConveyorProductMotionScript = preload("res://scripts/conveyor_product_motion.gd")
const PRODUCT_SIZE := Vector3(0.03795, 0.00400, 0.04600)
const DEFAULT_GAP_MM := 20.0
const ADJUSTED_GAP_MM := 60.0
const DEFAULT_PRODUCT_COUNT := 11
const ADJUSTED_PRODUCT_COUNT := 7
const DEFAULT_SPEED_MM_S := 100.0
const ADJUSTED_SPEED_MM_S := 200.0
const STOP_POSITION_Y_MM := 228.0
const TOLERANCE := 0.00003


func aggregate_world_bounds(root_node: Node) -> AABB:
	var minimum := Vector3(INF, INF, INF)
	var maximum := Vector3(-INF, -INF, -INF)
	for child in root_node.find_children("*", "MeshInstance3D", true, false):
		var mesh_instance := child as MeshInstance3D
		var bounds := mesh_instance.get_aabb()
		for corner_index in range(8):
			var corner := Vector3(
				bounds.position.x + bounds.size.x * float(corner_index & 1),
				bounds.position.y + bounds.size.y * float((corner_index >> 1) & 1),
				bounds.position.z + bounds.size.z * float((corner_index >> 2) & 1)
			)
			var world_corner := mesh_instance.to_global(corner)
			minimum = minimum.min(world_corner)
			maximum = maximum.max(world_corner)
	return AABB(minimum, maximum - minimum)


func projected_product_distances(flow: ConveyorProductMotionScript, axis: Vector3) -> Array[float]:
	var distances: Array[float] = []
	var origin := flow.get_product(0).global_position
	for index in range(flow.get_product_count()):
		distances.append((flow.get_product(index).global_position - origin).dot(axis))
	distances.sort()
	return distances


func spacing_is_exact(flow: ConveyorProductMotionScript, axis: Vector3, expected_pitch: float) -> bool:
	var distances := projected_product_distances(flow, axis)
	for index in range(1, distances.size()):
		if absf((distances[index] - distances[index - 1]) - expected_pitch) > TOLERANCE:
			return false
	return true


func _initialize() -> void:
	var packed: PackedScene = load("res://main.tscn")
	var scene := packed.instantiate() as Node3D
	var flow := scene.get_node("ProductFlow") as ConveyorProductMotionScript
	flow.set_physics_process(false)
	root.add_child(scene)
	await process_frame

	var conveyor := scene.get_node("Conveyor") as Node3D
	var travel_axis := conveyor.global_basis.x.normalized()
	var first_product := flow.get_product(0)
	var bounds := aggregate_world_bounds(first_product)
	var expected_belt_top := conveyor.global_position.y + flow.conveyor_model_height_m
	var default_pitch := flow.product_length_m + DEFAULT_GAP_MM / 1000.0

	print("Default product gap / count: ", flow.product_interval_mm, " mm / ", flow.get_product_count())
	print("Default pitch: ", default_pitch, " m")
	print("Product dimensions: ", bounds.size)
	print("Product bottom / belt top: ", bounds.position.y, " / ", expected_belt_top)

	var dimensions_ok := bounds.size.distance_to(PRODUCT_SIZE) <= TOLERANCE
	var contact_ok := absf(bounds.position.y - expected_belt_top) <= TOLERANCE
	var default_count_ok := flow.get_product_count() == DEFAULT_PRODUCT_COUNT
	var default_spacing_ok := spacing_is_exact(flow, travel_axis, default_pitch)

	flow.set_product_interval_mm(ADJUSTED_GAP_MM)
	var adjusted_pitch := flow.product_length_m + ADJUSTED_GAP_MM / 1000.0
	var adjusted_count_ok := flow.get_product_count() == ADJUSTED_PRODUCT_COUNT
	var adjusted_spacing_ok := spacing_is_exact(flow, travel_axis, adjusted_pitch)
	print("Adjusted product gap / count: ", flow.product_interval_mm, " mm / ", flow.get_product_count())

	flow.set_product_speed_mm_s(ADJUSTED_SPEED_MM_S)
	var speed_ok := absf(flow.product_speed_mps * 1000.0 - ADJUSTED_SPEED_MM_S) <= TOLERANCE
	flow.reset_flow()
	flow.stop_at_position_enabled = true
	flow.set_stop_position_y_mm(STOP_POSITION_Y_MM)
	flow.advance_motion(1.0)
	var stopped_index := flow.get_stopped_product_index()
	var stopped_product := flow.get_product(stopped_index)
	var stopped_y_mm := -stopped_product.global_position.z * 1000.0
	var station_stop_ok := flow.is_stopped_at_trigger() and absf(stopped_y_mm - STOP_POSITION_Y_MM) <= TOLERANCE
	var held_position := stopped_product.global_position
	flow.advance_motion(0.5)
	var station_hold_ok := stopped_product.global_position.distance_to(held_position) <= TOLERANCE
	var resume_event := InputEventKey.new()
	resume_event.keycode = KEY_Q
	resume_event.pressed = true
	scene.call("_input", resume_event)
	flow.advance_motion(0.01)
	var station_resume_ok := not flow.is_stopped_at_trigger() and stopped_product.global_position.distance_to(held_position) > TOLERANCE
	var release_event := InputEventKey.new()
	release_event.keycode = KEY_Q
	release_event.pressed = false
	scene.call("_input", release_event)

	flow.stop_at_position_enabled = false
	flow.reset_flow()
	var loop_seconds := flow.get_loop_seconds()
	flow.advance_motion(loop_seconds * 1.25)
	var loop_ok := absf(flow.get_progress_normalized() - 0.25) <= TOLERANCE
	var wrapped_spacing_ok := spacing_is_exact(flow, travel_axis, adjusted_pitch)

	if not dimensions_ok or not contact_ok or not default_count_ok or not default_spacing_ok \
		or not adjusted_count_ok or not adjusted_spacing_ok or not speed_ok or not station_stop_ok \
		or not station_hold_ok or not station_resume_ok or not loop_ok or not wrapped_spacing_ok:
		push_error("Product pool count, spacing, speed, station stop/resume, or loop verification failed")
		quit(1)
	else:
		print("Adjustable product pool, exact interval, station stop/resume, speed, and loop verification passed")
		quit(0)

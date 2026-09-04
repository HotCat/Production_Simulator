@tool
class_name LabelApplicationFixture
extends Node3D
## Visual nest used while the label is applied to the upright H89 product.
##
## The fixture is intentionally open on its +Z/front edge. Its base is a
## perimeter frame rather than a solid plate, leaving the centre hollow for a
## flat-laid product and for the gripper to approach through the opening. No
## physics collision shapes are used: this is a visual production simulator.

const PRODUCT_WIDTH_M := 0.03795
const PRODUCT_HEIGHT_M := 0.046
const PRODUCT_THICKNESS_M := 0.004
const BASE_WIDTH_M := 0.070
const FRAME_LENGTH_M := 0.080
const FRAME_RAIL_HEIGHT_M := 0.008
const GUIDE_THICKNESS_M := 0.006
const GUIDE_HEIGHT_M := 0.010
const REAR_STOP_THICKNESS_M := 0.006

var _base_material: StandardMaterial3D
var _guide_material: StandardMaterial3D
var _pad_material: StandardMaterial3D


func _ready() -> void:
	_build_materials()
	_build_fixture()


func _build_materials() -> void:
	_base_material = _material(Color("384650"), 0.38, 0.62)
	_guide_material = _material(Color("5d7180"), 0.32, 0.72)
	_pad_material = _material(Color("20282d"), 0.78, 0.08)


func _material(color: Color, roughness: float, metallic: float) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = roughness
	material.metallic = metallic
	return material


func _build_fixture() -> void:
	# Hollow base frame: side rails and rear rail support the product perimeter,
	# but the centre is completely open. The missing +Z/front rail is the jaw
	# access path during the label-application and unload operations.
	var guide_x := (PRODUCT_WIDTH_M + GUIDE_THICKNESS_M) * 0.5
	for side in [-1.0, 1.0]:
		_add_box("SideGuide_%s" % ("Left" if side < 0.0 else "Right"),
			Vector3(GUIDE_THICKNESS_M, FRAME_RAIL_HEIGHT_M, FRAME_LENGTH_M),
			Vector3(side * guide_x, FRAME_RAIL_HEIGHT_M * 0.5, 0.0), _guide_material)
		_add_box("GuidePad_%s" % ("Left" if side < 0.0 else "Right"),
			Vector3(0.002, GUIDE_HEIGHT_M * 0.72, FRAME_LENGTH_M * 0.88),
			Vector3(side * (guide_x - side * 0.0035), FRAME_RAIL_HEIGHT_M * 0.5, 0.0), _pad_material)
	# Rear stop only. The +Z edge is omitted so the gripper can enter and leave.
	_add_box("RearStop", Vector3(BASE_WIDTH_M, FRAME_RAIL_HEIGHT_M, REAR_STOP_THICKNESS_M),
		Vector3(0.0, FRAME_RAIL_HEIGHT_M * 0.5, -FRAME_LENGTH_M * 0.5), _guide_material)
	# Four low corner pads make the hollow opening easy to read without putting a
	# plate below the product's flat face.
	for side in [-1.0, 1.0]:
		for z_side in [-1.0, 1.0]:
			_add_box("CornerPad_%d_%d" % [int(side), int(z_side)], Vector3(0.010, 0.003, 0.010),
				Vector3(side * (BASE_WIDTH_M * 0.5 - 0.005), 0.0015, z_side * (FRAME_LENGTH_M * 0.5 - 0.005)), _pad_material)
	set_meta("product_envelope_mm", "37.95 x 46.00 x 4.00")
	set_meta("open_edge", "+Z gripper access")
	set_meta("base", "hollow perimeter frame for flat-laid product")
	set_meta("purpose", "flat product restraint during label application and unload")


func set_flat_tcp_reference(tcp_world_position: Vector3, tcp_world_basis: Basis, tool_clearance_m := 0.130) -> void:
	# The gripper fingers extend 130 mm from the TCP. Offset the fixture pocket
	# along the uploaded tool axis so the jaw remains clear of the frame rails.
	global_basis = tcp_world_basis.orthonormalized()
	global_position = tcp_world_position + global_basis * Vector3(0.0, 0.0, tool_clearance_m)
	set_meta("reference_tcp_world_m", str(tcp_world_position))
	set_meta("tool_clearance_mm", tool_clearance_m * 1000.0)


func _add_box(node_name: String, size: Vector3, position: Vector3, material: Material) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = size
	var instance := MeshInstance3D.new()
	instance.name = node_name
	instance.position = position
	instance.mesh = mesh
	instance.material_override = material
	add_child(instance)
	return instance

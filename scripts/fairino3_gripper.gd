@tool
class_name Fairino3Gripper
extends Node3D
## Visual parallel-jaw gripper for the Fairino3 TCP.
##
## The jaws close along the TCP/flange X axis and the fingers extend along
## local +Z (the flange/tool direction). This lets the jaws capture the thin
## 4 mm product edge while the 46 mm product dimension remains vertical.

const PRODUCT_WIDTH_M := 0.03795
const PRODUCT_HEIGHT_M := 0.046
const PRODUCT_THICKNESS_M := 0.004
const JAW_WIDTH_M := 0.006
const OPEN_INNER_GAP_M := 0.050

var _metal: StandardMaterial3D
var _dark: StandardMaterial3D
var _rubber: StandardMaterial3D
var _jaw_nodes: Array[MeshInstance3D] = []
var _pad_nodes: Array[MeshInstance3D] = []
var _jaw_closed := false
var _closed_inner_gap_m := PRODUCT_WIDTH_M


func _ready() -> void:
	_build_materials()
	_build_gripper()


func _build_materials() -> void:
	_metal = _make_material(Color("8e9aa3"), 0.28, 0.75)
	_dark = _make_material(Color("202a31"), 0.42, 0.35)
	_rubber = _make_material(Color("161a1d"), 0.82, 0.05)


func _make_material(color: Color, roughness: float, metallic: float) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = roughness
	material.metallic = metallic
	return material


func _build_gripper() -> void:
	# Flange adapter and compact body.
	_add_box("FlangeAdapter", Vector3(0.060, 0.008, 0.060), Vector3(0, 0, 0.004), _metal)
	_add_box("GripperBody", Vector3(0.070, 0.026, 0.042), Vector3(0, 0, 0.028), _dark)
	_add_box("JawGuide", Vector3(0.090, 0.018, 0.016), Vector3(0, 0, 0.052), _metal)

	# Two fingers leave a 6 mm opening: enough clearance for the 4 mm CAD
	# thickness while presenting a visible side-wall gripping configuration.
	var jaw_x := (OPEN_INNER_GAP_M + JAW_WIDTH_M) * 0.5
	for side in [-1.0, 1.0]:
		var x: float = side * jaw_x
		# Keep full-length fingers. The pickup controller shifts the product to
		# one jaw end so only a quarter of this length contacts the product.
		_jaw_nodes.append(_add_box("Jaw_%s" % ("Left" if side < 0 else "Right"), Vector3(0.006, 0.022, 0.070), Vector3(x, 0, 0.086), _metal))
		_pad_nodes.append(_add_box("GripPad_%s" % ("Left" if side < 0 else "Right"), Vector3(0.002, 0.018, 0.050), Vector3(x - side * 0.0035, 0, 0.088), _rubber))



func set_product_span(span_m: float) -> void:
	# Different pickup approaches close across different product dimensions:
	# thin-side uses the 37.95 mm width, while long-side uses the 46 mm height.
	_closed_inner_gap_m = maxf(span_m, 0.001)


func set_jaws_closed(closed: bool) -> void:
	_jaw_closed = closed
	# `gap` is the inner clearance between the two rubber pads. For the thin-side
	# strategy the jaws close to the product width, contacting its two long
	# outside edges without changing the product's upright orientation.
	var inner_gap := _closed_inner_gap_m if closed else OPEN_INNER_GAP_M
	var gap := (inner_gap + JAW_WIDTH_M) * 0.5
	for index in 2:
		var side := -1.0 if index == 0 else 1.0
		_jaw_nodes[index].position.x = side * gap
		_pad_nodes[index].position.x = side * gap - side * 0.0035


func are_jaws_closed() -> bool:
	return _jaw_closed


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

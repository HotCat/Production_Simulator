@tool
class_name LabelApplicationFixture
extends Node3D
## Visual nest used while the label is applied to the upright H89 product.
##
## The fixture is intentionally open on its +Z/front edge. The product sits in
## the shallow base pocket, while the two side guides and rear stop keep it
## vertical. No physics collision shapes are used: this is a visual production
## simulator and the opening must remain available for the gripper jaws.

const PRODUCT_WIDTH_M := 0.03795
const PRODUCT_HEIGHT_M := 0.046
const PRODUCT_THICKNESS_M := 0.004
const BASE_WIDTH_M := 0.070
const BASE_DEPTH_M := 0.064
const BASE_HEIGHT_M := 0.004
const GUIDE_THICKNESS_M := 0.006
const GUIDE_HEIGHT_M := 0.025
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
	# The fixture origin is the product's local origin at the conveyor/contact
	# plane. The product body starts at local Y=0 and rises to 46 mm.
	_add_box("NestBase", Vector3(BASE_WIDTH_M, BASE_HEIGHT_M, BASE_DEPTH_M),
		Vector3(0.0, -BASE_HEIGHT_M * 0.5, 0.0), _base_material)
	# Side guides are deliberately just outside the 37.95 mm product width.
	var guide_x := (PRODUCT_WIDTH_M + GUIDE_THICKNESS_M) * 0.5
	for side in [-1.0, 1.0]:
		_add_box("SideGuide_%s" % ("Left" if side < 0.0 else "Right"),
			Vector3(GUIDE_THICKNESS_M, GUIDE_HEIGHT_M, BASE_DEPTH_M * 0.72),
			Vector3(side * guide_x, GUIDE_HEIGHT_M * 0.5, 0.0), _guide_material)
		_add_box("GuidePad_%s" % ("Left" if side < 0.0 else "Right"),
			Vector3(0.002, GUIDE_HEIGHT_M * 0.72, BASE_DEPTH_M * 0.70),
			Vector3(side * (guide_x - side * 0.0035), GUIDE_HEIGHT_M * 0.5, 0.0), _pad_material)
	# Rear stop only. The +Z edge is omitted so the gripper can enter and leave
	# without colliding with a fourth wall.
	_add_box("RearStop", Vector3(BASE_WIDTH_M * 0.82, GUIDE_HEIGHT_M, REAR_STOP_THICKNESS_M),
		Vector3(0.0, GUIDE_HEIGHT_M * 0.5, -BASE_DEPTH_M * 0.5 + REAR_STOP_THICKNESS_M * 0.5), _guide_material)
	# A thin label-support ledge sits below the product, leaving its label face
	# exposed for the emitter/nozzle pass.
	_add_box("LabelSupportLedge", Vector3(PRODUCT_WIDTH_M * 0.90, 0.003, BASE_DEPTH_M * 0.55),
		Vector3(0.0, 0.002, BASE_DEPTH_M * 0.08), _pad_material)
	set_meta("product_envelope_mm", "37.95 x 46.00 x 4.00")
	set_meta("open_edge", "+Z gripper access")
	set_meta("purpose", "upright product restraint during label application")


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

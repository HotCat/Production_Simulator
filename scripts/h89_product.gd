@tool
class_name H89Product
extends Node3D
## Upright visual approximation of cads/H89-converted.dxf.
##
## The drawing is treated as the requested 37.95 x 46 x 4 mm envelope. The
## 46 mm side is vertical and the 4 mm thickness is along local Z, allowing a
## parallel jaw gripper to capture the long thin side wall.

const WIDTH_M := 0.03795
const HEIGHT_M := 0.046
const THICKNESS_M := 0.004

func _ready() -> void:
	var body_material := _material(Color("d8e0e5"), 0.36, 0.25)
	var edge_material := _material(Color("52626d"), 0.5, 0.45)
	var marker_material := _material(Color("f2b84b"), 0.3, 0.05)
	_add_box("H89Body", Vector3(WIDTH_M, HEIGHT_M, THICKNESS_M), Vector3(0, HEIGHT_M * 0.5, 0), body_material)
	# Raised side-wall rails make the 4 mm gripping edge readable in the
	# production-line view while retaining the exact cubic envelope.
	for side in [-1.0, 1.0]:
		_add_box("SideRail_%s" % ("Front" if side < 0 else "Back"), Vector3(WIDTH_M * 0.92, HEIGHT_M * 0.92, 0.00035), Vector3(0, HEIGHT_M * 0.5, side * (THICKNESS_M * 0.5 + 0.00018)), edge_material)
	# Two small fiducial bars provide stable visual references for hand-eye and
	# camera parameter setup; they are flush with the front face.
	for side in [-1.0, 1.0]:
		_add_box("Fiducial_%s" % ("Top" if side > 0 else "Bottom"), Vector3(0.012, 0.0012, 0.00025), Vector3(0, HEIGHT_M * (0.72 if side > 0 else 0.28), -THICKNESS_M * 0.5 - 0.0002), marker_material)


func _material(color: Color, roughness: float, metallic: float) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = roughness
	material.metallic = metallic
	return material


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

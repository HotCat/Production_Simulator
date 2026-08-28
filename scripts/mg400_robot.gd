class_name MG400Robot
extends Node3D
## Visual MG400 model with closed-form Cartesian inverse kinematics.
##
## Public coordinates use Godot's convention: X/Z are the floor and +Y is up.
## The source URDF uses ROS convention: X/Y are the floor and +Z is up.

signal target_reachability_changed(reachable: bool)

const J1_ORIGIN := Vector3(-0.0050000000, 0.0, 0.1090000000)
const J2_ORIGIN := Vector3(0.0435007595, -0.0357748756, 0.1189956826)
const J3_ORIGIN := Vector3(-0.0010512570, 0.0357748756, 0.1750011643)
const J4_1_ORIGIN := Vector3(0.1749697841, -0.0170000000, 0.0032518714)
const J4_ORIGIN := Vector3(0.0659996239, 0.0170000000, 0.0310008007)
const TOOL_OFFSET := Vector3(0.0, 0.0, -0.085)

const J2_2_ORIGIN := Vector3(0.0045288568, -0.0305000000, 0.1415000000)
const J3_2_ORIGIN := Vector3(-0.0010504975, 0.0065000000, 0.1749968470)
const J4_2_ORIGIN := Vector3(0.0678965856, 0.0005000000, 0.0119719999)

const J1_LIMIT := Vector2(-PI, PI)
const J2_LIMIT := Vector2(-0.14, 1.39)
const J3_LIMIT := Vector2(0.0, 1.39)
const J4_LIMIT := Vector2(-PI, PI)

# In the planar IK equation, J3_ORIGIN and J4_1_ORIGIN are the two arm
# vectors. J2_ORIGIN + J4_ORIGIN + TOOL_OFFSET is the fixed TCP offset.
const FIXED_XZ := Vector2(
	J2_ORIGIN.x + J4_ORIGIN.x + TOOL_OFFSET.x,
	J2_ORIGIN.z + J4_ORIGIN.z + TOOL_OFFSET.z
)
const ARM_A_XZ := Vector2(J3_ORIGIN.x, J3_ORIGIN.z)
const ARM_B_XZ := Vector2(J4_1_ORIGIN.x, J4_1_ORIGIN.z)

@export_range(0.1, 10.0, 0.1) var joint_speed := 2.8
@export var smooth_motion := true

var _urdf_root: Node3D
var _joints := {}
var _tcp: Marker3D
var _joint_angles := Vector4.ZERO
var _target_angles := Vector4.ZERO
var _last_reachable := true

var _white_material: StandardMaterial3D
var _blue_material: StandardMaterial3D
var _dark_material: StandardMaterial3D
var _metal_material: StandardMaterial3D


func _ready() -> void:
	_build_materials()
	_build_robot()
	apply_joint_angles(Vector4(0.0, 0.35, 0.7, 0.0))
	_target_angles = _joint_angles


func _process(delta: float) -> void:
	if not smooth_motion:
		return
	var next_angles := _joint_angles
	for index in range(4):
		next_angles[index] = move_toward(
			_joint_angles[index], _target_angles[index], joint_speed * delta
		)
	apply_joint_angles(next_angles)


## Commands the TCP in Godot world coordinates. tool_yaw is rotation about the
## world's up axis. Returns false when the requested pose had to be clamped.
func set_tcp_target_world(
	world_position: Vector3,
	tool_yaw: float = 0.0,
	immediate: bool = false
) -> bool:
	var target_urdf := _urdf_root.to_local(world_position)
	var result := _solve_ik_urdf(target_urdf, tool_yaw - global_rotation.y)
	_target_angles = result.angles
	if immediate or not smooth_motion:
		apply_joint_angles(_target_angles)
	var reachable: bool = result.reachable
	if reachable != _last_reachable:
		_last_reachable = reachable
		target_reachability_changed.emit(reachable)
	return reachable


## Immediately sets q1, q2, q3 and q4 in radians. Mimic joints are updated too.
func apply_joint_angles(angles: Vector4) -> void:
	_joint_angles = Vector4(
		clamp(angles.x, J1_LIMIT.x, J1_LIMIT.y),
		clamp(angles.y, J2_LIMIT.x, J2_LIMIT.y),
		clamp(angles.z, J3_LIMIT.x, J3_LIMIT.y),
		wrapf(angles.w, -PI, PI)
	)
	if _joints.is_empty():
		return

	_joints.j1.rotation.z = _joint_angles.x
	_joints.j2.rotation.y = _joint_angles.y
	_joints.j3.rotation.y = _joint_angles.z
	_joints.j3_1.rotation.y = -_joint_angles.y
	_joints.j4_1.rotation.y = -_joint_angles.z
	_joints.j4.rotation.z = _joint_angles.w

	# The second side of the physical parallelogram follows the URDF mimic tags.
	_joints.j2_2.rotation.y = _joint_angles.y
	_joints.j3_2.rotation.y = -_joint_angles.y
	_joints.j4_2.rotation.y = _joint_angles.z


func get_tcp_world_position() -> Vector3:
	return _tcp.global_position


func get_joint_angles() -> Vector4:
	return _joint_angles


## Converts a ROS/URDF position to Godot world coordinates. Useful for authoring
## machine points from MG400 datasheet coordinates.
func urdf_position_to_world(urdf_position: Vector3) -> Vector3:
	return _urdf_root.to_global(urdf_position)


func _solve_ik_urdf(target: Vector3, tool_yaw: float) -> Dictionary:
	var from_j1 := target - J1_ORIGIN
	var q1 := atan2(from_j1.y, from_j1.x)
	var radial := Vector2(from_j1.x, from_j1.y).length()
	var desired := Vector2(
		radial - FIXED_XZ.x,
		from_j1.z - FIXED_XZ.y
	)

	var length_a := ARM_A_XZ.length()
	var length_b := ARM_B_XZ.length()
	var distance := desired.length()
	var minimum_reach: float = absf(length_a - length_b)
	var maximum_reach: float = length_a + length_b
	var geometrically_reachable: bool = (
		distance >= minimum_reach - 0.000001
		and distance <= maximum_reach + 0.000001
	)

	# Project unreachable commands to the nearest point on the annular workspace.
	if distance < 0.000001:
		desired = Vector2(maximum_reach, 0.0)
		distance = maximum_reach
	else:
		var clamped_distance: float = clampf(
			distance, minimum_reach + 0.000001, maximum_reach
		)
		desired *= clamped_distance / distance
		distance = clamped_distance

	var cosine_a: float = clampf(
		(distance * distance + length_a * length_a - length_b * length_b)
		/ (2.0 * distance * length_a),
		-1.0,
		1.0
	)
	var triangle_angle := acos(cosine_a)
	var target_angle := atan2(desired.y, desired.x)
	var arm_a_rest_angle := atan2(ARM_A_XZ.y, ARM_A_XZ.x)
	var arm_b_rest_angle := atan2(ARM_B_XZ.y, ARM_B_XZ.x)

	var best_angles := Vector2.ZERO
	var best_penalty := INF
	var best_within_limits := false
	for elbow_sign in [1.0, -1.0]:
		var arm_a_angle: float = target_angle + elbow_sign * triangle_angle
		var remaining := desired - Vector2.from_angle(arm_a_angle) * length_a
		var arm_b_angle := atan2(remaining.y, remaining.x)
		# Godot's positive Y rotation subtracts angle in the URDF X/Z plane.
		var q2 := wrapf(arm_a_rest_angle - arm_a_angle, -PI, PI)
		var q3 := wrapf(arm_b_rest_angle - arm_b_angle, -PI, PI)
		var penalty := _limit_penalty(q2, J2_LIMIT) + _limit_penalty(q3, J3_LIMIT)
		# Prefer continuity if both elbow branches are legal.
		penalty += 0.0001 * (abs(q2 - _joint_angles.y) + abs(q3 - _joint_angles.z))
		if penalty < best_penalty:
			best_penalty = penalty
			best_angles = Vector2(q2, q3)
			best_within_limits = (
				q2 >= J2_LIMIT.x and q2 <= J2_LIMIT.y
				and q3 >= J3_LIMIT.x and q3 <= J3_LIMIT.y
			)

	var q4 := wrapf(tool_yaw - q1, -PI, PI)
	var all_within_limits := (
		q1 >= J1_LIMIT.x and q1 <= J1_LIMIT.y
		and best_within_limits
		and q4 >= J4_LIMIT.x and q4 <= J4_LIMIT.y
	)
	return {
		"angles": Vector4(
			clamp(q1, J1_LIMIT.x, J1_LIMIT.y),
			clamp(best_angles.x, J2_LIMIT.x, J2_LIMIT.y),
			clamp(best_angles.y, J3_LIMIT.x, J3_LIMIT.y),
			clamp(q4, J4_LIMIT.x, J4_LIMIT.y)
		),
		"reachable": geometrically_reachable and all_within_limits,
	}


func _limit_penalty(value: float, limits: Vector2) -> float:
	if value < limits.x:
		return limits.x - value
	if value > limits.y:
		return value - limits.y
	return 0.0


func _build_materials() -> void:
	_white_material = _make_material(Color("e7ebee"), 0.32, 0.18)
	_blue_material = _make_material(Color("2879b9"), 0.27, 0.32)
	_dark_material = _make_material(Color("252b31"), 0.34, 0.55)
	_metal_material = _make_material(Color("aab2b8"), 0.22, 0.70)


func _make_material(color: Color, roughness: float, metallic: float) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = roughness
	material.metallic = metallic
	return material


func _build_robot() -> void:
	_urdf_root = Node3D.new()
	_urdf_root.name = "URDF_Z_Up"
	# ROS Z-up -> Godot Y-up. This also maps ROS +Y to Godot -Z.
	_urdf_root.rotation.x = -PI * 0.5
	add_child(_urdf_root)

	_add_mesh(_urdf_root, "base_link", _dark_material)

	_joints.j1 = _add_joint(_urdf_root, "j1", J1_ORIGIN)
	_add_mesh(_joints.j1, "link1", _white_material)

	_joints.j2 = _add_joint(_joints.j1, "j2", J2_ORIGIN)
	_add_mesh(_joints.j2, "link2_1", _blue_material)

	_joints.j3 = _add_joint(_joints.j2, "j3", J3_ORIGIN)
	_joints.j3_1 = _add_joint(_joints.j3, "j3_1", Vector3.ZERO)
	_add_mesh(_joints.j3_1, "link3_1", _metal_material)

	_joints.j4_1 = _add_joint(_joints.j3_1, "j4_1", J4_1_ORIGIN)
	_add_mesh(_joints.j4_1, "link4_1", _blue_material)

	_joints.j4 = _add_joint(_joints.j4_1, "j4", J4_ORIGIN)
	_add_mesh(_joints.j4, "link5", _dark_material)

	_tcp = Marker3D.new()
	_tcp.name = "TCP"
	_tcp.position = TOOL_OFFSET
	_joints.j4.add_child(_tcp)
	_add_tcp_visual(_tcp)

	_joints.j2_2 = _add_joint(_joints.j1, "j2_2", J2_2_ORIGIN)
	_add_mesh(_joints.j2_2, "link2_2", _blue_material)
	_joints.j3_2 = _add_joint(_joints.j2_2, "j3_2", J3_2_ORIGIN)
	_add_mesh(_joints.j3_2, "link3_2", _metal_material)
	_joints.j4_2 = _add_joint(_joints.j3_2, "j4_2", J4_2_ORIGIN)
	_add_mesh(_joints.j4_2, "link4_2", _blue_material)
	_add_upper_arm_logo(_joints.j4_2)


func _add_joint(parent: Node3D, joint_name: String, origin: Vector3) -> Node3D:
	var joint := Node3D.new()
	joint.name = joint_name
	joint.position = origin
	parent.add_child(joint)
	return joint


func _add_mesh(parent: Node3D, mesh_name: String, material: Material) -> void:
	var mesh_path := "res://assets/mg400/%s.obj" % mesh_name
	var mesh_resource := load(mesh_path) as Mesh
	if mesh_resource == null:
		push_error("Missing MG400 mesh: %s. Run tools/convert_stl_to_obj.py." % mesh_path)
		return
	var instance := MeshInstance3D.new()
	instance.name = mesh_name
	instance.mesh = mesh_resource
	instance.material_override = material
	parent.add_child(instance)


func _add_tcp_visual(parent: Node3D) -> void:
	var sphere := SphereMesh.new()
	sphere.radius = 0.008
	sphere.height = 0.016
	var material := StandardMaterial3D.new()
	material.albedo_color = Color("43e1ff")
	material.emission_enabled = true
	material.emission = Color("1fbad6")
	material.emission_energy_multiplier = 1.8
	var instance := MeshInstance3D.new()
	instance.name = "TCPVisual"
	instance.mesh = sphere
	instance.material_override = material
	parent.add_child(instance)


func _add_upper_arm_logo(parent: Node3D) -> void:
	var logo_texture := load("res://material/Logo_Alt_2@2x.png") as Texture2D
	if logo_texture == null:
		push_warning("Could not load upper-arm logo texture")
		return
	# logo_position.png identifies the long diagonal blue link4_2 member. Its
	# groove is on the outward local -Y side and runs along local +X.
	var decal_basis := Basis(
		Vector3(1.0, 0.0, 0.0),
		Vector3(0.0, 0.0, 1.0),
		Vector3(0.0, -1.0, 0.0)
	)
	var plaque_mesh := QuadMesh.new()
	plaque_mesh.size = Vector2(0.074, 0.021)
	var plaque_material := StandardMaterial3D.new()
	plaque_material.albedo_color = Color("dce8ef")
	plaque_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	plaque_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	var plaque := MeshInstance3D.new()
	plaque.name = "UpperArmLogoPlaque"
	plaque.mesh = plaque_mesh
	plaque.material_override = plaque_material
	# The STL groove rim is at local Y ~= 0.0015 m and its recessed floor is
	# around Y ~= 0.0042 m. Seat the plaque against that floor, behind the rim.
	plaque.position = Vector3(0.0875, 0.00270, 0.0006)
	plaque.basis = decal_basis
	parent.add_child(plaque)

	var logo := Sprite3D.new()
	logo.name = "UpperArmLogo"
	logo.texture = logo_texture
	# 340 x 93 px at this scale gives a 68 x 18.6 mm logo, leaving a clear
	# margin inside the upper-arm groove.
	logo.pixel_size = 0.00020
	logo.position = Vector3(0.0875, 0.00240, 0.0006)
	logo.basis = decal_basis
	logo.modulate = Color(1.0, 1.0, 1.0, 0.96)
	logo.no_depth_test = false
	logo.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR
	parent.add_child(logo)

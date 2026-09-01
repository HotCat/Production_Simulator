class_name Fairino3Robot
extends Node3D
## Fairino3 V6 six-axis visual robot and numerical Cartesian IK controller.
##
## The link frames and joint origins mirror fairino3_v6.urdf from the official
## frcobot_ros2 repository. Public targets use Godot metres (Y-up); the URDF
## source is ROS metres (Z-up), converted by the child frame below.

signal target_reachability_changed(reachable: bool)

const LINK_DIR := "res://assets/fairino3_v6_obj/"
const JOINT_LIMITS := [
	Vector2(-3.0543, 3.0543), Vector2(-4.6251, 1.4835),
	Vector2(-2.8274, 2.8274), Vector2(-4.6251, 1.4835),
	Vector2(-3.0543, 3.0543), Vector2(-3.0543, 3.0543)
]
const JOINT_ORIGINS := [
	Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.14),
	Vector3(-0.28, 0.0, 0.0), Vector3(-0.24001, 0.0, 0.0),
	Vector3(0.0, 0.0, 0.102), Vector3(0.0, 0.0, 0.102)
]
const JOINT_RPY := [
	Vector3.ZERO, Vector3(PI * 0.5, 0.0, 0.0), Vector3.ZERO,
	Vector3.ZERO, Vector3(PI * 0.5, 0.0, 0.0), Vector3(-PI * 0.5, 0.0, 0.0)
]
# The URDF ends at wrist3_link's J6 origin, but the wrist3 STL continues along
# local +Z to its circular mounting face at Z=0.100 m. FAIRINO's operating TCP
# is defined at this J6 flange surface, not at the rear joint-center origin.
const J6_FLANGE_OFFSET := Vector3(0.0, 0.0, 0.100)
# The J6 flange face is normal to its local +Z axis. This downward-facing basis
# matches the operating pose: local +Z points along Godot world -Y, while the
# other axes remain aligned to world -X/-Z. The circular flange surface is
# therefore parallel to the horizontal base plane and faces the work surface.
const FLANGE_PARALLEL_TO_BASE_BASIS := Basis(
	Vector3.LEFT,
	Vector3.FORWARD,
	Vector3.DOWN
)

@export_range(0.1, 8.0, 0.1) var joint_speed := 2.8
@export_range(1, 80, 1) var ik_iterations := 35
@export_range(0.001, 0.2, 0.001) var ik_gain := 0.65
@export_range(0.001, 0.2, 0.001) var pose_damping := 0.01
@export var smooth_motion := true

var _urdf_root: Node3D
var _joint_frames: Array[Node3D] = []
var _joint_rotors: Array[Node3D] = []
var _tcp: Marker3D
var _joint_angles := PackedFloat32Array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
var _target_angles := PackedFloat32Array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
var _last_reachable := true
var _white_material: StandardMaterial3D
var _blue_material: StandardMaterial3D
var _dark_material: StandardMaterial3D


func _ready() -> void:
	_build_materials()
	_build_robot()
	apply_joint_angles(PackedFloat32Array([0.0, -0.75, 1.25, 0.0, 0.7, 0.0]))
	_target_angles = _joint_angles


func _process(delta: float) -> void:
	if not smooth_motion:
		return
	var next := _joint_angles.duplicate()
	for i in 6:
		next[i] = move_toward(_joint_angles[i], _target_angles[i], joint_speed * delta)
	apply_joint_angles(next)


func set_tcp_target_world(world_position: Vector3, tool_yaw: float = 0.0, immediate := false) -> bool:
	var target_local := _urdf_root.to_local(world_position)
	var solved := _solve_position_ik(target_local)
	# J6 is a tool-axis roll in this model; expose yaw as a convenient Cartesian
	# orientation control without sacrificing the position solution.
	var yaw_now := atan2(_tcp.global_basis.x.z, _tcp.global_basis.x.x)
	var yaw_delta := wrapf(tool_yaw - yaw_now, -PI, PI)
	solved[5] = clampf(solved[5] + yaw_delta, JOINT_LIMITS[5].x, JOINT_LIMITS[5].y)
	_target_angles = solved
	# Evaluate the candidate pose itself, even while normal motion is smoothed.
	# This keeps reachability feedback current during a gizmo drag or jog.
	var current_angles := _joint_angles.duplicate()
	apply_joint_angles(solved)
	var solved_error := _position_error(target_local)
	apply_joint_angles(current_angles)
	if immediate or not smooth_motion:
		apply_joint_angles(_target_angles)
	var reachable := solved_error < 0.0015
	if reachable != _last_reachable:
		_last_reachable = reachable
		target_reachability_changed.emit(reachable)
	return reachable


## Solves a complete world-space TCP pose. Unlike set_tcp_target_world(), this
## constrains all three orientation axes rather than approximating yaw with J6.
func set_tcp_target_pose_world(
	world_position: Vector3,
	world_basis: Basis,
	immediate := false
) -> bool:
	var target_local_position := _urdf_root.to_local(world_position)
	var target_local_basis := (
		_urdf_root.global_basis.inverse() * world_basis
	).orthonormalized()
	var solved := _solve_pose_ik(target_local_position, target_local_basis)
	_target_angles = solved
	var current_angles := _joint_angles.duplicate()
	apply_joint_angles(solved)
	var position_error := _position_error(target_local_position)
	var current_local_basis := (
		_urdf_root.global_basis.inverse() * _tcp.global_basis
	).orthonormalized()
	var orientation_error := _rotation_vector(current_local_basis, target_local_basis).length()
	apply_joint_angles(current_angles)
	if immediate or not smooth_motion:
		apply_joint_angles(_target_angles)
	var reachable := position_error < 0.0015 and orientation_error < deg_to_rad(1.0)
	if reachable != _last_reachable:
		_last_reachable = reachable
		target_reachability_changed.emit(reachable)
	return reachable


func get_tcp_world_position() -> Vector3:
	return _tcp.global_position


func get_tcp_world_basis() -> Basis:
	return _tcp.global_basis.orthonormalized()


func urdf_position_to_world(position_m: Vector3) -> Vector3:
	return _urdf_root.to_global(position_m)


func get_tcp_urdf_position() -> Vector3:
	return _urdf_root.to_local(_tcp.global_position)


func get_joint_angles() -> PackedFloat32Array:
	return _joint_angles.duplicate()


func apply_joint_angles(angles: PackedFloat32Array) -> void:
	if angles.size() < 6:
		return
	for i in 6:
		_joint_angles[i] = clampf(angles[i], JOINT_LIMITS[i].x, JOINT_LIMITS[i].y)
		_joint_rotors[i].rotation.z = _joint_angles[i]


## Sets a joint configuration as the active target. This is used by the
## individual-axis controls so Fairino3's smoothing loop does not pull the arm
## back toward the previous Cartesian target on the next frame.
func set_joint_angles_target(angles: PackedFloat32Array, immediate := true) -> void:
	if angles.size() < 6:
		return
	_target_angles = angles.duplicate()
	if immediate or not smooth_motion:
		apply_joint_angles(_target_angles)


func _solve_position_ik(target_local: Vector3) -> PackedFloat32Array:
	var solved := _joint_angles.duplicate()
	for iteration in ik_iterations:
		for i in range(5, -1, -1):
			apply_joint_angles(solved)
			var current_position := _urdf_root.to_local(_tcp.global_position)
			var error := target_local - current_position
			if error.length() < 0.0005:
				return solved
			# Finite-difference the actual imported URDF frame. This avoids making
			# assumptions about Euler rotation order while retaining all six axes.
			var probe := solved.duplicate()
			probe[i] += 0.0005
			apply_joint_angles(probe)
			var probe_position := _urdf_root.to_local(_tcp.global_position)
			var derivative := (probe_position - current_position) / 0.0005
			apply_joint_angles(solved)
			var denominator := derivative.length_squared() + 0.000001
			var step := clampf(derivative.dot(error) / denominator * ik_gain, -0.18, 0.18)
			solved[i] = clampf(solved[i] + step, JOINT_LIMITS[i].x, JOINT_LIMITS[i].y)
	apply_joint_angles(solved)
	return solved


func _solve_pose_ik(target_position: Vector3, target_basis: Basis) -> PackedFloat32Array:
	# For a downward-facing request, start from a known FR3 operating branch;
	# otherwise preserve the current configuration for smooth pose edits.
	var downward_seed := PackedFloat32Array([
		deg_to_rad(3.4), deg_to_rad(-115.1), deg_to_rad(-64.9),
		deg_to_rad(-88.0), deg_to_rad(91.0), deg_to_rad(86.6)
	])
	# The solver receives the target basis in ROS/URDF coordinates. World -Y
	# becomes URDF -Z through the Z-up → Y-up conversion.
	var solved := downward_seed if target_basis.z.z < -0.7 else _joint_angles.duplicate()
	# Quaternion angle extraction loses useful precision below roughly 0.001
	# radians with Godot's float transforms, so use a one-centiradian probe.
	var probe_delta := 0.01
	var position_weight := 25.0
	for iteration in maxi(ik_iterations * 8, 240):
		# Alternating the sweep direction reduces branch bias near a singularity.
		var joint_order := range(5, -1, -1) if iteration % 2 == 0 else range(0, 6)
		for joint in joint_order:
			apply_joint_angles(solved)
			var current_position := _urdf_root.to_local(_tcp.global_position)
			var current_basis := (
				_urdf_root.global_basis.inverse() * _tcp.global_basis
			).orthonormalized()
			var position_error := target_position - current_position
			var orientation_error := _rotation_vector(current_basis, target_basis)
			if position_error.length() < 0.0005 and orientation_error.length() < deg_to_rad(0.25):
				return solved
			var probe := solved.duplicate()
			probe[joint] += probe_delta
			apply_joint_angles(probe)
			var probe_position := _urdf_root.to_local(_tcp.global_position)
			var probe_basis := (
				_urdf_root.global_basis.inverse() * _tcp.global_basis
			).orthonormalized()
			var position_derivative := (probe_position - current_position) / probe_delta
			var orientation_derivative := _rotation_vector(current_basis, probe_basis) / probe_delta
			var numerator := (
				position_weight * position_derivative.dot(position_error)
				+ orientation_derivative.dot(orientation_error)
			)
			var denominator := (
				position_weight * position_derivative.length_squared()
				+ orientation_derivative.length_squared()
				+ pose_damping * pose_damping
			)
			var step := clampf(numerator / denominator * 0.7, -0.15, 0.15)
			solved[joint] = clampf(
				solved[joint] + step,
				JOINT_LIMITS[joint].x,
				JOINT_LIMITS[joint].y
			)
	apply_joint_angles(solved)
	return solved


func _rotation_vector(from_basis: Basis, to_basis: Basis) -> Vector3:
	var delta := (to_basis * from_basis.inverse()).orthonormalized()
	var quaternion := delta.get_rotation_quaternion().normalized()
	if quaternion.w < 0.0:
		quaternion = -quaternion
	var angle := quaternion.get_angle()
	if angle < 0.000001:
		return Vector3.ZERO
	return quaternion.get_axis() * angle


func _position_error(target_local: Vector3) -> float:
	return _urdf_root.to_local(_tcp.global_position).distance_to(target_local)


func _build_materials() -> void:
	_white_material = _make_material(Color("e7ebee"), 0.38, 0.15)
	_blue_material = _make_material(Color("1672b8"), 0.28, 0.28)
	_dark_material = _make_material(Color("27313a"), 0.34, 0.55)


func _make_material(color: Color, roughness: float, metallic: float) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = color
	m.roughness = roughness
	m.metallic = metallic
	return m


func _build_robot() -> void:
	_urdf_root = Node3D.new()
	_urdf_root.name = "URDF_Z_Up"
	_urdf_root.rotation.x = -PI * 0.5
	add_child(_urdf_root)
	var parent: Node3D = _urdf_root
	_add_mesh(parent, "base_link", _dark_material)
	var names := ["shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link"]
	for i in 6:
		var frame := Node3D.new()
		frame.name = "j%d_frame" % (i + 1)
		frame.position = JOINT_ORIGINS[i]
		frame.basis = Basis.from_euler(JOINT_RPY[i])
		parent.add_child(frame)
		_joint_frames.append(frame)
		var rotor := Node3D.new()
		rotor.name = "j%d" % (i + 1)
		frame.add_child(rotor)
		_joint_rotors.append(rotor)
		_add_mesh(rotor, names[i], _blue_material if i in [1, 2] else _white_material)
		parent = rotor
	_tcp = Marker3D.new()
	_tcp.name = "TCP"
	_tcp.position = J6_FLANGE_OFFSET
	_tcp.set_meta("frame", "Fairino3 J6 flange center")
	_tcp.set_meta("offset_from_j6_mm", 100.0)
	parent.add_child(_tcp)
	var tcp_mesh := SphereMesh.new()
	tcp_mesh.radius = 0.008
	tcp_mesh.height = 0.016
	var tcp := MeshInstance3D.new()
	tcp.name = "TCPVisual"
	tcp.mesh = tcp_mesh
	tcp.material_override = _make_material(Color("41e0ff"), 0.2, 0.1)
	_tcp.add_child(tcp)


func _add_mesh(parent: Node3D, link_name: String, material: Material) -> void:
	var mesh := load(LINK_DIR.path_join("%s.obj" % link_name)) as Mesh
	if mesh == null:
		push_error("Missing Fairino3 mesh: %s" % link_name)
		return
	var instance := MeshInstance3D.new()
	instance.name = link_name
	instance.mesh = mesh
	instance.material_override = material
	parent.add_child(instance)

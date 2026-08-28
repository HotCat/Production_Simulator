"""Build the installation-specific stand for the HIKROBOT GS200 camera.

The GLB origin is the center of the floor-mounted base plate. Geometry is
authored in Godot's Y-up coordinate convention, converted to Blender Z-up for
export, and returns to the same coordinates when Godot imports the GLB.

Run with:
    /Applications/Blender\ 4.5.app/Contents/MacOS/Blender \
        --background --python tools/build_hikrobot_gs200_bracket.py
"""

from __future__ import annotations

import math
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parent.parent
ASSET_DIR = ROOT / "assets" / "hikrobot_gs200_bracket"
SOURCE_DIR = ROOT / "source_assets" / "hikrobot_gs200_bracket"
BLEND_PATH = SOURCE_DIR / "hikrobot_gs200_bracket.blend"
GLB_PATH = ASSET_DIR / "hikrobot_gs200_bracket.glb"
RENDER_PATH = ROOT / "material" / "generated" / "hikrobot_gs200_bracket_blender_render.png"

# Installation coordinates are metres in the Godot scene.
BASE_WORLD_POSITION = Vector((0.440000, 0.0, -0.009776026))
CAMERA_WORLD_POSITION = Vector((0.262900, 0.302346, -0.009776026))
CAMERA_LOCAL_POSITION = CAMERA_WORLD_POSITION - BASE_WORLD_POSITION
CAMERA_BASIS_X = Vector((-0.9995292, -0.019541625, -0.02365536))
CAMERA_BASIS_Y = Vector((0.0, 0.7709579, -0.63688606))
CAMERA_BASIS_Z = Vector((0.03068308, -0.6365862, -0.77059495))

CAMERA_BODY_HEIGHT = 0.029
PLATE_THICKNESS = 0.004
PLATE_SIZE = Vector((0.028, PLATE_THICKNESS, 0.032))
PLATE_OFFSET = CAMERA_BODY_HEIGHT * 0.5 + PLATE_THICKNESS * 0.5
PLATE_CENTER = CAMERA_LOCAL_POSITION + CAMERA_BASIS_Y * PLATE_OFFSET
BALL_CENTER = CAMERA_LOCAL_POSITION + CAMERA_BASIS_Y * 0.0245

BASE_SIZE = Vector((0.100, 0.012, 0.120))
POST_SIZE = Vector((0.030, 0.348, 0.030))
POST_CENTER = Vector((0.0, BASE_SIZE.y + POST_SIZE.y * 0.5, 0.0))
BOOM_Y = 0.352
BOOM_INNER_X = CAMERA_LOCAL_POSITION.x - 0.024
BOOM_OUTER_X = 0.018
BOOM_SIZE = Vector((BOOM_OUTER_X - BOOM_INNER_X, 0.025, 0.025))
BOOM_CENTER = Vector(((BOOM_INNER_X + BOOM_OUTER_X) * 0.5, BOOM_Y, 0.0))


def reset_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for collection in list(bpy.data.collections):
        bpy.data.collections.remove(collection)


def make_material(
    name: str,
    color: tuple[float, float, float, float],
    metallic: float = 0.0,
    roughness: float = 0.5,
) -> bpy.types.Material:
    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.use_nodes = True
    principled = material.node_tree.nodes.get("Principled BSDF")
    principled.inputs["Base Color"].default_value = color
    principled.inputs["Metallic"].default_value = metallic
    principled.inputs["Roughness"].default_value = roughness
    return material


def godot_to_blender(point: Vector) -> Vector:
    """Convert a Godot Y-up point to the equivalent Blender Z-up point."""

    return Vector((point.x, -point.z, point.y))


def add_mesh(
    collection: bpy.types.Collection,
    name: str,
    godot_vertices: list[Vector],
    faces: list[tuple[int, ...]],
    material: bpy.types.Material,
    smooth_from_face: int | None = None,
) -> bpy.types.Object:
    mesh = bpy.data.meshes.new(f"{name}_Mesh")
    mesh.from_pydata([godot_to_blender(vertex) for vertex in godot_vertices], [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    collection.objects.link(obj)
    obj.data.materials.append(material)
    if smooth_from_face is not None:
        for polygon_index, polygon in enumerate(mesh.polygons):
            polygon.use_smooth = polygon_index >= smooth_from_face
    return obj


def add_box(
    collection: bpy.types.Collection,
    name: str,
    size: Vector,
    center: Vector,
    material: bpy.types.Material,
    axes: tuple[Vector, Vector, Vector] | None = None,
) -> bpy.types.Object:
    x_axis, y_axis, z_axis = axes or (
        Vector((1.0, 0.0, 0.0)),
        Vector((0.0, 1.0, 0.0)),
        Vector((0.0, 0.0, 1.0)),
    )
    half = size * 0.5
    vertices: list[Vector] = []
    for x_sign, y_sign, z_sign in (
        (-1, -1, -1),
        (1, -1, -1),
        (1, 1, -1),
        (-1, 1, -1),
        (-1, -1, 1),
        (1, -1, 1),
        (1, 1, 1),
        (-1, 1, 1),
    ):
        vertices.append(
            center
            + x_axis * (half.x * x_sign)
            + y_axis * (half.y * y_sign)
            + z_axis * (half.z * z_sign)
        )
    faces = [
        (0, 3, 2, 1),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ]
    return add_mesh(collection, name, vertices, faces, material)


def perpendicular_axes(axis: Vector) -> tuple[Vector, Vector]:
    direction = axis.normalized()
    reference = Vector((0.0, 1.0, 0.0))
    if abs(direction.dot(reference)) > 0.90:
        reference = Vector((1.0, 0.0, 0.0))
    first = direction.cross(reference).normalized()
    second = direction.cross(first).normalized()
    return first, second


def add_cylinder(
    collection: bpy.types.Collection,
    name: str,
    radius: float,
    length: float,
    center: Vector,
    axis: Vector,
    material: bpy.types.Material,
    segments: int = 48,
) -> bpy.types.Object:
    direction = axis.normalized()
    first, second = perpendicular_axes(direction)
    bottom = center - direction * (length * 0.5)
    top = center + direction * (length * 0.5)
    vertices: list[Vector] = [bottom, top]
    for ring_center in (bottom, top):
        for index in range(segments):
            angle = math.tau * index / segments
            vertices.append(ring_center + first * (radius * math.cos(angle)) + second * (radius * math.sin(angle)))
    faces: list[tuple[int, ...]] = []
    for index in range(segments):
        following = (index + 1) % segments
        faces.append((0, 2 + following, 2 + index))
        faces.append((1, 2 + segments + index, 2 + segments + following))
        faces.append((2 + index, 2 + following, 2 + segments + following, 2 + segments + index))
    return add_mesh(collection, name, vertices, faces, material, smooth_from_face=segments * 2)


def add_sphere(
    collection: bpy.types.Collection,
    name: str,
    radius: float,
    center: Vector,
    material: bpy.types.Material,
    rings: int = 16,
    segments: int = 32,
) -> bpy.types.Object:
    vertices = [center + Vector((0.0, radius, 0.0)), center + Vector((0.0, -radius, 0.0))]
    for ring in range(1, rings):
        latitude = math.pi * ring / rings
        y = radius * math.cos(latitude)
        radial = radius * math.sin(latitude)
        for segment in range(segments):
            angle = math.tau * segment / segments
            vertices.append(center + Vector((radial * math.cos(angle), y, radial * math.sin(angle))))
    faces: list[tuple[int, ...]] = []
    first_ring = 2
    for segment in range(segments):
        following = (segment + 1) % segments
        faces.append((0, first_ring + segment, first_ring + following))
    for ring in range(rings - 2):
        current = 2 + ring * segments
        following_ring = current + segments
        for segment in range(segments):
            following = (segment + 1) % segments
            faces.append((current + segment, following_ring + segment, following_ring + following, current + following))
    last_ring = 2 + (rings - 2) * segments
    for segment in range(segments):
        following = (segment + 1) % segments
        faces.append((1, last_ring + following, last_ring + segment))
    return add_mesh(collection, name, vertices, faces, material, smooth_from_face=0)


def add_rod_between(
    collection: bpy.types.Collection,
    name: str,
    radius: float,
    start: Vector,
    end: Vector,
    material: bpy.types.Material,
) -> bpy.types.Object:
    return add_cylinder(collection, name, radius, (end - start).length, (start + end) * 0.5, end - start, material)


def build_bracket() -> tuple[bpy.types.Collection, list[bpy.types.Object]]:
    collection = bpy.data.collections.new("GS200_Adjustable_Camera_Bracket")
    bpy.context.scene.collection.children.link(collection)

    aluminum = make_material("Silver Anodized Aluminum", (0.48, 0.54, 0.58, 1.0), 0.82, 0.22)
    machined = make_material("Machined Aluminum", (0.68, 0.72, 0.74, 1.0), 0.90, 0.16)
    black = make_material("Black Anodized Clamps", (0.018, 0.023, 0.029, 1.0), 0.66, 0.25)
    groove = make_material("Extrusion Grooves", (0.025, 0.030, 0.034, 1.0), 0.48, 0.34)
    steel = make_material("Stainless Fasteners", (0.55, 0.59, 0.62, 1.0), 0.94, 0.14)
    orange = make_material("Adjustment Markers", (0.95, 0.24, 0.035, 1.0), 0.12, 0.35)

    add_box(collection, "FloorBasePlate_100x120x12mm", BASE_SIZE, Vector((0.0, BASE_SIZE.y * 0.5, 0.0)), black)
    add_box(collection, "BaseMachinedTop", Vector((0.090, 0.0012, 0.110)), Vector((0.0, BASE_SIZE.y + 0.0006, 0.0)), machined)

    for x in (-0.037, 0.037):
        for z in (-0.047, 0.047):
            add_cylinder(collection, f"M6FloorAnchor_{x:+.3f}_{z:+.3f}", 0.0048, 0.0022, Vector((x, BASE_SIZE.y + 0.0011, z)), Vector((0.0, 1.0, 0.0)), steel)
            add_cylinder(collection, f"M6AnchorRecess_{x:+.3f}_{z:+.3f}", 0.0027, 0.0024, Vector((x, BASE_SIZE.y + 0.0013, z)), Vector((0.0, 1.0, 0.0)), groove)

    add_box(collection, "Vertical3030Extrusion", POST_SIZE, POST_CENTER, aluminum)
    for x_sign in (-1.0, 1.0):
        add_box(collection, f"PostXGroove_{x_sign:+.0f}", Vector((0.0008, POST_SIZE.y - 0.010, 0.006)), POST_CENTER + Vector((x_sign * 0.0152, 0.0, 0.0)), groove)
    for z_sign in (-1.0, 1.0):
        add_box(collection, f"PostZGroove_{z_sign:+.0f}", Vector((0.006, POST_SIZE.y - 0.010, 0.0008)), POST_CENTER + Vector((0.0, 0.0, z_sign * 0.0152)), groove)
    add_box(collection, "PostBaseClamp", Vector((0.046, 0.036, 0.046)), Vector((0.0, 0.030, 0.0)), black)
    add_box(collection, "PostTopCap", Vector((0.032, 0.004, 0.032)), Vector((0.0, 0.362, 0.0)), black)

    add_box(collection, "Horizontal2525Extrusion", BOOM_SIZE, BOOM_CENTER, aluminum)
    add_box(collection, "BoomTopGroove", Vector((BOOM_SIZE.x - 0.010, 0.0008, 0.005)), BOOM_CENTER + Vector((0.0, 0.0127, 0.0)), groove)
    add_box(collection, "BoomFrontGroove", Vector((BOOM_SIZE.x - 0.010, 0.005, 0.0008)), BOOM_CENTER + Vector((0.0, 0.0, -0.0127)), groove)
    add_box(collection, "PostBoomCornerClamp", Vector((0.048, 0.050, 0.048)), Vector((0.0, 0.346, 0.0)), black)
    add_box(collection, "BoomInnerEndCap", Vector((0.004, 0.027, 0.027)), Vector((BOOM_INNER_X - 0.002, BOOM_Y, 0.0)), black)

    carriage_center = Vector((CAMERA_LOCAL_POSITION.x, BOOM_Y, 0.0))
    add_box(collection, "SlidingBoomCarriage", Vector((0.034, 0.038, 0.041)), carriage_center, black)
    add_cylinder(collection, "CarriagePositionScale", 0.0040, 0.0022, carriage_center + Vector((0.0, 0.0200, -0.015)), Vector((0.0, 1.0, 0.0)), orange)
    add_cylinder(collection, "CarriageLockKnob", 0.0090, 0.014, carriage_center + Vector((0.0, 0.0, 0.027)), Vector((0.0, 0.0, 1.0)), black, 40)
    add_cylinder(collection, "CarriageLockScrew", 0.0025, 0.016, carriage_center + Vector((0.0, 0.0, 0.021)), Vector((0.0, 0.0, 1.0)), steel, 32)

    upper_rod = Vector((CAMERA_LOCAL_POSITION.x, BOOM_Y - 0.021, 0.0))
    add_rod_between(collection, "ArticulatedDropLink", 0.0042, upper_rod, BALL_CENTER, steel)
    add_sphere(collection, "CameraAdjustmentBall", 0.0095, BALL_CENTER, black)
    add_cylinder(collection, "BallClampCollar", 0.0115, 0.006, BALL_CENTER + CAMERA_BASIS_Y * 0.002, CAMERA_BASIS_Y, machined, 48)

    camera_axes = (CAMERA_BASIS_X, CAMERA_BASIS_Y, CAMERA_BASIS_Z)
    plate = add_box(collection, "CameraMountPlate_28x32x4mm", PLATE_SIZE, PLATE_CENTER, machined, camera_axes)
    plate["camera_body_interface"] = "Top-face M3 pattern, 25 mm pitch"
    plate["camera_model"] = "HIKROBOT MV-CS200-10GC"
    for z_offset in (-0.0125, 0.0125):
        bolt_center = CAMERA_LOCAL_POSITION + CAMERA_BASIS_Z * z_offset + CAMERA_BASIS_Y * (CAMERA_BODY_HEIGHT * 0.5 + PLATE_THICKNESS + 0.0008)
        add_cylinder(collection, f"M3CameraBolt_{z_offset:+.4f}", 0.0027, 0.0020, bolt_center, CAMERA_BASIS_Y, steel, 32)
        # Dark elongated inserts communicate fore/aft adjustment slots without
        # requiring fragile boolean geometry in the exported demonstration mesh.
        add_box(
            collection,
            f"CameraAdjustmentSlot_{z_offset:+.4f}",
            Vector((0.0050, 0.0007, 0.0080)),
            CAMERA_LOCAL_POSITION + CAMERA_BASIS_Z * z_offset + CAMERA_BASIS_Y * (CAMERA_BODY_HEIGHT * 0.5 + PLATE_THICKNESS + 0.00035),
            groove,
            camera_axes,
        )

    objects = list(collection.objects)
    for obj in objects:
        obj["units"] = "metres"
        obj["installation"] = "GS200 production-cell camera stand"
        obj["base_world_position_mm"] = "440.0, 0.0, -9.776"
    return collection, objects


def mesh_bounds(objects: list[bpy.types.Object]) -> tuple[Vector, Vector]:
    points = [Vector(vertex.co) for obj in objects if obj.type == "MESH" for vertex in obj.data.vertices]
    # Mesh vertices are Blender coordinates; convert them back for validation.
    godot_points = [Vector((point.x, point.z, -point.y)) for point in points]
    minimum = Vector(min(point[index] for point in godot_points) for index in range(3))
    maximum = Vector(max(point[index] for point in godot_points) for index in range(3))
    return minimum, maximum


def export_glb(objects: list[bpy.types.Object]) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.export_scene.gltf(
        filepath=str(GLB_PATH),
        export_format="GLB",
        use_selection=True,
        export_apply=True,
        export_cameras=False,
        export_lights=False,
    )


def point_at(obj: bpy.types.Object, target: Vector) -> None:
    obj.rotation_euler = (target - obj.location).to_track_quat("-Z", "Y").to_euler()


def create_preview() -> None:
    preview = bpy.data.collections.new("Preview_Setup_Not_Exported")
    bpy.context.scene.collection.children.link(preview)
    ground = make_material("Preview Ground", (0.025, 0.032, 0.043, 1.0), 0.20, 0.48)
    belt = make_material("Preview Conveyor Belt", (0.025, 0.24, 0.105, 1.0), 0.02, 0.50)
    camera_black = make_material("Preview Camera", (0.012, 0.016, 0.022, 1.0), 0.68, 0.24)
    lens_black = make_material("Preview Lens", (0.004, 0.006, 0.008, 1.0), 0.58, 0.20)

    add_box(preview, "PreviewGround", Vector((0.60, 0.004, 0.52)), Vector((-0.08, -0.002, 0.0)), ground)
    conveyor_center = Vector((0.27163061, 0.08125973, 0.0)) - BASE_WORLD_POSITION
    add_box(preview, "PreviewConveyor", Vector((0.200, 0.070, 0.800)), conveyor_center + Vector((0.0, 0.035, 0.0)), belt)
    camera_axes = (CAMERA_BASIS_X, CAMERA_BASIS_Y, CAMERA_BASIS_Z)
    add_box(preview, "PreviewGS200Body", Vector((0.029, 0.029, 0.042)), CAMERA_LOCAL_POSITION, camera_black, camera_axes)
    add_cylinder(
        preview,
        "Preview20MPLens",
        0.0165,
        0.052,
        CAMERA_LOCAL_POSITION + CAMERA_BASIS_Z * 0.047,
        CAMERA_BASIS_Z,
        lens_black,
        64,
    )

    bpy.ops.object.camera_add(location=godot_to_blender(Vector((0.480, 0.300, 0.720))))
    camera = bpy.context.object
    camera.name = "PreviewCamera"
    camera.data.lens = 58
    point_at(camera, godot_to_blender(Vector((-0.075, 0.190, 0.0))))
    preview.objects.link(camera)
    bpy.context.collection.objects.unlink(camera)
    bpy.context.scene.camera = camera

    for name, godot_location, energy, size in (
        ("KeyLight", Vector((-0.08, 0.52, 0.18)), 5.0, 0.24),
        ("FillLight", Vector((0.22, 0.30, -0.25)), 2.0, 0.18),
        ("RimLight", Vector((-0.25, 0.37, -0.10)), 3.0, 0.16),
    ):
        bpy.ops.object.light_add(type="AREA", location=godot_to_blender(godot_location))
        light = bpy.context.object
        light.name = name
        light.data.energy = energy
        light.data.shape = "DISK"
        light.data.size = size
        point_at(light, godot_to_blender(Vector((-0.08, 0.20, 0.0))))
        preview.objects.link(light)
        bpy.context.collection.objects.unlink(light)

    scene = bpy.context.scene
    scene.world.color = (0.010, 0.014, 0.023)
    scene.world.use_nodes = True
    scene.world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.010, 0.014, 0.023, 1.0)
    scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.28
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 900
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(RENDER_PATH)
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.view_settings.exposure = -0.6
    bpy.ops.render.render(write_still=True)


def validate(objects: list[bpy.types.Object]) -> None:
    minimum, maximum = mesh_bounds(objects)
    if abs(minimum.y) > 0.00001:
        raise RuntimeError(f"Bracket must contact the floor at Y=0, got {minimum.y}")
    plate = bpy.data.objects["CameraMountPlate_28x32x4mm"]
    plate_center_blender = sum((Vector(vertex.co) for vertex in plate.data.vertices), Vector()) / len(plate.data.vertices)
    plate_center_godot = Vector((plate_center_blender.x, plate_center_blender.z, -plate_center_blender.y))
    if (plate_center_godot - PLATE_CENTER).length > 0.00001:
        raise RuntimeError(f"Camera mounting plate moved: {plate_center_godot}, expected {PLATE_CENTER}")
    base_world_min_x = BASE_WORLD_POSITION.x - BASE_SIZE.x * 0.5
    conveyor_world_max_x = 0.27163061 + 0.200 * 0.5
    clearance = base_world_min_x - conveyor_world_max_x
    if clearance < 0.015:
        raise RuntimeError(f"Insufficient conveyor/base clearance: {clearance}")
    print("Bracket bounds (mm):", tuple(round((maximum[i] - minimum[i]) * 1000.0, 2) for i in range(3)))
    print("Base-to-conveyor clearance (mm):", round(clearance * 1000.0, 3))
    print("Camera plate center, bracket local (mm):", tuple(round(value * 1000.0, 3) for value in PLATE_CENTER))


def main() -> None:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    RENDER_PATH.parent.mkdir(parents=True, exist_ok=True)
    reset_scene()
    _, objects = build_bracket()
    validate(objects)
    export_glb(objects)
    create_preview()
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_PATH))
    print(f"Saved {BLEND_PATH}")
    print(f"Saved {GLB_PATH}")
    print(f"Saved {RENDER_PATH}")


if __name__ == "__main__":
    main()

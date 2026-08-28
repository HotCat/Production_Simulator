"""Build the dimensioned label stripper/emitter in Blender.

The Blender world origin is the center of the emitted label. Dimensions are
in metres. The support pad reaches Z=-0.1504 so placing the origin at the
requested MG400 Z=150.4 mm puts the pad exactly on the sandbox floor.

Run with:
    blender --background --python tools/build_label_stripper.py
"""

from __future__ import annotations

import math
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parent.parent
ASSET_DIR = ROOT / "assets" / "label_stripper"
SOURCE_DIR = ROOT / "source_assets" / "label_stripper"
GLB_PATH = ASSET_DIR / "label_stripper.glb"
BLEND_PATH = SOURCE_DIR / "label_stripper.blend"
RENDER_PATH = ROOT / "material" / "generated" / "label_stripper_blender_render.png"

BASE_LENGTH = 0.218
BASE_WIDTH = 0.190
HOUSING_DEPTH = 0.160
HOUSING_WIDTH = 0.100
HOUSING_HEIGHT = 0.120
GUIDE_SPAN = 0.120
MAST_HEIGHT = 0.188
LABEL_WIDTH = 0.120
ORIGIN_HEIGHT = 0.1504
MACHINE_BASE_Z = -0.080
PAD_HEIGHT = ORIGIN_HEIGHT + MACHINE_BASE_Z  # 70.4 mm


def reset_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for collection in list(bpy.data.collections):
        bpy.data.collections.remove(collection)


def make_material(
    name: str,
    color: tuple[float, float, float, float],
    metallic: float,
    roughness: float,
    emission: tuple[float, float, float, float] | None = None,
) -> bpy.types.Material:
    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.use_nodes = True
    principled = material.node_tree.nodes.get("Principled BSDF")
    principled.inputs["Base Color"].default_value = color
    principled.inputs["Metallic"].default_value = metallic
    principled.inputs["Roughness"].default_value = roughness
    if emission is not None:
        principled.inputs["Emission Color"].default_value = emission
        principled.inputs["Emission Strength"].default_value = 1.4
    return material


def move_to_collection(obj: bpy.types.Object, collection: bpy.types.Collection) -> None:
    for source in list(obj.users_collection):
        source.objects.unlink(obj)
    collection.objects.link(obj)


def add_box(
    collection: bpy.types.Collection,
    name: str,
    size: tuple[float, float, float],
    location: tuple[float, float, float],
    material: bpy.types.Material,
    bevel: float = 0.0,
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel > 0.0:
        modifier = obj.modifiers.new("Edge softening", "BEVEL")
        modifier.width = bevel
        modifier.segments = 3
        modifier.limit_method = "ANGLE"
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    obj.data.materials.append(material)
    move_to_collection(obj, collection)
    return obj


def add_cylinder(
    collection: bpy.types.Collection,
    name: str,
    radius: float,
    depth: float,
    location: tuple[float, float, float],
    material: bpy.types.Material,
    axis: str = "Z",
    vertices: int = 48,
) -> bpy.types.Object:
    rotation = {
        "X": (0.0, math.pi * 0.5, 0.0),
        "Y": (math.pi * 0.5, 0.0, 0.0),
        "Z": (0.0, 0.0, 0.0),
    }[axis]
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=depth,
        location=location,
        rotation=rotation,
    )
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(material)
    move_to_collection(obj, collection)
    for face in obj.data.polygons:
        face.use_smooth = True
    return obj


def add_uv_sphere(
    collection: bpy.types.Collection,
    name: str,
    radius: float,
    location: tuple[float, float, float],
    material: bpy.types.Material,
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=radius, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(material)
    move_to_collection(obj, collection)
    return obj


def add_ribbon_between(
    collection: bpy.types.Collection,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    width: float,
    thickness: float,
    material: bpy.types.Material,
) -> bpy.types.Object:
    a = Vector(start)
    b = Vector(end)
    delta = b - a
    length = math.sqrt(delta.x * delta.x + delta.z * delta.z)
    angle_y = -math.atan2(delta.z, delta.x)
    return add_box(
        collection,
        name,
        (length, width, thickness),
        tuple((a + b) * 0.5),
        material,
        bevel=0.0004,
        rotation=(0.0, angle_y, 0.0),
    )


def build_machine() -> tuple[bpy.types.Collection, list[bpy.types.Object]]:
    collection = bpy.data.collections.new("LabelStripper_218x190")
    bpy.context.scene.collection.children.link(collection)

    black = make_material("Black Powder Coat", (0.018, 0.022, 0.026, 1.0), 0.48, 0.30)
    panel = make_material("Control Panel", (0.055, 0.065, 0.072, 1.0), 0.35, 0.32)
    steel = make_material("Polished Stainless", (0.58, 0.64, 0.68, 1.0), 0.94, 0.14)
    brushed = make_material("Brushed Aluminum", (0.46, 0.50, 0.53, 1.0), 0.82, 0.25)
    rubber = make_material("Black Rubber", (0.006, 0.008, 0.010, 1.0), 0.10, 0.68)
    paper = make_material("Label Paper", (0.93, 0.95, 0.96, 1.0), 0.0, 0.62)
    backing = make_material("Backing Paper", (0.76, 0.78, 0.75, 1.0), 0.0, 0.70)
    label_line = make_material("Label Separation", (0.55, 0.62, 0.66, 1.0), 0.0, 0.62)
    red = make_material("Controls Red", (0.56, 0.018, 0.012, 1.0), 0.22, 0.27)
    yellow = make_material("Knob Marker", (0.95, 0.54, 0.025, 1.0), 0.18, 0.32)
    pad_mat = make_material("Machine Pad", (0.075, 0.095, 0.115, 1.0), 0.56, 0.33)
    origin_mat = make_material(
        "Emission Point",
        (0.08, 0.95, 0.85, 1.0),
        0.10,
        0.25,
        emission=(0.04, 0.65, 0.58, 1.0),
    )

    # The pad bottom is exactly -150.4 mm and its top meets the machine base.
    add_box(
        collection,
        "HeightPad_70_4mm",
        (0.228, 0.200, PAD_HEIGHT),
        (-0.091, 0.0, -ORIGIN_HEIGHT + PAD_HEIGHT * 0.5),
        pad_mat,
        bevel=0.006,
    )
    add_box(
        collection,
        "BasePlate_218x190",
        (BASE_LENGTH, BASE_WIDTH, 0.004),
        (-0.091, 0.0, MACHINE_BASE_Z + 0.002),
        black,
        bevel=0.003,
    )

    # 160 x 100 x 120 mm motor/control enclosure from the dimension photo.
    housing_center_x = -0.128
    add_box(
        collection,
        "MotorHousing_160x100x120",
        (HOUSING_DEPTH, HOUSING_WIDTH, HOUSING_HEIGHT),
        (housing_center_x, -0.035, MACHINE_BASE_Z + HOUSING_HEIGHT * 0.5 + 0.004),
        black,
        bevel=0.004,
    )
    add_box(
        collection,
        "ControlPanel",
        (0.112, 0.082, 0.003),
        (-0.136, -0.035, 0.0455),
        panel,
        bevel=0.002,
    )

    # Stainless U-handle on the housing lid.
    for x in (-0.168, -0.105):
        add_cylinder(collection, f"LidHandlePost_{x}", 0.003, 0.032, (x, -0.006, 0.061), steel, "Z", 24)
    add_cylinder(collection, "LidHandleGrip", 0.003, 0.063, (-0.1365, -0.006, 0.077), steel, "X", 24)

    # Speed knob and power switch.
    add_cylinder(collection, "SpeedKnob", 0.011, 0.012, (-0.145, -0.051, 0.058), rubber, "Z", 32)
    add_box(collection, "SpeedMarker", (0.003, 0.007, 0.001), (-0.145, -0.055, 0.0645), yellow, bevel=0.0004)
    add_cylinder(collection, "PowerSwitch", 0.009, 0.010, (-0.090, -0.051, 0.057), red, "Z", 32)

    # Front fasteners and drive hub.
    for x in (-0.202, -0.054):
        for z in (-0.060, 0.030):
            add_cylinder(collection, f"HousingFastener_{x}_{z}", 0.003, 0.002, (x, -0.086, z), steel, "Y", 20)
    add_cylinder(collection, "DriveHub", 0.017, 0.012, (-0.082, -0.087, -0.034), rubber, "Y", 40)
    add_cylinder(collection, "DriveHubCenter", 0.006, 0.015, (-0.082, -0.089, -0.034), steel, "Y", 32)

    # 120 mm front guide/peel assembly.
    add_box(collection, "PeelPlate", (0.072, GUIDE_SPAN, 0.004), (-0.050, 0.015, -0.003), brushed, bevel=0.0015)
    add_cylinder(collection, "PeelRoller", 0.010, GUIDE_SPAN, (-0.036, 0.015, -0.012), steel, "Y", 48)
    add_cylinder(collection, "PressureRoller", 0.012, GUIDE_SPAN, (-0.076, 0.015, -0.026), brushed, "Y", 48)
    for side in (-1.0, 1.0):
        add_box(
            collection,
            f"GuideBracket_{side:+.0f}",
            (0.064, 0.004, 0.036),
            (-0.055, 0.015 + side * (GUIDE_SPAN * 0.5 + 0.004), -0.014),
            black,
            bevel=0.002,
        )
    add_cylinder(collection, "GuideAdjustmentShaft", 0.003, 0.148, (-0.020, 0.015, 0.005), steel, "Y", 24)
    add_box(collection, "GuideAdjustmentHandle", (0.003, 0.003, 0.038), (-0.020, 0.091, 0.023), steel, bevel=0.001)

    # 188 mm roll mast, spindle, retaining plates, and roll.
    mast_center_z = MACHINE_BASE_Z + 0.004 + MAST_HEIGHT * 0.5
    for side in (-1.0, 1.0):
        add_box(
            collection,
            f"RollMast_{side:+.0f}_188mm",
            (0.022, 0.004, MAST_HEIGHT),
            (-0.166, side * 0.068, mast_center_z),
            black,
            bevel=0.002,
        )
    roll_center = Vector((-0.140, 0.0, 0.060))
    add_cylinder(collection, "RollSpindle", 0.006, 0.190, tuple(roll_center), steel, "Y", 40)
    add_cylinder(collection, "LabelRoll", 0.055, LABEL_WIDTH, tuple(roll_center), paper, "Y", 96)
    add_cylinder(collection, "RollCore", 0.016, 0.124, tuple(roll_center), backing, "Y", 48)
    for side in (-1.0, 1.0):
        add_cylinder(
            collection,
            f"RollRetainer_{side:+.0f}",
            0.064,
            0.003,
            (-0.140, side * 0.063, 0.060),
            brushed,
            "Y",
            64,
        )
    add_cylinder(collection, "SpindleHandle", 0.008, 0.055, (-0.140, -0.118, 0.060), rubber, "Y", 32)
    add_cylinder(collection, "SpindleLock", 0.006, 0.018, (-0.140, 0.083, 0.035), red, "Z", 24)

    # Label web follows the roll down to the peel edge.
    add_ribbon_between(
        collection,
        "LabelWebUpper",
        (-0.105, 0.0, 0.102),
        (-0.068, 0.0, 0.018),
        LABEL_WIDTH,
        0.0012,
        paper,
    )
    add_ribbon_between(
        collection,
        "LabelWebLower",
        (-0.068, 0.0, 0.018),
        (-0.035, 0.0, 0.000),
        LABEL_WIDTH,
        0.0012,
        paper,
    )
    add_ribbon_between(
        collection,
        "BackingReturn",
        (-0.035, 0.0, -0.002),
        (-0.080, 0.0, -0.055),
        LABEL_WIDTH,
        0.0008,
        backing,
    )

    # The emitted label is deliberately centered on Blender world origin.
    emitted = add_box(
        collection,
        "EmittedLabel_CENTER_ORIGIN",
        (0.070, LABEL_WIDTH, 0.0008),
        (0.0, 0.0, 0.0),
        paper,
        bevel=0.001,
    )
    emitted["coordinate_origin"] = "Emitted label center: (0, 0, 0)"
    emitted["label_width_mm"] = 120
    # Fine border makes the loose label readable against the peel plate.
    for y in (-LABEL_WIDTH * 0.5, LABEL_WIDTH * 0.5):
        add_box(collection, f"LabelEdge_{y:+.3f}", (0.068, 0.0007, 0.0003), (0.0, y, 0.00055), label_line)

    # A small center marker sits just below the label, visible only from low angles.
    add_uv_sphere(collection, "EmissionOriginMarker", 0.0023, (0.0, 0.0, -0.0022), origin_mat)
    bpy.ops.object.empty_add(type="PLAIN_AXES", radius=0.020, location=(0.0, 0.0, 0.0))
    origin = bpy.context.object
    origin.name = "EmittedLabelCenter_ORIGIN_0_0_0"
    move_to_collection(origin, collection)

    objects = list(collection.objects)
    for obj in objects:
        obj["units"] = "metres"
        obj["placement_mg400_mm"] = "X 53.0, Y 231.4, Z 150.4"
    return collection, objects


def export_glb(objects: list[bpy.types.Object]) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    meshes = [obj for obj in objects if obj.type == "MESH"]
    bpy.context.view_layer.objects.active = meshes[0]
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
    ground_mat = make_material("Preview Ground", (0.030, 0.040, 0.055, 1.0), 0.04, 0.58)
    add_box(preview, "PreviewGround", (0.75, 0.65, 0.006), (-0.075, 0.0, -ORIGIN_HEIGHT - 0.004), ground_mat, bevel=0.002)

    bpy.ops.object.camera_add(location=(0.58, -0.70, 0.43))
    camera = bpy.context.object
    camera.name = "PreviewCamera"
    camera.data.lens = 62
    point_at(camera, Vector((-0.085, 0.0, -0.020)))
    move_to_collection(camera, preview)
    bpy.context.scene.camera = camera

    bpy.ops.object.light_add(type="AREA", location=(0.05, -0.20, 0.52))
    key = bpy.context.object
    key.name = "KeyLight"
    key.data.energy = 105
    key.data.shape = "RECTANGLE"
    key.data.size = 0.40
    point_at(key, Vector((-0.09, 0.0, 0.0)))
    move_to_collection(key, preview)

    bpy.ops.object.light_add(type="AREA", location=(-0.36, 0.30, 0.18))
    fill = bpy.context.object
    fill.name = "FillLight"
    fill.data.energy = 65
    fill.data.size = 0.30
    point_at(fill, Vector((-0.10, 0.0, 0.0)))
    move_to_collection(fill, preview)

    scene = bpy.context.scene
    scene.world.color = (0.012, 0.018, 0.028)
    scene.world.use_nodes = True
    scene.world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.012, 0.018, 0.028, 1.0)
    scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.25
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(RENDER_PATH)
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.view_settings.exposure = -0.45
    bpy.ops.render.render(write_still=True)


def validate(objects: list[bpy.types.Object]) -> None:
    emitted = bpy.data.objects["EmittedLabel_CENTER_ORIGIN"]
    if emitted.location.length > 0.000001:
        raise RuntimeError(f"Emitted label center is not the origin: {emitted.location}")
    mesh_objects = [obj for obj in objects if obj.type == "MESH"]
    corners = [obj.matrix_world @ Vector(corner) for obj in mesh_objects for corner in obj.bound_box]
    minimum = Vector(min(c[i] for c in corners) for i in range(3))
    maximum = Vector(max(c[i] for c in corners) for i in range(3))
    if abs(minimum.z + ORIGIN_HEIGHT) > 0.0001:
        raise RuntimeError(f"Pad bottom {minimum.z} is not {-ORIGIN_HEIGHT}")
    print("Emitted-label center (m):", tuple(emitted.location))
    print("Model bounds min/max (m):", tuple(round(v, 5) for v in minimum), tuple(round(v, 5) for v in maximum))
    print("Pad height (mm):", round(PAD_HEIGHT * 1000.0, 3))


def main() -> None:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    RENDER_PATH.parent.mkdir(parents=True, exist_ok=True)
    reset_scene()
    _, objects = build_machine()
    validate(objects)
    export_glb(objects)
    create_preview()
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_PATH))
    print(f"Saved {BLEND_PATH}")
    print(f"Saved {GLB_PATH}")
    print(f"Saved {RENDER_PATH}")


if __name__ == "__main__":
    main()

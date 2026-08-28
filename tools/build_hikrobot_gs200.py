"""Build a HIKROBOT MV-CS200-10GC camera with a 20 MP C/CS lens.

The camera-body origin is its geometric center. Blender dimensions are metres.
The second-generation CS housing is exactly 29 x 42 x 29 mm (X, Y, Z), with
the optical axis pointing toward negative Blender Y. The attached lens is a
compact 20 MP 1-inch C/CS-mount lens with 33 mm maximum diameter.

Run with:
    blender --background --python tools/build_hikrobot_gs200.py
"""

from __future__ import annotations

import math
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parent.parent
ASSET_DIR = ROOT / "assets" / "hikrobot_gs200"
SOURCE_DIR = ROOT / "source_assets" / "hikrobot_gs200"
BLEND_PATH = SOURCE_DIR / "hikrobot_gs200.blend"
GLB_PATH = ASSET_DIR / "hikrobot_gs200.glb"
RENDER_PATH = ROOT / "material" / "generated" / "hikrobot_gs200_blender_render.png"

BODY_WIDTH = 0.029
BODY_DEPTH = 0.042
BODY_HEIGHT = 0.029
LENS_MAX_DIAMETER = 0.033
LENS_OPTICAL_LENGTH = 0.042
FRONT_FACE_Y = -BODY_DEPTH * 0.5
MOUNT_FACE_Y = FRONT_FACE_Y - 0.005


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
    emission: tuple[float, float, float, float] | None = None,
    emission_strength: float = 1.0,
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
        principled.inputs["Emission Strength"].default_value = emission_strength
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
    bevel_segments: int = 3,
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_cube_add(location=location, rotation=rotation)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel > 0.0:
        modifier = obj.modifiers.new("Machined edge", "BEVEL")
        modifier.width = bevel
        modifier.segments = bevel_segments
        modifier.limit_method = "ANGLE"
        modifier.harden_normals = True
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
    axis: str = "Y",
    vertices: int = 64,
    smooth: bool = True,
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
    if smooth:
        for face in obj.data.polygons:
            face.use_smooth = True
    return obj


def add_ring(
    collection: bpy.types.Collection,
    name: str,
    major_radius: float,
    minor_radius: float,
    location: tuple[float, float, float],
    material: bpy.types.Material,
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_torus_add(
        major_radius=major_radius,
        minor_radius=minor_radius,
        major_segments=72,
        minor_segments=12,
        location=location,
        rotation=(math.pi * 0.5, 0.0, 0.0),
    )
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(material)
    move_to_collection(obj, collection)
    for face in obj.data.polygons:
        face.use_smooth = True
    return obj


def add_text(
    collection: bpy.types.Collection,
    body: str,
    name: str,
    location: tuple[float, float, float],
    size: float,
    material: bpy.types.Material,
    rotation: tuple[float, float, float],
    extrude: float = 0.000018,
) -> bpy.types.Object:
    bpy.ops.object.text_add(location=location, rotation=rotation)
    obj = bpy.context.object
    obj.name = name
    obj.data.body = body
    obj.data.align_x = "CENTER"
    obj.data.align_y = "CENTER"
    obj.data.size = size
    obj.data.extrude = extrude
    obj.data.bevel_depth = 0.000005
    obj.data.materials.append(material)
    bpy.ops.object.convert(target="MESH")
    move_to_collection(obj, collection)
    return obj


def add_knurl_ring(
    collection: bpy.types.Collection,
    name: str,
    center_y: float,
    radius: float,
    length: float,
    material: bpy.types.Material,
    ridge_material: bpy.types.Material,
) -> None:
    add_cylinder(collection, name, radius, length, (0.0, center_y, 0.0), material, "Y", 96)
    for index in range(36):
        angle = math.tau * index / 36.0
        x = (radius - 0.00015) * math.cos(angle)
        z = (radius - 0.00015) * math.sin(angle)
        ridge = add_box(
            collection,
            f"{name}_Knurl_{index:02d}",
            (0.00020, length * 0.82, 0.00020),
            (x, center_y, z),
            ridge_material,
            bevel=0.00006,
        )
        ridge.rotation_euler.y = -angle


def build_camera() -> tuple[bpy.types.Collection, list[bpy.types.Object]]:
    collection = bpy.data.collections.new("HIKROBOT_MV_CS200_10GC_20MP")
    bpy.context.scene.collection.children.link(collection)

    black = make_material("Black Anodized Aluminum", (0.014, 0.017, 0.020, 1.0), 0.72, 0.25)
    panel = make_material("Machined Side Panels", (0.035, 0.040, 0.046, 1.0), 0.64, 0.30)
    dark = make_material("Internal Black", (0.003, 0.004, 0.006, 1.0), 0.12, 0.52)
    steel = make_material("C Mount Stainless", (0.67, 0.70, 0.72, 1.0), 0.92, 0.16)
    lens_black = make_material("Lens Black", (0.008, 0.010, 0.012, 1.0), 0.55, 0.24)
    knurl = make_material("Knurl Highlights", (0.075, 0.082, 0.088, 1.0), 0.66, 0.22)
    glass = make_material("Optical Glass", (0.018, 0.085, 0.115, 1.0), 0.25, 0.08)
    sensor = make_material("IMX183 Sensor", (0.035, 0.19, 0.16, 1.0), 0.35, 0.12)
    white = make_material("Printed White", (0.86, 0.89, 0.91, 1.0), 0.0, 0.38)
    orange = make_material("HIKROBOT Orange", (0.95, 0.22, 0.025, 1.0), 0.05, 0.35)
    led_green = make_material(
        "Status LED Green", (0.02, 0.55, 0.12, 1.0), 0.0, 0.22, (0.01, 0.65, 0.10, 1.0), 1.8
    )
    copper = make_material("Connector Contacts", (0.72, 0.34, 0.08, 1.0), 0.76, 0.20)

    body = add_box(
        collection,
        "BodyHousing_EXACT_29x42x29mm",
        (BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT),
        (0.0, 0.0, 0.0),
        black,
        bevel=0.00105,
        bevel_segments=4,
    )
    body["official_body_dimensions_mm"] = "29 x 29 x 42"
    body["model"] = "MV-CS200-10GC"
    body["sensor"] = "Sony IMX183, 5472 x 3648, 20 MP"

    # Slightly inset removable side panels and their fasteners.
    for side in (-1.0, 1.0):
        x = side * (BODY_WIDTH * 0.5 + 0.00003)
        add_box(collection, f"SidePanel_{side:+.0f}", (0.00010, 0.032, 0.023), (x, 0.001, 0.0), panel, 0.0007)
        for y in (-0.0145, 0.0145):
            for z in (-0.0102, 0.0102):
                add_cylinder(collection, f"SideScrew_{side:+.0f}_{y:+.3f}_{z:+.3f}", 0.0010, 0.00016, (x, y, z), dark, "X", 24)

    # HIKROBOT side wordmark and orange underline follow the official image.
    add_text(
        collection,
        "HIKROBOT",
        "HIKROBOT_Wordmark",
        (BODY_WIDTH * 0.5 + 0.00010, -0.001, 0.0010),
        0.0032,
        white,
        (math.pi * 0.5, 0.0, math.pi * 0.5),
    )
    add_box(
        collection,
        "HIKROBOT_Orange_Underline",
        (0.00010, 0.0175, 0.00045),
        (BODY_WIDTH * 0.5 + 0.00016, -0.001, -0.0033),
        orange,
        bevel=0.00012,
    )

    # Mounting holes on four faces make the second-generation body installable from any side.
    for z_side in (-1.0, 1.0):
        z = z_side * (BODY_HEIGHT * 0.5 + 0.00003)
        for y in (-0.0125, 0.0125):
            add_cylinder(collection, f"M3_Mount_Z{z_side:+.0f}_{y:+.3f}", 0.00155, 0.00016, (0.0, y, z), dark, "Z", 32)

    # Front flange, sensor cavity, C-mount threads, and four front screws.
    add_box(collection, "FrontFacePlate", (0.0285, 0.0012, 0.0285), (0.0, FRONT_FACE_Y - 0.0006, 0.0), panel, 0.0012)
    add_box(collection, "SensorCavity", (0.0158, 0.0005, 0.0115), (0.0, FRONT_FACE_Y - 0.00130, 0.0), dark, 0.0007)
    add_box(collection, "IMX183_1inch_SensorWindow", (0.0132, 0.00025, 0.0088), (0.0, FRONT_FACE_Y - 0.00158, 0.0), sensor, 0.00035)
    for x in (-0.0112, 0.0112):
        for z in (-0.0112, 0.0112):
            add_cylinder(collection, f"FrontScrew_{x:+.3f}_{z:+.3f}", 0.00115, 0.00035, (x, FRONT_FACE_Y - 0.00135, z), dark, "Y", 28)

    add_cylinder(collection, "C_Mount_Black_Boss", 0.0140, 0.0040, (0.0, FRONT_FACE_Y - 0.0030, 0.0), panel, "Y", 96)
    add_cylinder(collection, "C_Mount_Silver_Ring", 0.01365, 0.0018, (0.0, MOUNT_FACE_Y + 0.0009, 0.0), steel, "Y", 96)
    add_cylinder(collection, "C_Mount_Thread_Cavity", 0.0117, 0.0021, (0.0, MOUNT_FACE_Y - 0.0002, 0.0), dark, "Y", 96)
    for thread_index in range(8):
        add_ring(
            collection,
            f"C_Mount_Thread_{thread_index:02d}",
            0.01218,
            0.00012,
            (0.0, MOUNT_FACE_Y - 0.00025 + thread_index * 0.00026, 0.0),
            steel,
        )

    # 20 MP C/CS lens: 42 mm optical length from the camera mount face.
    lens_back = MOUNT_FACE_Y
    cursor = lens_back

    def lens_segment(name: str, diameter: float, length: float, material: bpy.types.Material) -> float:
        nonlocal cursor
        center = cursor - length * 0.5
        add_cylinder(collection, name, diameter * 0.5, length, (0.0, center, 0.0), material, "Y", 96)
        cursor -= length
        return center

    lens_segment("C_CS_5mm_Spacer", 0.0260, 0.0050, steel)
    lens_segment("LensRearBarrel", 0.0285, 0.0065, lens_black)
    focus_center = cursor - 0.00625
    add_knurl_ring(collection, "FocusRing", focus_center, LENS_MAX_DIAMETER * 0.5, 0.0125, lens_black, knurl)
    cursor -= 0.0125
    lens_segment("LensCenterBarrel", 0.0290, 0.0050, lens_black)
    iris_center = cursor - 0.0045
    add_knurl_ring(collection, "IrisRing", iris_center, 0.0157, 0.0090, lens_black, knurl)
    cursor -= 0.0090
    lens_segment("LensFrontBarrel", 0.0305, 0.0040, lens_black)

    # The segment sum above is exactly 42 mm.
    if abs(cursor - (lens_back - LENS_OPTICAL_LENGTH)) > 0.000001:
        raise RuntimeError("Lens segment lengths no longer sum to 42 mm")

    add_cylinder(collection, "LensFrontBezel", 0.0152, 0.0010, (0.0, cursor - 0.0005, 0.0), knurl, "Y", 96)
    add_cylinder(collection, "20MP_FrontGlass", 0.0122, 0.00035, (0.0, cursor - 0.00102, 0.0), glass, "Y", 96)
    add_ring(collection, "FrontGlassRetainer", 0.0129, 0.00055, (0.0, cursor - 0.00118, 0.0), lens_black)

    # Focus and iris index marks remain readable in close-up Godot views.
    for index, label in enumerate(("0.1", "0.3", "1", "INF")):
        y = focus_center + 0.0045 - index * 0.0027
        add_box(collection, f"FocusScaleTick_{index}", (0.00045, 0.0013, 0.00010), (0.0, y, 0.01635), white, 0.00005)
    add_text(collection, "20MP  C  16mm", "LensSpecification", (0.0, cursor + 0.0145, 0.01515), 0.00155, white, (0.0, 0.0, 0.0))

    # Rear I/O: recessed RJ45 GigE port, 6-pin Hirose connector and status LED.
    rear_y = BODY_DEPTH * 0.5
    add_box(collection, "RearPlate", (0.0276, 0.0010, 0.0276), (0.0, rear_y + 0.0005, 0.0), panel, 0.0010)
    add_box(collection, "RJ45_GigE_Recess", (0.0128, 0.0020, 0.0108), (-0.0068, rear_y + 0.0013, 0.0020), dark, 0.0010)
    add_box(collection, "RJ45_GigE_Shield", (0.0114, 0.0007, 0.0092), (-0.0068, rear_y + 0.00255, 0.0020), steel, 0.0006)
    add_box(collection, "RJ45_GigE_Opening", (0.0097, 0.00045, 0.0069), (-0.0068, rear_y + 0.00295, 0.0020), dark, 0.00045)
    for index in range(8):
        x = -0.0105 + index * 0.00105
        add_box(collection, f"RJ45_Contact_{index}", (0.00038, 0.00012, 0.0024), (x, rear_y + 0.00321, 0.0020), copper, 0.00004)

    add_cylinder(collection, "Hirose_IO_Outer", 0.0046, 0.0025, (0.0076, rear_y + 0.0018, -0.0065), steel, "Y", 48)
    add_cylinder(collection, "Hirose_IO_Insert", 0.0037, 0.0027, (0.0076, rear_y + 0.0022, -0.0065), dark, "Y", 48)
    for index in range(6):
        angle = math.tau * index / 6.0
        x = 0.0076 + 0.00225 * math.cos(angle)
        z = -0.0065 + 0.00225 * math.sin(angle)
        add_cylinder(collection, f"Hirose_Pin_{index}", 0.00024, 0.00020, (x, rear_y + 0.00362, z), copper, "Y", 16)
    add_cylinder(collection, "StatusLED", 0.00115, 0.0005, (0.0107, rear_y + 0.0011, 0.0090), led_green, "Y", 32)

    add_text(collection, "GigE", "RearGigELabel", (-0.0068, rear_y + 0.0033, 0.0097), 0.0013, white, (math.pi * 0.5, 0.0, 0.0))

    objects = list(collection.objects)
    for obj in objects:
        obj["units"] = "metres"
        obj["official_reference"] = "HIKROBOT CS Series Area Scan Camera"
        obj["camera_model"] = "MV-CS200-10GC"
    return collection, objects


def mesh_bounds(objects: list[bpy.types.Object]) -> tuple[Vector, Vector]:
    meshes = [obj for obj in objects if obj.type == "MESH"]
    corners = [obj.matrix_world @ Vector(corner) for obj in meshes for corner in obj.bound_box]
    minimum = Vector(min(c[index] for c in corners) for index in range(3))
    maximum = Vector(max(c[index] for c in corners) for index in range(3))
    return minimum, maximum


def export_glb(objects: list[bpy.types.Object]) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    meshes = [obj for obj in objects if obj.type == "MESH"]
    for obj in meshes:
        obj.select_set(True)
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
    ground_material = make_material("Preview Ground", (0.025, 0.032, 0.044, 1.0), 0.10, 0.48)
    add_box(preview, "PreviewGround", (0.18, 0.20, 0.003), (0.0, -0.022, -0.0190), ground_material, 0.001)

    bpy.ops.object.camera_add(location=(0.105, -0.155, 0.090))
    camera = bpy.context.object
    camera.name = "PreviewCamera"
    camera.data.lens = 61
    point_at(camera, Vector((0.0, -0.022, 0.0)))
    move_to_collection(camera, preview)
    bpy.context.scene.camera = camera

    bpy.ops.object.light_add(type="AREA", location=(0.015, -0.065, 0.120))
    key = bpy.context.object
    key.name = "KeyLight"
    key.data.energy = 2.8
    key.data.size = 0.090
    point_at(key, Vector((0.0, -0.020, 0.0)))
    move_to_collection(key, preview)

    bpy.ops.object.light_add(type="AREA", location=(-0.090, 0.055, 0.045))
    fill = bpy.context.object
    fill.name = "FillLight"
    fill.data.energy = 1.2
    fill.data.size = 0.075
    point_at(fill, Vector((0.0, 0.0, 0.0)))
    move_to_collection(fill, preview)

    bpy.ops.object.light_add(type="AREA", location=(0.0, 0.070, 0.090))
    rim = bpy.context.object
    rim.name = "RimLight"
    rim.data.energy = 1.6
    rim.data.size = 0.055
    point_at(rim, Vector((0.0, 0.0, 0.0)))
    move_to_collection(rim, preview)

    scene = bpy.context.scene
    scene.world.color = (0.010, 0.014, 0.023)
    scene.world.use_nodes = True
    scene.world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.010, 0.014, 0.023, 1.0)
    scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.25
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 820
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(RENDER_PATH)
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.view_settings.exposure = -0.80
    bpy.ops.render.render(write_still=True)


def validate(objects: list[bpy.types.Object]) -> None:
    body = bpy.data.objects["BodyHousing_EXACT_29x42x29mm"]
    expected_body = Vector((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT))
    if any(abs(body.dimensions[index] - expected_body[index]) > 0.000001 for index in range(3)):
        raise RuntimeError(f"Body dimensions changed: {body.dimensions}, expected {expected_body}")
    minimum, maximum = mesh_bounds(objects)
    dimensions = maximum - minimum
    if abs(dimensions.x - LENS_MAX_DIAMETER) > 0.0002 or abs(dimensions.z - LENS_MAX_DIAMETER) > 0.0002:
        raise RuntimeError(f"Unexpected lens envelope: {dimensions}")
    print("Camera body (mm):", tuple(round(value * 1000.0, 3) for value in body.dimensions))
    print("Camera + lens bounds (mm):", tuple(round(value * 1000.0, 3) for value in dimensions))
    print("Assembly min/max (m):", tuple(round(value, 5) for value in minimum), tuple(round(value, 5) for value in maximum))


def main() -> None:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    RENDER_PATH.parent.mkdir(parents=True, exist_ok=True)
    reset_scene()
    _, objects = build_camera()
    validate(objects)
    export_glb(objects)
    create_preview()
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_PATH))
    print(f"Saved {BLEND_PATH}")
    print(f"Saved {GLB_PATH}")
    print(f"Saved {RENDER_PATH}")


if __name__ == "__main__":
    main()

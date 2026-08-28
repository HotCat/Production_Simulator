"""Build the thin molded-plastic product shown in material/1.jpg ... 5.jpg.

The model origin is its geometric center. Blender dimensions are metres and
the exported bounds are exactly 37.95 x 46.00 x 4.00 mm (X, Y, Z).

Run with:
    blender --background --python tools/build_product.py
"""

from __future__ import annotations

import math
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parent.parent
ASSET_DIR = ROOT / "assets" / "product"
SOURCE_DIR = ROOT / "source_assets" / "product"
BLEND_PATH = SOURCE_DIR / "product.blend"
GLB_PATH = ASSET_DIR / "product.glb"
RENDER_PATH = ROOT / "material" / "generated" / "product_blender_render.png"

WIDTH = 0.03795
LENGTH = 0.04600
THICKNESS = 0.00400


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
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel > 0.0:
        modifier = obj.modifiers.new("Molded edge radius", "BEVEL")
        modifier.width = bevel
        modifier.segments = bevel_segments
        modifier.limit_method = "ANGLE"
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    obj.data.materials.append(material)
    move_to_collection(obj, collection)
    return obj


def rounded_outline() -> list[tuple[float, float]]:
    """Return the stepped product silhouette, counter-clockwise in XY."""
    half_w = WIDTH * 0.5
    half_l = LENGTH * 0.5
    points: list[tuple[float, float]] = []

    def arc(cx: float, cy: float, radius: float, start: float, end: float, count: int = 6) -> None:
        for index in range(count + 1):
            angle = start + (end - start) * index / count
            points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))

    # Start on the top edge and trace the right-hand side down around the base.
    points.append((0.0, half_l))
    points.append((half_w - 0.0055, half_l))
    arc(half_w - 0.0055, half_l - 0.0055, 0.0055, math.pi * 0.5, 0.0)
    points.append((half_w, 0.0030))
    points.append((0.0165, 0.0023))
    points.append((0.0146, 0.0005))
    points.append((0.0146, -0.0108))
    points.append((half_w, -half_l + 0.0048))
    arc(half_w - 0.0048, -half_l + 0.0048, 0.0048, 0.0, -math.pi * 0.5)
    points.append((-half_w + 0.0048, -half_l))
    arc(-half_w + 0.0048, -half_l + 0.0048, 0.0048, -math.pi * 0.5, -math.pi)
    points.append((-half_w, -0.0146))
    points.append((-0.0165, -0.0118))
    points.append((-0.0147, -0.0105))
    points.append((-0.0147, 0.0004))
    points.append((-0.0164, 0.0022))
    points.append((-half_w, 0.0030))
    points.append((-half_w, half_l - 0.0055))
    arc(-half_w + 0.0055, half_l - 0.0055, 0.0055, math.pi, math.pi * 0.5)
    return points


def add_extruded_polygon(
    collection: bpy.types.Collection,
    name: str,
    outline: list[tuple[float, float]],
    z_min: float,
    z_max: float,
    material: bpy.types.Material,
    bevel: float = 0.0,
    bevel_segments: int = 3,
) -> bpy.types.Object:
    count = len(outline)
    vertices = [(x, y, z_min) for x, y in outline] + [(x, y, z_max) for x, y in outline]
    faces: list[tuple[int, ...]] = [tuple(reversed(range(count))), tuple(range(count, count * 2))]
    for index in range(count):
        next_index = (index + 1) % count
        faces.append((index, next_index, next_index + count, index + count))
    mesh = bpy.data.meshes.new(f"{name}_Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    collection.objects.link(obj)
    obj.data.materials.append(material)
    if bevel > 0.0:
        modifier = obj.modifiers.new("Molded edge radius", "BEVEL")
        modifier.width = bevel
        modifier.segments = bevel_segments
        modifier.limit_method = "ANGLE"
        modifier.harden_normals = True
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.modifier_apply(modifier=modifier.name)
        obj.select_set(False)
    return obj


def rounded_rectangle_outline(width: float, height: float, radius: float, clipped: bool = False) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    points: list[tuple[float, float]] = []
    segments = 5
    corners = [
        (half_w - radius, half_h - radius, 0.0, math.pi * 0.5),
        (-half_w + radius, half_h - radius, math.pi * 0.5, math.pi),
        (-half_w + radius, -half_h + radius, math.pi, math.pi * 1.5),
    ]
    for cx, cy, start, end in corners:
        for index in range(segments + 1):
            angle = start + (end - start) * index / segments
            points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    if clipped:
        points.extend(((half_w - 0.0023, -half_h), (half_w, -half_h + 0.0023)))
    else:
        cx, cy = half_w - radius, -half_h + radius
        for index in range(segments + 1):
            angle = math.pi * 1.5 + math.pi * 0.5 * index / segments
            points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points


def add_text(
    collection: bpy.types.Collection,
    body: str,
    name: str,
    location: tuple[float, float, float],
    size: float,
    material: bpy.types.Material,
    rotation_z: float = 0.0,
) -> bpy.types.Object:
    bpy.ops.object.text_add(location=location, rotation=(0.0, 0.0, rotation_z))
    text = bpy.context.object
    text.name = name
    text.data.body = body
    text.data.align_x = "CENTER"
    text.data.align_y = "CENTER"
    text.data.size = size
    text.data.extrude = 0.000018
    text.data.bevel_depth = 0.000006
    text.data.materials.append(material)
    bpy.ops.object.convert(target="MESH")
    move_to_collection(text, collection)
    return text


def build_product() -> tuple[bpy.types.Collection, list[bpy.types.Object]]:
    collection = bpy.data.collections.new("Product_37_95x46x4mm")
    bpy.context.scene.collection.children.link(collection)

    shell = make_material("Warm White Molded Plastic", (0.82, 0.81, 0.75, 1.0), 0.0, 0.39)
    top = make_material("Satin White Top", (0.93, 0.92, 0.86, 1.0), 0.0, 0.34)
    seam = make_material("Molded Seam", (0.58, 0.58, 0.55, 1.0), 0.0, 0.58)
    print_gray = make_material("Printed Gray", (0.27, 0.29, 0.29, 1.0), 0.0, 0.52)
    recess = make_material("Port Recess", (0.055, 0.065, 0.068, 1.0), 0.10, 0.43)

    outline = rounded_outline()
    body = add_extruded_polygon(
        collection,
        "MoldedHousing_EXACT_BOUNDS",
        outline,
        -THICKNESS * 0.5,
        0.00155,
        shell,
        bevel=0.00042,
        bevel_segments=4,
    )
    body["dimensions_mm"] = "37.95 x 46.00 x 4.00"
    body["coordinate_origin"] = "Geometric center"

    # A shallow inset top skin gives the photographed two-part molded seam.
    inset = [(x * 0.982, y * 0.985) for x, y in outline]
    add_extruded_polygon(collection, "TopShell", inset, 0.00150, 0.00172, top, bevel=0.00028, bevel_segments=3)

    # Recessed label/battery panel with the characteristic clipped corner.
    panel_center = (0.0, 0.0093)
    panel_outer = rounded_rectangle_outline(0.0278, 0.0102, 0.0010, clipped=True)
    panel_outer = [(x + panel_center[0], y + panel_center[1]) for x, y in panel_outer]
    add_extruded_polygon(collection, "PanelBorder", panel_outer, 0.00173, 0.00180, seam)
    panel_inner = rounded_rectangle_outline(0.0272, 0.0096, 0.00078, clipped=True)
    panel_inner = [(x + panel_center[0], y + panel_center[1]) for x, y in panel_inner]
    add_extruded_polygon(collection, "RecessedPanel", panel_inner, 0.00178, 0.00184, top)

    # USB/charging opening and lower pill slot are read as real recesses.
    add_box(collection, "USB_Port_Recess", (0.0019, 0.0047, 0.00012), (-0.0111, -0.0050, 0.00184), recess, 0.00065, 5)
    add_box(collection, "Lower_Pill_Slot", (0.0068, 0.00215, 0.00012), (0.0016, -0.0181, 0.00184), recess, 0.00095, 6)
    add_box(collection, "Lower_Pill_Slot_Highlight", (0.0049, 0.00072, 0.00007), (0.0018, -0.0180, 0.00193), seam, 0.00034, 4)

    # Small molded latch/tab visible at the middle of the lower edge.
    add_box(collection, "LowerLatch", (0.0048, 0.00115, 0.00034), (0.0, -0.02234, 0.00055), top, 0.00035, 4)

    # Printed markings. The text is converted to mesh so Godot imports it.
    add_text(collection, "96008", "ServiceNumber", (0.0024, -0.0017, 0.00191), 0.00235, print_gray)
    add_text(collection, "www.js96008.com", "Website", (0.0018, -0.0050, 0.00191), 0.00142, print_gray)
    add_text(collection, "Bluetooth", "BluetoothText", (0.0030, -0.0100, 0.00191), 0.00225, print_gray)
    add_text(collection, "USB", "USBText", (-0.0128, -0.0090, 0.00191), 0.00130, print_gray, math.pi * 0.5)

    # Compact Bluetooth rune beside the wordmark.
    bpy.ops.mesh.primitive_cylinder_add(vertices=40, radius=0.00155, depth=0.00007, location=(-0.0039, -0.0100, 0.001945))
    bluetooth_disc = bpy.context.object
    bluetooth_disc.name = "BluetoothMark"
    bluetooth_disc.data.materials.append(print_gray)
    move_to_collection(bluetooth_disc, collection)
    for angle in (-0.62, 0.62):
        segment = add_box(collection, f"BluetoothRune_{angle:+.2f}", (0.00035, 0.0020, 0.000055), (-0.0039, -0.0100, 0.001968), top, 0.00008, 2)
        segment.rotation_euler.z = angle

    # Fine printed guide strokes above the service text.
    add_box(collection, "PrintRuleLeft", (0.0040, 0.00020, 0.00005), (-0.0091, 0.0001, 0.001975), print_gray, 0.00008, 2)
    add_box(collection, "PrintRuleRight", (0.0040, 0.00020, 0.00005), (0.0122, 0.0001, 0.001975), print_gray, 0.00008, 2)

    objects = list(collection.objects)
    for obj in objects:
        obj["units"] = "metres"
        obj["source_reference"] = "material/1.jpg ... material/5.jpg"
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
    ground_material = make_material("Preview Ground", (0.025, 0.032, 0.043, 1.0), 0.05, 0.58)
    add_box(preview, "PreviewGround", (0.12, 0.11, 0.002), (0.0, 0.0, -0.0032), ground_material, 0.001)

    bpy.ops.object.camera_add(location=(0.070, -0.085, 0.068))
    camera = bpy.context.object
    camera.name = "PreviewCamera"
    camera.data.lens = 58
    point_at(camera, Vector((0.0, 0.0, 0.0)))
    move_to_collection(camera, preview)
    bpy.context.scene.camera = camera

    bpy.ops.object.light_add(type="AREA", location=(-0.025, -0.030, 0.090))
    key = bpy.context.object
    key.name = "KeyLight"
    key.data.energy = 3.5
    key.data.size = 0.075
    point_at(key, Vector((0.0, 0.0, 0.0)))
    move_to_collection(key, preview)

    bpy.ops.object.light_add(type="AREA", location=(0.060, 0.035, 0.045))
    fill = bpy.context.object
    fill.name = "FillLight"
    fill.data.energy = 1.8
    fill.data.size = 0.055
    point_at(fill, Vector((0.0, 0.0, 0.0)))
    move_to_collection(fill, preview)

    scene = bpy.context.scene
    scene.world.color = (0.012, 0.018, 0.026)
    scene.world.use_nodes = True
    scene.world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.012, 0.018, 0.026, 1.0)
    scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.32
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1000
    scene.render.resolution_y = 760
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(RENDER_PATH)
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.view_settings.exposure = -0.65
    bpy.ops.render.render(write_still=True)


def main() -> None:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    RENDER_PATH.parent.mkdir(parents=True, exist_ok=True)
    reset_scene()
    _, objects = build_product()
    minimum, maximum = mesh_bounds(objects)
    dimensions = maximum - minimum
    expected = Vector((WIDTH, LENGTH, THICKNESS))
    print("Product bounds (m):", tuple(round(value, 7) for value in dimensions))
    print("Product min/max (m):", tuple(round(value, 7) for value in minimum), tuple(round(value, 7) for value in maximum))
    if any(abs(dimensions[index] - expected[index]) > 0.000005 for index in range(3)):
        raise RuntimeError(f"Unexpected product bounds: {dimensions}, expected {expected}")
    if (minimum + maximum).length > 0.00001:
        raise RuntimeError(f"Product origin is not its geometric center: min={minimum}, max={maximum}")
    export_glb(objects)
    create_preview()
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_PATH))
    print(f"Saved {BLEND_PATH}")
    print(f"Saved {GLB_PATH}")
    print(f"Saved {RENDER_PATH}")


if __name__ == "__main__":
    main()

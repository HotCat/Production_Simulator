"""Build the thin product label used by the MG400 pick-and-place demo.

The product's recessed label contour is 27.8 x 10.2 mm in the existing
product model.  This label is intentionally inset by 0.5 mm on every side so
it seats inside that contour without hiding the molded border.  Blender uses
metres and places the label origin on its upper (nozzle-facing) surface; the
label extends along local -Z.  Godot applies the same frame correction as the
SMT nozzle when attaching it to the TCP.
"""

from __future__ import annotations

import math
from pathlib import Path

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parent.parent
ASSET_DIR = ROOT / "assets" / "product_label"
SOURCE_DIR = ROOT / "source_assets" / "product_label"
BLEND_PATH = SOURCE_DIR / "product_label.blend"
GLB_PATH = ASSET_DIR / "product_label.glb"
RENDER_PATH = ROOT / "material" / "generated" / "product_label_blender_render.png"

LABEL_WIDTH = 0.0268
LABEL_HEIGHT = 0.0092
LABEL_THICKNESS = 0.00016


def reset_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for collection in list(bpy.data.collections):
        bpy.data.collections.remove(collection)


def make_material(name: str, color: tuple[float, float, float, float], roughness: float, metallic: float = 0.0):
    material = bpy.data.materials.new(name)
    material.diffuse_color = color
    material.use_nodes = True
    principled = material.node_tree.nodes.get("Principled BSDF")
    principled.inputs["Base Color"].default_value = color
    principled.inputs["Roughness"].default_value = roughness
    principled.inputs["Metallic"].default_value = metallic
    return material


def clipped_rectangle(width: float, height: float, radius: float, clip: float) -> list[tuple[float, float]]:
    """Counter-clockwise rounded rectangle with the product's clipped corner."""
    half_w = width * 0.5
    half_h = height * 0.5
    points: list[tuple[float, float]] = []

    def arc(cx: float, cy: float, start: float, end: float) -> None:
        for index in range(6):
            angle = start + (end - start) * index / 5.0
            points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))

    # Start at the upper-right and trace counter-clockwise.  The lower-right
    # corner is clipped to match the recessed product panel.
    arc(half_w - radius, half_h - radius, 0.0, math.pi * 0.5)
    arc(-half_w + radius, half_h - radius, math.pi * 0.5, math.pi)
    arc(-half_w + radius, -half_h + radius, math.pi, math.pi * 1.5)
    points.extend(((half_w - clip, -half_h), (half_w, -half_h + clip)))
    return points


def extruded_polygon(collection: bpy.types.Collection, name: str, outline: list[tuple[float, float]], z_min: float, z_max: float, material, bevel_width: float = 0.0):
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
    if bevel_width:
        bevel = obj.modifiers.new("Soft paper edge", "BEVEL")
        bevel.width = bevel_width
        bevel.segments = 3
        bevel.limit_method = "ANGLE"
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.modifier_apply(modifier=bevel.name)
        obj.select_set(False)
    return obj


def build() -> None:
    reset_scene()
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    RENDER_PATH.parent.mkdir(parents=True, exist_ok=True)
    collection = bpy.data.collections.new("ProductLabel_26_8x9_2x0_16mm")
    bpy.context.scene.collection.children.link(collection)

    border_material = make_material("Label printed border", (0.15, 0.18, 0.20, 1.0), 0.46)
    paper_material = make_material("Warm white label paper", (0.94, 0.91, 0.82, 1.0), 0.68)

    # A slightly larger backing creates a readable printed contour.  The paper
    # face remains inside the 27.8 x 10.2 mm recessed product panel.
    backing = extruded_polygon(
        collection,
        "LabelContour_27x9_4mm",
        clipped_rectangle(0.0270, 0.0094, 0.00075, 0.0019),
        -LABEL_THICKNESS,
        0.00002,
        border_material,
        bevel_width=0.00010,
    )
    backing["dimensions_mm"] = "27.0 x 9.4 x 0.16"

    face = extruded_polygon(
        collection,
        "LabelFace_26_8x9_2mm",
        clipped_rectangle(LABEL_WIDTH, LABEL_HEIGHT, 0.00068, 0.0018),
        -LABEL_THICKNESS + 0.000025,
        0.000045,
        paper_material,
        bevel_width=0.00008,
    )
    face["dimensions_mm"] = "26.8 x 9.2 x 0.16"

    # Add two tiny registration bars to make the label legible in the
    # simulator while retaining the blank, industrial appearance of the
    # reference label.
    for index, x in enumerate((-0.0080, -0.0066, -0.0052, -0.0038, 0.0055, 0.0069, 0.0083)):
        bpy.ops.mesh.primitive_cube_add(location=(x, -0.0022, 0.00006))
        mark = bpy.context.object
        mark.name = f"LabelMark_{index:02d}"
        mark.dimensions = (0.00042, 0.0030 if index % 2 else 0.0025, 0.000025)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        mark.data.materials.append(border_material)
        for source in list(mark.users_collection):
            source.objects.unlink(mark)
        collection.objects.link(mark)

    root = bpy.data.objects.new("ProductLabel_ROOT", None)
    collection.objects.link(root)
    root["dimensions_mm"] = "27.0 x 9.4 x 0.16"
    root["product_panel_reference_mm"] = "27.8 x 10.2"
    root["interface_origin"] = "upper nozzle-facing surface; label extends local -Z"
    for obj in list(collection.objects):
        if obj != root and obj.parent is None:
            obj.parent = root

    # Preview in Blender's Z-up frame.
    bpy.context.scene.render.engine = "BLENDER_EEVEE_NEXT"
    bpy.context.scene.render.resolution_x = 700
    bpy.context.scene.render.resolution_y = 500
    bpy.context.scene.render.resolution_percentage = 100
    bpy.context.scene.world.color = (0.035, 0.045, 0.06)
    bpy.ops.object.camera_add(location=(0.035, -0.030, 0.045))
    camera = bpy.context.object
    camera.rotation_euler = (Vector((0.0, 0.0, -0.004)) - camera.location).to_track_quat("-Z", "Y").to_euler()
    camera.data.clip_start = 0.0005
    camera.data.clip_end = 2.0
    bpy.context.scene.camera = camera
    bpy.ops.object.light_add(type="AREA", location=(0.02, -0.02, 0.04))
    bpy.context.object.data.energy = 500
    bpy.context.object.data.size = 0.06
    bpy.context.scene.render.filepath = str(RENDER_PATH)
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_PATH))
    bpy.ops.render.render(write_still=True)

    bpy.ops.object.select_all(action="DESELECT")
    root.select_set(True)
    for child in root.children_recursive:
        child.select_set(True)
    bpy.context.view_layer.objects.active = root
    bpy.ops.export_scene.gltf(filepath=str(GLB_PATH), export_format="GLB", use_selection=True, export_apply=True)


if __name__ == "__main__":
    build()

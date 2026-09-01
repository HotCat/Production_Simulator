"""Build a compact SMT pick-and-place vacuum nozzle.

The model uses the common small-nozzle envelope used by desktop/compact SMT
heads: 6 mm shank, 8 mm barrel, 8 x 8 mm square mounting block, 12 mm silicone
cup, and 25 mm overall height.  Blender local origin is the mounting interface
and the source mesh extends along Blender local -Z.  The Godot attachment
applies the corresponding frame correction so the pickup axis points down in
world space.
"""

from __future__ import annotations

import math
from pathlib import Path

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parent.parent
ASSET_DIR = ROOT / "assets" / "smt_nozzle"
SOURCE_DIR = ROOT / "source_assets" / "smt_nozzle"
BLEND_PATH = SOURCE_DIR / "smt_nozzle.blend"
GLB_PATH = ASSET_DIR / "smt_nozzle.glb"
RENDER_PATH = ROOT / "material" / "generated" / "smt_nozzle_blender_render.png"


def reset_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for collection in list(bpy.data.collections):
        bpy.data.collections.remove(collection)


def material(name: str, color: tuple[float, float, float, float], metallic=0.0, roughness=0.45):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = color
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    return mat


def bevel(obj: bpy.types.Object, width: float, segments=3) -> None:
    mod = obj.modifiers.new("Soft machined edges", "BEVEL")
    mod.width = width
    mod.segments = segments
    mod.limit_method = "ANGLE"
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=mod.name)


def cyl(name: str, radius: float, depth: float, z: float, mat, vertices=48, bevel_width=0.0):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=depth, location=(0, 0, z - depth * 0.5))
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(mat)
    if bevel_width:
        bevel(obj, bevel_width)
    return obj


def box(name: str, size: tuple[float, float, float], z: float, mat, bevel_width=0.0):
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, z - size[2] * 0.5))
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(mat)
    if bevel_width:
        bevel(obj, bevel_width)
    return obj


def torus(name: str, major: float, minor: float, z: float, mat):
    bpy.ops.mesh.primitive_torus_add(major_radius=major, minor_radius=minor, major_segments=48, minor_segments=12, location=(0, 0, z))
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(mat)
    return obj


def build() -> None:
    reset_scene()
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    collection = bpy.data.collections.new("SMT_Nozzle_6mm_25mm")
    bpy.context.scene.collection.children.link(collection)

    black = material("Anodized black mounting block", (0.025, 0.032, 0.040, 1), metallic=0.55, roughness=0.32)
    dark = material("Dark steel barrel", (0.07, 0.085, 0.10, 1), metallic=0.72, roughness=0.27)
    steel = material("Spring and shaft steel", (0.47, 0.52, 0.56, 1), metallic=0.88, roughness=0.22)
    green = material("Silicone suction cup", (0.06, 0.72, 0.22, 1), metallic=0.0, roughness=0.38)

    # All dimensions are metres. The interface plane is z=0 and the pickup tip
    # points toward local -Z (Godot world down after parenting to the TCP).
    mount = box("MountingBlock_8x8x6mm", (0.008, 0.008, 0.006), 0.0, black, 0.00045)
    mount["dimensions_mm"] = "8 x 8 x 6"
    # Two shallow side ears reproduce the keyed clamp shape in the reference.
    box("MountEarLeft", (0.0015, 0.0085, 0.004), -0.003, black, 0.00018).location.x = -0.0047
    box("MountEarRight", (0.0015, 0.0085, 0.004), -0.003, black, 0.00018).location.x = 0.0047
    cyl("Barrel_8mm", 0.004, 0.008, -0.006, dark, bevel_width=0.00028)
    cyl("Collar_9mm", 0.0045, 0.0017, -0.010, black, bevel_width=0.00018)
    cyl("SpringShaft_6mm", 0.003, 0.006, -0.0135, steel, bevel_width=0.00015)
    # A visible helical spring, represented by a torus stack for robust GLB import.
    for i in range(5):
        torus("SpringCoil_%02d" % i, 0.00325, 0.00022, -0.0116 - i * 0.00115, steel)
    cyl("TipStem_3mm", 0.0018, 0.004, -0.018, steel, bevel_width=0.00012)
    cyl("SiliconeCup_12mm", 0.006, 0.0042, -0.0218, green, vertices=64, bevel_width=0.00038)
    cyl("CupNeck_7mm", 0.0035, 0.0024, -0.0188, green, vertices=64, bevel_width=0.00022)
    torus("CupLip", 0.00535, 0.00065, -0.0237, green)
    cyl("VacuumTip_4mm", 0.0020, 0.0012, -0.0255, dark, vertices=48, bevel_width=0.00012)

    # Parent all geometry under a single origin object for clean Godot transforms.
    root = bpy.data.objects.new("SMT_Nozzle_ROOT", None)
    collection.objects.link(root)
    root["overall_dimensions_mm"] = "12 diameter x 25.5 length"
    root["mounting_shank_mm"] = "6"
    root["interface_origin"] = "mounting plane; pickup axis local -Z"
    for obj in list(bpy.context.scene.objects):
        if obj != root and obj.type == "MESH" and obj.parent is None:
            obj.parent = root
    root.location = (0, 0, 0)

    bpy.context.scene.render.engine = "BLENDER_EEVEE_NEXT"
    bpy.context.scene.render.resolution_x = 700
    bpy.context.scene.render.resolution_y = 700
    bpy.context.scene.render.resolution_percentage = 100
    bpy.context.scene.render.film_transparent = False
    bpy.context.scene.world.color = (0.035, 0.045, 0.06)
    bpy.ops.object.camera_add(location=(0.045, -0.055, -0.028))
    camera = bpy.context.object
    camera.rotation_euler = (Vector((0.0, 0.0, -0.012)) - camera.location).to_track_quat("-Z", "Y").to_euler()
    camera.data.clip_start = 0.001
    camera.data.clip_end = 10.0
    bpy.context.scene.camera = camera
    bpy.ops.object.light_add(type="AREA", location=(0.03, -0.03, 0.03))
    bpy.context.object.data.energy = 550
    bpy.context.object.data.shape = "DISK"
    bpy.context.object.data.size = 0.08
    bpy.ops.object.light_add(type="AREA", location=(-0.03, 0.02, -0.02))
    bpy.context.object.data.energy = 300
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

"""Build the dimensionally accurate tabletop conveyor in Blender.

Run with:
    blender --background --python tools/build_conveyor.py

Outputs:
    assets/conveyor/conveyor.blend
    assets/conveyor/conveyor.glb
    material/generated/conveyor_blender_render.png
"""

from __future__ import annotations

import math
from pathlib import Path

import bpy
from mathutils import Vector


ROOT = Path(__file__).resolve().parent.parent
ASSET_DIR = ROOT / "assets" / "conveyor"
SOURCE_DIR = ROOT / "source_assets" / "conveyor"
BLEND_PATH = SOURCE_DIR / "conveyor.blend"
GLB_PATH = ASSET_DIR / "conveyor.glb"
RENDER_PATH = ROOT / "material" / "generated" / "conveyor_blender_render.png"

LENGTH = 0.800
WIDTH = 0.200
HEIGHT = 0.070


def reset_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for collection in list(bpy.data.collections):
        if collection.name != "Collection":
            bpy.data.collections.remove(collection)
    bpy.context.scene.collection.children.unlink(bpy.data.collections["Collection"])
    bpy.data.collections.remove(bpy.data.collections["Collection"])


def make_material(
    name: str,
    color: tuple[float, float, float, float],
    metallic: float,
    roughness: float,
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
    for source_collection in list(obj.users_collection):
        source_collection.objects.unlink(obj)
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
        modifier = obj.modifiers.new("Edge softening", "BEVEL")
        modifier.width = bevel
        modifier.segments = bevel_segments
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
    axis: str = "Y",
    vertices: int = 64,
) -> bpy.types.Object:
    rotation = (math.pi * 0.5, 0.0, 0.0) if axis == "Y" else (0.0, math.pi * 0.5, 0.0)
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=depth,
        end_fill_type="NGON",
        location=location,
        rotation=rotation,
    )
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(material)
    move_to_collection(obj, collection)
    for polygon in obj.data.polygons:
        polygon.use_smooth = polygon.normal.z < 0.95
    return obj


def add_rail_details(
    collection: bpy.types.Collection,
    aluminum: bpy.types.Material,
    groove: bpy.types.Material,
) -> None:
    for side in (-1.0, 1.0):
        add_box(
            collection,
            f"SideRail_{'L' if side < 0 else 'R'}",
            (0.680, 0.024, 0.052),
            (0.0, side * 0.088, 0.035),
            aluminum,
            bevel=0.002,
        )
        # Dark recessed lines suggest the longitudinal T-slots in 3060 profile.
        outer_y = side * 0.0994
        for groove_index, groove_z in enumerate((0.017, 0.035, 0.053), start=1):
            add_box(
                collection,
                f"RailGroove_{'L' if side < 0 else 'R'}_{groove_index}",
                (0.640, 0.0012, 0.0032),
                (0.0, outer_y, groove_z),
                groove,
                bevel=0.0005,
            )


def add_end_hardware(
    collection: bpy.types.Collection,
    end_cap: bpy.types.Material,
    fastener: bpy.types.Material,
    steel: bpy.types.Material,
) -> None:
    roller_x = 0.369
    for x_side in (-1.0, 1.0):
        x = x_side * 0.370
        for y_side in (-1.0, 1.0):
            y = y_side * 0.088
            cap = add_box(
                collection,
                f"EndCap_{'Infeed' if x_side < 0 else 'Outfeed'}_{'L' if y_side < 0 else 'R'}",
                (0.060, 0.024, HEIGHT),
                (x, y, HEIGHT * 0.5),
                end_cap,
                bevel=0.009,
                bevel_segments=5,
            )
            cap["mechanical_part"] = "end_cap"
            outer_y = y_side * 0.0990
            for screw_x in (-0.013, 0.013):
                for screw_z in (0.020, 0.050):
                    add_cylinder(
                        collection,
                        f"Fastener_{x_side:+.0f}_{y_side:+.0f}_{screw_x:+.3f}_{screw_z:.3f}",
                        radius=0.0032,
                        depth=0.0020,
                        location=(x + screw_x, outer_y, screw_z),
                        material=fastener,
                        axis="Y",
                        vertices=24,
                    )

        # Roller axle and dark bearing rings remain visible on both side faces.
        add_cylinder(
            collection,
            f"Axle_{'Infeed' if x_side < 0 else 'Outfeed'}",
            radius=0.0055,
            depth=0.199,
            location=(x_side * roller_x, 0.0, 0.039),
            material=steel,
            axis="Y",
            vertices=32,
        )
        for y_side in (-1.0, 1.0):
            add_cylinder(
                collection,
                f"Bearing_{x_side:+.0f}_{y_side:+.0f}",
                radius=0.0075,
                depth=0.0022,
                location=(x_side * roller_x, y_side * 0.0989, 0.039),
                material=fastener,
                axis="Y",
                vertices=32,
            )


def add_belt_and_drive(
    collection: bpy.types.Collection,
    belt: bpy.types.Material,
    dark: bpy.types.Material,
    steel: bpy.types.Material,
) -> None:
    roller_x = 0.369
    roller_z = 0.039
    roller_radius = 0.031

    # Steel cores are visible at the narrow belt edges.
    for x_side in (-1.0, 1.0):
        add_cylinder(
            collection,
            f"RollerCore_{'Infeed' if x_side < 0 else 'Outfeed'}",
            radius=0.027,
            depth=0.184,
            location=(x_side * roller_x, 0.0, roller_z),
            material=steel,
            axis="Y",
        )
        add_cylinder(
            collection,
            f"BeltTurn_{'Infeed' if x_side < 0 else 'Outfeed'}",
            radius=roller_radius,
            depth=0.170,
            location=(x_side * roller_x, 0.0, roller_z),
            material=belt,
            axis="Y",
            vertices=96,
        )

    add_box(
        collection,
        "BeltTop",
        (roller_x * 2.0, 0.170, 0.006),
        (0.0, 0.0, HEIGHT - 0.003),
        belt,
        bevel=0.0015,
        bevel_segments=3,
    )
    add_box(
        collection,
        "BeltReturn",
        (roller_x * 2.0, 0.170, 0.004),
        (0.0, 0.0, 0.010),
        belt,
        bevel=0.001,
    )
    add_box(
        collection,
        "InteriorDeck",
        (0.680, 0.166, 0.010),
        (0.0, 0.0, 0.049),
        dark,
        bevel=0.002,
    )
    # Compact internal drive motor, visible only through the lower frame gap.
    add_cylinder(
        collection,
        "IntegratedDriveMotor",
        radius=0.017,
        depth=0.115,
        location=(0.280, 0.0, 0.028),
        material=dark,
        axis="Y",
        vertices=48,
    )
    for x in (-0.220, 0.0, 0.220):
        add_box(
            collection,
            f"LowerCrossBrace_{x:+.3f}",
            (0.018, 0.170, 0.010),
            (x, 0.0, 0.012),
            steel,
            bevel=0.0015,
        )


def create_model() -> tuple[bpy.types.Collection, list[bpy.types.Object]]:
    model_collection = bpy.data.collections.new("Conveyor_800x200x70mm")
    bpy.context.scene.collection.children.link(model_collection)

    aluminum = make_material("Anodized Aluminum", (0.62, 0.68, 0.72, 1.0), 0.78, 0.22)
    end_cap = make_material("Machined End Caps", (0.78, 0.82, 0.84, 1.0), 0.88, 0.17)
    steel = make_material("Roller Steel", (0.36, 0.40, 0.43, 1.0), 0.92, 0.16)
    belt = make_material("Green Conveyor Belt", (0.025, 0.24, 0.105, 1.0), 0.02, 0.52)
    dark = make_material("Dark Interior", (0.018, 0.024, 0.028, 1.0), 0.35, 0.40)
    fastener = make_material("Black Fasteners", (0.006, 0.008, 0.010, 1.0), 0.72, 0.24)

    add_rail_details(model_collection, aluminum, dark)
    add_belt_and_drive(model_collection, belt, dark, steel)
    add_end_hardware(model_collection, end_cap, fastener, steel)

    objects = list(model_collection.objects)
    for obj in objects:
        obj["units"] = "metres"
        obj["conveyor_dimensions_mm"] = "800 x 200 x 70"
    return model_collection, objects


def model_bounds(objects: list[bpy.types.Object]) -> tuple[Vector, Vector]:
    corners = [obj.matrix_world @ Vector(corner) for obj in objects for corner in obj.bound_box]
    minimum = Vector(min(c[i] for c in corners) for i in range(3))
    maximum = Vector(max(c[i] for c in corners) for i in range(3))
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


def point_camera(camera: bpy.types.Object, target: Vector) -> None:
    direction = target - camera.location
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()


def create_render_setup(model_objects: list[bpy.types.Object]) -> None:
    render_collection = bpy.data.collections.new("Render_Setup_Not_Exported")
    bpy.context.scene.collection.children.link(render_collection)
    ground_material = make_material("Preview Ground", (0.035, 0.045, 0.060, 1.0), 0.05, 0.62)

    ground = add_box(
        render_collection,
        "PreviewGround",
        (1.50, 1.00, 0.006),
        (0.0, 0.0, -0.006),
        ground_material,
        bevel=0.002,
    )
    ground.hide_render = False

    bpy.ops.object.camera_add(location=(1.20, -1.30, 0.75))
    camera = bpy.context.object
    camera.name = "PreviewCamera"
    camera.data.lens = 58
    point_camera(camera, Vector((0.0, 0.0, 0.030)))
    move_to_collection(camera, render_collection)
    bpy.context.scene.camera = camera

    bpy.ops.object.light_add(type="AREA", location=(0.1, -0.25, 0.95))
    key = bpy.context.object
    key.name = "KeyLight"
    key.data.energy = 115
    key.data.shape = "RECTANGLE"
    key.data.size = 0.75
    key.data.size_y = 0.55
    point_camera(key, Vector((0.0, 0.0, 0.02)))
    move_to_collection(key, render_collection)

    bpy.ops.object.light_add(type="AREA", location=(-0.55, 0.55, 0.36))
    fill = bpy.context.object
    fill.name = "FillLight"
    fill.data.energy = 65
    fill.data.size = 0.55
    point_camera(fill, Vector((0.0, 0.0, 0.03)))
    move_to_collection(fill, render_collection)

    world = bpy.context.scene.world
    world.color = (0.018, 0.024, 0.035)
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.018, 0.024, 0.035, 1.0)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.28

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(RENDER_PATH)
    scene.render.film_transparent = False
    scene.render.image_settings.color_mode = "RGBA"
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.view_settings.exposure = -0.7
    bpy.ops.render.render(write_still=True)


def main() -> None:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    RENDER_PATH.parent.mkdir(parents=True, exist_ok=True)
    reset_scene()
    model_collection, model_objects = create_model()
    minimum, maximum = model_bounds(model_objects)
    dimensions = maximum - minimum
    print(
        "Conveyor bounds (m):",
        tuple(round(value, 6) for value in dimensions),
        "min=",
        tuple(round(value, 6) for value in minimum),
        "max=",
        tuple(round(value, 6) for value in maximum),
    )
    tolerance = 0.0002
    expected = Vector((LENGTH, WIDTH, HEIGHT))
    if any(abs(dimensions[i] - expected[i]) > tolerance for i in range(3)):
        raise RuntimeError(f"Unexpected conveyor bounds: {dimensions}, expected {expected}")

    export_glb(model_objects)
    create_render_setup(model_objects)
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_PATH))
    print(f"Saved {BLEND_PATH}")
    print(f"Saved {GLB_PATH}")
    print(f"Saved {RENDER_PATH}")


if __name__ == "__main__":
    main()

# MG400 Cartesian IK demo (Godot 4)

This Godot 4 project animates the separate MG400 URDF link meshes and drives
the tool centre point (TCP) from a Cartesian world-space target. It is the
first-stage robot sandbox for the later label-stripper/conveyor demonstration.

The scene also contains an 800 × 200 × 70 mm tabletop conveyor positioned
lengthwise beneath the MG400 work envelope. Its editable Blender source is
`source_assets/conveyor/conveyor.blend`; Godot loads the exported
`assets/conveyor/conveyor.glb`.

A thin molded product based on `material/1.jpg` through `material/5.jpg` starts
at the conveyor center. Its verified imported dimensions are exactly
37.95 × 46.00 × 4.00 mm, and its bottom face rests on the belt surface.

A dimensioned label stripper/emitter is anchored by its emitted-label center
at MG400 coordinates `X 53.0, Y 231.4, Z 150.4 mm`. In Godot Y-up world space
that is `(0.0530, 0.1504, -0.2314) m`. The model faces the emitted label toward
the robot, and its 70.4 mm pad reaches exactly to the floor.

The supplied `material/Logo_Alt_2@2x.png` is mounted along the narrow groove on
the long diagonal blue `link4_2` upper-arm member identified in
`material/logo_position.png`, so it follows that linkage during IK motion.

## Run

Open this directory in Godot 4 and run `main.tscn`, or use:

```sh
/Applications/Godot.app/Contents/MacOS/Godot --path .
```

The default mode follows an automatic Cartesian figure-eight. Press Space for
manual jog mode, then use the arrow keys for the floor plane, R/F for height,
and Q/E for tool yaw. Press `0` or Mac Delete to return the target to a known
reachable point.

### Camera controls

- Two-finger trackpad motion: orbit yaw and pitch
- Control + two-finger motion: change camera distance
- Shift + two-finger motion: pan horizontally and vertically
- `C`: reset the camera view
- `Show info` checkbox: show or hide the status and controls overlay
- Middle-mouse drag offers the same orbit behavior, with Control and Shift as
  zoom and pan modifiers. The mouse wheel also zooms.

The information overlay continuously displays the camera world position plus
its orbit yaw, pitch, distance, and focus point. These values completely
describe the current view and can be used to reproduce a placement screenshot.

## Cartesian API

The reusable robot component is `scripts/mg400_robot.gd` (`MG400Robot`). Its
main command accepts standard Godot world coordinates (Y-up):

```gdscript
var reachable := robot.set_tcp_target_world(
    Vector3(world_x, world_y, world_z),
    tool_yaw_radians
)
```

Use the optional third argument `true` to apply the pose immediately. The
normal path is smoothed at `joint_speed`. An unreachable target is projected
to the physical workspace and the method returns `false`.

`urdf_position_to_world(Vector3(x, y, z))` converts ROS/MG400 Z-up coordinates
to Godot world coordinates. All dimensions are metres.

## Model and IK notes

- The animated model uses the nine separate meshes in
  `MG400_ROS/mg400_description/meshes`. The single root-level SolidWorks STL is
  a fused assembly and cannot articulate.
- `tools/convert_stl_to_obj.py` is a dependency-free converter used because
  Godot does not import binary STL as a runtime mesh. Rebuild assets with:

  ```sh
  python3 tools/convert_stl_to_obj.py \
    MG400_ROS/mg400_description/meshes assets/mg400
  ```

- The solver is closed-form for the MG400 parallelogram. It uses the URDF joint
  origins and limits, resolves both elbow branches, chooses a legal continuous
  branch, and applies the URDF mimic joints to both sides of the mechanism.
- Verify the solver headlessly with:

  ```sh
  /Applications/Godot.app/Contents/MacOS/Godot \
    --headless --path . --script tests/test_ik.gd
  ```

## Rebuild the conveyor

The conveyor is generated as real Blender geometry at metre scale. Rebuild the
`.blend`, `.glb`, and Blender preview with:

```sh
/Applications/Blender\ 4.5.app/Contents/MacOS/Blender \
  --background --python tools/build_conveyor.py
```

The original photo, the HeyRoute-generated modeling reference, and the Blender
preview are stored under `material/`.

## Rebuild the label stripper

The emitted-label center is Blender local origin `(0,0,0)`. Rebuild its Blender
source, GLB, and preview with:

```sh
/Applications/Blender\ 4.5.app/Contents/MacOS/Blender \
  --background --python tools/build_label_stripper.py
```

Files:

- Editable source: `source_assets/label_stripper/label_stripper.blend`
- Godot model: `assets/label_stripper/label_stripper.glb`
- Generated modeling reference: `material/generated/label_stripper_reference.png`
- Blender preview: `material/generated/label_stripper_blender_render.png`

Verify its origin and floor contact with:

```sh
/Applications/Godot.app/Contents/MacOS/Godot \
  --headless --path . --script tests/test_label_stripper.gd
```

## Rebuild the product

The product uses its geometric center as the Blender/Godot origin. Rebuild its
editable Blender source, GLB, and preview with:

```sh
/Applications/Blender\ 4.5.app/Contents/MacOS/Blender \
  --background --python tools/build_product.py
```

Files:

- Editable source: `source_assets/product/product.blend`
- Godot model: `assets/product/product.glb`
- Generated modeling sheet: `material/generated/product_reference.png`
- Blender preview: `material/generated/product_blender_render.png`

Verify its exact size, belt contact, and centered placement with:

```sh
/Applications/Godot.app/Contents/MacOS/Godot \
  --headless --path . --script tests/test_product.gd
```

## HIKROBOT MV-CS200-10GC industrial camera

The reusable camera asset models the official second-generation HIKROBOT CS
housing with a compact 20 MP 1-inch C/CS-mount lens. The requested “GS200” is
the `MV-CS200-10GC`: Sony IMX183, 5472 × 3648, 5.9 fps, GigE. Its verified body
dimensions are 29 × 29 × 42 mm; the attached lens has a 33 mm maximum diameter
and 42 mm optical length.

The nominal installation reference is the requested MG400 coordinate
`X 262.9, Y 117.0, Z 291.3 mm`. The camera model in `main.tscn` uses the later
visually adjusted installation pose at Godot position
`(0.2629, 0.302346, -0.009776) m`, angled toward the conveyor work area. The
placement regression test records the complete basis so accidental scene
transform changes are detected.

Official source: [HIKROBOT CS Series Area Scan Camera](https://www.hikrobotics.com/en/machinevision/visionproduct?typeId=78&id=134)

Files:

- Godot model: `assets/hikrobot_gs200/hikrobot_gs200.glb`
- Editable source: `source_assets/hikrobot_gs200/hikrobot_gs200.blend`
- Godot preview scene: `scenes/hikrobot_gs200_preview.tscn`
- Official image: `material/generated/hikrobot_gs200_official.png`
- Generated modeling sheet: `material/generated/hikrobot_gs200_reference.png`
- Blender preview: `material/generated/hikrobot_gs200_blender_render.png`

Rebuild and verify:

```sh
/Applications/Blender\ 4.5.app/Contents/MacOS/Blender \
  --background --python tools/build_hikrobot_gs200.py

/Applications/Godot.app/Contents/MacOS/Godot \
  --headless --path . --script tests/test_hikrobot_gs200.gd
```

Run the standalone interactive camera preview:

```sh
/Applications/Godot.app/Contents/MacOS/Godot \
  --path . scenes/hikrobot_gs200_preview.tscn
```

## GS200 adjustable camera bracket

The camera is supported by a floor-mounted aluminum-extrusion stand designed
around its restored scene pose. A 100 × 120 × 12 mm base is centered at Godot
world position `(0.4400, 0, -0.009776) m`, immediately outside the conveyor.
Its nearest edge retains 18.37 mm of clearance from the conveyor frame.

The stand uses a 30 × 30 mm vertical post, 25 × 25 mm horizontal boom, sliding
carriage, lock knob, articulated drop link, and ball joint. The final
28 × 32 × 4 mm plate follows the camera's tilted basis and meets its top-face
M3 mounting pattern without changing the camera position or orientation.

Files:

- Godot model: `assets/hikrobot_gs200_bracket/hikrobot_gs200_bracket.glb`
- Editable source: `source_assets/hikrobot_gs200_bracket/hikrobot_gs200_bracket.blend`
- Blender preview: `material/generated/hikrobot_gs200_bracket_blender_render.png`

Rebuild and verify:

```sh
/Applications/Blender\ 4.5.app/Contents/MacOS/Blender \
  --background --python tools/build_hikrobot_gs200_bracket.py

/Applications/Godot.app/Contents/MacOS/Godot \
  --headless --path . --script tests/test_hikrobot_gs200_bracket.gd
```

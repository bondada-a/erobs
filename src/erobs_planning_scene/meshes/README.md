# Using 3D Mesh Files in Planning Scene

## Supported Formats
- ✅ **STL** (`.stl`) - Most common, recommended
- ✅ **DAE** (`.dae`) - Collada format
- ✅ **OBJ** (`.obj`) - Wavefront format
- ❌ **STEP** (`.step`) - Not directly supported, convert to STL first

## Converting STEP to STL

### Option 1: FreeCAD (Recommended)
```bash
# Install FreeCAD
sudo apt install freecad

# Convert via GUI:
# 1. Open STEP file in FreeCAD
# 2. File -> Export -> Select STL format
# 3. Save to this meshes/ directory
```

### Option 2: Online converters
- https://products.aspose.app/3d/conversion/step-to-stl
- Upload STEP, download STL

### Option 3: Command line (if you have OpenCASCADE)
```bash
python convert_step_to_stl.py your_file.step
```

## Directory Structure

Place your mesh files here:
```
erobs_planning_scene/
└── meshes/
    ├── robot_base.stl
    ├── table.stl
    └── custom_fixture.obj
```

## Using Meshes in YAML

Edit `config/beamline_scene.yaml`:

```yaml
obstacles:
  - name: "custom_part"
    type: "mesh"
    frame: "map"
    mesh: "package://erobs_planning_scene/meshes/custom_part.stl"
    scale: [1.0, 1.0, 1.0]  # Optional: scale x, y, z
    pose:
      x: 0.5
      y: 0.3
      z: 0.2
      roll: 0.0
      pitch: 0.0
      yaw: 1.5708  # 90 degrees
```

### Scale Parameter

If your mesh is in wrong units:
- **STL in mm** → Use `scale: [0.001, 0.001, 0.001]` (convert to meters)
- **STL in inches** → Use `scale: [0.0254, 0.0254, 0.0254]` (convert to meters)
- **STL already in meters** → Use `scale: [1.0, 1.0, 1.0]` or omit

## Installation

The mesh loader requires `trimesh`:

```bash
pip3 install trimesh
```

## Tips

- **Keep meshes simple**: Fewer triangles = faster collision checking
- **Use collision meshes**: Simplify CAD models for collision (decimation)
- **Check orientation**: May need to rotate in CAD before exporting
- **File size**: Keep STL files under 1MB for good performance

## Example Full Config

```yaml
obstacles:
  # Primitive box
  - name: "robot_base"
    type: "box"
    frame: "map"
    pose: {x: 0.0, y: 0.0, z: -0.127, roll: 0.0, pitch: 0.0, yaw: 0.0}
    size: [0.203, 0.203, 0.254]

  # Custom mesh
  - name: "complex_fixture"
    type: "mesh"
    frame: "map"
    mesh: "package://erobs_planning_scene/meshes/fixture.stl"
    scale: [0.001, 0.001, 0.001]  # Convert mm to meters
    pose: {x: 0.6, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}
```

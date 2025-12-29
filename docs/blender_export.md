# Exporting to Blender

This guide explains how to export trajectory data from Patrol Solver and visualize it in Blender using the Blender MCP (Model Context Protocol).

> **⚠️ Known Issue**: The solver currently generates a maze but does not produce valid paths. The trajectory data may be incomplete or incorrect. This is a work in progress.

## Overview

Patrol Solver generates trajectory JSON files that contain complete movement data for entities navigating through waypoints. This data can be imported into Blender for 3D visualization, animation, and analysis.

## Generating Trajectory Data

### Using the Mix Task

The easiest way to generate trajectory data is using the `patrol_solve` mix task:

```bash
# From the umbrella root
mix patrol_solve --waypoints 5 --maze --grid-width 10 --grid-height 10 --output /tmp/blender_trajectory.json
```

### Using the Solver API

```elixir
alias AriaPatrolSolver.Solver

{:ok, result} = Solver.solve(
  num_waypoints: 5,
  maze_mode: true,
  grid_width: 10,
  grid_height: 10,
  entity_speed: 2.0,
  output_path: "/tmp/blender_trajectory.json"
)

# Access the trajectory JSON
trajectory_json = result.trajectory_json
```

### Using Test Scripts

Several test scripts are available that generate trajectory data:

```bash
# Test with maze navigation
mix run apps/aria_patrol_solver/scripts/test_plan_reset_solve.exs

# Test with minimal maze
mix run apps/aria_patrol_solver/scripts/test_planner_maze_debug.exs

# Test grid-based patrol
mix run apps/aria_patrol_solver/scripts/test_grid_blender.exs
```

## Trajectory JSON Format

The trajectory JSON file has the following structure:

```json
{
  "metadata": {
    "total_steps": 24,
    "total_time": 0.0
  },
  "steps": [
    {
      "step": 0,
      "time": 0.0,
      "entities": [
        {
          "id": "entity1",
          "position": [0.0, 0.0, 0.0],
          "position_index": 0,
          "rotation": [0.602, 0.143, 0.785],
          "rotation_index": 3,
          "speed": 2.0,
          "movement_type": "walking"
        }
      ],
      "waypoints": [
        {
          "id": "wp1",
          "position": [0.0, 0.0, 0.0],
          "position_index": 0,
          "rotation": [0.0, 1.0, 0.0],
          "rotation_index": 0,
          "reached_by": []
        }
      ]
    }
  ]
}
```

### Data Fields

#### Metadata
- `total_steps`: Total number of trajectory steps
- `total_time`: Total simulation time in seconds

#### Step Data
Each step contains:
- `step`: Step number (0-indexed)
- `time`: Simulation time at this step
- `entities`: Array of entity states
- `waypoints`: Array of waypoint states

#### Entity State
- `id`: Unique entity identifier (e.g., "entity1")
- `position`: 3D position `[x, y, z]` in world coordinates
- `position_index`: Quantized position index (for grid/sphere quantization)
- `rotation`: 3D rotation vector `[rx, ry, rz]`
- `rotation_index`: Quantized rotation index
- `speed`: Movement speed in units/second
- `movement_type`: Type of movement (e.g., "walking")

#### Waypoint State
- `id`: Unique waypoint identifier (e.g., "wp1")
- `position`: 3D position `[x, y, z]` in world coordinates
- `position_index`: Quantized position index
- `rotation`: 3D rotation vector `[rx, ry, rz]`
- `rotation_index`: Quantized rotation index
- `reached_by`: Array of entity IDs that have reached this waypoint

## Visualizing in Blender

### Using Blender MCP

The recommended way to visualize trajectory data is using the Blender MCP server. Here's a Python script that can be executed via MCP:

```python
import bpy
import json
from mathutils import Vector

# Clear existing scene
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Load trajectory JSON
with open("/tmp/blender_trajectory.json", 'r') as f:
    data = json.load(f)

# Create materials
entity_mat = bpy.data.materials.new(name="EntityMaterial")
entity_mat.use_nodes = True
entity_mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0, 1, 0, 1)  # Green

waypoint_mat = bpy.data.materials.new(name="WaypointMaterial")
waypoint_mat.use_nodes = True
waypoint_mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (1, 0, 0, 1)  # Red

# Extract paths and waypoints
entity_paths = {}
waypoints = {}

for step in data['steps']:
    for entity in step.get('entities', []):
        if entity['id'] not in entity_paths:
            entity_paths[entity['id']] = []
        entity_paths[entity['id']].append(tuple(entity['position']))
    
    for waypoint in step.get('waypoints', []):
        waypoints[waypoint['id']] = tuple(waypoint['position'])

# Create waypoints (red cones)
for wp_id, pos in waypoints.items():
    bpy.ops.mesh.primitive_cone_add(radius1=0.3, depth=0.6, location=pos)
    bpy.context.active_object.data.materials.append(waypoint_mat)
    bpy.context.active_object.name = f"Waypoint_{wp_id}"

# Create entities with animation
fps = 30
for entity_id, path_points in entity_paths.items():
    if path_points:
        # Create entity sphere
        bpy.ops.mesh.primitive_uv_sphere_add(radius=0.3, location=path_points[0])
        entity_obj = bpy.context.active_object
        entity_obj.name = f"Entity_{entity_id}"
        entity_obj.data.materials.append(entity_mat)
        
        # Animate movement
        if len(path_points) > 1:
            for i, pos in enumerate(path_points):
                frame = i * fps
                bpy.context.scene.frame_set(frame)
                entity_obj.location = pos
                entity_obj.keyframe_insert(data_path="location", frame=frame)
            
            # Set interpolation to linear
            if entity_obj.animation_data and entity_obj.animation_data.action:
                for fcurve in entity_obj.animation_data.action.fcurves:
                    for kf in fcurve.keyframe_points:
                        kf.interpolation = 'LINEAR'
            
            bpy.context.scene.frame_end = len(path_points) * fps

# Create path curves
for entity_id, path_points in entity_paths.items():
    if len(path_points) < 2:
        continue
    
    curve = bpy.data.curves.new(name=f"Path_{entity_id}", type='CURVE')
    curve.dimensions = '3D'
    spline = curve.splines.new('POLY')
    spline.points.add(len(path_points) - 1)
    
    for i, point in enumerate(path_points):
        spline.points[i].co = (*point, 1)
    
    path_obj = bpy.data.objects.new(f"Path_{entity_id}", curve)
    bpy.context.collection.objects.link(path_obj)
    path_obj.data.materials.append(bpy.data.materials.new(name="PathMaterial"))
    curve.bevel_depth = 0.05

# Setup camera
all_points = [p for path in entity_paths.values() for p in path]
all_points.extend(list(waypoints.values()))

if all_points:
    # Calculate bounding box center
    min_x = min(p[0] for p in all_points)
    max_x = max(p[0] for p in all_points)
    min_y = min(p[1] for p in all_points)
    max_y = max(p[1] for p in all_points)
    min_z = min(p[2] for p in all_points)
    max_z = max(p[2] for p in all_points)
    
    center = ((min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2)
    size = max(max_x - min_x, max_y - min_y, max_z - min_z)
    distance = max(size * 1.5, 15)
    
    # Position camera
    cam = bpy.data.objects.get("Camera")
    if cam:
        cam.location = (center[0] + distance * 0.6, center[1] - distance * 0.6, center[2] + distance * 0.8)
        direction = Vector(center) - Vector(cam.location)
        cam.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()

# Add lighting
bpy.ops.object.light_add(type='SUN', location=(5, -5, 10))
bpy.context.active_object.data.energy = 3.0

print(f"Visualization complete: {len(waypoints)} waypoints, {len(entity_paths)} entities")
```

### Manual Import

If you prefer to work directly in Blender:

1. **Open Blender** and create a new scene
2. **Load the JSON file** using a Python script in Blender's Text Editor
3. **Run the script** to create objects, materials, and animations
4. **Adjust camera and lighting** as needed

## Advanced Visualization

### Adding Obstacles/Walls

For maze mode, you can add wall visualization:

```python
# Example: Add walls for a 10x10 grid maze
walls = [
    (1, 1), (1, 3), (1, 5),  # Wall positions
    # ... more wall positions
]

wall_mat = bpy.data.materials.new(name="WallMaterial")
wall_mat.use_nodes = True
wall_mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0.3, 0.3, 0.3, 1)

for x, y in walls:
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(x, y, 0.5))
    bpy.context.active_object.data.materials.append(wall_mat)
```

### Custom Materials

You can customize materials for different entity types:

```python
# Create custom material with emission
mat = bpy.data.materials.new(name="CustomEntityMaterial")
mat.use_nodes = True
bsdf = mat.node_tree.nodes["Principled BSDF"]
bsdf.inputs[0].default_value = (0, 0.8, 1, 1)  # Cyan
bsdf.inputs["Emission Strength"].default_value = 0.5
```

### Animation Timing

Adjust animation timing based on trajectory step duration:

```python
# Calculate frame timing from trajectory data
fps = 30
frames_per_step = 1  # Adjust based on your needs

for i, pos in enumerate(path_points):
    frame = i * frames_per_step
    bpy.context.scene.frame_set(frame)
    entity_obj.location = pos
    entity_obj.keyframe_insert(data_path="location", frame=frame)
```

## Troubleshooting

### Common Issues

1. **Empty trajectory**: Ensure the solver completed successfully and generated a solution
2. **Missing waypoints**: Check that waypoints are included in the trajectory JSON
3. **Animation not playing**: Verify keyframes are inserted and timeline is set correctly
4. **Objects at origin**: Verify position data is correctly extracted from JSON

### Debugging

Enable verbose logging when generating trajectory:

```elixir
require Logger
Logger.configure(level: :debug)

{:ok, result} = Solver.solve(...)
```

Check the trajectory JSON structure:

```bash
# Pretty print JSON
cat /tmp/blender_trajectory.json | python3 -m json.tool | head -50

# Count steps
cat /tmp/blender_trajectory.json | python3 -c "import json, sys; print(len(json.load(sys.stdin)['steps']))"
```

## Examples

### Minimal Example

```elixir
# Generate trajectory
{:ok, result} = Solver.solve(
  num_waypoints: 3,
  maze_mode: true,
  grid_width: 5,
  grid_height: 5,
  output_path: "/tmp/simple_trajectory.json"
)

# Load in Blender using the script above
```

### Complex Maze Example

```elixir
# Generate complex maze with obstacles
{:ok, result} = Solver.solve(
  num_waypoints: 10,
  maze_mode: true,
  grid_width: 20,
  grid_height: 20,
  obstacles: generate_labyrinth_obstacles(20, 20),
  entity_speed: 3.0,
  output_path: "/tmp/maze_trajectory.json"
)
```

## Integration with Blender MCP

When using Blender MCP, you can execute the visualization script directly:

```python
# Via MCP, the script can be executed in one call
# The trajectory JSON path should be passed as a parameter
trajectory_path = "/tmp/blender_trajectory.json"
# ... (use the script above)
```

## See Also

- [README.md](../README.md) - Main Patrol Solver documentation
- [SolutionTracker](../lib/aria_patrol_solver/domains/locomotion/visualization/solution_tracker.ex) - Trajectory tracking implementation
- [Blender MCP Documentation](https://github.com/your-repo/blender-mcp) - Blender MCP server documentation


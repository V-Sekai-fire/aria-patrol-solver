# Patrol Solver

A standalone application for solving patrol waypoint problems with shortest-path optimization using Aria Planner.

> **⚠️ Known Issue**: The solver currently generates a maze but does not produce valid paths. The trajectory data may be incomplete or incorrect. This is a work in progress.

## Overview

Patrol Solver generates scattered waypoints on a Fibonacci sphere, optimizes their visitation order using a nearest-neighbor heuristic to minimize total travel distance, and generates trajectory data for visualization in Blender.

## Features

- **Shortest Path Optimization**: Uses nearest-neighbor algorithm to find optimal waypoint visitation order
- **2D Maze Mode**: Default mode for mouse maze navigation with no upward movement
- **Navmesh Support**: Automatic navmesh generation from grid or import from physics engines (Godot/Unity)
- **Pathfinding**: A* pathfinding on navmesh ensures entities only move along walkable surfaces
- **3D Mode**: Optional 3D waypoint generation using Fibonacci sphere quantization
- **Trajectory Export**: Generates JSON trajectory data compatible with Blender visualization
- **Configurable Parameters**: Customize waypoint count, grid/sphere resolution, obstacles, and entity speed

## Usage

### As a Mix Task

From the umbrella root:

```bash
mix patrol_solve
mix patrol_solve --waypoints 10
mix patrol_solve -w 10 -s 200 -v 3.0 -o /tmp/patrol.json
```

### Options

- `--waypoints`, `-w`: Number of waypoints to generate (default: 7)
- `--maze`, `-m`: Use 2D maze mode - no upward movement allowed (default: true)
- `--grid-width`: Grid width for maze mode (default: 10)
- `--grid-height`: Grid height for maze mode (default: 10)
- `--sphere-points`, `-s`: Number of Fibonacci sphere points for 3D mode (default: 100)
- `--speed`, `-v`: Entity movement speed (default: 2.0)
- `--output`, `-o`: Output path for trajectory JSON (default: `/tmp/blender_trajectory.json`)

### Programmatic Usage

```elixir
alias AriaPatrolSolver.Solver

# Maze mode (2D, no upward movement)
{:ok, result} = Solver.solve(
  num_waypoints: 10,
  maze_mode: true,
  grid_width: 15,
  grid_height: 15,
  entity_speed: 3.0,
  output_path: "/tmp/patrol.json"
)

# 3D mode (Fibonacci sphere)
{:ok, result} = Solver.solve(
  num_waypoints: 10,
  maze_mode: false,
  sphere_points: 200,
  entity_speed: 3.0,
  output_path: "/tmp/patrol.json"
)

# Access results
result.trajectory          # Full trajectory data
result.trajectory_json     # JSON-compatible map
result.optimized_sequence  # Optimized waypoint order
result.total_distance      # Total travel distance
```

## Architecture

- **`AriaPatrolSolver.Solver`**: Main solver module that orchestrates the patrol problem solving
- **`AriaPatrolSolver.ShortestPath`**: Path optimization using nearest-neighbor heuristic
- **`AriaPatrolSolver.CLI`**: Command-line interface for the application
- **`Mix.Tasks.PatrolSolve`**: Mix task for easy execution

## Dependencies

- `aria_planner`: Core planning domain and locomotion functionality
- `jason`: JSON encoding for trajectory export

## Output

The solver generates:

1. **Trajectory JSON**: Saved to the specified output path (default: `/tmp/blender_trajectory.json`)
2. **Console Output**: Summary statistics and trajectory data with markers for parsing

The trajectory JSON can be used with Blender MCP for 3D visualization of the patrol path.

## Documentation

- **[Blender Export Guide](docs/blender_export.md)**: Complete guide on exporting trajectory data and visualizing in Blender


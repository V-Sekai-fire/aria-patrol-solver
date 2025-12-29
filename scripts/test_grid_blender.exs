# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

# Script to test grid-based patrol solver and visualize in Blender

alias AriaPatrolSolver.Solver

require Logger
Logger.configure(level: :info)

IO.puts("=" <> String.duplicate("=", 80))
IO.puts("Grid-Based Patrol Solver - Blender Visualization Test")
IO.puts("=" <> String.duplicate("=", 80))

# Solve patrol problem with grid mode
IO.puts("\n[1/2] Solving patrol problem with grid generation...")
{:ok, result} = Solver.solve(
  num_waypoints: 8,
  maze_mode: true,
  grid_width: 15,
  grid_height: 15,
  entity_speed: 2.0,
  output_path: "/tmp/blender_trajectory.json"
)

IO.puts("✓ Solved!")
IO.puts("  Waypoints: #{result.num_waypoints}")
IO.puts("  Optimized sequence: #{Enum.join(result.optimized_sequence, " -> ")} -> start")
IO.puts("  Total distance: #{:erlang.float_to_binary(result.total_distance, decimals: 4)} units")
IO.puts("  Trajectory steps: #{length(result.trajectory)}")
IO.puts("  Output file: #{result.output_path}")

IO.puts("\n[2/2] Trajectory data ready for Blender visualization")
IO.puts("  Use Blender MCP to load: #{result.output_path}")

IO.puts("\n" <> String.duplicate("=", 80))
IO.puts("SUCCESS! Ready for Blender visualization")
IO.puts(String.duplicate("=", 80))

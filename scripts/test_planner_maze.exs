# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

# Script to test aria planner solving a maze navigation problem

alias AriaPatrolSolver.Solver

require Logger
Logger.configure(level: :info)

IO.puts("=" <> String.duplicate("=", 80))
IO.puts("Aria Planner - Maze Navigation Test")
IO.puts("=" <> String.duplicate("=", 80))

# Solve patrol problem with grid mode using aria planner
IO.puts("\n[1/2] Solving maze navigation with aria planner...")
{:ok, result} = Solver.solve(
  num_waypoints: 5,  # Smaller for testing
  maze_mode: true,
  grid_width: 10,
  grid_height: 10,
  entity_speed: 2.0,
  output_path: "/tmp/blender_trajectory.json"
)

IO.puts("\n✓ Solved!")
IO.puts("  Waypoints: #{result.num_waypoints}")
IO.puts("  Optimized sequence: #{Enum.join(result.optimized_sequence, " -> ")} -> start")
IO.puts("  Total distance: #{:erlang.float_to_binary(result.total_distance, decimals: 4)} units")
IO.puts("  Trajectory steps: #{length(result.trajectory)}")
IO.puts("  Output file: #{result.output_path}")

IO.puts("\n[2/2] Trajectory data ready for Blender visualization")
IO.puts("=" <> String.duplicate("=", 80))

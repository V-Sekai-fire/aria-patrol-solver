# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

# Script to test navmesh generation and solve patrol in maze mode

Code.require_file("lib/aria_patrol_solver/solver.ex", __DIR__ <> "/../")

alias AriaPatrolSolver.Solver

require Logger
Logger.configure(level: :info)

IO.puts("=" <> String.duplicate("=", 80))
IO.puts("Test Navmesh Generation and Maze Patrol")
IO.puts("=" <> String.duplicate("=", 80))

# Solve patrol with navmesh
IO.puts("\n[1/3] Solving patrol with navmesh...")
opts = [
  num_waypoints: 5,
  maze_mode: true,
  grid_width: 10,
  grid_height: 10,
  obstacles: [{2, 2}, {5, 3}, {7, 7}, {3, 8}],  # Some obstacles in the maze
  entity_speed: 2.0,
  output_path: Path.join(System.tmp_dir!(), "blender_trajectory.json")
]

case Solver.solve(opts) do
  {:ok, result} ->
    IO.puts("✓ Patrol solved successfully")
    IO.puts("  Waypoints: #{result.num_waypoints}")
    IO.puts("  Optimized sequence: #{Enum.join(result.optimized_sequence, " -> ")} -> start")
    IO.puts("  Total distance: #{:erlang.float_to_binary(result.total_distance, decimals: 4)} units")
    IO.puts("  Trajectory steps: #{length(result.trajectory)}")

    IO.puts("\n" <> String.duplicate("=", 80))
    IO.puts("SUCCESS! Patrol solved with navmesh")
    IO.puts("  Trajectory JSON: #{result.output_path}")
    IO.puts(String.duplicate("=", 80))

  {:error, reason} ->
    IO.puts("✗ Failed to solve patrol: #{reason}")
    System.halt(1)
end

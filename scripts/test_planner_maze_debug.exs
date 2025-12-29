# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

# Script to test aria planner solving a maze navigation problem with verbose logging

alias AriaPatrolSolver.Solver

require Logger
Logger.configure(level: :info)

IO.puts("=" <> String.duplicate("=", 80))
IO.puts("Aria Planner - Maze Navigation Test (Debug Mode)")
IO.puts("=" <> String.duplicate("=", 80))

# Solve patrol problem with grid mode using aria planner
IO.puts("\n[1/2] Solving maze navigation with aria planner...")
IO.puts("Starting at: #{DateTime.utc_now() |> DateTime.to_iso8601()}")

try do
  {:ok, result} = Solver.solve(
    num_waypoints: 5,  # More waypoints for interesting patrol
    maze_mode: true,
    grid_width: 10,  # Larger grid
    grid_height: 10,  # Larger grid
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
rescue
  e ->
    IO.puts("\n✗ ERROR: #{inspect(e)}")
    IO.puts("Stacktrace:")
    # credo:disable-for-next-line
    IO.inspect(__STACKTRACE__)
    System.halt(1)
end

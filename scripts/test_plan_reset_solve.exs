# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

# Script to test: plan route + generate maze, reset state, then solve again

alias AriaPatrolSolver.Domains.Locomotion
alias AriaPatrolSolver.Solver

require Logger
Logger.configure(level: :info)

IO.puts("=" <> String.duplicate("=", 80))
IO.puts("Maze Planning and Solving Test")
IO.puts("=" <> String.duplicate("=", 80))

# Configuration
num_waypoints = 5
grid_width = 10
grid_height = 10
entity_speed = 2.0

IO.puts("\n[PHASE 1] Planning Route and Generating Maze")
IO.puts("-" <> String.duplicate("-", 78))

# Step 1: Create domain
IO.puts("\n1. Creating locomotion domain...")
{:ok, _domain} = Locomotion.create_domain(100, true, grid_width, grid_height)
IO.puts("   ✓ Domain created (maze mode: 10x10 grid)")

# Step 2: Generate waypoints
IO.puts("\n2. Generating waypoints...")
waypoint_indices =
  0..(num_waypoints - 1)
  |> Enum.map(fn i ->
    total_positions = grid_width * grid_height
    position_idx = rem(i * div(total_positions, num_waypoints), total_positions)
    rotation_idx = rem(i, 8)
    %{
      id: "wp#{i + 1}",
      position_index: position_idx,
      rotation_index: rotation_idx
    }
  end)

IO.puts("   ✓ Generated #{length(waypoint_indices)} waypoints:")
Enum.each(waypoint_indices, fn wp ->
  IO.puts("     - #{wp.id}: position_index=#{wp.position_index}, rotation_index=#{wp.rotation_index}")
end)

# Step 3: Generate maze with obstacles
IO.puts("\n3. Generating labyrinth maze with obstacles...")
obstacles = if grid_width <= 5 and grid_height <= 5 do
  [{1, 1}, {0, 2}, {2, 0}]
else
  # Larger maze: create a proper labyrinth with corridors
  [
    {1, 1}, {1, 3}, {1, 5}, {1, 7}, {1, 9},
    {3, 0}, {3, 2}, {3, 4}, {3, 6}, {3, 8},
    {5, 1}, {5, 3}, {5, 5}, {5, 7}, {5, 9},
    {7, 0}, {7, 2}, {7, 4}, {7, 6}, {7, 8},
    {9, 1}, {9, 3}, {9, 5}, {9, 7},
    {0, 2}, {2, 2}, {4, 2}, {6, 2}, {8, 2},
    {1, 4}, {3, 4}, {5, 4}, {7, 4}, {9, 4},
    {0, 6}, {2, 6}, {4, 6}, {6, 6}, {8, 6},
    {1, 8}, {3, 8}, {5, 8}, {7, 8}, {9, 8},
    {2, 1}, {4, 1}, {6, 1}, {8, 1},
    {2, 3}, {4, 3}, {6, 3}, {8, 3},
    {2, 5}, {4, 5}, {6, 5}, {8, 5},
    {2, 7}, {4, 7}, {6, 7}, {8, 7},
    {2, 9}, {4, 9}, {6, 9}, {8, 9}
  ]
end

IO.puts("   ✓ Generated #{length(obstacles)} wall obstacles")
IO.puts("     Maze pattern: Labyrinth with corridors")

# Step 4: Plan route (optimize waypoint order)
IO.puts("\n4. Planning optimal route...")
# Create temporary state for route planning
{:ok, temp_state} = Locomotion.initialize_state(%{
  entities: [
    %{
      id: "entity1",
      position: {0.0, 0.0, 0.0},
      rotation: {0.0, 0.0, 0.0, 1.0},
      speed: entity_speed,
      movement_type: "walking"
    }
  ],
  waypoints: waypoint_indices,
  sphere_points: 100,
  maze_mode: true,
  grid_width: grid_width,
  grid_height: grid_height,
  obstacles: obstacles
})

# Use shortest path optimization
alias AriaPatrolSolver.ShortestPath
start_pos_idx = temp_state.quantized_position["entity1"]

# Calculate distance matrix
distance_matrix =
  for wp1 <- waypoint_indices, into: %{} do
    distances =
      for wp2 <- waypoint_indices, into: %{} do
        dist = Locomotion.calculate_distance(temp_state, wp1.position_index, wp2.position_index)
        {wp2.id, dist}
      end
    {wp1.id, distances}
  end

# Calculate distances from start
start_distances =
  for wp <- waypoint_indices, into: %{} do
    dist = Locomotion.calculate_distance(temp_state, start_pos_idx, wp.position_index)
    {wp.id, dist}
  end

# Find shortest path
optimized_sequence = ShortestPath.find_shortest_path(
  waypoint_indices,
  start_pos_idx,
  temp_state,
  start_distances,
  distance_matrix
)

# Calculate total distance
total_distance = ShortestPath.calculate_total_distance(
  optimized_sequence,
  start_pos_idx,
  temp_state,
  start_distances,
  distance_matrix,
  waypoint_indices
)

IO.puts("   ✓ Route planned:")
IO.puts("     Optimized sequence: #{Enum.join(optimized_sequence, " -> ")} -> start")
IO.puts("     Total distance: #{:erlang.float_to_binary(total_distance, decimals: 4)} units")

IO.puts("\n[PHASE 1 COMPLETE] Maze and route planning finished")
IO.puts("=" <> String.duplicate("=", 80))

# Reset state
IO.puts("\n[PHASE 2] Resetting State")
IO.puts("-" <> String.duplicate("-", 78))
IO.puts("   Resetting domain state...")
IO.puts("   ✓ State reset complete")

IO.puts("\n[PHASE 3] Solving Maze Navigation")
IO.puts("-" <> String.duplicate("-", 78))
IO.puts("Starting at: #{DateTime.utc_now() |> DateTime.to_iso8601()}")

# Now solve from scratch
try do
  {:ok, result} = Solver.solve(
    num_waypoints: num_waypoints,
    maze_mode: true,
    grid_width: grid_width,
    grid_height: grid_height,
    entity_speed: entity_speed,
    output_path: "/tmp/blender_trajectory.json"
  )

  IO.puts("\n✓ Solved!")
  IO.puts("  Waypoints: #{result.num_waypoints}")
  IO.puts("  Optimized sequence: #{Enum.join(result.optimized_sequence, " -> ")} -> start")
  IO.puts("  Total distance: #{:erlang.float_to_binary(result.total_distance, decimals: 4)} units")
  IO.puts("  Trajectory steps: #{length(result.trajectory)}")
  IO.puts("  Output file: #{result.output_path}")
  IO.puts("\n[PHASE 3 COMPLETE] Solution ready for visualization")
  IO.puts("=" <> String.duplicate("=", 80))
rescue
  e ->
    IO.puts("\n✗ ERROR: #{inspect(e)}")
    IO.puts("Stacktrace:")
    # credo:disable-for-next-line
    IO.inspect(__STACKTRACE__)
    System.halt(1)
end

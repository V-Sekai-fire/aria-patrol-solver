# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Solver do
  @moduledoc """
  Main solver for patrol waypoint problems with shortest-path optimization.
  """

  alias AriaCore.Plan
  alias AriaCore.Planner.{Actions, LazyRefinement, Methods}
  alias AriaPatrolSolver.Domains.Locomotion
  alias AriaPatrolSolver.Domains.Locomotion.{Commands, Multigoals, Tasks}
  alias AriaPatrolSolver.Domains.Locomotion.Visualization.SolutionTracker
  alias AriaPatrolSolver.ShortestPath

  @doc """
  Solve a patrol problem with scattered waypoints.

  ## Parameters
  - `opts`: Keyword list with options:
    - `:num_waypoints` - Number of waypoints to generate (default: 7)
    - `:sphere_points` - Number of Fibonacci sphere points for 3D mode (default: 100)
    - `:maze_mode` - If true, use 2D grid maze mode (default: false)
    - `:grid_width` - Grid width for maze mode (default: 10)
    - `:grid_height` - Grid height for maze mode (default: 10)
    - `:entity_speed` - Entity movement speed (default: 2.0)
    - `:output_path` - Path to save trajectory JSON (default: System.tmp_dir!/"blender_trajectory.json")

  ## Returns
  - `{:ok, trajectory_data}` on success
  - `{:error, reason}` on failure
  """
  @spec solve(keyword()) :: {:ok, map()} | {:error, String.t()}
  def solve(opts \\ []) do
    num_waypoints = Keyword.get(opts, :num_waypoints, 7)
    maze_mode = Keyword.get(opts, :maze_mode, true)  # Default to maze mode
    grid_width = Keyword.get(opts, :grid_width, 10)
    grid_height = Keyword.get(opts, :grid_height, 10)
    sphere_points = Keyword.get(opts, :sphere_points, 100)
    entity_speed = Keyword.get(opts, :entity_speed, 2.0)
    output_path = Keyword.get(opts, :output_path, Path.join(System.tmp_dir!(), "blender_trajectory.json"))

    try do
      # Step 1: Initialize locomotion domain
      {:ok, _domain} = Locomotion.create_domain(sphere_points, maze_mode, grid_width, grid_height)

      # Step 2: Generate waypoints
      waypoint_indices = if maze_mode do
        generate_maze_waypoints(num_waypoints, grid_width, grid_height)
      else
        generate_waypoints(num_waypoints, sphere_points)
      end

      # Step 3: Create initial state
      {:ok, domain_state} =
        initialize_state(waypoint_indices, maze_mode, grid_width, grid_height, sphere_points, entity_speed)

      # Step 4: Optimize waypoint order
      {optimized_sequence, total_distance} = optimize_waypoints(waypoint_indices, domain_state)

      # Step 5: Create plan
      {:ok, plan} = create_plan(optimized_sequence, entity_speed)

      # Step 6: Use aria planner to solve the maze
      plan_with_graph = solve_with_planner(plan, optimized_sequence, domain_state, entity_speed)

      # Step 7: Track trajectory
      {:ok, trajectory} = SolutionTracker.track_trajectory(plan_with_graph, domain_state)

      # Step 8: Export trajectory
      trajectory_json = SolutionTracker.export_to_json(trajectory)
      trajectory_json_string = Jason.encode!(trajectory_json, pretty: true)
      File.write!(output_path, trajectory_json_string)

      {:ok, %{
        trajectory: trajectory,
        trajectory_json: trajectory_json,
        output_path: output_path,
        optimized_sequence: optimized_sequence,
        total_distance: total_distance,
        num_waypoints: num_waypoints
      }}
    rescue
      e -> {:error, "Failed to solve patrol: #{inspect(e)}"}
    end
  end

  defp generate_waypoints(num_waypoints, sphere_points) do
    0..(num_waypoints - 1)
    |> Enum.map(fn i ->
      position_idx = rem(i * div(sphere_points, num_waypoints), sphere_points)
      rotation_idx = rem(i * div(sphere_points, num_waypoints) + 5, sphere_points)
      %{
        id: "wp#{i + 1}",
        position_index: position_idx,
        rotation_index: rotation_idx
      }
    end)
  end

  defp generate_maze_waypoints(num_waypoints, grid_width, grid_height) do
    total_positions = grid_width * grid_height

    # Generate random waypoint positions on the grid
    waypoint_positions =
      0..(num_waypoints - 1)
      |> Enum.map(fn i ->
        # Distribute waypoints across the grid
        position_idx = rem(i * div(total_positions, num_waypoints), total_positions)
        # Use 8 directions for rotation in 2D (N, NE, E, SE, S, SW, W, NW)
        rotation_idx = rem(i, 8)
        %{
          id: "wp#{i + 1}",
          position_index: position_idx,
          rotation_index: rotation_idx
        }
      end)

    waypoint_positions
  end

  defp initialize_state(waypoint_indices, maze_mode, grid_width, grid_height, sphere_points, entity_speed) do
    # Generate obstacles for a proper labyrinth (maze with corridors)
    # Create a pattern that forms a solvable maze with paths between cells
    obstacles = if grid_width <= 5 and grid_height <= 5 do
      # Small maze: simple labyrinth pattern
      [{1, 1}, {0, 2}, {2, 0}]
    else
      # Larger maze: create a proper labyrinth with corridors
      # Pattern creates walls that form a maze-like structure
      # This creates corridors and dead ends, making it a true labyrinth
      labyrinth_walls = [
        # Create vertical corridors (walls that block horizontal movement)
        {1, 1}, {1, 3}, {1, 5}, {1, 7}, {1, 9},
        {3, 0}, {3, 2}, {3, 4}, {3, 6}, {3, 8},
        {5, 1}, {5, 3}, {5, 5}, {5, 7}, {5, 9},
        {7, 0}, {7, 2}, {7, 4}, {7, 6}, {7, 8},
        {9, 1}, {9, 3}, {9, 5}, {9, 7},
        # Create horizontal corridors (walls that block vertical movement)
        {0, 2}, {2, 2}, {4, 2}, {6, 2}, {8, 2},
        {1, 4}, {3, 4}, {5, 4}, {7, 4}, {9, 4},
        {0, 6}, {2, 6}, {4, 6}, {6, 6}, {8, 6},
        {1, 8}, {3, 8}, {5, 8}, {7, 8}, {9, 8},
        # Add some strategic walls to create dead ends and force pathfinding
        {2, 1}, {4, 1}, {6, 1}, {8, 1},
        {2, 3}, {4, 3}, {6, 3}, {8, 3},
        {2, 5}, {4, 5}, {6, 5}, {8, 5},
        {2, 7}, {4, 7}, {6, 7}, {8, 7},
        {2, 9}, {4, 9}, {6, 9}, {8, 9}
      ]
      |> Enum.filter(fn {x, y} ->
        # Filter to only include coordinates within grid bounds
        x >= 0 and x < grid_width and y >= 0 and y < grid_height and
        # Don't block the starting position (0, 0)
        not ({x, y} == {0, 0})
      end)
      |> Enum.uniq()  # Remove duplicates

      labyrinth_walls
    end

    initial_state_params = %{
      maze_mode: maze_mode,
      grid_width: grid_width,
      grid_height: grid_height,
      grid_spacing: 1.0,
      obstacles: obstacles,
      sphere_points: sphere_points,
      rotation_points: 8,  # 8 directions for 2D maze
      entities: [
        %{
          id: "entity1",
          position: {0.0, 0.0, 0.0},  # Start at grid origin
          rotation: %{x: 0.0, y: 0.0, z: 0.0, w: 1.0},
          speed: entity_speed,
          movement_type: "walking"
        }
      ],
      waypoints: waypoint_indices
    }

    Locomotion.initialize_state(initial_state_params)
  end

  defp optimize_waypoints(waypoint_indices, domain_state) do
    start_pos_idx = domain_state.quantized_position["entity1"]

    # Calculate distance matrix
    distance_matrix =
      for wp1 <- waypoint_indices, into: %{} do
        distances =
          for wp2 <- waypoint_indices, into: %{} do
            dist = Locomotion.calculate_distance(domain_state, wp1.position_index, wp2.position_index)
            {wp2.id, dist}
          end
        {wp1.id, distances}
      end

    # Calculate distances from start
    start_distances =
      for wp <- waypoint_indices, into: %{} do
        dist = Locomotion.calculate_distance(domain_state, start_pos_idx, wp.position_index)
        {wp.id, dist}
      end

    # Find shortest path
    optimized_sequence = ShortestPath.find_shortest_path(
      waypoint_indices,
      start_pos_idx,
      domain_state,
      start_distances,
      distance_matrix
    )

    total_distance = ShortestPath.calculate_total_distance(
      optimized_sequence,
      start_pos_idx,
      domain_state,
      start_distances,
      distance_matrix,
      waypoint_indices
    )

    {optimized_sequence, total_distance}
  end

  defp create_plan(optimized_sequence, entity_speed) do
    plan_attrs = %{
      name: "patrol_scattered_waypoints",
      persona_id: "test_persona",
      domain_type: "locomotion",
      objectives: [{"patrol", "entity1", optimized_sequence}],
      entity_capabilities: %{"entity1" => %{speed: entity_speed, movement_type: "walking"}}
    }

    Plan.create(plan_attrs)
  end

  # Use aria planner to solve the maze navigation problem
  defp solve_with_planner(plan, optimized_sequence, domain_state, entity_speed) do
    require Logger
    Logger.info("Using aria planner to solve maze navigation...")
    IO.puts("=" <> String.duplicate("=", 80))
    IO.puts("PLANNER: Starting maze navigation solver")
    IO.puts("=" <> String.duplicate("=", 80))

    # Store domain state in process dictionary for access by methods/actions
    domain_state_key = :patrol_solver_domain_state
    Process.put(domain_state_key, domain_state)

    # Build domain spec for planner
    IO.puts("PLANNER: Building domain spec...")
    domain_spec = build_domain_spec(domain_state_key)
    IO.puts("PLANNER: Domain spec built")

    # Convert domain state to planner initial state params
    IO.puts("PLANNER: Building planner state...")
    initial_state_params = build_planner_state(domain_state, entity_speed)
    IO.puts("PLANNER: Planner state built")

    # Create initial tasks from objectives
    IO.puts("PLANNER: Building initial tasks...")
    initial_tasks = build_initial_tasks(plan, optimized_sequence)
    IO.puts("PLANNER: Initial tasks: #{inspect(initial_tasks)}")
    Logger.info("Initial tasks: #{inspect(initial_tasks)}")

    # Update domain spec with initial tasks
    domain_spec = Map.put(domain_spec, :initial_tasks, initial_tasks)

    # Solve using lazy refinement planner
    IO.puts("PLANNER: Starting lazy refinement planner...")
    Logger.info("Starting planner...")

    result = try do
      IO.puts("PLANNER: Calling LazyRefinement.run_lazy_refineahead...")
      result = LazyRefinement.run_lazy_refineahead(domain_spec, initial_state_params, plan, [])
      IO.puts("PLANNER: Planner returned: #{inspect(result)}")
      result
    catch
      kind, reason ->
        IO.puts("PLANNER: ERROR - #{kind}: #{inspect(reason)}")
        Logger.error("Planner error: #{kind} - #{inspect(reason)}")
        {:error, "Planner error: #{kind} - #{inspect(reason)}"}
    after
      Process.delete(domain_state_key)
      IO.puts("PLANNER: Cleaned up process dictionary")
    end

    case result do
      {:ok, solved_plan} ->
        Logger.info("Planner completed successfully!")
        solved_plan

      {:error, reason} ->
        Logger.error("Planner failed: #{inspect(reason)}")
        raise "Planner failed to solve maze: #{inspect(reason)}"
    end
  end

  # Build domain spec with methods and actions for planner
  defp build_domain_spec(domain_state_key) do
    # Helper to convert string task/action names to atoms
    # Tasks and actions should already be in correct format (no state)
    normalize_task_names = fn task_tuples ->
      Enum.map(task_tuples, fn
        {"t_" <> _ = task_name, entity, arg1, arg2} ->
          # Handle 4-argument tasks like t_follow_path
          {String.to_atom(task_name), entity, arg1, arg2}

        {"t_" <> _ = task_name, entity, arg} ->
          {String.to_atom(task_name), entity, arg}

        {"t_" <> _ = task_name, entity} ->
          {String.to_atom(task_name), entity}

        {"c_" <> _ = action_name, entity, arg1, arg2} ->
          # Handle 3-argument actions like c_move_and_rotate
          {String.to_atom(action_name), entity, arg1, arg2}

        {"c_" <> _ = action_name, entity, arg} ->
          {String.to_atom(action_name), entity, arg}

        {"c_" <> _ = action_name, entity} ->
          {String.to_atom(action_name), entity}

        tuple when is_tuple(tuple) ->
          # Already in correct format (atoms)
          tuple

        other -> other
      end)
    end

    # Register task methods - access domain state from process dictionary
    # Note: Method names must match task names (with t_ prefix)
    methods = Methods.new()
    |> Methods.add_task_method(:t_navigate_to, fn planner_state, _task_name, entity, waypoint ->
      domain_state = Process.get(domain_state_key)
      # planner_state and task_name are passed by planner but we use domain_state from process dict
      Tasks.NavigateTo.t_navigate_to(domain_state, entity, waypoint)
      |> normalize_task_names.()
    end)
    |> Methods.add_task_method(:t_navigate_path, fn planner_state, _task_name, entity, waypoint_seq ->
      domain_state = Process.get(domain_state_key)
      # planner_state and task_name are passed by planner but we use domain_state from process dict
      Tasks.NavigatePath.t_navigate_path(domain_state, entity, waypoint_seq)
      |> normalize_task_names.()
    end)
    |> Methods.add_task_method(:t_patrol, fn planner_state, _task_name, entity, waypoint_seq ->
      domain_state = Process.get(domain_state_key)
      # planner_state and task_name are passed by planner but we use domain_state from process dict
      Tasks.Patrol.t_patrol(domain_state, entity, waypoint_seq)
      |> normalize_task_names.()
    end)
    |> Methods.add_task_method(:t_return_to_start, fn planner_state, _task_name, entity, start_pos ->
      domain_state = Process.get(domain_state_key)
      # planner_state and task_name are passed by planner but we use domain_state from process dict
      Tasks.ReturnToStart.t_return_to_start(domain_state, entity, start_pos)
      |> normalize_task_names.()
    end)
    |> Methods.add_task_method(:t_global_pathfind, fn planner_state, _task_name, entity, target_pos ->
      domain_state = Process.get(domain_state_key)
      # planner_state and task_name are passed by planner but we use domain_state from process dict
      Tasks.GlobalPathfind.t_global_pathfind(domain_state, entity, target_pos)
      |> normalize_task_names.()
    end)
    |> Methods.add_task_method(:t_follow_path, fn planner_state, _task_name, entity, path_indices, final_target ->
      domain_state = Process.get(domain_state_key)
      # planner_state and task_name are passed by planner but we use domain_state from process dict
      Tasks.FollowPath.t_follow_path(domain_state, entity, path_indices, final_target)
      |> normalize_task_names.()
    end)
    |> Methods.add_multigoal_method(:t_navigate_to, fn planner_state, entity, waypoint ->
      domain_state = Process.get(domain_state_key)
      # planner_state is passed by planner but we use domain_state from process dict
      Multigoals.NavigateTo.m_navigate_to(domain_state, entity, waypoint)
      |> normalize_task_names.()
    end)
    |> Methods.add_multigoal_method(:t_navigate_path, fn planner_state, entity, waypoint_seq ->
      domain_state = Process.get(domain_state_key)
      # planner_state is passed by planner but we use domain_state from process dict
      Multigoals.NavigatePath.m_navigate_path(domain_state, entity, waypoint_seq)
      |> normalize_task_names.()
    end)
    |> Methods.add_multigoal_method(:t_patrol, fn planner_state, entity, waypoint_seq ->
      domain_state = Process.get(domain_state_key)
      # planner_state is passed by planner but we use domain_state from process dict
      Multigoals.Patrol.m_patrol(domain_state, entity, waypoint_seq)
      |> normalize_task_names.()
    end)

    # Register actions - wrap to handle state conversion and duration
    actions = Actions.new()
    |> Actions.add_action(:c_move_to, fn planner_state, _action_name, entity, target_pos ->
      domain_state = Process.get(domain_state_key)
      case Commands.MoveTo.c_move_to(domain_state, entity, target_pos) do
        {:ok, new_domain_state} ->
          duration = get_action_duration(new_domain_state)
          Process.put(domain_state_key, new_domain_state)
          {:ok, planner_state, duration}
        error -> error
      end
    end)
    |> Actions.add_action(:c_rotate_to, fn planner_state, _action_name, entity, target_rot ->
      domain_state = Process.get(domain_state_key)
      case Commands.RotateTo.c_rotate_to(domain_state, entity, target_rot) do
        {:ok, new_domain_state} ->
          duration = get_action_duration(new_domain_state)
          Process.put(domain_state_key, new_domain_state)
          {:ok, planner_state, duration}
        error -> error
      end
    end)
    |> Actions.add_action(:c_move_and_rotate, fn planner_state, _action_name, entity, target_pos, target_rot ->
      domain_state = Process.get(domain_state_key)
      case Commands.MoveAndRotate.c_move_and_rotate(domain_state, entity, target_pos, target_rot) do
        {:ok, new_domain_state} ->
          duration = get_action_duration(new_domain_state)
          Process.put(domain_state_key, new_domain_state)
          {:ok, planner_state, duration}
        error -> error
      end
    end)
    |> Actions.add_action(:c_mark_waypoint_reached, fn planner_state, _action_name, entity, waypoint ->
      domain_state = Process.get(domain_state_key)
      case Commands.MarkWaypointReached.c_mark_waypoint_reached(domain_state, entity, waypoint) do
        {:ok, new_domain_state} ->
          Process.put(domain_state_key, new_domain_state)
          {:ok, planner_state, 0}
        error -> error
      end
    end)

    %{
      methods: methods,
      actions: actions,
      initial_tasks: []  # Will be set by caller
    }
  end

  # Get action duration from domain state metadata
  defp get_action_duration(domain_state) do
    metadata = Map.get(domain_state, :planner_metadata, %{})
    duration_seconds = Map.get(metadata, :last_action_duration, 0.5)

    # Handle :infinity case (shouldn't happen due to speed checks, but be safe)
    case duration_seconds do
      :infinity -> 0  # Return 0ms for infinity (shouldn't happen)
      duration when is_number(duration) -> trunc(duration * 1000)  # Convert to milliseconds
      _ -> 500  # Default to 500ms if invalid value
    end
  end

  # Convert domain state to planner initial state params
  defp build_planner_state(domain_state, entity_speed) do
    # Convert domain state facts to planner facts format
    facts = %{
      "entity1" => %{
        quantized_position: domain_state.quantized_position["entity1"],
        quantized_rotation: domain_state.quantized_rotation["entity1"],
        entity_speed: entity_speed,
        movement_type: domain_state.movement_type["entity1"]
      }
    }

    # Add waypoint reached facts
    waypoint_facts =
      for entity_id <- Map.keys(domain_state.quantized_position),
          waypoint <- domain_state.waypoints do
        waypoint_id = Map.get(waypoint, :id) || Map.get(waypoint, "id")
        reached = Map.get(domain_state.waypoint_reached, {entity_id, waypoint_id}, false)
        {"#{entity_id}_#{waypoint_id}", %{waypoint_reached: reached}}
      end
      |> Enum.into(%{})

    facts = Map.merge(facts, waypoint_facts)

    %{
      current_time: DateTime.utc_now(),
      timeline: %{},
      entity_capabilities: %{"entity1" => %{speed: entity_speed, movement_type: "walking"}},
      facts: facts
    }
  end

  # Build initial tasks from plan objectives
  defp build_initial_tasks(plan, _optimized_sequence) do
    objectives = Map.get(plan, :objectives, [])

    Enum.map(objectives, fn objective ->
      normalize_objective(objective)
    end)
  end

  defp normalize_objective({"patrol", entity, waypoint_sequence}) do
    {:t_patrol, entity, waypoint_sequence}
  end

  defp normalize_objective({"navigate_path", entity, waypoint_sequence}) do
    {:t_navigate_path, entity, waypoint_sequence}
  end

  defp normalize_objective({"navigate_to", entity, waypoint}) do
    {:t_navigate_to, entity, waypoint}
  end

  defp normalize_objective(objective) when is_tuple(objective) and tuple_size(objective) >= 2 do
    task_name = elem(objective, 0)

    if is_binary(task_name) do
      List.to_tuple([String.to_atom("t_#{task_name}") | Tuple.to_list(objective) |> tl()])
    else
      objective
    end
  end

  defp normalize_objective(objective) do
    objective
  end
end

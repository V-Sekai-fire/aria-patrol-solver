# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Visualization.SolutionTracker do
  @moduledoc """
  Tracks solution trajectory during locomotion planning execution.

  Captures entity positions, rotations, and waypoint status at each step
  for visualization purposes.
  """

  alias AriaPatrolSolver.Domains.Locomotion.FibonacciSphere
  alias AriaPatrolSolver.Domains.Locomotion.Predicates.{
    QuantizedPosition,
    QuantizedRotation,
    WaypointReached
  }

  @type trajectory_step :: %{
          step: integer(),
          time: float(),
          entities: [entity_state()],
          waypoints: [waypoint_state()]
        }

  @type entity_state :: %{
          id: String.t(),
          position: {float(), float(), float()},
          position_index: integer(),
          rotation: {float(), float(), float()},
          rotation_index: integer(),
          speed: float(),
          movement_type: String.t()
        }

  @type waypoint_state :: %{
          id: String.t(),
          position: {float(), float(), float()},
          position_index: integer(),
          rotation: {float(), float(), float()},
          rotation_index: integer(),
          reached_by: [String.t()]
        }

  @doc """
  Tracks trajectory from a completed plan's solution graph.

  ## Parameters
  - `plan`: Plan struct with solution_graph_data
  - `domain_state`: Initial locomotion domain state

  ## Returns
  - `{:ok, trajectory}` where trajectory is a list of trajectory_step maps
  """
  @spec track_trajectory(map(), map()) :: {:ok, [trajectory_step()]} | {:error, String.t()}
  def track_trajectory(plan, domain_state) do
    try do
      solution_graph = Map.get(plan, :solution_graph_data, %{})
      # Handle both maze mode (grid) and 3D mode (sphere)
      sphere = Map.get(domain_state, :fibonacci_sphere) || domain_state.position_space

      # Extract action nodes in execution order
      actions = extract_actions_in_order(solution_graph)

      # Build trajectory by simulating state changes
      trajectory =
        actions
        |> Enum.reduce({[], domain_state, 0, 0.0}, fn action, {acc, state, step, time} ->
          new_state = apply_action_to_state(state, action)
          step_data = capture_step(new_state, sphere, step, time)
          {[step_data | acc], new_state, step + 1, time + get_action_duration(action)}
        end)
        |> elem(0)
        |> Enum.reverse()

      {:ok, trajectory}
    rescue
      e -> {:error, "Failed to track trajectory: #{inspect(e)}"}
    end
  end

  @doc """
  Exports trajectory to JSON format for Node.js rendering.

  ## Parameters
  - `trajectory`: List of trajectory_step maps

  ## Returns
  - JSON-encodable map structure with tuples converted to arrays
  """
  @spec export_to_json([trajectory_step()]) :: map()
  def export_to_json(trajectory) do
    %{
      steps: Enum.map(trajectory, &convert_step_to_json/1),
      metadata: %{
        total_steps: length(trajectory),
        total_time: get_total_time(trajectory)
      }
    }
  end

  # Private: Convert step data to JSON-compatible format (tuples -> arrays)
  defp convert_step_to_json(step) do
    %{
      step: step.step,
      time: step.time,
      entities: Enum.map(step.entities, &convert_entity_to_json/1),
      waypoints: Enum.map(step.waypoints, &convert_waypoint_to_json/1)
    }
  end

  # Private: Convert entity to JSON format
  defp convert_entity_to_json(entity) do
    # Handle both tuple and list formats
    position = case entity.position do
      {px, py, pz} -> [px, py, pz]
      [px, py, pz] when is_number(px) -> [px, py, pz]
      _ -> [0.0, 0.0, 0.0]
    end

    rotation = case entity.rotation do
      {rx, ry, rz} -> [rx, ry, rz]
      [rx, ry, rz] when is_number(rx) -> [rx, ry, rz]
      _ -> [0.0, 0.0, 1.0]
    end

    %{
      id: entity.id,
      position: position,
      position_index: entity.position_index,
      rotation: rotation,
      rotation_index: entity.rotation_index,
      speed: entity.speed,
      movement_type: entity.movement_type
    }
  end

  # Private: Convert waypoint to JSON format
  defp convert_waypoint_to_json(waypoint) do
    # Handle both tuple and list formats
    position = case waypoint.position do
      {px, py, pz} -> [px, py, pz]
      [px, py, pz] when is_number(px) -> [px, py, pz]
      _ -> [0.0, 0.0, 0.0]
    end

    rotation = case waypoint.rotation do
      {rx, ry, rz} -> [rx, ry, rz]
      [rx, ry, rz] when is_number(rx) -> [rx, ry, rz]
      _ -> [0.0, 0.0, 1.0]
    end

    %{
      id: waypoint.id,
      position: position,
      position_index: waypoint.position_index,
      rotation: rotation,
      rotation_index: waypoint.rotation_index,
      reached_by: waypoint.reached_by
    }
  end

  # Private: Extract action nodes in execution order from solution graph
  defp extract_actions_in_order(solution_graph) do
    solution_graph
    |> Map.to_list()
    |> Enum.filter(fn {_node_id, node} -> node.type == :A end)
    |> Enum.sort_by(fn {node_id, node} ->
      # Sort by start_time if available, otherwise by node_id
      # Always return an integer for consistent sorting
      start_time = Map.get(node, :start_time)

      if is_struct(start_time, DateTime) do
        # Convert DateTime to unix timestamp (microseconds) for numeric sorting
        # Use a large offset to ensure timestamps sort after node_ids
        try do
          1_000_000_000_000_000 + DateTime.to_unix(start_time, :microsecond)
        rescue
          _ -> node_id  # Fallback to node_id if conversion fails
        end
      else
        # Use node_id directly (will be < 1_000_000_000_000_000, so sorts first)
        node_id
      end
    end)
    |> Enum.map(fn {_node_id, node} -> node end)
  end

  # Private: Apply action to state and return new state
  defp apply_action_to_state(state, action) do
    action_info = Map.get(action, :info, {})
    action_name = elem(action_info, 0)

    case action_name do
      "c_move_to" ->
        entity = elem(action_info, 1)
        target_pos = elem(action_info, 2)
        QuantizedPosition.set(state, entity, target_pos)

      "c_rotate_to" ->
        entity = elem(action_info, 1)
        target_rot = elem(action_info, 2)
        QuantizedRotation.set(state, entity, target_rot)

      "c_move_and_rotate" ->
        entity = elem(action_info, 1)
        target_pos = elem(action_info, 2)
        target_rot = elem(action_info, 3)
        state
        |> QuantizedPosition.set(entity, target_pos)
        |> QuantizedRotation.set(entity, target_rot)

      "c_mark_waypoint_reached" ->
        entity = elem(action_info, 1)
        waypoint = elem(action_info, 2)
        WaypointReached.set(state, entity, waypoint, true)

      _ ->
        state
    end
  end

  # Private: Capture step data from current state
  defp capture_step(state, position_space, step, time) do
    entities = capture_entity_states(state, position_space)
    waypoints = capture_waypoint_states(state, position_space)

    %{
      step: step,
      time: time,
      entities: entities,
      waypoints: waypoints
    }
  end

  # Private: Capture entity states
  defp capture_entity_states(state, position_space) do
    alias AriaPatrolSolver.Domains.Locomotion.{FibonacciSphere, MazeGrid}

    state.quantized_position
    |> Map.keys()
    |> Enum.map(fn entity_id ->
      pos_idx = QuantizedPosition.get(state, entity_id)
      rot_idx = QuantizedRotation.get(state, entity_id)

      # Handle both grid (maze mode) and sphere (3D mode)
      pos = if state.maze_mode do
        MazeGrid.get_point(position_space, pos_idx) || {0.0, 0.0, 0.0}
      else
        FibonacciSphere.get_point(position_space, pos_idx) || {0.0, 0.0, 0.0}
      end

      # Rotation space is always a sphere (even in maze mode, it's a small 8-direction sphere)
      rotation_space = state.rotation_space
      rot = FibonacciSphere.get_point(rotation_space, rot_idx) || {0.0, 0.0, 1.0}

      %{
        id: entity_id,
        position: pos,
        position_index: pos_idx,
        rotation: rot,
        rotation_index: rot_idx,
        speed: Map.get(state.entity_speed, entity_id, 1.0),
        movement_type: Map.get(state.movement_type, entity_id, "walking")
      }
    end)
  end

  # Private: Capture waypoint states
  defp capture_waypoint_states(state, position_space) do
    state.waypoints
    |> Enum.map(fn waypoint ->
      process_waypoint(state, position_space, waypoint)
    end)
  end

  defp process_waypoint(state, position_space, waypoint) do
    alias AriaPatrolSolver.Domains.Locomotion.{FibonacciSphere, MazeGrid}

    waypoint_id = Map.get(waypoint, :id) || Map.get(waypoint, "id")
    pos_idx = Map.get(waypoint, :position_index) || Map.get(waypoint, "position_index") || 0
    rot_idx = Map.get(waypoint, :rotation_index) || Map.get(waypoint, "rotation_index") || 0

    pos = get_waypoint_position(state, position_space, pos_idx)
    rot = get_waypoint_rotation(state, rot_idx)
    reached_by = find_entities_at_waypoint(state, waypoint_id)

    %{
      id: waypoint_id,
      position: pos,
      position_index: pos_idx,
      rotation: rot,
      rotation_index: rot_idx,
      reached_by: reached_by
    }
  end

  defp get_waypoint_position(state, position_space, pos_idx) do
    alias AriaPatrolSolver.Domains.Locomotion.{FibonacciSphere, MazeGrid}

    if state.maze_mode do
      MazeGrid.get_point(position_space, pos_idx) || {0.0, 0.0, 0.0}
    else
      FibonacciSphere.get_point(position_space, pos_idx) || {0.0, 0.0, 0.0}
    end
  end

  defp get_waypoint_rotation(state, rot_idx) do
    alias AriaPatrolSolver.Domains.Locomotion.FibonacciSphere

    rotation_space = state.rotation_space
    FibonacciSphere.get_point(rotation_space, rot_idx) || {0.0, 0.0, 1.0}
  end

  defp find_entities_at_waypoint(state, waypoint_id) do
    state.quantized_position
    |> Map.keys()
    |> Enum.filter(fn entity_id ->
      WaypointReached.get(state, entity_id, waypoint_id)
    end)
  end

  # Private: Get action duration from action node
  defp get_action_duration(action) do
    duration = Map.get(action, :duration, 0.0)
    # Handle non-numeric durations (shouldn't happen, but be safe)
    if is_number(duration) do
      duration / 1000.0
    else
      0.0
    end
  end

  # Private: Calculate total time from trajectory
  defp get_total_time(trajectory) do
    case trajectory do
      [] -> 0.0
      [last_step | _] -> Map.get(last_step, :time, 0.0)
    end
  end
end

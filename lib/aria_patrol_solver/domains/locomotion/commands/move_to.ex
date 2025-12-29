# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Commands.MoveTo do
  @moduledoc """
  Command: c_move_to(entity, target_position_index)

  Move entity to a quantized target position.

  Preconditions:
  - Entity exists
  - Entity can move (movement_type allows it)
  - Entity has positive speed
  - Target position index is valid

  Effects:
  - quantized_position[entity] = target_position_index
  - Temporal duration = distance / speed
  """

  alias AriaPatrolSolver.Domains.Locomotion
  alias AriaPatrolSolver.Domains.Locomotion.FibonacciSphere
  alias AriaPatrolSolver.Domains.Locomotion.MazeGrid
  alias AriaPatrolSolver.Domains.Locomotion.Navmesh
  alias AriaPatrolSolver.Domains.Locomotion.Predicates.{
    EntitySpeed,
    MovementType,
    QuantizedPosition
  }

  @spec c_move_to(
          state :: map(),
          entity :: String.t(),
          target_position_index :: integer()
        ) :: {:ok, map()} | {:error, String.t()}
  def c_move_to(state, entity, target_position_index)
      when is_binary(entity) and is_integer(target_position_index) and
             target_position_index >= 0 do
    with :ok <- check_entity_exists(state, entity),
         :ok <- check_can_move(state, entity),
         :ok <- check_speed_positive(state, entity),
         :ok <- check_valid_position_index(state, target_position_index),
         :ok <- check_no_upward_movement(state, entity, target_position_index),
         :ok <- check_navmesh_path(state, entity, target_position_index) do
      # Calculate duration BEFORE updating position (need old position for distance)
      current_pos_idx = QuantizedPosition.get(state, entity)
      distance = Locomotion.calculate_distance(state, current_pos_idx, target_position_index)
      speed = EntitySpeed.get(state, entity)
      duration = Locomotion.calculate_duration(distance, speed)

      # Now update position
      new_state = QuantizedPosition.set(state, entity, target_position_index)

      # Store duration in metadata for temporal planner
      metadata = Map.get(new_state, :planner_metadata, %{})
      metadata = Map.put(metadata, :last_action_duration, duration)
      new_state = Map.put(new_state, :planner_metadata, metadata)

      {:ok, new_state}
    else
      error -> error
    end
  end

  defp check_entity_exists(state, entity) do
    if Map.has_key?(state.quantized_position, entity) do
      :ok
    else
      {:error, "Entity #{entity} does not exist"}
    end
  end

  defp check_can_move(state, entity) do
    movement_type = MovementType.get(state, entity)

    if movement_type in ["walking", "running", "flying", "swimming"] do
      :ok
    else
      {:error, "Entity #{entity} movement_type '#{movement_type}' does not allow movement"}
    end
  end

  defp check_speed_positive(state, entity) do
    speed = EntitySpeed.get(state, entity)

    if speed > 0.0 do
      :ok
    else
      {:error, "Entity #{entity} has non-positive speed: #{speed}"}
    end
  end

  defp check_valid_position_index(state, index) do
    if state.maze_mode do
      grid = state.position_space
      if index < grid.n do
        :ok
      else
        {:error, "Invalid position index #{index} (max: #{grid.n - 1})"}
      end
    else
      sphere = Map.get(state, :fibonacci_sphere) || state.position_space
      if index < sphere.n do
        :ok
      else
        {:error, "Invalid position index #{index} (max: #{sphere.n - 1})"}
      end
    end
  end

  defp check_no_upward_movement(state, entity, target_position_index) do
    if state.maze_mode do
      check_maze_upward_movement(state, entity, target_position_index)
    else
      :ok  # 3D mode allows upward movement
    end
  end

  defp check_maze_upward_movement(state, entity, target_position_index) do
    current_pos_idx = QuantizedPosition.get(state, entity)
    grid = state.position_space

    from_point = MazeGrid.get_point(grid, current_pos_idx)
    to_point = MazeGrid.get_point(grid, target_position_index)

    case {from_point, to_point} do
      {nil, _} -> :ok  # Can't check, allow it
      {_, nil} -> :ok  # Can't check, allow it
      {fp, tp} when is_tuple(fp) and is_tuple(tp) ->
        if MazeGrid.upward_movement?(fp, tp) do
          {:error, "Entity #{entity} cannot move upward in maze mode"}
        else
          :ok
        end
    end
  end

  defp check_navmesh_path(state, entity, target_position_index) do
    if state.navmesh do
      validate_navmesh_path(state, entity, target_position_index)
    else
      :ok  # No navmesh, allow movement
    end
  end

  defp validate_navmesh_path(state, entity, target_position_index) do
    current_pos_idx = QuantizedPosition.get(state, entity)
    grid = state.position_space

    from_point = MazeGrid.get_point(grid, current_pos_idx)
    to_point = MazeGrid.get_point(grid, target_position_index)

    case {from_point, to_point} do
      {nil, _} -> :ok  # Can't check, allow it
      {_, nil} -> :ok  # Can't check, allow it
      {fp, tp} when is_tuple(fp) and is_tuple(tp) ->
        check_walkable_and_path(state, entity, fp, tp)
    end
  end

  defp check_walkable_and_path(state, entity, from_point, to_point) do
    from_walkable = Navmesh.walkable?(state.navmesh, from_point)
    to_walkable = Navmesh.walkable?(state.navmesh, to_point)

    if from_walkable && to_walkable do
      # Check if path exists (optional - can be expensive)
      case Navmesh.find_path(state.navmesh, from_point, to_point) do
        {:ok, _path} -> :ok
        {:error, reason} -> {:error, "No navmesh path: #{reason}"}
      end
    else
      {:error, "Entity #{entity} start or target position is not walkable on navmesh"}
    end
  end
end

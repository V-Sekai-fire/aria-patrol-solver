# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Commands.MoveAndRotate do
  @moduledoc """
  Command: c_move_and_rotate(entity, position_index, rotation_index)

  Move and rotate entity simultaneously to target quantized position and orientation.

  Preconditions:
  - Entity exists
  - Entity can move
  - Entity has positive speed
  - Both indices are valid

  Effects:
  - quantized_position[entity] = position_index
  - quantized_rotation[entity] = rotation_index
  - Temporal duration = max(movement_duration, rotation_duration)
  """

  alias AriaPatrolSolver.Domains.Locomotion
  alias AriaPatrolSolver.Domains.Locomotion.FibonacciSphere
  alias AriaPatrolSolver.Domains.Locomotion.Predicates.{
    EntitySpeed,
    MovementType,
    QuantizedPosition,
    QuantizedRotation
  }

  @spec c_move_and_rotate(
          state :: map(),
          entity :: String.t(),
          position_index :: integer(),
          rotation_index :: integer()
        ) :: {:ok, map()} | {:error, String.t()}
  def c_move_and_rotate(state, entity, position_index, rotation_index)
      when is_binary(entity) and is_integer(position_index) and position_index >= 0 and
             is_integer(rotation_index) and rotation_index >= 0 do
    with :ok <- check_entity_exists(state, entity),
         :ok <- check_can_move(state, entity),
         :ok <- check_speed_positive(state, entity),
         :ok <- check_valid_indices(state, position_index, rotation_index) do
      # Update both position and rotation
      new_state =
        state
        |> QuantizedPosition.set(entity, position_index)
        |> QuantizedRotation.set(entity, rotation_index)

      # Calculate combined duration
      # Use position_space for distance calculation (could be grid or sphere)
      position_space = state.position_space
      current_pos_idx = QuantizedPosition.get(state, entity)
      distance = Locomotion.calculate_distance(state, current_pos_idx, position_index)
      speed = EntitySpeed.get(state, entity)
      movement_duration = Locomotion.calculate_duration(distance, speed)
      rotation_duration = Map.get(state, :rotation_duration, 0.5)
      combined_duration = max(movement_duration, rotation_duration)

      # Store duration in metadata
      metadata = Map.get(new_state, :planner_metadata, %{})
      metadata = Map.put(metadata, :last_action_duration, combined_duration)
      new_state = Map.put(new_state, :planner_metadata, metadata)

      {:ok, new_state}
    else
      error -> error
    end
  end

  defp check_entity_exists(state, entity) do
    if Map.has_key?(state.quantized_position, entity) and
         Map.has_key?(state.quantized_rotation, entity) do
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

  defp check_valid_indices(state, position_index, rotation_index) do
    # Use position_space for position validation (could be grid or sphere)
    position_space = state.position_space
    # Use rotation_space for rotation validation (always a sphere, even in maze mode)
    rotation_space = state.rotation_space

    cond do
      position_index >= position_space.n ->
        {:error, "Invalid position index #{position_index} (max: #{position_space.n - 1})"}

      rotation_index >= rotation_space.n ->
        {:error, "Invalid rotation index #{rotation_index} (max: #{rotation_space.n - 1})"}

      true ->
        :ok
    end
  end
end

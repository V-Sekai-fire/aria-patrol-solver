# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Commands.RotateTo do
  @moduledoc """
  Command: c_rotate_to(entity, target_rotation_index)

  Rotate entity to a quantized target orientation.

  Preconditions:
  - Entity exists
  - Target rotation index is valid

  Effects:
  - quantized_rotation[entity] = target_rotation_index
  - Temporal duration = rotation_duration (configurable)
  """

  alias AriaPatrolSolver.Domains.Locomotion.Predicates.QuantizedRotation

  @spec c_rotate_to(
          state :: map(),
          entity :: String.t(),
          target_rotation_index :: integer()
        ) :: {:ok, map()} | {:error, String.t()}
  def c_rotate_to(state, entity, target_rotation_index)
      when is_binary(entity) and is_integer(target_rotation_index) and
             target_rotation_index >= 0 do
    with :ok <- check_entity_exists(state, entity),
         :ok <- check_valid_rotation_index(state, target_rotation_index) do
      new_state = QuantizedRotation.set(state, entity, target_rotation_index)

      # Calculate rotation duration (typically faster than movement)
      rotation_duration = Map.get(state, :rotation_duration, 0.5)

      # Store duration in metadata for temporal planner
      metadata = Map.get(new_state, :planner_metadata, %{})
      metadata = Map.put(metadata, :last_action_duration, rotation_duration)
      new_state = Map.put(new_state, :planner_metadata, metadata)

      {:ok, new_state}
    else
      error -> error
    end
  end

  defp check_entity_exists(state, entity) do
    if Map.has_key?(state.quantized_rotation, entity) do
      :ok
    else
      {:error, "Entity #{entity} does not exist"}
    end
  end

  defp check_valid_rotation_index(state, index) do
    # Use rotation_space (always a sphere, even in maze mode)
    rotation_space = state.rotation_space

    if index < rotation_space.n do
      :ok
    else
      {:error, "Invalid rotation index #{index} (max: #{rotation_space.n - 1})"}
    end
  end
end

# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Predicates.QuantizedRotation do
  @moduledoc """
  Quantized Rotation predicate for locomotion domain.

  Represents the quantized rotation (forward direction) of an entity as a Fibonacci sphere index.
  """

  @doc """
  Gets the quantized rotation index of an entity from state.
  """
  @spec get(state :: map(), entity :: String.t()) :: integer()
  def get(state, entity) do
    Map.get(state.quantized_rotation, entity, 0)
  end

  @doc """
  Sets the quantized rotation index of an entity in state.
  """
  @spec set(state :: map(), entity :: String.t(), rotation_index :: integer()) :: map()
  def set(state, entity, rotation_index) when is_integer(rotation_index) and rotation_index >= 0 do
    new_rotations = Map.put(state.quantized_rotation, entity, rotation_index)
    Map.put(state, :quantized_rotation, new_rotations)
  end
end

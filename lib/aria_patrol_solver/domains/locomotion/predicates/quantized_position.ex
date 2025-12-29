# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Predicates.QuantizedPosition do
  @moduledoc """
  Quantized Position predicate for locomotion domain.

  Represents the quantized 3D position of an entity as a Fibonacci sphere index.
  """

  @doc """
  Gets the quantized position index of an entity from state.
  """
  @spec get(state :: map(), entity :: String.t()) :: integer()
  def get(state, entity) do
    Map.get(state.quantized_position, entity, 0)
  end

  @doc """
  Sets the quantized position index of an entity in state.
  """
  @spec set(state :: map(), entity :: String.t(), position_index :: integer()) :: map()
  def set(state, entity, position_index) when is_integer(position_index) and position_index >= 0 do
    new_positions = Map.put(state.quantized_position, entity, position_index)
    Map.put(state, :quantized_position, new_positions)
  end
end

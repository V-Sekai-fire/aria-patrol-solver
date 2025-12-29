# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Predicates.MovementType do
  @moduledoc """
  Movement Type predicate for locomotion domain.

  Represents the movement type of an entity: "walking", "running", "jumping",
  "climbing", "flying", "swimming".
  """

  @valid_types ["walking", "running", "jumping", "climbing", "flying", "swimming"]

  @doc """
  Gets the movement type of an entity from state.
  """
  @spec get(state :: map(), entity :: String.t()) :: String.t()
  def get(state, entity) do
    Map.get(state.movement_type, entity, "walking")
  end

  @doc """
  Sets the movement type of an entity in state.
  """
  @spec set(state :: map(), entity :: String.t(), movement_type :: String.t()) :: map()
  def set(state, entity, movement_type) when movement_type in @valid_types do
    new_types = Map.put(state.movement_type, entity, movement_type)
    Map.put(state, :movement_type, new_types)
  end
end

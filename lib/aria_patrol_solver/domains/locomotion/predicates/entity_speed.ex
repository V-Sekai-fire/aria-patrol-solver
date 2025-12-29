# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Predicates.EntitySpeed do
  @moduledoc """
  Entity Speed predicate for locomotion domain.

  Represents the movement speed of an entity (units per second).
  """

  @doc """
  Gets the speed of an entity from state.
  """
  @spec get(state :: map(), entity :: String.t()) :: float()
  def get(state, entity) do
    Map.get(state.entity_speed, entity, 1.0)
  end

  @doc """
  Sets the speed of an entity in state.
  """
  @spec set(state :: map(), entity :: String.t(), speed :: float()) :: map()
  def set(state, entity, speed) when is_float(speed) and speed >= 0.0 do
    new_speeds = Map.put(state.entity_speed, entity, speed)
    Map.put(state, :entity_speed, new_speeds)
  end
end

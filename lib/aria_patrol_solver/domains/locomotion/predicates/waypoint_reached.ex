# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Predicates.WaypointReached do
  @moduledoc """
  Waypoint Reached predicate for locomotion domain.

  Represents whether an entity has reached a waypoint.
  """

  @doc """
  Gets whether an entity has reached a waypoint.
  """
  @spec get(state :: map(), entity :: String.t(), waypoint :: String.t()) :: boolean()
  def get(state, entity, waypoint) do
    Map.get(state.waypoint_reached, {entity, waypoint}, false)
  end

  @doc """
  Sets whether an entity has reached a waypoint.
  """
  @spec set(state :: map(), entity :: String.t(), waypoint :: String.t(), reached :: boolean()) ::
          map()
  def set(state, entity, waypoint, reached) when is_boolean(reached) do
    new_reached = Map.put(state.waypoint_reached, {entity, waypoint}, reached)
    Map.put(state, :waypoint_reached, new_reached)
  end
end

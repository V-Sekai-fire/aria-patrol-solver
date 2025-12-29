# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Commands.MarkWaypointReached do
  @moduledoc """
  Command: c_mark_waypoint_reached(entity, waypoint)

  Mark that an entity has reached a waypoint.

  Preconditions:
  - Entity exists
  - Waypoint exists

  Effects:
  - waypoint_reached[entity, waypoint] = true
  """

  alias AriaPatrolSolver.Domains.Locomotion.Predicates.WaypointReached

  @spec c_mark_waypoint_reached(
          state :: map(),
          entity :: String.t(),
          waypoint :: String.t()
        ) :: {:ok, map()}
  def c_mark_waypoint_reached(state, entity, waypoint)
      when is_binary(entity) and is_binary(waypoint) do
    new_state = WaypointReached.set(state, entity, waypoint, true)
    {:ok, new_state}
  end
end

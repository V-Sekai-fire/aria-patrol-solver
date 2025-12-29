# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Multigoals.NavigateTo do
  @moduledoc """
  Multigoal: m_navigate_to(state, entity, waypoint)

  Navigate entity to a target waypoint (goal-based planning).

  Returns subtasks to achieve the goal of reaching the waypoint.
  """

  alias AriaPatrolSolver.Domains.Locomotion.Predicates.WaypointReached

  @spec m_navigate_to(
          state :: map(),
          entity :: String.t(),
          waypoint :: String.t()
        ) :: [tuple()]
  def m_navigate_to(state, entity, waypoint) do
    if WaypointReached.get(state, entity, waypoint) do
      []
    else
      # Decompose into navigation task
      [{"t_navigate_to", entity, waypoint}]
    end
  end
end

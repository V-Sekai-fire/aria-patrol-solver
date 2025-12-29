# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Multigoals.NavigatePath do
  @moduledoc """
  Multigoal: m_navigate_path(state, entity, waypoint_sequence)

  Navigate entity through a sequence of waypoints (goal-based planning).

  Returns subtasks to achieve the goal of reaching all waypoints.
  """

  @spec m_navigate_path(
          state :: map(),
          entity :: String.t(),
          waypoint_sequence :: [String.t()]
        ) :: [tuple()]
  def m_navigate_path(_state, _entity, []) do
    []
  end

  def m_navigate_path(_state, entity, waypoint_sequence) do
    # Decompose into navigation path task
    [{"t_navigate_path", entity, waypoint_sequence}]
  end
end

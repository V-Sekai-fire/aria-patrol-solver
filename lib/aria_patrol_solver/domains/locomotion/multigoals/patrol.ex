# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Multigoals.Patrol do
  @moduledoc """
  Multigoal: m_patrol(state, entity, waypoint_sequence)

  Patrol entity through a sequence of waypoints, returning to start (goal-based planning).

  A patrol is a continuous loop: start -> waypoints -> return to start.
  """

  @spec m_patrol(
          state :: map(),
          entity :: String.t(),
          waypoint_sequence :: [String.t()]
        ) :: [tuple()]
  def m_patrol(_state, _entity, []) do
    []
  end

  def m_patrol(state, entity, waypoint_sequence) do
    # Get starting position
    alias AriaPatrolSolver.Domains.Locomotion.Predicates.QuantizedPosition
    start_pos = QuantizedPosition.get(state, entity)

    # Create patrol path: navigate through waypoints, then return to start
    # Decompose into navigation path task followed by return to start
    [
      {"t_navigate_path", entity, waypoint_sequence},
      {"t_return_to_start", entity, start_pos}
    ]
  end
end

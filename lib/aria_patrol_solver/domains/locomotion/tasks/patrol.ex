# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Tasks.Patrol do
  @moduledoc """
  Task: t_patrol(state, entity, waypoint_sequence)

  Patrol entity through a sequence of waypoints, returning to start.
  """

  alias AriaPatrolSolver.Domains.Locomotion.Predicates.QuantizedPosition

  @spec t_patrol(
          state :: map(),
          entity :: String.t(),
          waypoint_sequence :: [String.t()]
        ) :: [tuple()]
  def t_patrol(_state, _entity, []) do
    []
  end

  def t_patrol(state, entity, waypoint_sequence) do
    # Get starting position
    start_pos = QuantizedPosition.get(state, entity)

    # Patrol: navigate through waypoints, then return to start
    [
      {"t_navigate_path", entity, waypoint_sequence},
      {"t_return_to_start", entity, start_pos}
    ]
  end
end

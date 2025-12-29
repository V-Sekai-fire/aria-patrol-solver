# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Tasks.NavigatePath do
  @moduledoc """
  Task: t_navigate_path(state, entity, waypoint_sequence)

  Navigate entity through a sequence of waypoints.
  """

  @spec t_navigate_path(
          state :: map(),
          entity :: String.t(),
          waypoint_sequence :: [String.t()]
        ) :: [tuple()]
  def t_navigate_path(_state, _entity, []) do
    []
  end

  def t_navigate_path(_state, entity, [waypoint | rest]) do
    [
      {"t_navigate_to", entity, waypoint},
      {"t_navigate_path", entity, rest}
    ]
  end
end

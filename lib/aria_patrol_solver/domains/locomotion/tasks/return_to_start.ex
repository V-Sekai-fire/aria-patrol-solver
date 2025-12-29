# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Tasks.ReturnToStart do
  @moduledoc """
  Task: t_return_to_start(state, entity, start_position_index)

  Return entity to the starting position.
  """

  alias AriaPatrolSolver.Domains.Locomotion.Predicates.QuantizedPosition

  @spec t_return_to_start(
          state :: map(),
          entity :: String.t(),
          start_position_index :: integer()
        ) :: [tuple()]
  def t_return_to_start(state, entity, start_position_index) do
    current_pos = QuantizedPosition.get(state, entity)

    if current_pos == start_position_index do
      # Already at start
      []
    else
      # Move to start position
      [{"c_move_to", entity, start_position_index}]
    end
  end
end

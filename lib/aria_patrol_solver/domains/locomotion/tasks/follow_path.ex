# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Tasks.FollowPath do
  @moduledoc """
  Task: t_follow_path(state, entity, path_indices, final_target_index)

  Local path following: executes a pre-computed path step-by-step.
  This is the low-level navigation that follows the global path.

  Takes a sequence of intermediate position indices and moves through them,
  then moves to the final target.
  """

  @spec t_follow_path(
          state :: map(),
          entity :: String.t(),
          path_indices :: [integer()],
          final_target_index :: integer()
        ) :: [tuple()]
  def t_follow_path(_state, entity, [], final_target_index) do
    # Path complete, move to final target
    [{"c_move_to", entity, final_target_index}]
  end

  def t_follow_path(_state, entity, [next_pos | rest], final_target_index) do
    # Move to next position in path, then continue following
    [
      {"c_move_to", entity, next_pos},
      {"t_follow_path", entity, rest, final_target_index}
    ]
  end
end

# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Tasks.GlobalPathfind do
  @moduledoc """
  Task: t_global_pathfind(state, entity, target_position_index)

  Global path planning: finds a path from current position to target using navmesh.
  This is the high-level pathfinding that computes the route through the maze.

  Returns subtasks to follow the computed path.
  """

  alias AriaPatrolSolver.Domains.Locomotion.{
    MazeGrid,
    Navmesh,
    Predicates.QuantizedPosition
  }

  @spec t_global_pathfind(
          state :: map(),
          entity :: String.t(),
          target_position_index :: integer()
        ) :: [tuple()]
  def t_global_pathfind(state, entity, target_position_index) do
    current_pos_idx = QuantizedPosition.get(state, entity)

    # If already at target, no path needed
    if current_pos_idx == target_position_index do
      []
    else
      # Get 3D positions from indices
      grid = state.position_space
      from_point = MazeGrid.get_point(grid, current_pos_idx)
      to_point = MazeGrid.get_point(grid, target_position_index)

      case {from_point, to_point} do
        {nil, _} ->
          # Can't get points, fallback to direct movement
          [{"c_move_to", entity, target_position_index}]

        {_, nil} ->
          # Can't get points, fallback to direct movement
          [{"c_move_to", entity, target_position_index}]

        {fp, tp} when is_tuple(fp) and is_tuple(tp) ->
          process_navmesh_path(state, fp, tp, grid, entity, target_position_index)
      end
    end
  end

  defp process_navmesh_path(state, from_point, to_point, grid, entity, target_position_index) do
    # Use navmesh to find path (global path planning)
    case find_path_with_navmesh(state, from_point, to_point) do
      {:ok, path_positions} when length(path_positions) > 2 ->
        build_path_subtasks(path_positions, grid, entity, target_position_index)

      {:ok, _path_positions} ->
        # Direct path (from -> to), just move to target
        [{"c_move_to", entity, target_position_index}]

      {:error, _reason} ->
        # Fallback: try direct movement (might fail if blocked)
        [{"c_move_to", entity, target_position_index}]
    end
  end

  defp build_path_subtasks(path_positions, grid, entity, target_position_index) do
    # Convert 3D positions back to position indices
    path_indices = Enum.map(path_positions, fn pos ->
      MazeGrid.find_nearest_index(grid, pos)
    end)
    |> Enum.uniq()  # Remove duplicates
    # Remove first position (current) - keep target as final
    intermediate_path = Enum.drop(path_indices, 1)

    case intermediate_path do
      [] ->
        # No intermediate points, just move to target
        [{"c_move_to", entity, target_position_index}]

      _ ->
        [{"t_follow_path", entity, intermediate_path, target_position_index}]
    end
  end

  defp find_path_with_navmesh(state, from_point, to_point) do
    if state.navmesh do
      Navmesh.find_path(state.navmesh, from_point, to_point)
    else
      # No navmesh, return direct path
      {:ok, [from_point, to_point]}
    end
  end
end

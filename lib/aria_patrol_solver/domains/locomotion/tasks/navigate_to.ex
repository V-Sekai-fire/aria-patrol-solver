# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Tasks.NavigateTo do
  @moduledoc """
  Task: t_navigate_to(state, entity, waypoint)

  Navigate entity to a target waypoint (position + rotation).
  """

  alias AriaPatrolSolver.Domains.Locomotion.Predicates.{
    QuantizedPosition,
    QuantizedRotation,
    WaypointReached
  }

  @spec t_navigate_to(
          state :: map(),
          entity :: String.t(),
          waypoint :: String.t()
        ) :: [tuple()]
  def t_navigate_to(state, entity, waypoint) do
    if WaypointReached.get(state, entity, waypoint) do
      []
    else
      # Find waypoint definition
      waypoint_def = find_waypoint(state, waypoint)

      case waypoint_def do
        nil -> []
        _ -> generate_navigation_subtasks(state, entity, waypoint, waypoint_def)
      end
    end
  end

  defp generate_navigation_subtasks(state, entity, waypoint, waypoint_def) do
    current_pos = QuantizedPosition.get(state, entity)
    current_rot = QuantizedRotation.get(state, entity)
    target_pos = Map.get(waypoint_def, :position_index) || Map.get(waypoint_def, "position_index")
    target_rot = Map.get(waypoint_def, :rotation_index) || Map.get(waypoint_def, "rotation_index")

    cond do
      current_pos == target_pos and current_rot == target_rot ->
        # Already at waypoint, mark as reached
        [{"c_mark_waypoint_reached", entity, waypoint}]

      current_pos == target_pos ->
        # At position but need to rotate, then mark as reached
        [
          {"c_rotate_to", entity, target_rot},
          {"c_mark_waypoint_reached", entity, waypoint}
        ]

      true ->
        # Need movement (and possibly rotation) - use global path planning for movement
        [
          {"t_global_pathfind", entity, target_pos},
          {"c_rotate_to", entity, target_rot},
          {"c_mark_waypoint_reached", entity, waypoint}
        ]
    end
  end

  defp find_waypoint(state, waypoint_id) do
    Enum.find_value(state.waypoints, fn waypoint ->
      id = Map.get(waypoint, :id) || Map.get(waypoint, "id")
      if id == waypoint_id, do: waypoint
    end)
  end
end

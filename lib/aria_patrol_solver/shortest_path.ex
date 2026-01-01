# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.ShortestPath do
  @moduledoc """
  Shortest path optimization using nearest-neighbor heuristic for waypoint patrol.
  """

  alias AriaPatrolSolver.Domains.Locomotion

  @doc """
  Find shortest path through waypoints using nearest-neighbor heuristic.

  ## Parameters
  - `waypoints`: List of waypoint maps with `:id`, `:position_index`, `:rotation_index`
  - `start_pos_idx`: Starting position index
  - `state`: Domain state
  - `start_distances`: Map of waypoint_id => distance from start
  - `distance_matrix`: Map of waypoint_id => %{waypoint_id => distance}

  ## Returns
  - List of waypoint IDs in optimized order
  """
  @spec find_shortest_path(
          waypoints :: [map()],
          start_pos_idx :: integer(),
          state :: map(),
          start_distances :: map(),
          distance_matrix :: map()
        ) :: [String.t()]
  def find_shortest_path(waypoints, _start_pos_idx, _state, start_distances, distance_matrix) do
    # Start with nearest waypoint to start position
    {first_wp_id, _first_dist} = Enum.min_by(start_distances, fn {_id, dist} -> dist end)

    # Build path using nearest-neighbor
    path = [first_wp_id]
    current_wp_id = first_wp_id
    remaining_count = length(waypoints) - 1

    # Greedily select nearest unvisited waypoint
    Enum.reduce(1..remaining_count, {path, current_wp_id}, fn _i, {acc_path, current_id} ->
      select_next_waypoint(waypoints, acc_path, current_id, distance_matrix)
    end)
    |> elem(0)
    |> Enum.reverse()
  end

  defp select_next_waypoint(waypoints, acc_path, current_id, distance_matrix) do
    unvisited = Enum.reject(waypoints, fn wp -> wp.id in acc_path end)

    case unvisited do
      [] ->
        {acc_path, current_id}

      _ ->
        distances_from_current = distance_matrix[current_id]
        {next_wp_id, _next_dist} =
          unvisited
          |> Enum.map(fn wp -> {wp.id, distances_from_current[wp.id]} end)
          |> Enum.min_by(fn {_id, dist} -> dist end)

        {[next_wp_id | acc_path], next_wp_id}
    end
  end

  @doc """
  Calculate total distance of a waypoint sequence including return to start.

  ## Parameters
  - `waypoint_sequence`: List of waypoint IDs in order
  - `start_pos_idx`: Starting position index
  - `state`: Domain state
  - `start_distances`: Map of waypoint_id => distance from start
  - `distance_matrix`: Map of waypoint_id => %{waypoint_id => distance}
  - `waypoints`: List of waypoint maps

  ## Returns
  - Total distance as float
  """
  @spec calculate_total_distance(
          waypoint_sequence :: [String.t()],
          start_pos_idx :: integer(),
          state :: map(),
          start_distances :: map(),
          distance_matrix :: map(),
          waypoints :: [map()]
        ) :: float()
  def calculate_total_distance(
        waypoint_sequence,
        start_pos_idx,
        state,
        start_distances,
        distance_matrix,
        waypoints
      ) do
    # Distance from start to first waypoint
    first_wp_id = List.first(waypoint_sequence)
    total = start_distances[first_wp_id] || 0.0

    # Distances between consecutive waypoints
    total =
      waypoint_sequence
      |> Enum.chunk_every(2, 1, :discard)
      |> Enum.reduce(total, fn [from_id, to_id], acc ->
        dist = distance_matrix[from_id][to_id] || 0.0
        acc + dist
      end)

    # Distance from last waypoint back to start
    last_wp_id = List.last(waypoint_sequence)
    last_wp = Enum.find(waypoints, &(&1.id == last_wp_id))
    return_dist = if last_wp do
      Locomotion.calculate_distance(state, last_wp.position_index, start_pos_idx)
    else
      0.0
    end

    total + return_dist
  end
end

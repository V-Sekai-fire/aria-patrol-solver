# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.CLI do
  @moduledoc """
  Command-line interface for Patrol Solver.
  """

  require Logger
  alias AriaPatrolSolver.Solver

  @doc """
  Main entry point for CLI execution.
  """
  @spec main([String.t()]) :: :ok
  def main(args) do
    Logger.configure(level: :info)

    opts = parse_args(args)

    IO.puts("=" <> String.duplicate("=", 80))
    IO.puts("Scatter Waypoints and Solve Patrol with Aria Planner")
    IO.puts("=" <> String.duplicate("=", 80))

    case Solver.solve(opts) do
      {:ok, result} ->
        print_success(result)

      {:error, reason} ->
        IO.puts("✗ Error: #{reason}")
        System.halt(1)
    end
  end

  defp parse_args(args) do
    {opts, _args, _invalid} =
      OptionParser.parse(args,
        switches: [
          waypoints: :integer,
          sphere_points: :integer,
          speed: :float,
          output: :string,
          maze: :boolean,
          grid_width: :integer,
          grid_height: :integer
        ],
        aliases: [
          w: :waypoints,
          s: :sphere_points,
          v: :speed,
          o: :output,
          m: :maze
        ]
      )

    [
      num_waypoints: Keyword.get(opts, :waypoints, 7),
      maze_mode: Keyword.get(opts, :maze, true),  # Default to maze mode
      grid_width: Keyword.get(opts, :grid_width, 10),
      grid_height: Keyword.get(opts, :grid_height, 10),
      sphere_points: Keyword.get(opts, :sphere_points, 100),
      entity_speed: Keyword.get(opts, :speed, 2.0),
      output_path: Keyword.get(opts, :output, Path.join(System.tmp_dir!(), "blender_trajectory.json"))
    ]
  end

  defp print_success(result) do
    IO.puts("\n" <> String.duplicate("=", 80))
    IO.puts("SUCCESS! Optimized patrol solution ready")
    IO.puts("  Waypoints: #{result.num_waypoints}")
    IO.puts("  Optimized sequence: #{Enum.join(result.optimized_sequence, " -> ")} -> start")
    IO.puts("  Total distance: #{:erlang.float_to_binary(result.total_distance, decimals: 4)} units")
    IO.puts("  Trajectory steps: #{length(result.trajectory)}")
    IO.puts("  Data file: #{result.output_path}")
    IO.puts("  Use trajectory data above to create Blender visualization via MCP")
    IO.puts(String.duplicate("=", 80))

    # Output trajectory JSON to stdout
    trajectory_json_string = Jason.encode!(result.trajectory_json, pretty: true)
    IO.puts("\n" <> String.duplicate("=", 80))
    IO.puts("TRAJECTORY_DATA_START")
    IO.puts(trajectory_json_string)
    IO.puts("TRAJECTORY_DATA_END")
    IO.puts(String.duplicate("=", 80))
  end
end

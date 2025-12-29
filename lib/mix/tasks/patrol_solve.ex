# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule Mix.Tasks.PatrolSolve do
  @moduledoc """
  Mix task to solve patrol waypoint problems.

  ## Examples

      mix patrol_solve
      mix patrol_solve --waypoints 10
      mix patrol_solve -w 10 -s 200 -v 3.0 -o /tmp/patrol.json

  ## Options

    - `--waypoints`, `-w` - Number of waypoints (default: 7)
    - `--maze`, `-m` - Use 2D maze mode (no upward movement) (default: true)
    - `--grid-width` - Grid width for maze mode (default: 10)
    - `--grid-height` - Grid height for maze mode (default: 10)
    - `--sphere-points`, `-s` - Number of Fibonacci sphere points for 3D mode (default: 100)
    - `--speed`, `-v` - Entity movement speed (default: 2.0)
    - `--output`, `-o` - Output path for trajectory JSON (default: /tmp/blender_trajectory.json)
  """
  use Mix.Task

  alias AriaPatrolSolver.CLI

  @shortdoc "Solve patrol waypoint problems with shortest-path optimization"

  @impl Mix.Task
  def run(args) do
    # Ensure dependencies are loaded
    Mix.Task.run("app.start", [])

    # Parse and run
    CLI.main(args)
  end
end

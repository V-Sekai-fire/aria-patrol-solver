# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Application do
  @moduledoc """
  Application supervisor for Patrol Solver.
  """
  use Application
  require Logger

  @impl true
  def start(_type, _args) do
    Logger.info("Starting AriaPatrolSolver.Application...")

    children = []

    opts = [strategy: :one_for_one, name: AriaPatrolSolver.Supervisor]

    case Supervisor.start_link(children, opts) do
      {:ok, pid} ->
        Logger.info("AriaPatrolSolver.Supervisor started")
        {:ok, pid}

      {:error, reason} ->
        Logger.error("Failed to start AriaPatrolSolver.Supervisor: #{inspect(reason)}")
        {:error, reason}
    end
  end
end

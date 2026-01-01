# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.MixProject do
  use Mix.Project

  def project do
    [
      app: :aria_patrol_solver,
      version: "0.1.0",
      elixir: "~> 1.17",
      start_permanent: Mix.env() == :prod,
      deps: deps()
    ]
  end

  def application do
    [
      extra_applications: [:logger],
      mod: {AriaPatrolSolver.Application, []}
    ]
  end

  defp deps do
    [
      {:aria_planner, git: "https://github.com/V-Sekai-fire/aria-planner.git", branch: "main"},
      {:jason, "~> 1.4"}
    ]
  end
end

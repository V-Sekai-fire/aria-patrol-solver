# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.HDDL do
  @moduledoc """
  HDDL file path helpers for aria_patrol_solver.

  Provides path utilities for storing domains and problems as HDDL files.
  Use `AriaPlanner.HDDL` directly for import/export operations.
  """

  @doc """
  Gets the default HDDL directory for storing domains and problems.
  """
  @spec hddl_dir() :: String.t()
  def hddl_dir do
    Path.join([:code.priv_dir(:aria_patrol_solver), "hddl"])
  end

  @doc """
  Gets the domain HDDL file path.
  """
  @spec domain_path(String.t()) :: String.t()
  def domain_path(domain_name) do
    Path.join([hddl_dir(), "domains", "#{domain_name}.hddl"])
  end

  @doc """
  Gets the problem HDDL file path.
  """
  @spec problem_path(String.t()) :: String.t()
  def problem_path(problem_name) do
    Path.join([hddl_dir(), "problems", "#{problem_name}.hddl"])
  end

  @doc """
  Ensures HDDL directories exist.
  """
  @spec ensure_directories() :: :ok
  def ensure_directories do
    dir = hddl_dir()
    File.mkdir_p!(Path.join(dir, "domains"))
    File.mkdir_p!(Path.join(dir, "problems"))
    :ok
  end
end


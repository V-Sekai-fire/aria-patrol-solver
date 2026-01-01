# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.HDDL do
  @moduledoc """
  Converts locomotion domain map to PlanningDomain struct for HDDL export.

  Use `AriaPlanner.HDDL` directly for import/export operations.
  This module only provides the domain conversion logic.
  """

  alias AriaCore.PlanningDomain

  @doc """
  Converts locomotion domain map to PlanningDomain struct.
  """
  @spec to_planning_domain(map()) :: PlanningDomain.t()
  def to_planning_domain(domain) do
    attrs = %{
      id: UUIDv7.generate(),
      domain_type: "navigation",
      name: "locomotion",
      description: "3D/2D Locomotion planning domain with quantization",
      entities: [],
      tasks: extract_tasks(domain),
      actions: extract_actions(domain),
      commands: extract_commands(domain),
      multigoals: [],
      state: :active,
      version: 1,
      metadata: %{}
    }

    case PlanningDomain.create(attrs) do
      {:ok, planning_domain} -> planning_domain
      error -> raise "Failed to create PlanningDomain: #{inspect(error)}"
    end
  end

  defp extract_tasks(domain) do
    methods = Map.get(domain, :methods, [])
    Enum.filter(methods, &(&1.type == "task"))
    |> Enum.map(fn method ->
      %{
        name: method.name,
        arity: method.arity,
        description: Map.get(method, :decomposition, "")
      }
    end)
  end

  defp extract_actions(domain) do
    actions = Map.get(domain, :actions, [])
    Enum.map(actions, fn action ->
      %{
        name: action.name,
        arity: action.arity,
        preconditions: action.preconditions || [],
        effects: action.effects || []
      }
    end)
  end

  defp extract_commands(domain) do
    # Commands are similar to actions but with side effects
    # For now, return empty list - commands are handled separately
    []
  end
end


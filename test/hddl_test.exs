# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.HDDLTest do
  use ExUnit.Case
  doctest AriaPatrolSolver.HDDL

  alias AriaPatrolSolver.Domains.Locomotion
  alias AriaPatrolSolver.HDDL
  alias AriaPlanner.HDDL, as: PlannerHDDL

  describe "HDDL domain loading" do
    test "loads locomotion_3d domain from HDDL file" do
      {:ok, domain} = Locomotion.create_domain(100, false, 10, 10)

      assert domain.type == "navigation" or domain.type == "locomotion"
      assert Map.has_key?(domain, :maze_mode)
      assert domain.maze_mode == false
    end

    test "loads locomotion_maze domain from HDDL file" do
      {:ok, domain} = Locomotion.create_domain(100, true, 10, 10)

      assert domain.type == "navigation" or domain.type == "locomotion"
      assert Map.has_key?(domain, :maze_mode)
      assert domain.maze_mode == true
    end

    test "HDDL domain files exist" do
      domain_3d_path = HDDL.domain_path("locomotion_3d")
      domain_maze_path = HDDL.domain_path("locomotion_maze")

      assert File.exists?(domain_3d_path), "locomotion_3d.hddl should exist"
      assert File.exists?(domain_maze_path), "locomotion_maze.hddl should exist"
    end

    test "can parse HDDL domain files" do
      domain_3d_path = HDDL.domain_path("locomotion_3d")
      domain_maze_path = HDDL.domain_path("locomotion_maze")

      assert {:ok, _domain} = PlannerHDDL.import_from_file(domain_3d_path)
      assert {:ok, _domain} = PlannerHDDL.import_from_file(domain_maze_path)
    end
  end

  describe "HDDL plan export/import" do
    test "exports plan to HDDL file" do
      alias AriaCore.Plan

      plan_attrs = %{
        name: "test_patrol_plan",
        persona_id: "test_persona",
        domain_type: "locomotion",
        objectives: [{"patrol", "entity1", ["wp1", "wp2", "wp3"]}],
        entity_capabilities: %{"entity1" => %{speed: 2.0, movement_type: "walking"}},
        execution_status: "planned"
      }

      {:ok, plan} = Plan.create(plan_attrs)

      path = HDDL.problem_path("test_patrol_plan")
      assert :ok = PlannerHDDL.export_to_file(plan, path)
      assert File.exists?(path)
    end

    test "imports plan from HDDL file" do
      alias AriaCore.Plan

      plan_attrs = %{
        name: "test_import_plan",
        persona_id: "test_persona",
        domain_type: "locomotion",
        objectives: [{"patrol", "entity1", ["wp1", "wp2"]}],
        entity_capabilities: %{"entity1" => %{speed: 2.0, movement_type: "walking"}},
        execution_status: "planned"
      }

      {:ok, original_plan} = Plan.create(plan_attrs)

      # Export
      path = HDDL.problem_path("test_import_plan")
      assert :ok = PlannerHDDL.export_to_file(original_plan, path)

      # Import
      assert {:ok, imported_plan} = PlannerHDDL.import_from_file(path)
      assert %Plan{} = imported_plan
      assert imported_plan.name == original_plan.name
      assert imported_plan.domain_type == original_plan.domain_type
    end

    test "roundtrip: export then import plan" do
      alias AriaCore.Plan

      plan_attrs = %{
        name: "test_roundtrip_plan",
        persona_id: "test_persona",
        domain_type: "locomotion",
        objectives: [{"navigate_to", "entity1", "wp1"}],
        entity_capabilities: %{"entity1" => %{speed: 1.5, movement_type: "running"}},
        execution_status: "planned"
      }

      {:ok, original_plan} = Plan.create(plan_attrs)

      # Export
      path = HDDL.problem_path("test_roundtrip_plan")
      assert :ok = PlannerHDDL.export_to_file(original_plan, path)

      # Import
      assert {:ok, imported_plan} = PlannerHDDL.import_from_file(path)

      # Verify key fields match
      assert imported_plan.name == original_plan.name
      assert imported_plan.domain_type == original_plan.domain_type
      assert imported_plan.persona_id == original_plan.persona_id
    end
  end

  describe "HDDL directory structure" do
    test "HDDL directories exist" do
      HDDL.ensure_directories()

      dir = HDDL.hddl_dir()
      assert File.exists?(dir)
      assert File.exists?(Path.join(dir, "domains"))
      assert File.exists?(Path.join(dir, "problems"))
    end
  end

  describe "domain conversion" do
    test "converts domain map to PlanningDomain struct" do
      alias AriaPatrolSolver.Domains.Locomotion.HDDL, as: LocomotionHDDL

      domain = %{
        type: "locomotion",
        predicates: ["quantized_position", "quantized_rotation"],
        actions: [
          %{name: "a_move_to", arity: 2, preconditions: [], effects: []}
        ],
        methods: [
          %{name: "navigate_to", type: "task", arity: 2, decomposition: "navigate"}
        ],
        goal_methods: []
      }

      planning_domain = LocomotionHDDL.to_planning_domain(domain)

      assert %AriaCore.PlanningDomain{} = planning_domain
      assert planning_domain.domain_type == "navigation"
      assert planning_domain.name == "locomotion"
    end
  end
end


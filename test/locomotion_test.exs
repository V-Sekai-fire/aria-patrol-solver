# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.LocomotionTest do
  use ExUnit.Case, async: true

  alias AriaPatrolSolver.Domains.Locomotion
  alias AriaPatrolSolver.Domains.Locomotion.Commands.{
    MarkWaypointReached,
    MoveAndRotate,
    MoveTo,
    RotateTo
  }
  alias AriaPatrolSolver.Domains.Locomotion.FibonacciSphere
  alias AriaPatrolSolver.Domains.Locomotion.Multigoals.NavigatePath, as: MNavigatePath
  alias AriaPatrolSolver.Domains.Locomotion.Multigoals.NavigateTo, as: MNavigateTo
  alias AriaPatrolSolver.Domains.Locomotion.Predicates.{
    EntitySpeed,
    MovementType,
    QuantizedPosition,
    QuantizedRotation,
    WaypointReached
  }
  alias AriaPatrolSolver.Domains.Locomotion.Quantization
  alias AriaPatrolSolver.Domains.Locomotion.Tasks.{NavigatePath, NavigateTo}

  describe "FibonacciSphere" do
    test "generates sphere with correct number of points" do
      sphere = FibonacciSphere.generate(100)
      assert sphere.n == 100
      assert length(sphere.points) == 100
    end

    test "generates unit vectors" do
      sphere = FibonacciSphere.generate(50)
      {x, y, z} = Enum.at(sphere.points, 0)
      magnitude = :math.sqrt(x * x + y * y + z * z)
      assert abs(magnitude - 1.0) < 0.001
    end

    test "finds nearest index for position" do
      sphere = FibonacciSphere.generate(100)
      position = {1.0, 0.0, 0.0}
      index = FibonacciSphere.find_nearest_index(sphere, position)
      assert index >= 0
      assert index < 100
    end

    test "gets point at index" do
      sphere = FibonacciSphere.generate(50)
      point = FibonacciSphere.get_point(sphere, 0)
      assert is_tuple(point)
      assert tuple_size(point) == 3
    end

    test "normalizes vectors" do
      normalized = FibonacciSphere.normalize({3.0, 4.0, 0.0})
      {x, y, z} = normalized
      magnitude = :math.sqrt(x * x + y * y + z * z)
      assert abs(magnitude - 1.0) < 0.001
    end

    test "handles zero vector normalization" do
      normalized = FibonacciSphere.normalize({0.0, 0.0, 0.0})
      assert normalized == {0.0, 0.0, 1.0}
    end
  end

  describe "Quantization" do
    setup do
      sphere = FibonacciSphere.generate(100)
      %{sphere: sphere}
    end

    test "quantizes position to index", %{sphere: sphere} do
      position = {1.0, 0.0, 0.0}
      index = Quantization.quantize_position(sphere, position)
      assert index >= 0
      assert index < 100
    end

    test "dequantizes position index", %{sphere: sphere} do
      index = 0
      position = Quantization.dequantize_position(sphere, index)
      assert is_tuple(position)
      assert tuple_size(position) == 3
    end

    test "extracts forward direction from quaternion" do
      # Identity quaternion (no rotation) should point forward
      quaternion = {0.0, 0.0, 0.0, 1.0}
      forward = Quantization.quaternion_to_forward(quaternion)
      assert is_tuple(forward)
      assert tuple_size(forward) == 3
    end

    test "quantizes rotation to index", %{sphere: sphere} do
      quaternion = {0.0, 0.0, 0.0, 1.0}
      index = Quantization.quantize_rotation(sphere, quaternion)
      assert index >= 0
      assert index < 100
    end

    test "encodes combined state", %{sphere: sphere} do
      pos_idx = 5
      rot_idx = 10
      state_id = Quantization.encode_combined_state(sphere, pos_idx, rot_idx)
      assert state_id == 5 * 100 + 10
    end

    test "decodes combined state", %{sphere: sphere} do
      state_id = 5 * 100 + 10
      {pos_idx, rot_idx} = Quantization.decode_combined_state(sphere, state_id)
      assert pos_idx == 5
      assert rot_idx == 10
    end
  end

  describe "domain creation" do
    test "creates planning domain with correct structure" do
      {:ok, domain} = Locomotion.create_domain()

      assert domain.type == "locomotion"
      assert "quantized_position" in domain.predicates
      assert "quantized_rotation" in domain.predicates
      assert "entity_speed" in domain.predicates
      assert "movement_type" in domain.predicates
      assert "waypoint_reached" in domain.predicates
      assert length(domain.actions) >= 4
    end

    test "domain has required actions" do
      {:ok, domain} = Locomotion.create_domain()
      action_names = Enum.map(domain.actions, & &1.name)

      assert "a_move_to" in action_names
      assert "a_rotate_to" in action_names
      assert "a_move_and_rotate" in action_names
      assert "a_mark_waypoint_reached" in action_names
    end

    test "domain has task methods" do
      {:ok, domain} = Locomotion.create_domain()
      method_names = Enum.map(domain.methods, & &1.name)

      assert "navigate_to" in method_names
      assert "navigate_path" in method_names
    end

    test "domain has goal methods" do
      {:ok, domain} = Locomotion.create_domain()
      goal_method_names = Enum.map(domain.goal_methods, & &1.name)

      assert "navigate_to" in goal_method_names
      assert "navigate_path" in goal_method_names
    end
  end

  describe "state initialization" do
    test "initializes state with entities and waypoints" do
      {:ok, state} = Locomotion.initialize_state(%{
        entities: [
          %{
            id: "entity1",
            position: {1.0, 0.0, 0.0},
            rotation: {0.0, 0.0, 0.0, 1.0},
            speed: 2.0,
            movement_type: "walking"
          }
        ],
        waypoints: [
          %{
            id: "waypoint1",
            position_index: 10,
            rotation_index: 20
          }
        ],
        sphere_points: 100
      })

      assert state.fibonacci_sphere.n == 100
      assert Map.has_key?(state.quantized_position, "entity1")
      assert Map.has_key?(state.quantized_rotation, "entity1")
      assert Map.has_key?(state.entity_speed, "entity1")
      assert Map.has_key?(state.movement_type, "entity1")
      assert length(state.waypoints) == 1
    end

    test "quantizes initial positions and rotations" do
      {:ok, state} = Locomotion.initialize_state(%{
        entities: [
          %{
            id: "entity1",
            position: {1.0, 0.0, 0.0},
            rotation: {0.0, 0.0, 0.0, 1.0}
          }
        ],
        sphere_points: 50
      })

      pos_idx = QuantizedPosition.get(state, "entity1")
      rot_idx = QuantizedRotation.get(state, "entity1")
      assert pos_idx >= 0
      assert pos_idx < 50
      assert rot_idx >= 0
      assert rot_idx < 50
    end
  end

  describe "predicates" do
    setup do
      {:ok, state} = Locomotion.initialize_state(%{
        entities: [
          %{
            id: "entity1",
            position: {0.0, 0.0, 0.0},
            rotation: {0.0, 0.0, 0.0, 1.0},
            speed: 2.0,
            movement_type: "walking"
          }
        ],
        sphere_points: 100
      })

      %{state: state}
    end

    test "QuantizedPosition get/set", %{state: state} do
      assert QuantizedPosition.get(state, "entity1") >= 0
      new_state = QuantizedPosition.set(state, "entity1", 50)
      assert QuantizedPosition.get(new_state, "entity1") == 50
    end

    test "QuantizedRotation get/set", %{state: state} do
      assert QuantizedRotation.get(state, "entity1") >= 0
      new_state = QuantizedRotation.set(state, "entity1", 30)
      assert QuantizedRotation.get(new_state, "entity1") == 30
    end

    test "EntitySpeed get/set", %{state: state} do
      assert EntitySpeed.get(state, "entity1") == 2.0
      new_state = EntitySpeed.set(state, "entity1", 3.0)
      assert EntitySpeed.get(new_state, "entity1") == 3.0
    end

    test "MovementType get/set", %{state: state} do
      assert MovementType.get(state, "entity1") == "walking"
      new_state = MovementType.set(state, "entity1", "running")
      assert MovementType.get(new_state, "entity1") == "running"
    end

    test "WaypointReached get/set", %{state: state} do
      assert WaypointReached.get(state, "entity1", "waypoint1") == false
      new_state = WaypointReached.set(state, "entity1", "waypoint1", true)
      assert WaypointReached.get(new_state, "entity1", "waypoint1") == true
    end
  end

  describe "commands" do
    setup do
      {:ok, state} = Locomotion.initialize_state(%{
        entities: [
          %{
            id: "entity1",
            position: {1.0, 0.0, 0.0},
            rotation: {0.0, 0.0, 0.0, 1.0},
            speed: 2.0,
            movement_type: "walking"
          }
        ],
        sphere_points: 100
      })

      %{state: state}
    end

    test "c_move_to moves entity to target position", %{state: state} do
      target_pos = 50
      {:ok, new_state} = MoveTo.c_move_to(state, "entity1", target_pos)

      assert QuantizedPosition.get(new_state, "entity1") == target_pos
      assert Map.has_key?(new_state.planner_metadata, :last_action_duration)
    end

    test "c_move_to validates entity exists", %{state: state} do
      {:error, msg} = MoveTo.c_move_to(state, "nonexistent", 10)
      assert String.contains?(msg, "does not exist")
    end

    test "c_move_to validates movement type", %{state: state} do
      # Set invalid movement type
      state = MovementType.set(state, "entity1", "jumping")
      {:error, msg} = MoveTo.c_move_to(state, "entity1", 10)
      assert String.contains?(msg, "does not allow movement")
    end

    test "c_rotate_to rotates entity", %{state: state} do
      target_rot = 30
      {:ok, new_state} = RotateTo.c_rotate_to(state, "entity1", target_rot)

      assert QuantizedRotation.get(new_state, "entity1") == target_rot
      assert Map.has_key?(new_state.planner_metadata, :last_action_duration)
    end

    test "c_move_and_rotate moves and rotates simultaneously", %{state: state} do
      pos_idx = 50
      rot_idx = 30
      {:ok, new_state} = MoveAndRotate.c_move_and_rotate(state, "entity1", pos_idx, rot_idx)

      assert QuantizedPosition.get(new_state, "entity1") == pos_idx
      assert QuantizedRotation.get(new_state, "entity1") == rot_idx
    end

    test "c_mark_waypoint_reached marks waypoint", %{state: state} do
      state = Map.put(state, :waypoints, [
        %{id: "wp1", position_index: 10, rotation_index: 20}
      ])
      state = Map.put(state, :waypoint_reached, %{})

      {:ok, new_state} = MarkWaypointReached.c_mark_waypoint_reached(state, "entity1", "wp1")
      assert WaypointReached.get(new_state, "entity1", "wp1") == true
    end
  end

  describe "tasks" do
    setup do
      {:ok, state} = Locomotion.initialize_state(%{
        entities: [
          %{
            id: "entity1",
            position: {1.0, 0.0, 0.0},
            rotation: {0.0, 0.0, 0.0, 1.0},
            speed: 2.0,
            movement_type: "walking"
          }
        ],
        waypoints: [
          %{
            id: "wp1",
            position_index: 10,
            rotation_index: 20
          }
        ],
        sphere_points: 100
      })

      %{state: state}
    end

    test "t_navigate_to generates subtasks when not at waypoint", %{state: state} do
      subtasks = NavigateTo.t_navigate_to(state, "entity1", "wp1")
      assert is_list(subtasks)
      assert subtasks != []
    end

    test "t_navigate_to returns empty when waypoint already reached", %{state: state} do
      state = WaypointReached.set(state, "entity1", "wp1", true)
      subtasks = NavigateTo.t_navigate_to(state, "entity1", "wp1")
      assert subtasks == []
    end

    test "t_navigate_path generates path subtasks", %{state: state} do
      waypoint_sequence = ["wp1"]
      subtasks = NavigatePath.t_navigate_path(state, "entity1", waypoint_sequence)
      assert is_list(subtasks)
    end

    test "t_navigate_path returns empty for empty sequence", %{state: state} do
      subtasks = NavigatePath.t_navigate_path(state, "entity1", [])
      assert subtasks == []
    end
  end

  describe "multigoals" do
    setup do
      {:ok, state} = Locomotion.initialize_state(%{
        entities: [
          %{
            id: "entity1",
            position: {1.0, 0.0, 0.0},
            rotation: {0.0, 0.0, 0.0, 1.0},
            speed: 2.0,
            movement_type: "walking"
          }
        ],
        waypoints: [
          %{
            id: "wp1",
            position_index: 10,
            rotation_index: 20
          }
        ],
        sphere_points: 100
      })

      %{state: state}
    end

    test "m_navigate_to generates goal subtasks", %{state: state} do
      goals = MNavigateTo.m_navigate_to(state, "entity1", "wp1")
      assert is_list(goals)
      assert goals != []
    end

    test "m_navigate_to returns empty when waypoint reached", %{state: state} do
      state = WaypointReached.set(state, "entity1", "wp1", true)
      goals = MNavigateTo.m_navigate_to(state, "entity1", "wp1")
      assert goals == []
    end

    test "m_navigate_path generates path goals", %{state: state} do
      waypoint_sequence = ["wp1"]
      goals = MNavigatePath.m_navigate_path(state, "entity1", waypoint_sequence)
      assert is_list(goals)
    end
  end

  describe "distance and duration calculations" do
    test "calculate_distance computes distance between indices" do
      {:ok, state} = Locomotion.initialize_state(%{
        entities: [],
        sphere_points: 100
      })

      distance = Locomotion.calculate_distance(state, 0, 1)
      assert is_float(distance)
      assert distance >= 0.0
    end

    test "calculate_duration computes duration from distance and speed" do
      duration = Locomotion.calculate_duration(10.0, 2.0)
      assert duration == 5.0
    end

    test "calculate_duration returns infinity for zero speed" do
      assert Locomotion.calculate_duration(10.0, 0.0) == :infinity
    end
  end
end

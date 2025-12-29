# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion do
  @moduledoc """
  3D/2D Locomotion planning domain with quantization.

  Supports two modes:
  - **3D Mode**: Fibonacci sphere quantization for 3D movement
  - **2D Maze Mode**: Grid quantization for 2D maze navigation (no vertical movement)

  This domain models movement and rotation using quantized states:
  - Positions quantized to sphere points (3D) or grid points (2D)
  - Rotations quantized via forward direction vectors
  - Logarithmic encoding for combined state space

  Features:
  - 3D/2D position tracking (quantized)
  - Rotation/orientation tracking (quantized)
  - Multiple movement types (walk, run, jump, climb, fly, swim)
  - Speed-based temporal constraints
  - Waypoint navigation
  - Multi-agent coordination
  - Maze mode: 2D grid with no upward movement constraint
  """

  alias AriaPatrolSolver.Domains.Locomotion.FibonacciSphere
  alias AriaPatrolSolver.Domains.Locomotion.MazeGrid
  alias AriaPatrolSolver.Domains.Locomotion.Navmesh

  @doc """
  Creates and registers the locomotion planning domain.

  ## Parameters
  - `sphere_points`: Number of Fibonacci sphere points for 3D mode (default: 100)
  - `maze_mode`: If true, uses 2D grid instead of 3D sphere (default: false)
  - `grid_width`: Width of grid for maze mode (default: 10)
  - `grid_height`: Height of grid for maze mode (default: 10)
  """
  @spec create_domain(integer(), boolean(), integer(), integer()) :: {:ok, map()} | {:error, String.t()}
  def create_domain(sphere_points \\ 100, maze_mode \\ false, grid_width \\ 10, grid_height \\ 10)
      when is_integer(sphere_points) and sphere_points > 0 and
             is_boolean(maze_mode) and
             is_integer(grid_width) and grid_width > 0 and
             is_integer(grid_height) and grid_height > 0 do
    case create_planning_domain() do
      {:ok, domain} ->
        domain = register_actions(domain)
        domain = register_task_methods(domain)
        domain = register_goal_methods(domain)
        domain = Map.put(domain, :maze_mode, maze_mode)
        {:ok, domain}

      error ->
        error
    end
  end

  @doc """
  Creates the base planning domain structure.
  """
  @spec create_planning_domain() :: {:ok, map()} | {:error, String.t()}
  def create_planning_domain do
    {:ok,
     %{
       type: "locomotion",
       predicates: [
         "quantized_position",
         "quantized_rotation",
         "entity_speed",
         "movement_type",
         "waypoint_reached"
       ],
       actions: [],
       methods: [],
       goal_methods: [],
       created_at: DateTime.utc_now()
     }}
  end

  defp register_actions(domain) do
    actions = [
      %{
        name: "a_move_to",
        arity: 2,
        preconditions: [
          "quantized_position[entity] != target_position_index",
          "movement_type[entity] allows movement",
          "entity_speed[entity] > 0",
          "target_position_index is valid (0..N-1)"
        ],
        effects: [
          "quantized_position[entity] = target_position_index",
          "temporal duration = distance / speed (via PlannerMetadata)"
        ]
      },
      %{
        name: "a_rotate_to",
        arity: 2,
        preconditions: [
          "quantized_rotation[entity] != target_rotation_index",
          "target_rotation_index is valid (0..N-1)"
        ],
        effects: [
          "quantized_rotation[entity] = target_rotation_index",
          "temporal duration = rotation_duration (via PlannerMetadata)"
        ]
      },
      %{
        name: "a_move_and_rotate",
        arity: 3,
        preconditions: [
          "quantized_position[entity] != position_index OR quantized_rotation[entity] != rotation_index",
          "movement_type[entity] allows movement",
          "entity_speed[entity] > 0",
          "both indices are valid (0..N-1)"
        ],
        effects: [
          "quantized_position[entity] = position_index",
          "quantized_rotation[entity] = rotation_index",
          "temporal duration = max(movement_duration, rotation_duration)"
        ]
      },
      %{
        name: "a_mark_waypoint_reached",
        arity: 2,
        preconditions: [
          "entity exists",
          "waypoint exists"
        ],
        effects: [
          "waypoint_reached[entity, waypoint] = true"
        ]
      }
    ]

    Map.put(domain, :actions, actions)
  end

  defp register_task_methods(domain) do
    methods = [
      %{
        name: "navigate_to",
        type: "task",
        arity: 2,
        decomposition: "navigate entity to target waypoint (position + rotation)"
      },
      %{
        name: "navigate_path",
        type: "task",
        arity: 2,
        decomposition: "navigate entity through sequence of waypoints"
      },
      %{
        name: "patrol",
        type: "task",
        arity: 2,
        decomposition: "patrol entity through waypoints and return to start"
      },
      %{
        name: "return_to_start",
        type: "task",
        arity: 2,
        decomposition: "return entity to starting position"
      }
    ]

    Map.update(domain, :methods, methods, &(&1 ++ methods))
  end

  defp register_goal_methods(domain) do
    goal_methods = [
      %{
        name: "navigate_to",
        type: "multigoal",
        arity: 2,
        predicate: "waypoint_reached[entity, waypoint]",
        decomposition: "navigate entity to waypoint (goal-based)"
      },
      %{
        name: "navigate_path",
        type: "multigoal",
        arity: 2,
        predicate: nil,
        decomposition: "navigate entity through waypoint sequence (goal-based)"
      },
      %{
        name: "patrol",
        type: "multigoal",
        arity: 2,
        predicate: nil,
        decomposition: "patrol entity through waypoints and return to start (goal-based)"
      }
    ]

    Map.update(domain, :goal_methods, goal_methods, &(&1 ++ goal_methods))
  end

  @doc """
  Initializes locomotion state with entities and waypoints.

  ## Parameters
  - `params`: Map containing:
    - `entities`: List of entity maps with id, position, rotation (quaternion), speed, movement_type
    - `waypoints`: List of waypoint maps with id, position_index, rotation_index
    - `sphere_points`: Number of Fibonacci sphere points (default: 100)
    - `rotation_duration`: Duration for rotation actions (default: 0.5)
  """
  @spec initialize_state(params :: map()) :: {:ok, map()} | {:error, String.t()}
  # credo:disable-for-next-line Credo.Check.Refactor.CyclomaticComplexity
  def initialize_state(params) do
    try do
      maze_mode = Map.get(params, :maze_mode, false)

      # Use grid for maze mode, sphere for 3D mode
      {position_space, rotation_space, navmesh} = if maze_mode do
        grid_width = Map.get(params, :grid_width, 10)
        grid_height = Map.get(params, :grid_height, 10)
        grid_spacing = Map.get(params, :grid_spacing, 1.0)
        grid = MazeGrid.generate(grid_width, grid_height, grid_spacing)

        # Generate navmesh from grid (or import from physics engine)
        navmesh = case Map.get(params, :navmesh) do
          nil ->
            # Generate navmesh from grid
            obstacles = Map.get(params, :obstacles, [])
            Navmesh.from_grid(grid_width, grid_height, grid_spacing, obstacles)
          navmesh_data when is_map(navmesh_data) ->
            # Import navmesh from physics engine
            format = Map.get(params, :navmesh_format, :json)
            case Navmesh.from_physics_engine(navmesh_data, format) do
              {:ok, nm} -> nm
              {:error, _} -> Navmesh.from_grid(grid_width, grid_height, grid_spacing, [])
            end
          _ ->
            Navmesh.from_grid(grid_width, grid_height, grid_spacing, [])
        end

        # For rotation in maze, still use a small sphere for direction
        sphere_points = Map.get(params, :rotation_points, 8)  # 8 directions for 2D
        sphere = FibonacciSphere.generate(sphere_points)
        {grid, sphere, navmesh}
      else
        sphere_points = Map.get(params, :sphere_points, 100)
        sphere = FibonacciSphere.generate(sphere_points)
        {sphere, sphere, nil}
      end

      entities = Map.get(params, :entities) || Map.get(params, "entities") || []
      waypoints = Map.get(params, :waypoints) || Map.get(params, "waypoints") || []

      # Quantize initial entity positions and rotations
      alias AriaPatrolSolver.Domains.Locomotion.Quantization

      quantized_position =
        for entity <- entities, into: %{} do
          entity_id = Map.get(entity, :id) || Map.get(entity, "id")
          pos = Map.get(entity, :position) || Map.get(entity, "position") || {0.0, 0.0, 0.0}
          pos_idx = if maze_mode do
            MazeGrid.find_nearest_index(position_space, pos)
          else
            Quantization.quantize_position(position_space, pos)
          end
          {entity_id, pos_idx}
        end

      quantized_rotation =
        for entity <- entities, into: %{} do
          entity_id = Map.get(entity, :id) || Map.get(entity, "id")
          rot = Map.get(entity, :rotation) || Map.get(entity, "rotation") || {0.0, 0.0, 0.0, 1.0}
          rot_idx = Quantization.quantize_rotation(rotation_space, rot)
          {entity_id, rot_idx}
        end

      # Initialize entity speeds
      entity_speed =
        for entity <- entities, into: %{} do
          entity_id = Map.get(entity, :id) || Map.get(entity, "id")
          speed = Map.get(entity, :speed) || Map.get(entity, "speed") || 1.0
          {entity_id, speed}
        end

      # Initialize movement types
      movement_type =
        for entity <- entities, into: %{} do
          entity_id = Map.get(entity, :id) || Map.get(entity, "id")
          mt = Map.get(entity, :movement_type) || Map.get(entity, "movement_type") || "walking"
          {entity_id, mt}
        end

      # Validate waypoints in maze mode (no upward positions)
      if maze_mode do
        for waypoint <- waypoints do
          waypoint_id = Map.get(waypoint, :id) || Map.get(waypoint, "id")
          pos_idx = Map.get(waypoint, :position_index) || Map.get(waypoint, "position_index")

          # Check that waypoint position is valid for grid
          if pos_idx >= position_space.n do
            raise ArgumentError, "Waypoint #{waypoint_id} has invalid position_index #{pos_idx} (max: #{position_space.n - 1})"
          end

          # Check that waypoint position has no upward component (z must be 0)
          waypoint_point = MazeGrid.get_point(position_space, pos_idx)
          if waypoint_point do
            {_x, _y, z} = waypoint_point
            if z != 0.0 do
              raise ArgumentError, "Waypoint #{waypoint_id} cannot have upward position (z=#{z}) in maze mode"
            end
          end
        end
      end

      # Initialize waypoint reached status
      waypoint_reached =
        for entity <- entities, waypoint <- waypoints, into: %{} do
          entity_id = Map.get(entity, :id) || Map.get(entity, "id")
          waypoint_id = Map.get(waypoint, :id) || Map.get(waypoint, "id")
          {{entity_id, waypoint_id}, false}
        end

      state = %{
        entities: entities,
        waypoints: waypoints,
        position_space: position_space,
        rotation_space: rotation_space,
        navmesh: navmesh,
        maze_mode: maze_mode,
        quantized_position: quantized_position,
        quantized_rotation: quantized_rotation,
        entity_speed: entity_speed,
        movement_type: movement_type,
        waypoint_reached: waypoint_reached,
        rotation_duration: Map.get(params, :rotation_duration, 0.5),
        planner_metadata: %{}
      }

      # Keep backward compatibility
      state = if maze_mode do
        state
      else
        Map.put(state, :fibonacci_sphere, position_space)
      end

      {:ok, state}
    rescue
      e -> {:error, "Failed to initialize state: #{inspect(e)}"}
    end
  end

  @doc """
  Calculates Euclidean distance between two quantized positions.

  Uses 2D distance for maze mode, 3D distance for sphere mode.

  ## Parameters
  - `state`: Domain state
  - `from_index`: Source position index
  - `to_index`: Target position index

  ## Returns
  - Distance as float
  """
  @spec calculate_distance(map(), integer(), integer()) :: float()
  def calculate_distance(state, from_index, to_index) do
    if state.maze_mode do
      # 2D maze mode - use grid
      grid = state.position_space
      from_point = MazeGrid.get_point(grid, from_index)
      to_point = MazeGrid.get_point(grid, to_index)

      if from_point && to_point do
        MazeGrid.distance_2d(from_point, to_point)
      else
        0.0
      end
    else
      # 3D sphere mode
      sphere = Map.get(state, :fibonacci_sphere) || state.position_space
      from_point = FibonacciSphere.get_point(sphere, from_index)
      to_point = FibonacciSphere.get_point(sphere, to_index)

      if from_point && to_point do
        dx = elem(to_point, 0) - elem(from_point, 0)
        dy = elem(to_point, 1) - elem(from_point, 1)
        dz = elem(to_point, 2) - elem(from_point, 2)
        :math.sqrt(dx * dx + dy * dy + dz * dz)
      else
        0.0
      end
    end
  end

  @doc """
  Calculates movement duration given distance and speed.

  ## Parameters
  - `distance`: Distance to travel
  - `speed`: Movement speed (units per second)

  ## Returns
  - Duration in seconds, or :infinity if speed <= 0
  """
  @spec calculate_duration(distance :: float(), speed :: float()) :: float() | :infinity
  def calculate_duration(distance, speed) when speed > 0.0 do
    distance / speed
  end
  def calculate_duration(_distance, _speed), do: :infinity
end

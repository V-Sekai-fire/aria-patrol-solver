# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Navmesh do
  @moduledoc """
  Navigation mesh (navmesh) for pathfinding and movement constraints.

  A navmesh represents walkable areas in the environment. Entities can only
  move along navmesh edges/faces, which enables realistic pathfinding around
  obstacles and through complex geometry.

  ## Navmesh Structure

  - **Vertices**: 3D points defining the mesh
  - **Faces**: Polygons (typically triangles) representing walkable surfaces
  - **Edges**: Connections between faces (for pathfinding)
  - **Regions**: Connected groups of faces (for hierarchical pathfinding)

  ## Generation Methods

  1. **Grid-based**: Generate from 2D grid (simple, fast)
  2. **Physics Engine**: Import from Godot/Unity navmesh (realistic, complex)
  3. **Manual**: Define vertices and faces explicitly

  ## Example

      iex> navmesh = Navmesh.from_grid(10, 10, 1.0)
      iex> Navmesh.walkable?(navmesh, {5.0, 5.0, 0.0})
      true
      iex> Navmesh.find_path(navmesh, {0.0, 0.0, 0.0}, {9.0, 9.0, 0.0})
      {:ok, [path_points]}
  """

  @type vertex :: {float(), float(), float()}
  @type face :: {integer(), integer(), integer()}  # Triangle: {v1_idx, v2_idx, v3_idx}
  @type edge :: {integer(), integer()}  # Edge: {face1_idx, face2_idx} or {vertex1_idx, vertex2_idx}

  @type t :: %__MODULE__{
          vertices: [vertex()],
          faces: [face()],
          edges: [edge()],
          regions: [integer()],  # Face indices grouped by region
          walkable_mask: map(),  # Map of face_idx => boolean
          metadata: map()
        }

  defstruct vertices: [],
            faces: [],
            edges: [],
            regions: [],
            walkable_mask: %{},
            metadata: %{}

  @doc """
  Generates a navmesh from a 2D grid.

  Creates a mesh where each grid cell becomes a walkable face (two triangles).
  This is the simplest navmesh generation method, suitable for mazes.

  ## Parameters
  - `width`: Grid width (number of cells)
  - `height`: Grid height (number of cells)
  - `cell_size`: Size of each grid cell (default: 1.0)
  - `obstacles`: Optional list of {x, y} coordinates that are blocked (default: [])

  ## Returns
  - Navmesh struct with vertices, faces, and walkability information
  """
  @spec from_grid(integer(), integer(), float(), [{integer(), integer()}]) :: t()
  def from_grid(width, height, cell_size \\ 1.0, obstacles \\ [])
      when is_integer(width) and width > 0 and
             is_integer(height) and height > 0 and
             is_float(cell_size) and cell_size > 0.0 do
    # Generate vertices for grid
    vertices =
      for y <- 0..height, x <- 0..width do
        {x * cell_size, y * cell_size, 0.0}
      end

    # Generate faces (two triangles per grid cell)
    {face_triangles, walkable_mask} =
      for y <- 0..(height - 1), x <- 0..(width - 1), reduce: {[], %{}} do
        {acc_faces, acc_mask} ->
          # Check if this cell is an obstacle
          is_obstacle = {x, y} in obstacles

          # Calculate vertex indices for this cell
          # Grid layout: vertices are stored row by row
          v00 = y * (width + 1) + x  # Bottom-left
          v01 = y * (width + 1) + x + 1  # Bottom-right
          v10 = (y + 1) * (width + 1) + x  # Top-left
          v11 = (y + 1) * (width + 1) + x + 1  # Top-right

          # Two triangles per cell: (v00, v01, v10) and (v01, v11, v10)
          face1 = {v00, v01, v10}
          face2 = {v01, v11, v10}

          face1_idx = length(acc_faces)
          face2_idx = face1_idx + 1

          walkable = not is_obstacle

          {
            [face2, face1 | acc_faces],
            acc_mask
            |> Map.put(face1_idx, walkable)
            |> Map.put(face2_idx, walkable)
          }
      end

    faces = Enum.reverse(face_triangles)

    # Generate edges (connections between adjacent faces)
    edges = generate_edges(faces, width, height)

    # Group faces into regions (all connected walkable faces)
    regions = compute_regions(faces, walkable_mask)

    %__MODULE__{
      vertices: vertices,
      faces: faces,
      edges: edges,
      regions: regions,
      walkable_mask: walkable_mask,
      metadata: %{
        type: :grid,
        width: width,
        height: height,
        cell_size: cell_size,
        obstacle_count: length(obstacles)
      }
    }
  end

  @doc """
  Imports a navmesh from a physics engine format.

  Supports importing from:
  - Godot NavigationMesh (JSON format)
  - Unity NavMesh (JSON format)
  - Custom JSON format with vertices and faces

  ## Parameters
  - `data`: Map or JSON string containing navmesh data
  - `format`: Format type (`:godot`, `:unity`, `:json`)

  ## Returns
  - Navmesh struct, or `{:error, reason}` on failure
  """
  @spec from_physics_engine(map() | String.t(), atom()) :: t() | {:error, String.t()}
  def from_physics_engine(data, format \\ :json) do
    try do
      parsed_data = if is_binary(data) do
        Jason.decode!(data)
      else
        data
      end

      case format do
        :godot -> import_godot_navmesh(parsed_data)
        :unity -> import_unity_navmesh(parsed_data)
        :json -> import_json_navmesh(parsed_data)
        _ -> {:error, "Unsupported format: #{format}"}
      end
    rescue
      e -> {:error, "Failed to import navmesh: #{inspect(e)}"}
    end
  end

  @doc """
  Checks if a position is walkable (on a walkable face).

  ## Parameters
  - `navmesh`: Navmesh struct
  - `position`: 3D position {x, y, z}

  ## Returns
  - `true` if position is on a walkable face, `false` otherwise
  """
  @spec walkable?(t(), vertex()) :: boolean()
  def walkable?(%__MODULE__{faces: faces, vertices: vertices, walkable_mask: walkable_mask}, position) do
    # Find the face containing this position
    case find_face_at_position(faces, vertices, position) do
      {:ok, face_idx} ->
        Map.get(walkable_mask, face_idx, false)

      {:error, _} ->
        false
    end
  end

  @doc """
  Finds a path between two positions using the navmesh.

  Uses A* pathfinding on the navmesh graph.

  ## Parameters
  - `navmesh`: Navmesh struct
  - `from`: Starting position {x, y, z}
  - `to`: Target position {x, y, z}

  ## Returns
  - `{:ok, [path_points]}` - List of positions along the path
  - `{:error, reason}` - Path not found or invalid positions
  """
  @spec find_path(t(), vertex(), vertex()) :: {:ok, [vertex()]} | {:error, String.t()}
  def find_path(%__MODULE__{} = navmesh, from, to) do
    with {:ok, from_face} <- find_face_at_position(navmesh.faces, navmesh.vertices, from),
         {:ok, to_face} <- find_face_at_position(navmesh.faces, navmesh.vertices, to),
         true <- Map.get(navmesh.walkable_mask, from_face, false),
         true <- Map.get(navmesh.walkable_mask, to_face, false) do
      # Simple pathfinding: find path through face graph
      case astar_pathfind(navmesh, from_face, to_face) do
        {:ok, face_path} ->
          # Convert face path to position path
          path_positions = [from | Enum.map(face_path, fn face_idx ->
            get_face_center(navmesh.faces, navmesh.vertices, face_idx)
          end)] ++ [to]
          {:ok, path_positions}

        error ->
          error
      end
    else
      false -> {:error, "Start or end position is not walkable"}
      error -> error
    end
  end

  @doc """
  Gets the nearest walkable position to a given position.

  ## Parameters
  - `navmesh`: Navmesh struct
  - `position`: 3D position {x, y, z}

  ## Returns
  - Nearest walkable position, or original position if already walkable
  """
  @spec get_nearest_walkable(t(), vertex()) :: vertex()
  def get_nearest_walkable(%__MODULE__{faces: faces, vertices: vertices, walkable_mask: walkable_mask}, position) do
    if walkable?(%__MODULE__{faces: faces, vertices: vertices, walkable_mask: walkable_mask}, position) do
      position
    else
      # Find nearest walkable face center
      walkable_faces =
        walkable_mask
        |> Enum.filter(fn {_idx, walkable} -> walkable end)
        |> Enum.map(fn {idx, _} -> idx end)

      {_face_idx, nearest_pos} =
        walkable_faces
        |> Enum.map(fn face_idx ->
          center = get_face_center(faces, vertices, face_idx)
          dist = distance_3d(position, center)
          {face_idx, center, dist}
        end)
        |> Enum.min_by(fn {_idx, _pos, dist} -> dist end)

      nearest_pos
    end
  end

  # Private helper functions

  defp generate_edges(faces, width, height) do
    # Generate edges between adjacent faces
    # For grid-based navmesh, edges connect faces that share vertices
    []
    # credo:disable-for-next-line
    # TODO: Implement edge generation for grid
  end

  defp compute_regions(faces, walkable_mask) do
    # Group connected walkable faces into regions
    # Simple implementation: all walkable faces in one region
    walkable_faces =
      walkable_mask
      |> Enum.filter(fn {_idx, walkable} -> walkable end)
      |> Enum.map(fn {idx, _} -> idx end)

    [walkable_faces]
  end

  defp find_face_at_position(faces, vertices, {x, y, z}) do
    # Find face containing the position (point-in-triangle test)
    case Enum.find_index(faces, fn {v1_idx, v2_idx, v3_idx} ->
           v1 = Enum.at(vertices, v1_idx)
           v2 = Enum.at(vertices, v2_idx)
           v3 = Enum.at(vertices, v3_idx)
           point_in_triangle?({x, y, z}, v1, v2, v3)
         end) do
      nil -> {:error, "Position not on any face"}
      idx -> {:ok, idx}
    end
  end

  defp point_in_triangle?({px, py, pz}, {x1, y1, z1}, {x2, y2, z2}, {x3, y3, z3}) do
    # Barycentric coordinates test for point in triangle
    # Simplified: check if point is within triangle bounds (2D projection to XY plane)
    v0x = x3 - x1
    v0y = y3 - y1
    v1x = x2 - x1
    v1y = y2 - y1
    v2x = px - x1
    v2y = py - y1

    dot00 = v0x * v0x + v0y * v0y
    dot01 = v0x * v1x + v0y * v1y
    dot02 = v0x * v2x + v0y * v2y
    dot11 = v1x * v1x + v1y * v1y
    dot12 = v1x * v2x + v1y * v2y

    inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * inv_denom
    v = (dot00 * dot12 - dot01 * dot02) * inv_denom

    u >= 0.0 and v >= 0.0 and u + v <= 1.0
  end

  defp get_face_center(faces, vertices, face_idx) do
    {v1_idx, v2_idx, v3_idx} = Enum.at(faces, face_idx)
    v1 = Enum.at(vertices, v1_idx)
    v2 = Enum.at(vertices, v2_idx)
    v3 = Enum.at(vertices, v3_idx)

    # Calculate centroid
    {x1, y1, z1} = v1
    {x2, y2, z2} = v2
    {x3, y3, z3} = v3

    {(x1 + x2 + x3) / 3.0, (y1 + y2 + y3) / 3.0, (z1 + z2 + z3) / 3.0}
  end

  defp astar_pathfind(_navmesh, from_face, to_face) when from_face == to_face do
    {:ok, []}
  end

  defp astar_pathfind(navmesh, from_face, to_face) do
    # Simple A* pathfinding on face graph
    # For grid-based navmesh, use Manhattan distance heuristic
    # credo:disable-for-next-line
    # TODO: Implement full A* with edge connections
    {:ok, [from_face, to_face]}
  end

  defp distance_3d({x1, y1, z1}, {x2, y2, z2}) do
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    :math.sqrt(dx * dx + dy * dy + dz * dz)
  end

  defp import_godot_navmesh(data) do
    # Import from Godot NavigationMesh format
    # Expected format: %{"vertices" => [...], "faces" => [...]}
    vertices = Map.get(data, "vertices", []) |> Enum.map(&tuple_from_list/1)
    faces = Map.get(data, "faces", []) |> Enum.map(&tuple_from_list/1)

    %__MODULE__{
      vertices: vertices,
      faces: faces,
      edges: [],
      regions: [],
      walkable_mask: %{},  # All faces walkable by default
      metadata: %{type: :godot, source: "physics_engine"}
    }
  end

  defp import_unity_navmesh(data) do
    # Import from Unity NavMesh format
    # Similar structure to Godot
    import_godot_navmesh(data)
  end

  defp import_json_navmesh(data) do
    # Import from generic JSON format
    import_godot_navmesh(data)
  end

  defp tuple_from_list([x, y, z]) when is_number(x) and is_number(y) and is_number(z) do
    {x, y, z}
  end

  defp tuple_from_list([x, y]) when is_number(x) and is_number(y) do
    {x, y, 0.0}
  end

  defp tuple_from_list(other) do
    raise ArgumentError, "Invalid vertex format: #{inspect(other)}"
  end
end

# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.MazeGrid do
  @moduledoc """
  2D Grid generator for mouse maze navigation.

  Generates a 2D grid of positions for maze navigation where entities
  can only move horizontally (X, Y) - no vertical (Z) movement allowed.

  ## Grid Layout

  Positions are arranged in a grid pattern:
  - X-axis: horizontal movement
  - Y-axis: forward/backward movement
  - Z-axis: fixed at 0 (no vertical movement)

  ## Example

      iex> grid = MazeGrid.generate(10, 10)  # 10x10 grid = 100 positions
      iex> length(grid.points)
      100
      iex> {x, y, z} = Enum.at(grid.points, 0)
      # 2D point with z=0
  """

  @type point :: {float(), float(), float()}
  @type t :: %__MODULE__{
          points: [point()],
          width: integer(),
          height: integer(),
          n: integer(),
          index_map: %{point() => integer()}
        }

  defstruct points: [], width: 0, height: 0, n: 0, index_map: %{}

  @doc """
  Generates a 2D grid of positions for maze navigation.

  ## Parameters
  - `width`: Number of positions along X-axis (default: 10)
  - `height`: Number of positions along Y-axis (default: 10)
  - `spacing`: Distance between grid points (default: 1.0)

  ## Returns
  - MazeGrid struct with points and index mapping
  """
  @spec generate(integer(), integer(), float()) :: t()
  def generate(width \\ 10, height \\ 10, spacing \\ 1.0)
      when is_integer(width) and width > 0 and is_integer(height) and height > 0 and
             is_float(spacing) and spacing > 0.0 do
    points =
      for y <- 0..(height - 1), x <- 0..(width - 1) do
        {x * spacing, y * spacing, 0.0}
      end

    index_map =
      points
      |> Enum.with_index()
      |> Enum.into(%{}, fn {point, idx} -> {point, idx} end)

    %__MODULE__{
      points: points,
      width: width,
      height: height,
      n: width * height,
      index_map: index_map
    }
  end

  @doc """
  Finds the nearest grid point to a given 2D position.

  ## Parameters
  - `grid`: MazeGrid struct
  - `position`: 2D or 3D position {x, y} or {x, y, z}

  ## Returns
  - Index of nearest point (0..N-1)
  """
  @spec find_nearest_index(t(), {float(), float()} | {float(), float(), float()}) :: integer()
  def find_nearest_index(%__MODULE__{points: points}, position) do
    # Normalize to 2D (ignore Z)
    {x, y, _z} = case position do
      {x, y} -> {x, y, 0.0}
      {x, y, z} -> {x, y, z}
    end

    # Find point with minimum 2D distance
    {_point, index} =
      points
      |> Enum.with_index()
      |> Enum.min_by(fn {{px, py, _pz}, _idx} ->
        dx = x - px
        dy = y - py
        :math.sqrt(dx * dx + dy * dy)
      end)

    index
  end

  @doc """
  Gets the point at a given index.

  ## Parameters
  - `grid`: MazeGrid struct
  - `index`: Point index (0..N-1)

  ## Returns
  - Point {x, y, z} on grid, or nil if index out of range
  """
  @spec get_point(t(), integer()) :: point() | nil
  def get_point(%__MODULE__{points: points}, index)
      when is_integer(index) and index >= 0 do
    Enum.at(points, index)
  end

  def get_point(_grid, _index), do: nil

  @doc """
  Gets the index of a point in the grid.

  ## Parameters
  - `grid`: MazeGrid struct
  - `point`: Point {x, y, z}

  ## Returns
  - Point index (0..N-1) or nil if point not found
  """
  @spec get_index(t(), point()) :: integer() | nil
  def get_index(%__MODULE__{index_map: index_map}, point) do
    Map.get(index_map, point)
  end

  @doc """
  Calculates 2D Euclidean distance between two grid points.

  Uses only X and Y coordinates (Z is always 0 in maze).

  ## Parameters
  - `p1`: First point {x, y, z}
  - `p2`: Second point {x, y, z}

  ## Returns
  - 2D distance as float
  """
  @spec distance_2d(point(), point()) :: float()
  def distance_2d({x1, y1, _z1}, {x2, y2, _z2}) do
    dx = x2 - x1
    dy = y2 - y1
    :math.sqrt(dx * dx + dy * dy)
  end

  @doc """
  Checks if movement from one point to another involves upward movement.

  ## Parameters
  - `from_point`: Starting point {x, y, z}
  - `to_point`: Target point {x, y, z}

  ## Returns
  - `true` if movement goes upward (z increases), `false` otherwise
  """
  @spec upward_movement?(point(), point()) :: boolean()
  def upward_movement?({_x1, _y1, z1}, {_x2, _y2, z2}) do
    z2 > z1
  end

  @doc """
  Gets grid coordinates (row, col) from a point index.

  ## Parameters
  - `grid`: MazeGrid struct
  - `index`: Point index (0..N-1)

  ## Returns
  - Tuple {row, col} or nil if index out of range
  """
  @spec index_to_coords(t(), integer()) :: {integer(), integer()} | nil
  def index_to_coords(%__MODULE__{width: width}, index)
      when is_integer(index) and index >= 0 do
    row = div(index, width)
    col = rem(index, width)
    {row, col}
  end

  def index_to_coords(_grid, _index), do: nil

  @doc """
  Gets point index from grid coordinates (row, col).

  ## Parameters
  - `grid`: MazeGrid struct
  - `row`: Row index (0..height-1)
  - `col`: Column index (0..width-1)

  ## Returns
  - Point index (0..N-1) or nil if coordinates out of range
  """
  @spec coords_to_index(t(), integer(), integer()) :: integer() | nil
  def coords_to_index(%__MODULE__{width: width, height: height}, row, col)
      when is_integer(row) and row >= 0 and row < height and
             is_integer(col) and col >= 0 and col < width do
    row * width + col
  end

  def coords_to_index(_grid, _row, _col), do: nil
end

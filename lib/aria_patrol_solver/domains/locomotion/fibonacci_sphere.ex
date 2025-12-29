# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.FibonacciSphere do
  @moduledoc """
  Fibonacci sphere generator for uniform point distribution on unit sphere.

  Generates N points uniformly distributed on the surface of a unit sphere
  using the golden angle method. This enables quantization of 3D positions
  and rotation directions for discrete state space planning.

  ## Algorithm

  Uses the golden angle (approximately 137.508 degrees) derived from the
  golden ratio to distribute points evenly across the sphere surface.

  ## Example

      iex> sphere = FibonacciSphere.generate(100)
      iex> length(sphere.points)
      100
      iex> {x, y, z} = Enum.at(sphere.points, 0)
      # Unit vector on sphere surface
  """

  @type point :: {float(), float(), float()}
  @type t :: %__MODULE__{
          points: [point()],
          n: integer(),
          index_map: %{point() => integer()}
        }

  defstruct points: [], n: 0, index_map: %{}

  # Golden angle in radians (approximately 137.508 degrees)
  @golden_angle 2.39996322972865332

  @doc """
  Generates N points uniformly distributed on a unit sphere.

  ## Parameters
  - `n`: Number of points to generate (default: 100)

  ## Returns
  - FibonacciSphere struct with points and index mapping
  """
  @spec generate(integer()) :: t()
  def generate(n \\ 100) when is_integer(n) and n > 0 do
    points =
      for i <- 0..(n - 1) do
        generate_point(i, n)
      end

    index_map =
      points
      |> Enum.with_index()
      |> Enum.into(%{}, fn {point, idx} -> {point, idx} end)

    %__MODULE__{
      points: points,
      n: n,
      index_map: index_map
    }
  end

  @doc """
  Finds the nearest point on the sphere to a given 3D position.

  ## Parameters
  - `sphere`: FibonacciSphere struct
  - `position`: 3D position {x, y, z}

  ## Returns
  - Index of nearest point (0..N-1)
  """
  @spec find_nearest_index(t(), {float(), float(), float()}) :: integer()
  def find_nearest_index(%__MODULE__{points: points}, position) do
    # Normalize position to unit sphere
    normalized = normalize(position)

    # Find point with minimum angular distance
    {_point, index} =
      points
      |> Enum.with_index()
      |> Enum.min_by(fn {point, _idx} ->
        angular_distance(normalized, point)
      end)

    index
  end

  @doc """
  Gets the point at a given index.

  ## Parameters
  - `sphere`: FibonacciSphere struct
  - `index`: Point index (0..N-1)

  ## Returns
  - Point {x, y, z} on unit sphere, or nil if index out of range
  """
  @spec get_point(t(), integer()) :: point() | nil
  def get_point(%__MODULE__{points: points}, index)
      when is_integer(index) and index >= 0 do
    Enum.at(points, index)
  end

  def get_point(_sphere, _index), do: nil

  @doc """
  Calculates the angular distance between two points on the unit sphere.

  Uses dot product: cos(angle) = dot(p1, p2) for unit vectors.

  ## Parameters
  - `p1`: First point {x, y, z}
  - `p2`: Second point {x, y, z}

  ## Returns
  - Angular distance in radians
  """
  @spec angular_distance(point(), point()) :: float()
  def angular_distance({x1, y1, z1}, {x2, y2, z2}) do
    dot_product = x1 * x2 + y1 * y2 + z1 * z2
    # Clamp to [-1, 1] to avoid numerical errors
    clamped = max(-1.0, min(1.0, dot_product))
    :math.acos(clamped)
  end

  @doc """
  Normalizes a 3D vector to unit length.

  ## Parameters
  - `vector`: 3D vector {x, y, z}

  ## Returns
  - Normalized unit vector {x, y, z}
  """
  @spec normalize({float(), float(), float()}) :: point()
  def normalize({x, y, z}) do
    magnitude = :math.sqrt(x * x + y * y + z * z)

    if magnitude > 1.0e-10 do
      {x / magnitude, y / magnitude, z / magnitude}
    else
      # Zero vector - return arbitrary direction (0, 0, 1)
      {0.0, 0.0, 1.0}
    end
  end

  # Private: Generate a single point on the sphere using golden angle
  defp generate_point(i, n) do
    # Golden angle method for uniform distribution
    y = 1.0 - (2.0 * i) / (n - 1.0)
    radius = :math.sqrt(1.0 - y * y)
    theta = @golden_angle * i

    x = radius * :math.cos(theta)
    z = radius * :math.sin(theta)

    # Normalize to ensure unit length (accounting for floating point errors)
    normalize({x, y, z})
  end
end

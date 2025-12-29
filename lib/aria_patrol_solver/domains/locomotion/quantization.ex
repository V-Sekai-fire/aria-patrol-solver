# SPDX-License-Identifier: MIT
# Copyright (c) 2025-present K. S. Ernest (iFire) Lee

defmodule AriaPatrolSolver.Domains.Locomotion.Quantization do
  @moduledoc """
  Quantization utilities for 3D positions and rotations.

  Provides functions to quantize continuous 3D positions and rotation
  quaternions to discrete Fibonacci sphere indices, and dequantize them
  back. Also supports logarithmic encoding of combined position+rotation
  states for efficient state space representation.

  ## Quantization Strategy

  - **Position**: Map continuous {x, y, z} to nearest Fibonacci sphere point → index
  - **Rotation**: Extract forward direction from quaternion, map to nearest sphere point → index
  - **Combined State**: Use logarithmic encoding: `state_id = position_index * N + rotation_index`
  """

  alias AriaPatrolSolver.Domains.Locomotion.FibonacciSphere

  @type position :: {float(), float(), float()}
  @type quaternion :: {float(), float(), float(), float()}
  @type sphere :: FibonacciSphere.t()

  @doc """
  Quantizes a 3D position to a Fibonacci sphere index.

  ## Parameters
  - `sphere`: FibonacciSphere struct
  - `position`: 3D position {x, y, z}

  ## Returns
  - Index (0..N-1) of nearest sphere point
  """
  @spec quantize_position(sphere(), position()) :: integer()
  def quantize_position(sphere, position) do
    FibonacciSphere.find_nearest_index(sphere, position)
  end

  @doc """
  Dequantizes a position index back to a 3D position.

  Note: This returns the sphere point, which may need to be scaled
  to match the original position's magnitude.

  ## Parameters
  - `sphere`: FibonacciSphere struct
  - `index`: Position index (0..N-1)

  ## Returns
  - 3D position {x, y, z} on unit sphere, or nil if invalid index
  """
  @spec dequantize_position(sphere(), integer()) :: position() | nil
  def dequantize_position(sphere, index) do
    FibonacciSphere.get_point(sphere, index)
  end

  @doc """
  Extracts the forward direction vector from a quaternion.

  In Godot/3D game conventions, forward is typically -Z axis.
  This function extracts the forward vector that the quaternion
  would rotate the default forward direction to.

  ## Parameters
  - `quaternion`: Rotation quaternion {x, y, z, w}

  ## Returns
  - Forward direction vector {x, y, z} (unit vector)
  """
  @spec quaternion_to_forward(quaternion()) :: position()
  def quaternion_to_forward({qx, qy, qz, qw}) do
    # Default forward direction in Godot: {0, 0, -1}
    # Rotate this vector by the quaternion
    # For a quaternion q and vector v, rotated = q * v * q^-1
    # Simplified forward extraction:
    # forward_x = 2 * (qx * qz + qw * qy)
    # forward_y = 2 * (qy * qz - qw * qx)
    # forward_z = 1 - 2 * (qx * qx + qy * qy)

    fx = 2.0 * (qx * qz + qw * qy)
    fy = 2.0 * (qy * qz - qw * qx)
    fz = 1.0 - 2.0 * (qx * qx + qy * qy)

    # Normalize to unit vector
    FibonacciSphere.normalize({fx, fy, fz})
  end

  @doc """
  Quantizes a rotation quaternion to a Fibonacci sphere index.

  Extracts the forward direction vector and finds the nearest sphere point.

  ## Parameters
  - `sphere`: FibonacciSphere struct
  - `quaternion`: Rotation quaternion {x, y, z, w}

  ## Returns
  - Index (0..N-1) of nearest sphere point for forward direction
  """
  @spec quantize_rotation(sphere(), quaternion() | map()) :: integer()
  def quantize_rotation(sphere, %{x: qx, y: qy, z: qz, w: qw}) do
    quantize_rotation(sphere, {qx, qy, qz, qw})
  end

  def quantize_rotation(sphere, {qx, qy, qz, qw}) do
    forward = quaternion_to_forward({qx, qy, qz, qw})
    FibonacciSphere.find_nearest_index(sphere, forward)
  end

  @doc """
  Dequantizes a rotation index back to a forward direction vector.

  ## Parameters
  - `sphere`: FibonacciSphere struct
  - `index`: Rotation index (0..N-1)

  ## Returns
  - Forward direction vector {x, y, z} on unit sphere, or nil if invalid
  """
  @spec dequantize_rotation(sphere(), integer()) :: position() | nil
  def dequantize_rotation(sphere, index) do
    FibonacciSphere.get_point(sphere, index)
  end

  @doc """
  Encodes position and rotation indices into a single combined state ID.

  Uses logarithmic encoding: `state_id = position_index * N + rotation_index`
  where N is the number of sphere points.

  ## Parameters
  - `sphere`: FibonacciSphere struct (provides N)
  - `position_index`: Position index (0..N-1)
  - `rotation_index`: Rotation index (0..N-1)

  ## Returns
  - Combined state ID (0..N²-1)
  """
  @spec encode_combined_state(sphere(), integer(), integer()) :: integer()
  def encode_combined_state(%FibonacciSphere{n: n}, position_index, rotation_index)
      when position_index >= 0 and position_index < n and
             rotation_index >= 0 and rotation_index < n do
    position_index * n + rotation_index
  end

  def encode_combined_state(_sphere, _position_index, _rotation_index) do
    raise ArgumentError, "Invalid indices for state encoding"
  end

  @doc """
  Decodes a combined state ID back to position and rotation indices.

  ## Parameters
  - `sphere`: FibonacciSphere struct (provides N)
  - `state_id`: Combined state ID (0..N²-1)

  ## Returns
  - {position_index, rotation_index} tuple
  """
  @spec decode_combined_state(sphere(), integer()) :: {integer(), integer()}
  def decode_combined_state(%FibonacciSphere{n: n}, state_id)
      when state_id >= 0 and state_id < n * n do
    position_index = div(state_id, n)
    rotation_index = rem(state_id, n)
    {position_index, rotation_index}
  end

  def decode_combined_state(_sphere, _state_id) do
    raise ArgumentError, "Invalid state_id for decoding"
  end
end

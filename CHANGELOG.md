# Changelog

All notable changes to this project will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [m2s2-math 0.2.0 / m2s2-geometry 0.1.0] — 2026-07-01

### Added

#### m2s2-math

##### Vectors
- `Vector<T, D>`: `distance`, `distance_squared`, `lerp`, `reflect`, `project_onto`, `reject_from`, `clamp_length`, `angle_between` (dimension-generic, Float-only)
- `Vector2Ops`: `cross` (2D perp-dot product)

##### Matrices
- `Matrix2x2`/`Matrix3x3`/`Matrix4x4`: `trace` (any numeric `T`), `determinant` (cofactor expansion, any numeric `T`), `inverse` (Float-only, Gauss-Jordan elimination with partial pivoting)

#### New crate: m2s2-geometry

Geometric primitives and intersection queries, built on `m2s2-math` and split into
its own crate so consumers that only need vectors/matrices/quaternions (e.g. a
renderer) aren't forced to depend on collision/geometry code.

- `Ray2`/`Ray3`: `point_at`
- `Aabb2`/`Aabb3`: `from_center_half_extents`, `center`, `half_extents`, `size`, `contains_point`, `intersects_aabb`, `union`, `closest_point`, `intersects_circle`/`intersects_sphere`, `intersects_ray` (slab method), `sweep_aabb` (swept AABB vs static AABB)
- `Circle`/`Sphere`: `contains_point`, `closest_point`, `intersects_circle`/`intersects_sphere`, `intersects_aabb`, `intersects_ray` (quadratic), `sweep_circle`/`sweep_sphere` (continuous collision, relative-velocity solve)
- `Plane`: `from_point_normal`, `from_points`, `signed_distance`, `intersects_ray`, `intersects_plane`
- `Triangle3`: `intersects_ray` (double-sided Möller–Trumbore)
- `Obb2`/`Obb3`: `axes`, `contains_point`, `intersects_obb` (SAT — 4-axis in 2D, full 15-axis in 3D), `intersects_aabb`, `intersects_ray`

## [0.1.0] — 2026-06-16

Initial release.

### Added

#### Vectors
- Generic `Vector<T, const D: usize>` with type aliases `Vector2`, `Vector3`, `Vector4` (and `i32`, `i64`, `f32`, `f64` variants)
- Arithmetic: `Add`, `Sub`, `Mul<T>`, `Div<T>`, `Neg`, `AddAssign`, `SubAssign`
- `Index` / `IndexMut`, `get`, `set`, `as_slice`, `from_slice`, `dimension`
- `Vector2<T>`: `rotate_90_cw`, `rotate_90_ccw` (integer-friendly pivot rotation)
- Float traits (`Vector2Ops`, `Vector3Ops`, `Vector4Ops`): `length`, `length_squared`, `normalize`, `dot`, `cross` (3D), `perpendicular` (2D), `perspective_divide` (4D → 3D)

#### Matrices
- `Matrix2x2`, `Matrix3x3`, `Matrix4x4` (macro-generated, stable Rust)
- Type aliases for `i32`, `i64`, `f32`, `f64`
- Arithmetic: `Add`, `Sub`, `Mul<T>`, `Neg`
- Matrix × matrix and matrix × vector multiplication
- `identity`, `transpose`, `from_slice`, `from_2d_array`, `get`, `get_mut`, `set`, `as_slice`
- `Index` / `IndexMut` (row slices)

#### Transforms — `Transform4x4` trait on `Matrix4x4<f32/f64>`
- `rotation_x`, `rotation_y`, `rotation_z`, `rotation_axis_angle`
- `translation`, `scale`, `uniform_scale`
- Perspective projection — four conventions:
  - `perspective_rh_zo` (Vulkan, Metal, D3D12 RH)
  - `perspective_rh_no` (OpenGL)
  - `perspective_lh_zo` (D3D9 / D3D11 / D3D12 LH)
  - `perspective_lh_no`
- Orthographic projection — four conventions:
  - `ortho_rh_zo` (Vulkan, Metal)
  - `ortho_rh_no` (OpenGL)
  - `ortho_lh_zo` (D3D)
  - `ortho_lh_no`
- View matrix — `look_at_rh` and `look_at_lh`

#### Transforms — `Transform3x3` trait on `Matrix3x3<f32/f64>`
- `translation_2d`, `rotation_2d`, `scale_2d`, `uniform_scale_2d`

#### Transforms — `Transform2x2` trait on `Matrix2x2<f32/f64>`
- `rotation_2d`

#### Quaternions
- Generic `Quaternion<T: Float>` with `Quaternionf32` / `Quaternionf64` aliases
- Construction: `new`, `identity`, `from_axis_angle`, `from_euler_xyz`
- Arithmetic: `Mul` (Hamilton product), `Mul<T>`, `Add`, `Sub`, `Neg`
- Methods: `conjugate`, `norm`, `norm_squared`, `normalize`, `dot`, `inverse`
- Rotation: `rotate_vector`, `to_matrix4x4`, `to_matrix3x3`
- Conversion: `to_axis_angle`, `to_euler_xyz`
- Interpolation: `lerp` (normalized linear), `slerp` (spherical linear, shortest path)

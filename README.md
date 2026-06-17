# m2s2-math

A linear algebra library for game engines and Vulkan/Metal/OpenGL/D3D renderers, written in Rust.

Provides vectors, matrices, and quaternions with zero heap allocation and stable-Rust compatibility.

## Features

- **Vectors** — `Vector2`, `Vector3`, `Vector4` (generic over any element type); dot, cross, normalize, length, perpendicular, perspective divide
- **Matrices** — `Matrix2x2`, `Matrix3x3`, `Matrix4x4` (generic); add, sub, scalar mul, matrix mul, mat×vec, transpose, identity
- **Transforms** — rotation (X/Y/Z, axis-angle), translation, scale — convention-independent
- **Projection** — perspective and orthographic in all four graphics API conventions:

  | Method suffix | Z NDC range | Handedness | Use for |
  |--------------|-------------|------------|---------|
  | `_rh_zo`     | [0, 1]      | Right      | Vulkan, Metal, D3D12 RH |
  | `_rh_no`     | [−1, 1]     | Right      | OpenGL |
  | `_lh_zo`     | [0, 1]      | Left       | D3D9 / D3D11 / D3D12 LH |
  | `_lh_no`     | [−1, 1]     | Left       | (rare) |

- **Look-at** — `look_at_rh` (Vulkan / OpenGL / Metal) and `look_at_lh` (D3D)
- **Quaternions** — `Quaternion<T>` with Hamilton product, conjugate, inverse, normalize, axis-angle, Euler angles, `rotate_vector`, `to_matrix4x4`, `to_matrix3x3`, `lerp`, `slerp`

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
m2s2-math = "0.1"
```

### Vectors

```rust
use m2s2_math::{Vector3f32, Vector3Ops};

let a = Vector3f32::new(1.0, 0.0, 0.0);
let b = Vector3f32::new(0.0, 1.0, 0.0);
let c = a.cross(&b);          // (0, 0, 1)
let d = a + b;                // (1, 1, 0)
let n = d.normalize();
println!("{}", n.length());   // 1.0
```

### Matrices & Transforms

```rust
use m2s2_math::{Matrix4x4f32, Vector3f32, Vector4f32};
use m2s2_math::Transform4x4;

// Vulkan perspective (right-handed, depth [0, 1])
let proj = Matrix4x4f32::perspective_rh_zo(
    std::f32::consts::FRAC_PI_2, // 90° fov
    16.0 / 9.0,                  // aspect ratio
    0.1,                         // near
    1000.0,                      // far
);

// View matrix (right-handed, for Vulkan / OpenGL / Metal)
let view = Matrix4x4f32::look_at_rh(
    Vector3f32::new(0.0, 2.0, 5.0),  // eye
    Vector3f32::new(0.0, 0.0, 0.0),  // target
    Vector3f32::new(0.0, 1.0, 0.0),  // up
);

// Row-major storage: call .transpose() before uploading to Vulkan/GPU
let proj_gpu = proj.transpose();
```

### Quaternions

```rust
use m2s2_math::{Quaternionf32, Vector3f32};

let axis  = Vector3f32::new(0.0, 1.0, 0.0);
let q     = Quaternionf32::from_axis_angle(axis, std::f32::consts::FRAC_PI_4);
let mat   = q.to_matrix4x4();         // use in shader uploads
let qlerp = Quaternionf32::slerp(Quaternionf32::identity(), q, 0.5);
```

## Storage convention

Matrices are stored **row-major** in memory (`data[row * cols + col]`). CPU-side `M * v` treats `v` as a column vector and gives correct results directly.

Before uploading to a GPU API that expects column-major layout (Vulkan, OpenGL), call `.transpose()` on the matrix.

## License

MIT — see [LICENSE](LICENSE).

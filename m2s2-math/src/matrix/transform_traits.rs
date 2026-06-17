use crate::vector::Vector3;
use num_traits::Float;

/// Trait for 4x4 transformation matrices.
///
/// Projection and view methods come in four convention variants:
///
/// - `rh_no` — Right-handed, Z NDC [-1, 1]  (OpenGL)
/// - `rh_zo` — Right-handed, Z NDC [ 0, 1]  (Vulkan, Metal, D3D12 RH)
/// - `lh_zo` — Left-handed,  Z NDC [ 0, 1]  (D3D9/11/12 LH)
/// - `lh_no` — Left-handed,  Z NDC [-1, 1]  (rare; included for completeness)
///
/// Convention-independent operations (rotation, translation, scale) have no suffix.
pub trait Transform4x4<T>
where
    T: Float + Copy,
{
    // --- Perspective projection ---

    /// Right-handed, Z maps to [0, 1]. Use for Vulkan, Metal, D3D12 RH.
    fn perspective_rh_zo(fov_y_radians: T, aspect_ratio: T, near: T, far: T) -> Self;

    /// Right-handed, Z maps to [-1, 1]. Use for OpenGL.
    fn perspective_rh_no(fov_y_radians: T, aspect_ratio: T, near: T, far: T) -> Self;

    /// Left-handed, Z maps to [0, 1]. Use for D3D9/11/12 LH.
    fn perspective_lh_zo(fov_y_radians: T, aspect_ratio: T, near: T, far: T) -> Self;

    /// Left-handed, Z maps to [-1, 1].
    fn perspective_lh_no(fov_y_radians: T, aspect_ratio: T, near: T, far: T) -> Self;

    // --- Orthographic projection ---

    /// Right-handed, Z maps to [-1, 1]. Use for OpenGL.
    fn ortho_rh_no(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self;

    /// Right-handed, Z maps to [0, 1]. Use for Vulkan, Metal, D3D12 RH.
    fn ortho_rh_zo(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self;

    /// Left-handed, Z maps to [0, 1]. Use for D3D9/11/12 LH.
    fn ortho_lh_zo(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self;

    /// Left-handed, Z maps to [-1, 1].
    fn ortho_lh_no(left: T, right: T, bottom: T, top: T, near: T, far: T) -> Self;

    // --- View (look-at) ---

    /// Right-handed look-at view matrix. Camera looks toward -Z (OpenGL, Vulkan, Metal).
    fn look_at_rh(eye: Vector3<T>, target: Vector3<T>, up: Vector3<T>) -> Self;

    /// Left-handed look-at view matrix. Camera looks toward +Z (D3D).
    fn look_at_lh(eye: Vector3<T>, target: Vector3<T>, up: Vector3<T>) -> Self;

    // --- Convention-independent transforms ---

    fn translation(translation: Vector3<T>) -> Self;
    fn uniform_scale(scale: T) -> Self;
    fn scale(scale: Vector3<T>) -> Self;
    fn rotation_x(angle_radians: T) -> Self;
    fn rotation_y(angle_radians: T) -> Self;
    fn rotation_z(angle_radians: T) -> Self;
    fn rotation_axis_angle(axis: Vector3<T>, angle_radians: T) -> Self;
}

/// Trait for 3x3 transformation matrices (2D + homogeneous).
pub trait Transform3x3<T>
where
    T: Float + Copy,
{
    fn translation_2d(translation: crate::vector::Vector2<T>) -> Self;
    fn rotation_2d(angle_radians: T) -> Self;
    fn scale_2d(scale: crate::vector::Vector2<T>) -> Self;
    fn uniform_scale_2d(scale: T) -> Self;
}

/// Trait for 2x2 rotation matrices.
pub trait Transform2x2<T>
where
    T: Float + Copy,
{
    fn rotation_2d(angle_radians: T) -> Self;
}

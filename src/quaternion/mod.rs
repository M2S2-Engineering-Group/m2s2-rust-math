use std::ops::{Add, Mul, Neg, Sub};

use num_traits::Float;

use crate::matrix::{Matrix3x3, Matrix4x4};
use crate::vector::Vector3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion<T> {
    pub w: T,
    pub x: T,
    pub y: T,
    pub z: T,
}

pub type Quaternionf32 = Quaternion<f32>;
pub type Quaternionf64 = Quaternion<f64>;

impl<T: Float + Copy> Quaternion<T> {
    pub fn new(w: T, x: T, y: T, z: T) -> Self {
        Quaternion { w, x, y, z }
    }

    pub fn identity() -> Self {
        Quaternion {
            w: T::one(),
            x: T::zero(),
            y: T::zero(),
            z: T::zero(),
        }
    }

    pub fn from_axis_angle(axis: Vector3<T>, angle: T) -> Self {
        let half = angle / (T::one() + T::one());
        let s = half.sin();
        let a = axis.as_slice();
        Quaternion {
            w: half.cos(),
            x: a[0] * s,
            y: a[1] * s,
            z: a[2] * s,
        }
    }

    /// Constructs from Euler angles (roll=X, pitch=Y, yaw=Z) applied in ZYX order.
    pub fn from_euler_xyz(roll: T, pitch: T, yaw: T) -> Self {
        let two = T::one() + T::one();
        let (sr, cr) = (roll / two).sin_cos();
        let (sp, cp) = (pitch / two).sin_cos();
        let (sy, cy) = (yaw / two).sin_cos();
        Quaternion {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    pub fn conjugate(&self) -> Self {
        Quaternion {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn norm_squared(&self) -> T {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn norm(&self) -> T {
        self.norm_squared().sqrt()
    }

    pub fn normalize(&self) -> Self {
        let n = self.norm();
        if n <= T::epsilon() {
            return *self;
        }
        Quaternion {
            w: self.w / n,
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
        }
    }

    pub fn dot(&self, other: &Self) -> T {
        self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn inverse(&self) -> Self {
        let ns = self.norm_squared();
        self.conjugate() * (T::one() / ns)
    }

    pub fn rotate_vector(&self, v: Vector3<T>) -> Vector3<T> {
        let q = self.normalize();
        let s = v.as_slice();
        let qv = Quaternion::new(T::zero(), s[0], s[1], s[2]);
        let result = q * qv * q.conjugate();
        Vector3::new(result.x, result.y, result.z)
    }

    pub fn to_matrix4x4(&self) -> Matrix4x4<T> {
        let q = self.normalize();
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);
        let two = T::one() + T::one();
        Matrix4x4::from_2d_array([
            [
                T::one() - two * (y * y + z * z),
                two * (x * y - w * z),
                two * (x * z + w * y),
                T::zero(),
            ],
            [
                two * (x * y + w * z),
                T::one() - two * (x * x + z * z),
                two * (y * z - w * x),
                T::zero(),
            ],
            [
                two * (x * z - w * y),
                two * (y * z + w * x),
                T::one() - two * (x * x + y * y),
                T::zero(),
            ],
            [T::zero(), T::zero(), T::zero(), T::one()],
        ])
    }

    pub fn to_matrix3x3(&self) -> Matrix3x3<T> {
        let q = self.normalize();
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);
        let two = T::one() + T::one();
        Matrix3x3::from_2d_array([
            [
                T::one() - two * (y * y + z * z),
                two * (x * y - w * z),
                two * (x * z + w * y),
            ],
            [
                two * (x * y + w * z),
                T::one() - two * (x * x + z * z),
                two * (y * z - w * x),
            ],
            [
                two * (x * z - w * y),
                two * (y * z + w * x),
                T::one() - two * (x * x + y * y),
            ],
        ])
    }

    pub fn to_axis_angle(&self) -> (Vector3<T>, T) {
        let q = self.normalize();
        let angle = two_acos(q.w);
        let s = (T::one() - q.w * q.w).sqrt();
        if s <= T::epsilon() {
            (Vector3::new(T::one(), T::zero(), T::zero()), T::zero())
        } else {
            (Vector3::new(q.x / s, q.y / s, q.z / s), angle)
        }
    }

    pub fn to_euler_xyz(&self) -> (T, T, T) {
        let q = self.normalize();
        let two = T::one() + T::one();
        let sinr_cosp = two * (q.w * q.x + q.y * q.z);
        let cosr_cosp = T::one() - two * (q.x * q.x + q.y * q.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        let sinp = two * (q.w * q.y - q.z * q.x);
        let pitch = if sinp.abs() >= T::one() {
            (T::one()).atan2(T::zero()) * sinp.signum()
        } else {
            sinp.asin()
        };

        let siny_cosp = two * (q.w * q.z + q.x * q.y);
        let cosy_cosp = T::one() - two * (q.y * q.y + q.z * q.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }

    pub fn lerp(a: Self, b: Self, t: T) -> Self {
        let one_minus_t = T::one() - t;
        Quaternion {
            w: a.w * one_minus_t + b.w * t,
            x: a.x * one_minus_t + b.x * t,
            y: a.y * one_minus_t + b.y * t,
            z: a.z * one_minus_t + b.z * t,
        }
        .normalize()
    }

    pub fn slerp(a: Self, b: Self, t: T) -> Self {
        let mut dot = a.dot(&b);

        // Ensure shortest path
        let b = if dot < T::zero() {
            dot = -dot;
            -b
        } else {
            b
        };

        // Fall back to lerp if quaternions are very close
        let threshold = T::one() - T::from(1e-6).unwrap_or(T::epsilon());
        if dot > threshold {
            return Self::lerp(a, b, t);
        }

        let theta_0 = dot.acos();
        let theta = theta_0 * t;
        let sin_theta = theta.sin();
        let sin_theta_0 = theta_0.sin();

        let s0 = (theta_0 - theta).sin() / sin_theta_0;
        let s1 = sin_theta / sin_theta_0;

        Quaternion {
            w: a.w * s0 + b.w * s1,
            x: a.x * s0 + b.x * s1,
            y: a.y * s0 + b.y * s1,
            z: a.z * s0 + b.z * s1,
        }
    }
}

fn two_acos<T: Float>(w: T) -> T {
    let clamped = if w > T::one() {
        T::one()
    } else if w < -T::one() {
        -T::one()
    } else {
        w
    };
    let two = T::one() + T::one();
    two * clamped.acos()
}

impl<T: Float + Copy> Mul for Quaternion<T> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Quaternion {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl<T: Float + Copy> Mul<T> for Quaternion<T> {
    type Output = Self;
    fn mul(self, scalar: T) -> Self {
        Quaternion {
            w: self.w * scalar,
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl<T: Float + Copy> Add for Quaternion<T> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Quaternion {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl<T: Float + Copy> Sub for Quaternion<T> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Quaternion {
            w: self.w - rhs.w,
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl<T: Float + Copy> Neg for Quaternion<T> {
    type Output = Self;
    fn neg(self) -> Self {
        Quaternion {
            w: -self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vector::{Vector3f32, vector_ops::Vector3Ops};
    use std::f32::consts::PI;

    const EPS: f32 = 1e-5;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    fn quat_approx_eq(a: Quaternionf32, b: Quaternionf32) -> bool {
        approx_eq(a.w, b.w) && approx_eq(a.x, b.x) && approx_eq(a.y, b.y) && approx_eq(a.z, b.z)
    }

    #[test]
    fn test_identity() {
        let q = Quaternionf32::identity();
        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 0.0);
        assert_eq!(q.y, 0.0);
        assert_eq!(q.z, 0.0);
    }

    #[test]
    fn test_identity_rotates_nothing() {
        let q = Quaternionf32::identity();
        let v = Vector3f32::new(1.0, 2.0, 3.0);
        let result = q.rotate_vector(v);
        assert!(approx_eq(result.x(), 1.0));
        assert!(approx_eq(result.y(), 2.0));
        assert!(approx_eq(result.z(), 3.0));
    }

    #[test]
    fn test_from_axis_angle_z_90() {
        let axis = Vector3f32::new(0.0, 0.0, 1.0);
        let q = Quaternionf32::from_axis_angle(axis, PI / 2.0);
        let v = Vector3f32::new(1.0, 0.0, 0.0);
        let rotated = q.rotate_vector(v);
        assert!(approx_eq(rotated.x(), 0.0));
        assert!(approx_eq(rotated.y(), 1.0));
        assert!(approx_eq(rotated.z(), 0.0));
    }

    #[test]
    fn test_from_axis_angle_x_90() {
        let axis = Vector3f32::new(1.0, 0.0, 0.0);
        let q = Quaternionf32::from_axis_angle(axis, PI / 2.0);
        let v = Vector3f32::new(0.0, 1.0, 0.0);
        let rotated = q.rotate_vector(v);
        assert!(approx_eq(rotated.x(), 0.0));
        assert!(approx_eq(rotated.y(), 0.0));
        assert!(approx_eq(rotated.z(), 1.0));
    }

    #[test]
    fn test_inverse_gives_identity() {
        let axis = Vector3f32::new(0.0, 1.0, 0.0);
        let q = Quaternionf32::from_axis_angle(axis, PI / 4.0);
        let result = q * q.inverse();
        let identity = Quaternionf32::identity();
        assert!(quat_approx_eq(result, identity));
    }

    #[test]
    fn test_conjugate_of_unit_is_inverse() {
        let axis = Vector3f32::new(1.0, 0.0, 0.0);
        let q = Quaternionf32::from_axis_angle(axis, PI / 3.0);
        let result = q * q.conjugate();
        assert!(approx_eq(result.w, 1.0));
        assert!(approx_eq(result.x, 0.0));
        assert!(approx_eq(result.y, 0.0));
        assert!(approx_eq(result.z, 0.0));
    }

    #[test]
    fn test_normalize() {
        let q = Quaternionf32::new(2.0, 0.0, 0.0, 0.0);
        let n = q.normalize();
        assert!(approx_eq(n.norm(), 1.0));
        assert!(approx_eq(n.w, 1.0));
    }

    #[test]
    fn test_hamilton_product() {
        let i = Quaternionf32::new(0.0, 1.0, 0.0, 0.0);
        let j = Quaternionf32::new(0.0, 0.0, 1.0, 0.0);
        let k = i * j;
        // i*j = k
        assert!(approx_eq(k.w, 0.0));
        assert!(approx_eq(k.x, 0.0));
        assert!(approx_eq(k.y, 0.0));
        assert!(approx_eq(k.z, 1.0));
    }

    #[test]
    fn test_to_matrix4x4_identity() {
        let q = Quaternionf32::identity();
        let m = q.to_matrix4x4();
        let expected = crate::Matrix4x4f32::identity();
        for (a, b) in m.as_slice().iter().zip(expected.as_slice().iter()) {
            assert!(approx_eq(*a, *b));
        }
    }

    #[test]
    fn test_to_matrix4x4_rotation_z_90() {
        let axis = Vector3f32::new(0.0, 0.0, 1.0);
        let q = Quaternionf32::from_axis_angle(axis, PI / 2.0);
        let m = q.to_matrix4x4();
        let v = crate::Vector4f32::new(1.0, 0.0, 0.0, 0.0);
        let result = m * v;
        assert!(approx_eq(result.as_slice()[0], 0.0));
        assert!(approx_eq(result.as_slice()[1], 1.0));
        assert!(approx_eq(result.as_slice()[2], 0.0));
    }

    #[test]
    fn test_to_matrix3x3_identity() {
        let q = Quaternionf32::identity();
        let m = q.to_matrix3x3();
        let expected = crate::Matrix3x3f32::identity();
        for (a, b) in m.as_slice().iter().zip(expected.as_slice().iter()) {
            assert!(approx_eq(*a, *b));
        }
    }

    #[test]
    fn test_slerp_endpoints() {
        let axis = Vector3f32::new(0.0, 1.0, 0.0);
        let a = Quaternionf32::from_axis_angle(axis, 0.0);
        let b = Quaternionf32::from_axis_angle(axis, PI / 2.0);
        let start = Quaternionf32::slerp(a, b, 0.0);
        let end = Quaternionf32::slerp(a, b, 1.0);
        assert!(quat_approx_eq(start, a));
        assert!(quat_approx_eq(end, b));
    }

    #[test]
    fn test_slerp_midpoint() {
        // 0 → PI/2, midpoint should be PI/4. Avoids antipodal ambiguity at PI.
        let axis = Vector3f32::new(0.0, 0.0, 1.0);
        let a = Quaternionf32::from_axis_angle(axis, 0.0);
        let b = Quaternionf32::from_axis_angle(axis, PI / 2.0);
        let mid = Quaternionf32::slerp(a, b, 0.5);
        let expected = Quaternionf32::from_axis_angle(axis, PI / 4.0);
        assert!(quat_approx_eq(mid, expected));
    }

    #[test]
    fn test_lerp_endpoints() {
        let axis = Vector3f32::new(1.0, 0.0, 0.0);
        let a = Quaternionf32::identity();
        let b = Quaternionf32::from_axis_angle(axis, PI / 2.0);
        let start = Quaternionf32::lerp(a, b, 0.0);
        let end = Quaternionf32::lerp(a, b, 1.0);
        assert!(quat_approx_eq(start, a));
        assert!(quat_approx_eq(end, b));
    }

    #[test]
    fn test_to_axis_angle_roundtrip() {
        let axis = Vector3f32::new(0.0, 1.0, 0.0);
        let angle = PI / 3.0;
        let q = Quaternionf32::from_axis_angle(axis, angle);
        let (out_axis, out_angle) = q.to_axis_angle();
        assert!(approx_eq(out_angle, angle));
        assert!(approx_eq(out_axis.x(), axis.x()));
        assert!(approx_eq(out_axis.y(), axis.y()));
        assert!(approx_eq(out_axis.z(), axis.z()));
    }

    #[test]
    fn test_euler_roundtrip() {
        let roll = 0.1f32;
        let pitch = 0.2f32;
        let yaw = 0.3f32;
        let q = Quaternionf32::from_euler_xyz(roll, pitch, yaw);
        let (r, p, y) = q.to_euler_xyz();
        assert!(approx_eq(r, roll));
        assert!(approx_eq(p, pitch));
        assert!(approx_eq(y, yaw));
    }

    #[test]
    fn test_composition_of_rotations() {
        // Two 90-degree rotations around Z should equal 180-degree rotation around Z
        let axis = Vector3f32::new(0.0, 0.0, 1.0);
        let q90 = Quaternionf32::from_axis_angle(axis, PI / 2.0);
        let q180 = Quaternionf32::from_axis_angle(axis, PI);
        let composed = q90 * q90;
        // Both should rotate (1,0,0) to (-1,0,0)
        let v = Vector3f32::new(1.0, 0.0, 0.0);
        let r1 = composed.rotate_vector(v);
        let r2 = q180.rotate_vector(v);
        assert!(approx_eq(r1.x(), r2.x()));
        assert!(approx_eq(r1.y(), r2.y()));
        assert!(approx_eq(r1.z(), r2.z()));
    }
}

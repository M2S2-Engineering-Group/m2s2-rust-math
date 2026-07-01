use crate::vector::{Vector2, Vector3, Vector4};
use num_traits::Float;

/// Trait for 3D vector operations
pub trait Vector3Ops<T>
where
    T: Float + Copy,
{
    /// Calculate the length (magnitude) of the vector
    fn length(&self) -> T;

    /// Calculate the squared length (avoids sqrt for performance)
    fn length_squared(&self) -> T;

    /// Normalize the vector (return unit vector in same direction)
    fn normalize(&self) -> Self;

    /// Calculate dot product with another vector
    fn dot(&self, other: &Self) -> T;

    /// Calculate cross product with another vector
    fn cross(&self, other: &Self) -> Self;

    /// Get individual components
    fn x(&self) -> T;
    fn y(&self) -> T;
    fn z(&self) -> T;
}

/// Trait for 2D vector operations
pub trait Vector2Ops<T>
where
    T: Float + Copy,
{
    /// Calculate the length (magnitude) of the vector
    fn length(&self) -> T;

    /// Calculate the squared length (avoids sqrt for performance)
    fn length_squared(&self) -> T;

    /// Normalize the vector (return unit vector in same direction)
    fn normalize(&self) -> Self;

    /// Calculate dot product with another vector
    fn dot(&self, other: &Self) -> T;

    /// Get individual components
    fn x(&self) -> T;
    fn y(&self) -> T;

    /// Rotate 90 degrees counter-clockwise
    fn perpendicular(&self) -> Self;

    /// 2D "cross product" (perp dot product): the Z component of the 3D cross
    /// product if both vectors were embedded in the XY plane. Positive when
    /// `other` is counter-clockwise from `self`; useful for winding/orientation
    /// tests and signed area calculations.
    fn cross(&self, other: &Self) -> T;
}

// Generic implementation for Vector3<T> where T: Float
impl<T> Vector3Ops<T> for Vector3<T>
where
    T: Float + Copy,
{
    fn length(&self) -> T {
        (self.data[0] * self.data[0] + self.data[1] * self.data[1] + self.data[2] * self.data[2])
            .sqrt()
    }

    fn length_squared(&self) -> T {
        self.data[0] * self.data[0] + self.data[1] * self.data[1] + self.data[2] * self.data[2]
    }

    fn normalize(&self) -> Self {
        let len = self.length();
        if len <= T::epsilon() {
            *self // Return original if zero vector
        } else {
            Vector3::new(self.data[0] / len, self.data[1] / len, self.data[2] / len)
        }
    }

    fn dot(&self, other: &Self) -> T {
        self.data[0] * other.data[0] + self.data[1] * other.data[1] + self.data[2] * other.data[2]
    }

    fn cross(&self, other: &Self) -> Self {
        Vector3::new(
            self.data[1] * other.data[2] - self.data[2] * other.data[1],
            self.data[2] * other.data[0] - self.data[0] * other.data[2],
            self.data[0] * other.data[1] - self.data[1] * other.data[0],
        )
    }

    fn x(&self) -> T {
        self.data[0]
    }
    fn y(&self) -> T {
        self.data[1]
    }
    fn z(&self) -> T {
        self.data[2]
    }
}

// Generic implementation for Vector2<T> where T: Float
impl<T> Vector2Ops<T> for Vector2<T>
where
    T: Float + Copy,
{
    fn length(&self) -> T {
        (self.data[0] * self.data[0] + self.data[1] * self.data[1]).sqrt()
    }

    fn length_squared(&self) -> T {
        self.data[0] * self.data[0] + self.data[1] * self.data[1]
    }

    fn normalize(&self) -> Self {
        let len = self.length();
        if len <= T::epsilon() {
            *self
        } else {
            Vector2::new(self.data[0] / len, self.data[1] / len)
        }
    }

    fn dot(&self, other: &Self) -> T {
        self.data[0] * other.data[0] + self.data[1] * other.data[1]
    }

    fn x(&self) -> T {
        self.data[0]
    }
    fn y(&self) -> T {
        self.data[1]
    }

    fn perpendicular(&self) -> Self {
        Vector2::new(-self.data[1], self.data[0])
    }

    fn cross(&self, other: &Self) -> T {
        self.data[0] * other.data[1] - self.data[1] * other.data[0]
    }
}

/// Trait for 4D vector / homogeneous coordinate operations
pub trait Vector4Ops<T>
where
    T: Float + Copy,
{
    fn length(&self) -> T;
    fn length_squared(&self) -> T;
    fn normalize(&self) -> Self;
    fn dot(&self, other: &Self) -> T;
    fn x(&self) -> T;
    fn y(&self) -> T;
    fn z(&self) -> T;
    fn w(&self) -> T;
    /// Divides xyz by w to convert from homogeneous to 3D coordinates.
    fn perspective_divide(&self) -> Vector3<T>;
}

impl<T> Vector4Ops<T> for Vector4<T>
where
    T: Float + Copy,
{
    fn length(&self) -> T {
        (self.data[0] * self.data[0]
            + self.data[1] * self.data[1]
            + self.data[2] * self.data[2]
            + self.data[3] * self.data[3])
            .sqrt()
    }

    fn length_squared(&self) -> T {
        self.data[0] * self.data[0]
            + self.data[1] * self.data[1]
            + self.data[2] * self.data[2]
            + self.data[3] * self.data[3]
    }

    fn normalize(&self) -> Self {
        let len = self.length();
        if len <= T::epsilon() {
            *self
        } else {
            Vector4::new(
                self.data[0] / len,
                self.data[1] / len,
                self.data[2] / len,
                self.data[3] / len,
            )
        }
    }

    fn dot(&self, other: &Self) -> T {
        self.data[0] * other.data[0]
            + self.data[1] * other.data[1]
            + self.data[2] * other.data[2]
            + self.data[3] * other.data[3]
    }

    fn x(&self) -> T {
        self.data[0]
    }
    fn y(&self) -> T {
        self.data[1]
    }
    fn z(&self) -> T {
        self.data[2]
    }
    fn w(&self) -> T {
        self.data[3]
    }

    fn perspective_divide(&self) -> Vector3<T> {
        let w = self.data[3];
        Vector3::new(self.data[0] / w, self.data[1] / w, self.data[2] / w)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vector::{Vector2f32, Vector3f32, Vector3f64, Vector4f32};

    #[test]
    fn test_vector3_f32_operations() {
        let v = Vector3f32::new(3.0, 4.0, 0.0);
        assert_eq!(v.length(), 5.0);

        let normalized = v.normalize();
        assert!((normalized.length() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_vector3_f64_operations() {
        let v = Vector3f64::new(3.0, 4.0, 0.0);
        assert_eq!(v.length(), 5.0);

        let normalized = v.normalize();
        assert!((normalized.length() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_vector2_operations() {
        let v = Vector2f32::new(3.0, 4.0);
        assert_eq!(v.length(), 5.0);

        let perp = v.perpendicular();
        assert_eq!(perp.dot(&v), 0.0);
    }

    #[test]
    fn test_cross_product_generic() {
        let v1 = Vector3f64::new(1.0, 0.0, 0.0);
        let v2 = Vector3f64::new(0.0, 1.0, 0.0);
        let cross = v1.cross(&v2);
        assert_eq!(cross.as_slice(), [0.0, 0.0, 1.0]);
    }

    #[test]
    fn test_vector3_dot_product() {
        let v1 = Vector3f32::new(1.0, 2.0, 3.0);
        let v2 = Vector3f32::new(4.0, 5.0, 6.0);
        assert_eq!(v1.dot(&v2), 32.0); // 1*4 + 2*5 + 3*6
    }

    #[test]
    fn test_vector3_cross_parallel_vectors() {
        let v1 = Vector3f32::new(1.0, 0.0, 0.0);
        let v2 = Vector3f32::new(2.0, 0.0, 0.0);
        let cross = v1.cross(&v2);
        assert_eq!(cross.as_slice(), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_normalize_zero_vector() {
        let zero = Vector3f32::new(0.0, 0.0, 0.0);
        let result = zero.normalize();
        assert_eq!(result.as_slice(), [0.0, 0.0, 0.0]);

        let zero2 = Vector2f32::new(0.0, 0.0);
        let result2 = zero2.normalize();
        assert_eq!(result2.as_slice(), [0.0, 0.0]);
    }

    #[test]
    fn test_vector2_normalize() {
        let v = Vector2f32::new(3.0, 4.0);
        let n = v.normalize();
        assert!((n.length() - 1.0).abs() < 1e-6);
        assert!((n.x() - 0.6).abs() < 1e-6);
        assert!((n.y() - 0.8).abs() < 1e-6);
    }

    #[test]
    fn test_vector4_ops() {
        let v = Vector4f32::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(v.x(), 1.0);
        assert_eq!(v.y(), 2.0);
        assert_eq!(v.z(), 3.0);
        assert_eq!(v.w(), 4.0);
        assert!((v.length_squared() - 30.0).abs() < 1e-6);

        let v2 = Vector4f32::new(1.0, 0.0, 0.0, 0.0);
        let n = v2.normalize();
        assert!((n.length() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_vector4_dot() {
        let v1 = Vector4f32::new(1.0, 2.0, 3.0, 4.0);
        let v2 = Vector4f32::new(5.0, 6.0, 7.0, 8.0);
        assert_eq!(v1.dot(&v2), 70.0); // 1*5 + 2*6 + 3*7 + 4*8
    }

    #[test]
    fn test_vector2_cross() {
        let x = Vector2f32::new(1.0, 0.0);
        let y = Vector2f32::new(0.0, 1.0);
        assert_eq!(x.cross(&y), 1.0); // counter-clockwise -> positive
        assert_eq!(y.cross(&x), -1.0); // clockwise -> negative
        assert_eq!(x.cross(&x), 0.0); // parallel -> zero
    }

    #[test]
    fn test_vector4_perspective_divide() {
        let v = Vector4f32::new(4.0, 6.0, 8.0, 2.0);
        let p = v.perspective_divide();
        assert_eq!(p.as_slice(), [2.0, 3.0, 4.0]);
    }
}

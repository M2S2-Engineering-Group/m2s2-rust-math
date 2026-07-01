use crate::vector::Vector;
use num_traits::Float;

/// Dimension-generic Float operations shared by Vector2/3/4 (and any future Vector<T, D>).
impl<T, const D: usize> Vector<T, D>
where
    T: Float + Copy,
{
    fn dot_generic(&self, other: &Self) -> T {
        self.data
            .iter()
            .zip(other.data.iter())
            .fold(T::zero(), |acc, (&a, &b)| acc + a * b)
    }

    /// Squared distance to another vector (avoids sqrt for performance).
    pub fn distance_squared(&self, other: &Self) -> T {
        self.data
            .iter()
            .zip(other.data.iter())
            .fold(T::zero(), |acc, (&a, &b)| acc + (a - b) * (a - b))
    }

    /// Distance to another vector.
    pub fn distance(&self, other: &Self) -> T {
        self.distance_squared(other).sqrt()
    }

    /// Linearly interpolate between self and other. t=0 -> self, t=1 -> other.
    pub fn lerp(&self, other: &Self, t: T) -> Self {
        let mut data = self.data;
        for ((dst, &s), &o) in data.iter_mut().zip(self.data.iter()).zip(other.data.iter()) {
            *dst = s + (o - s) * t;
        }
        Vector { data }
    }

    /// Reflects self about a normal (normal is assumed to be unit length).
    pub fn reflect(&self, normal: &Self) -> Self {
        let two = T::one() + T::one();
        let d = self.dot_generic(normal);
        let mut data = self.data;
        for ((dst, &s), &n) in data
            .iter_mut()
            .zip(self.data.iter())
            .zip(normal.data.iter())
        {
            *dst = s - two * d * n;
        }
        Vector { data }
    }

    /// Projects self onto other. Returns the zero vector if `other` is (near) zero length.
    pub fn project_onto(&self, other: &Self) -> Self {
        let denom = other.dot_generic(other);
        if denom <= T::epsilon() {
            return Vector {
                data: [T::zero(); D],
            };
        }
        let scale = self.dot_generic(other) / denom;
        let mut data = self.data;
        for (dst, &o) in data.iter_mut().zip(other.data.iter()) {
            *dst = o * scale;
        }
        Vector { data }
    }

    /// The component of self perpendicular to other (self minus its projection onto other).
    pub fn reject_from(&self, other: &Self) -> Self {
        let proj = self.project_onto(other);
        let mut data = self.data;
        for ((dst, &s), &p) in data.iter_mut().zip(self.data.iter()).zip(proj.data.iter()) {
            *dst = s - p;
        }
        Vector { data }
    }

    /// Clamps the vector's length to `max`, preserving direction. Vectors already
    /// shorter than `max` (or zero) are returned unchanged.
    pub fn clamp_length(&self, max: T) -> Self {
        let len_sq = self.dot_generic(self);
        if len_sq <= max * max || len_sq <= T::epsilon() {
            return *self;
        }
        let len = len_sq.sqrt();
        let scale = max / len;
        let mut data = self.data;
        for (dst, &s) in data.iter_mut().zip(self.data.iter()) {
            *dst = s * scale;
        }
        Vector { data }
    }

    /// Angle between self and other, in radians, in [0, PI]. Returns 0 if either
    /// vector is (near) zero length.
    pub fn angle_between(&self, other: &Self) -> T {
        let len_product = (self.dot_generic(self) * other.dot_generic(other)).sqrt();
        if len_product <= T::epsilon() {
            return T::zero();
        }
        let cos_angle = self.dot_generic(other) / len_product;
        let clamped = if cos_angle > T::one() {
            T::one()
        } else if cos_angle < -T::one() {
            -T::one()
        } else {
            cos_angle
        };
        clamped.acos()
    }
}

#[cfg(test)]
mod tests {
    use crate::vector::vector_ops::Vector2Ops;
    use crate::vector::{Vector2f32, Vector3f32};
    use std::f32::consts::PI;

    const EPS: f32 = 1e-5;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    #[test]
    fn test_distance() {
        let a = Vector2f32::new(0.0, 0.0);
        let b = Vector2f32::new(3.0, 4.0);
        assert!(approx_eq(a.distance(&b), 5.0));
        assert!(approx_eq(a.distance_squared(&b), 25.0));
        assert!(approx_eq(a.distance(&a), 0.0));
    }

    #[test]
    fn test_lerp() {
        let a = Vector3f32::new(0.0, 0.0, 0.0);
        let b = Vector3f32::new(10.0, 20.0, 30.0);
        assert_eq!(a.lerp(&b, 0.0).as_slice(), a.as_slice());
        assert_eq!(a.lerp(&b, 1.0).as_slice(), b.as_slice());
        let mid = a.lerp(&b, 0.5);
        assert!(approx_eq(mid[0], 5.0));
        assert!(approx_eq(mid[1], 10.0));
        assert!(approx_eq(mid[2], 15.0));
    }

    #[test]
    fn test_reflect() {
        // Incoming (1, -1) hits a horizontal surface (normal (0, 1)) -> (1, 1).
        let incoming = Vector2f32::new(1.0, -1.0);
        let normal = Vector2f32::new(0.0, 1.0);
        let reflected = incoming.reflect(&normal);
        assert!(approx_eq(reflected[0], 1.0));
        assert!(approx_eq(reflected[1], 1.0));
    }

    #[test]
    fn test_reflect_straight_on() {
        // Straight into the surface bounces straight back.
        let incoming = Vector2f32::new(0.0, -1.0);
        let normal = Vector2f32::new(0.0, 1.0);
        let reflected = incoming.reflect(&normal);
        assert!(approx_eq(reflected[0], 0.0));
        assert!(approx_eq(reflected[1], 1.0));
    }

    #[test]
    fn test_project_and_reject() {
        let v = Vector2f32::new(3.0, 4.0);
        let onto = Vector2f32::new(1.0, 0.0);
        let proj = v.project_onto(&onto);
        assert!(approx_eq(proj[0], 3.0));
        assert!(approx_eq(proj[1], 0.0));

        let rej = v.reject_from(&onto);
        assert!(approx_eq(rej[0], 0.0));
        assert!(approx_eq(rej[1], 4.0));

        // proj + rej == v
        assert!(approx_eq(proj[0] + rej[0], v[0]));
        assert!(approx_eq(proj[1] + rej[1], v[1]));
    }

    #[test]
    fn test_project_onto_zero_vector() {
        let v = Vector2f32::new(3.0, 4.0);
        let zero = Vector2f32::new(0.0, 0.0);
        let proj = v.project_onto(&zero);
        assert_eq!(proj.as_slice(), [0.0, 0.0]);
    }

    #[test]
    fn test_clamp_length() {
        let v = Vector2f32::new(6.0, 8.0); // length 10
        let clamped = v.clamp_length(5.0);
        assert!(approx_eq(clamped.length(), 5.0));
        assert!(approx_eq(clamped[0], 3.0));
        assert!(approx_eq(clamped[1], 4.0));

        // Already under the max: unchanged.
        let short = Vector2f32::new(1.0, 0.0);
        let unclamped = short.clamp_length(5.0);
        assert_eq!(unclamped.as_slice(), short.as_slice());

        // Zero vector: unchanged, no NaN.
        let zero = Vector2f32::new(0.0, 0.0);
        assert_eq!(zero.clamp_length(5.0).as_slice(), [0.0, 0.0]);
    }

    #[test]
    fn test_angle_between() {
        let x = Vector2f32::new(1.0, 0.0);
        let y = Vector2f32::new(0.0, 1.0);
        assert!(approx_eq(x.angle_between(&y), PI / 2.0));

        let neg_x = Vector2f32::new(-1.0, 0.0);
        assert!(approx_eq(x.angle_between(&neg_x), PI));

        assert!(approx_eq(x.angle_between(&x), 0.0));

        let scaled = Vector2f32::new(5.0, 0.0);
        assert!(approx_eq(x.angle_between(&scaled), 0.0));
    }

    #[test]
    fn test_angle_between_zero_vector() {
        let x = Vector2f32::new(1.0, 0.0);
        let zero = Vector2f32::new(0.0, 0.0);
        assert!(approx_eq(x.angle_between(&zero), 0.0));
    }

    #[test]
    fn test_vector3_reflect() {
        let incoming = Vector3f32::new(1.0, -1.0, 0.0);
        let normal = Vector3f32::new(0.0, 1.0, 0.0);
        let reflected = incoming.reflect(&normal);
        assert!(approx_eq(reflected[0], 1.0));
        assert!(approx_eq(reflected[1], 1.0));
        assert!(approx_eq(reflected[2], 0.0));
    }
}

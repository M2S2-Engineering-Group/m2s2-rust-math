use m2s2_math::{Vector2, Vector3};
use num_traits::Float;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray2<T> {
    pub origin: Vector2<T>,
    pub direction: Vector2<T>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray3<T> {
    pub origin: Vector3<T>,
    pub direction: Vector3<T>,
}

pub type Ray2f32 = Ray2<f32>;
pub type Ray2f64 = Ray2<f64>;
pub type Ray3f32 = Ray3<f32>;
pub type Ray3f64 = Ray3<f64>;

impl<T: Float + Copy> Ray2<T> {
    pub fn new(origin: Vector2<T>, direction: Vector2<T>) -> Self {
        Ray2 { origin, direction }
    }

    /// The point at distance `t` along the ray: `origin + direction * t`.
    pub fn point_at(&self, t: T) -> Vector2<T> {
        self.origin + self.direction * t
    }
}

impl<T: Float + Copy> Ray3<T> {
    pub fn new(origin: Vector3<T>, direction: Vector3<T>) -> Self {
        Ray3 { origin, direction }
    }

    /// The point at distance `t` along the ray: `origin + direction * t`.
    pub fn point_at(&self, t: T) -> Vector3<T> {
        self.origin + self.direction * t
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ray2_point_at() {
        let ray = Ray2::new(Vector2::new(1.0, 1.0), Vector2::new(2.0, 0.0));
        let p = ray.point_at(3.0);
        assert_eq!(p.as_slice(), [7.0, 1.0]);
    }

    #[test]
    fn test_ray3_point_at() {
        let ray = Ray3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 2.0, 3.0));
        let p = ray.point_at(2.0);
        assert_eq!(p.as_slice(), [2.0, 4.0, 6.0]);
    }
}

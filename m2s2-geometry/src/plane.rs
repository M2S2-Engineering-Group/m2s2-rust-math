use m2s2_math::{Vector3, Vector3Ops};
use num_traits::Float;

use crate::ray::Ray3;

/// A plane in 3D, stored in normal form: points `p` on the plane satisfy
/// `dot(normal, p) == d`. `normal` is kept unit-length.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane<T> {
    pub normal: Vector3<T>,
    pub d: T,
}

pub type Planef32 = Plane<f32>;
pub type Planef64 = Plane<f64>;

impl<T: Float + Copy> Plane<T> {
    pub fn from_point_normal(point: Vector3<T>, normal: Vector3<T>) -> Self {
        let n = normal.normalize();
        Plane {
            normal: n,
            d: n.dot(&point),
        }
    }

    /// Constructs a plane from three points, wound counter-clockwise when
    /// viewed from the side the normal points toward.
    pub fn from_points(a: Vector3<T>, b: Vector3<T>, c: Vector3<T>) -> Self {
        let normal = (b - a).cross(&(c - a));
        Self::from_point_normal(a, normal)
    }

    /// Positive on the side the normal points toward, negative on the other side.
    pub fn signed_distance(&self, p: &Vector3<T>) -> T {
        self.normal.dot(p) - self.d
    }

    /// Ray/plane intersection distance, or `None` if the ray is parallel to the
    /// plane or points away from it.
    pub fn intersects_ray(&self, ray: &Ray3<T>) -> Option<T> {
        let denom = self.normal.dot(&ray.direction);
        if denom.abs() <= T::epsilon() {
            return None;
        }
        let t = (self.d - self.normal.dot(&ray.origin)) / denom;
        if t < T::zero() { None } else { Some(t) }
    }

    /// Line of intersection with another plane, as `(point_on_line, direction)`
    /// with `direction` unit-length. Returns `None` if the planes are parallel
    /// (including coincident).
    pub fn intersects_plane(&self, other: &Self) -> Option<(Vector3<T>, Vector3<T>)> {
        let direction = self.normal.cross(&other.normal);
        if direction.length_squared() <= T::epsilon() {
            return None;
        }

        let n1_dot_n2 = self.normal.dot(&other.normal);
        let denom = T::one() - n1_dot_n2 * n1_dot_n2;
        let a = (self.d - other.d * n1_dot_n2) / denom;
        let b = (other.d - self.d * n1_dot_n2) / denom;
        let point = self.normal * a + other.normal * b;

        Some((point, direction.normalize()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-4;
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    #[test]
    fn test_from_point_normal_and_signed_distance() {
        let plane =
            Plane::from_point_normal(Vector3::new(0.0, 5.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        assert!(approx_eq(
            plane.signed_distance(&Vector3::new(0.0, 5.0, 0.0)),
            0.0
        ));
        assert!(approx_eq(
            plane.signed_distance(&Vector3::new(0.0, 8.0, 0.0)),
            3.0
        ));
        assert!(approx_eq(
            plane.signed_distance(&Vector3::new(0.0, 2.0, 0.0)),
            -3.0
        ));
    }

    #[test]
    fn test_from_points() {
        let plane = Plane::from_points(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        );
        // XZ plane; normal should be +/- Y.
        assert!(approx_eq(plane.normal.x(), 0.0));
        assert!(approx_eq(plane.normal.z(), 0.0));
        assert!(approx_eq(plane.normal.y().abs(), 1.0));
    }

    #[test]
    fn test_intersects_ray_hit() {
        let plane =
            Plane::from_point_normal(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        let ray = Ray3::new(Vector3::new(0.0, 10.0, 0.0), Vector3::new(0.0, -1.0, 0.0));
        let t = plane.intersects_ray(&ray).expect("should hit");
        assert!(approx_eq(t, 10.0));
    }

    #[test]
    fn test_intersects_ray_parallel() {
        let plane =
            Plane::from_point_normal(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        let ray = Ray3::new(Vector3::new(0.0, 5.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        assert!(plane.intersects_ray(&ray).is_none());
    }

    #[test]
    fn test_intersects_ray_pointing_away() {
        let plane =
            Plane::from_point_normal(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        let ray = Ray3::new(Vector3::new(0.0, 10.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        assert!(plane.intersects_ray(&ray).is_none());
    }

    #[test]
    fn test_intersects_plane() {
        let xy = Plane::from_point_normal(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let xz = Plane::from_point_normal(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        let (point, direction) = xy.intersects_plane(&xz).expect("should intersect");
        assert!(approx_eq(xy.signed_distance(&point), 0.0));
        assert!(approx_eq(xz.signed_distance(&point), 0.0));
        assert!(approx_eq(direction.length(), 1.0));
        // Direction must be perpendicular to both normals.
        assert!(approx_eq(direction.dot(&xy.normal), 0.0));
        assert!(approx_eq(direction.dot(&xz.normal), 0.0));
    }

    #[test]
    fn test_intersects_plane_parallel() {
        let a = Plane::from_point_normal(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        let b = Plane::from_point_normal(Vector3::new(0.0, 5.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        assert!(a.intersects_plane(&b).is_none());
    }
}

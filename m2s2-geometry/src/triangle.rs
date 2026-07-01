use m2s2_math::{Vector3, Vector3Ops};
use num_traits::Float;

use crate::ray::Ray3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Triangle3<T> {
    pub a: Vector3<T>,
    pub b: Vector3<T>,
    pub c: Vector3<T>,
}

pub type Triangle3f32 = Triangle3<f32>;
pub type Triangle3f64 = Triangle3<f64>;

impl<T: Float + Copy> Triangle3<T> {
    pub fn new(a: Vector3<T>, b: Vector3<T>, c: Vector3<T>) -> Self {
        Triangle3 { a, b, c }
    }

    /// Double-sided Moller-Trumbore ray/triangle intersection. Rejects only on
    /// a near-zero determinant (ray parallel to the triangle's plane), not on
    /// the sign of the determinant, so backfaces register hits too.
    pub fn intersects_ray(&self, ray: &Ray3<T>) -> Option<T> {
        let edge1 = self.b - self.a;
        let edge2 = self.c - self.a;
        let h = ray.direction.cross(&edge2);
        let det = edge1.dot(&h);
        if det.abs() <= T::epsilon() {
            return None; // ray parallel to the triangle's plane
        }
        let f = T::one() / det;
        let s = ray.origin - self.a;
        let u = f * s.dot(&h);
        if u < T::zero() || u > T::one() {
            return None;
        }
        let q = s.cross(&edge1);
        let v = f * ray.direction.dot(&q);
        if v < T::zero() || u + v > T::one() {
            return None;
        }
        let t = f * edge2.dot(&q);
        if t < T::zero() { None } else { Some(t) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-4;
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    fn test_triangle() -> Triangle3f32 {
        Triangle3::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        )
    }

    #[test]
    fn test_intersects_ray_hit_centroid() {
        let tri = test_triangle();
        let ray = Ray3::new(
            Vector3::new(1.0 / 3.0, 1.0 / 3.0, 5.0),
            Vector3::new(0.0, 0.0, -1.0),
        );
        let t = tri.intersects_ray(&ray).expect("should hit");
        assert!(approx_eq(t, 5.0));
        let hit = ray.point_at(t);
        assert!(approx_eq(hit[2], 0.0));
    }

    #[test]
    fn test_intersects_ray_miss_outside_triangle() {
        let tri = test_triangle();
        // Hits the triangle's plane (z=0) but well outside its bounds.
        let ray = Ray3::new(Vector3::new(5.0, 5.0, 5.0), Vector3::new(0.0, 0.0, -1.0));
        assert!(tri.intersects_ray(&ray).is_none());
    }

    #[test]
    fn test_intersects_ray_parallel_to_plane() {
        let tri = test_triangle();
        let ray = Ray3::new(Vector3::new(0.2, 0.2, 1.0), Vector3::new(1.0, 0.0, 0.0));
        assert!(tri.intersects_ray(&ray).is_none());
    }

    #[test]
    fn test_intersects_ray_pointing_away() {
        let tri = test_triangle();
        let ray = Ray3::new(Vector3::new(0.2, 0.2, -5.0), Vector3::new(0.0, 0.0, -1.0));
        assert!(tri.intersects_ray(&ray).is_none());
    }

    #[test]
    fn test_intersects_ray_hits_backface() {
        let tri = test_triangle();
        let ray = Ray3::new(Vector3::new(0.2, 0.2, -5.0), Vector3::new(0.0, 0.0, 1.0));
        let t = tri
            .intersects_ray(&ray)
            .expect("backface hit should still register");
        assert!(approx_eq(t, 5.0));
    }

    #[test]
    fn test_intersects_ray_hits_edge_boundary() {
        let tri = test_triangle();
        let ray = Ray3::new(Vector3::new(0.5, 0.0, 5.0), Vector3::new(0.0, 0.0, -1.0));
        let t = tri
            .intersects_ray(&ray)
            .expect("boundary hit should register");
        assert!(approx_eq(t, 5.0));
    }
}

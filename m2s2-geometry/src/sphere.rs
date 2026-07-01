use m2s2_math::{Vector2, Vector2Ops, Vector3, Vector3Ops};
use num_traits::Float;

use crate::aabb::{Aabb2, Aabb3};
use crate::ray::{Ray2, Ray3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Circle<T> {
    pub center: Vector2<T>,
    pub radius: T,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sphere<T> {
    pub center: Vector3<T>,
    pub radius: T,
}

pub type Circlef32 = Circle<f32>;
pub type Circlef64 = Circle<f64>;
pub type Spheref32 = Sphere<f32>;
pub type Spheref64 = Sphere<f64>;

impl<T: Float + Copy> Circle<T> {
    pub fn new(center: Vector2<T>, radius: T) -> Self {
        Circle { center, radius }
    }

    pub fn contains_point(&self, p: &Vector2<T>) -> bool {
        self.center.distance_squared(p) <= self.radius * self.radius
    }

    /// The point on the circle's boundary closest to `p`. Degenerates to
    /// `center` if `p` coincides with `center`.
    pub fn closest_point(&self, p: &Vector2<T>) -> Vector2<T> {
        self.center + (*p - self.center).normalize() * self.radius
    }

    pub fn intersects_circle(&self, other: &Self) -> bool {
        let r = self.radius + other.radius;
        self.center.distance_squared(&other.center) <= r * r
    }

    pub fn intersects_aabb(&self, aabb: &Aabb2<T>) -> bool {
        aabb.intersects_circle(self)
    }

    /// Quadratic ray/circle intersection. Returns the nearest non-negative
    /// hit distance, or `None` if the ray misses.
    pub fn intersects_ray(&self, ray: &Ray2<T>) -> Option<T> {
        let oc = ray.origin - self.center;
        let a = ray.direction.dot(&ray.direction);
        let two = T::one() + T::one();
        let b = two * oc.dot(&ray.direction);
        let c = oc.dot(&oc) - self.radius * self.radius;
        let disc = b * b - two * two * a * c;
        if disc < T::zero() {
            return None;
        }
        let sqrt_disc = disc.sqrt();
        let t0 = (-b - sqrt_disc) / (two * a);
        let t1 = (-b + sqrt_disc) / (two * a);
        if t0 >= T::zero() {
            Some(t0)
        } else if t1 >= T::zero() {
            Some(t1)
        } else {
            None
        }
    }

    /// Time-of-impact in `[0, 1]` of two moving circles, or `None` if they
    /// don't collide during the move. Returns `Some(0)` if already overlapping.
    pub fn sweep_circle(
        &self,
        velocity: &Vector2<T>,
        other: &Self,
        other_velocity: &Vector2<T>,
    ) -> Option<T> {
        sweep_spheres(
            self.center,
            self.radius,
            *velocity,
            other.center,
            other.radius,
            *other_velocity,
            Vector2Ops::dot,
        )
    }
}

impl<T: Float + Copy> Sphere<T> {
    pub fn new(center: Vector3<T>, radius: T) -> Self {
        Sphere { center, radius }
    }

    pub fn contains_point(&self, p: &Vector3<T>) -> bool {
        self.center.distance_squared(p) <= self.radius * self.radius
    }

    /// The point on the sphere's surface closest to `p`. Degenerates to
    /// `center` if `p` coincides with `center`.
    pub fn closest_point(&self, p: &Vector3<T>) -> Vector3<T> {
        self.center + (*p - self.center).normalize() * self.radius
    }

    pub fn intersects_sphere(&self, other: &Self) -> bool {
        let r = self.radius + other.radius;
        self.center.distance_squared(&other.center) <= r * r
    }

    pub fn intersects_aabb(&self, aabb: &Aabb3<T>) -> bool {
        aabb.intersects_sphere(self)
    }

    /// Quadratic ray/sphere intersection. Returns the nearest non-negative
    /// hit distance, or `None` if the ray misses.
    pub fn intersects_ray(&self, ray: &Ray3<T>) -> Option<T> {
        let oc = ray.origin - self.center;
        let a = ray.direction.dot(&ray.direction);
        let two = T::one() + T::one();
        let b = two * oc.dot(&ray.direction);
        let c = oc.dot(&oc) - self.radius * self.radius;
        let disc = b * b - two * two * a * c;
        if disc < T::zero() {
            return None;
        }
        let sqrt_disc = disc.sqrt();
        let t0 = (-b - sqrt_disc) / (two * a);
        let t1 = (-b + sqrt_disc) / (two * a);
        if t0 >= T::zero() {
            Some(t0)
        } else if t1 >= T::zero() {
            Some(t1)
        } else {
            None
        }
    }

    /// Time-of-impact in `[0, 1]` of two moving spheres, or `None` if they
    /// don't collide during the move. Returns `Some(0)` if already overlapping.
    pub fn sweep_sphere(
        &self,
        velocity: &Vector3<T>,
        other: &Self,
        other_velocity: &Vector3<T>,
    ) -> Option<T> {
        sweep_spheres(
            self.center,
            self.radius,
            *velocity,
            other.center,
            other.radius,
            *other_velocity,
            Vector3Ops::dot,
        )
    }
}

/// Shared time-of-impact solver for two moving spheres/circles, generic over
/// Vector2/Vector3 via a caller-supplied `dot` function.
fn sweep_spheres<T, V>(
    center_a: V,
    radius_a: T,
    vel_a: V,
    center_b: V,
    radius_b: T,
    vel_b: V,
    dot: impl Fn(&V, &V) -> T,
) -> Option<T>
where
    T: Float + Copy,
    V: std::ops::Sub<Output = V> + Copy,
{
    let rel_pos = center_a - center_b;
    let rel_vel = vel_a - vel_b;
    let r = radius_a + radius_b;
    let two = T::one() + T::one();

    let c = dot(&rel_pos, &rel_pos) - r * r;
    if c <= T::zero() {
        return Some(T::zero()); // already overlapping
    }

    let a = dot(&rel_vel, &rel_vel);
    if a <= T::epsilon() {
        return None; // no relative motion, and not already overlapping
    }

    let b = two * dot(&rel_pos, &rel_vel);
    let disc = b * b - two * two * a * c;
    if disc < T::zero() {
        return None;
    }

    let t = (-b - disc.sqrt()) / (two * a);
    if t < T::zero() || t > T::one() {
        None
    } else {
        Some(t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-5;
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    #[test]
    fn test_contains_point() {
        let c = Circle::new(Vector2::new(0.0, 0.0), 2.0);
        assert!(c.contains_point(&Vector2::new(1.0, 1.0)));
        assert!(c.contains_point(&Vector2::new(2.0, 0.0))); // boundary
        assert!(!c.contains_point(&Vector2::new(3.0, 0.0)));
    }

    #[test]
    fn test_closest_point() {
        let c = Circle::new(Vector2::new(0.0, 0.0), 2.0);
        let cp = c.closest_point(&Vector2::new(10.0, 0.0));
        assert!(approx_eq(cp[0], 2.0));
        assert!(approx_eq(cp[1], 0.0));
    }

    #[test]
    fn test_intersects_circle() {
        let a = Circle::new(Vector2::new(0.0, 0.0), 1.0);
        let touching = Circle::new(Vector2::new(2.0, 0.0), 1.0);
        let separate = Circle::new(Vector2::new(10.0, 0.0), 1.0);
        assert!(a.intersects_circle(&touching));
        assert!(!a.intersects_circle(&separate));
    }

    #[test]
    fn test_sphere_intersects_ray_hit() {
        let s = Sphere::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
        let ray = Ray3::new(Vector3::new(-5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let t = s.intersects_ray(&ray).expect("should hit");
        assert!(approx_eq(t, 4.0));
    }

    #[test]
    fn test_sphere_intersects_ray_miss() {
        let s = Sphere::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
        let ray = Ray3::new(Vector3::new(-5.0, 5.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        assert!(s.intersects_ray(&ray).is_none());
    }

    #[test]
    fn test_sphere_intersects_ray_origin_inside() {
        let s = Sphere::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
        let ray = Ray3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let t = s
            .intersects_ray(&ray)
            .expect("should hit exiting the sphere");
        assert!(approx_eq(t, 1.0));
    }

    #[test]
    fn test_sphere_intersects_ray_tangent() {
        let s = Sphere::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
        let ray = Ray3::new(Vector3::new(-5.0, 1.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let t = s
            .intersects_ray(&ray)
            .expect("tangent ray should still register a hit");
        assert!(approx_eq(t, 5.0));
    }

    #[test]
    fn test_sweep_sphere_already_overlapping() {
        let a = Sphere::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
        let b = Sphere::new(Vector3::new(0.5, 0.0, 0.0), 1.0);
        let t = a
            .sweep_sphere(
                &Vector3::new(0.0, 0.0, 0.0),
                &b,
                &Vector3::new(0.0, 0.0, 0.0),
            )
            .expect("already overlapping");
        assert!(approx_eq(t, 0.0));
    }

    #[test]
    fn test_sweep_sphere_head_on() {
        let a = Sphere::new(Vector3::new(-10.0, 0.0, 0.0), 1.0);
        let b = Sphere::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
        // a moves toward b at 10 units/step; they touch when centers are 2 apart,
        // i.e. a's center reaches x = -2, a distance of 8 out of 10 -> t = 0.8
        let t = a
            .sweep_sphere(
                &Vector3::new(10.0, 0.0, 0.0),
                &b,
                &Vector3::new(0.0, 0.0, 0.0),
            )
            .expect("should collide");
        assert!(approx_eq(t, 0.8));
    }

    #[test]
    fn test_sweep_sphere_misses() {
        let a = Sphere::new(Vector3::new(-10.0, 10.0, 0.0), 1.0);
        let b = Sphere::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
        assert!(
            a.sweep_sphere(
                &Vector3::new(10.0, 0.0, 0.0),
                &b,
                &Vector3::new(0.0, 0.0, 0.0)
            )
            .is_none()
        );
    }

    #[test]
    fn test_sweep_sphere_no_relative_motion() {
        let a = Sphere::new(Vector3::new(-10.0, 0.0, 0.0), 1.0);
        let b = Sphere::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
        assert!(
            a.sweep_sphere(
                &Vector3::new(0.0, 0.0, 0.0),
                &b,
                &Vector3::new(0.0, 0.0, 0.0)
            )
            .is_none()
        );
    }
}

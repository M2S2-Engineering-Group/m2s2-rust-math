use m2s2_math::{Vector2, Vector3};
use num_traits::Float;

use crate::ray::{Ray2, Ray3};
use crate::sphere::{Circle, Sphere};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb2<T> {
    pub min: Vector2<T>,
    pub max: Vector2<T>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb3<T> {
    pub min: Vector3<T>,
    pub max: Vector3<T>,
}

pub type Aabb2f32 = Aabb2<f32>;
pub type Aabb2f64 = Aabb2<f64>;
pub type Aabb3f32 = Aabb3<f32>;
pub type Aabb3f64 = Aabb3<f64>;

impl<T: Float + Copy> Aabb2<T> {
    pub fn new(min: Vector2<T>, max: Vector2<T>) -> Self {
        Aabb2 { min, max }
    }

    pub fn from_center_half_extents(center: Vector2<T>, half_extents: Vector2<T>) -> Self {
        Aabb2 {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    pub fn center(&self) -> Vector2<T> {
        let two = T::one() + T::one();
        (self.min + self.max) / two
    }

    pub fn half_extents(&self) -> Vector2<T> {
        let two = T::one() + T::one();
        (self.max - self.min) / two
    }

    pub fn size(&self) -> Vector2<T> {
        self.max - self.min
    }

    pub fn contains_point(&self, p: &Vector2<T>) -> bool {
        p[0] >= self.min[0] && p[0] <= self.max[0] && p[1] >= self.min[1] && p[1] <= self.max[1]
    }

    pub fn intersects_aabb(&self, other: &Self) -> bool {
        self.min[0] <= other.max[0]
            && self.max[0] >= other.min[0]
            && self.min[1] <= other.max[1]
            && self.max[1] >= other.min[1]
    }

    pub fn union(&self, other: &Self) -> Self {
        Aabb2 {
            min: Vector2::new(self.min[0].min(other.min[0]), self.min[1].min(other.min[1])),
            max: Vector2::new(self.max[0].max(other.max[0]), self.max[1].max(other.max[1])),
        }
    }

    /// The point on (or in) the box closest to `p`.
    pub fn closest_point(&self, p: &Vector2<T>) -> Vector2<T> {
        Vector2::new(
            p[0].max(self.min[0]).min(self.max[0]),
            p[1].max(self.min[1]).min(self.max[1]),
        )
    }

    pub fn intersects_circle(&self, circle: &Circle<T>) -> bool {
        let closest = self.closest_point(&circle.center);
        closest.distance_squared(&circle.center) <= circle.radius * circle.radius
    }

    /// Slab-method ray/box intersection. Returns the entry distance along the
    /// ray, or `None` if the ray misses the box. If the ray origin is already
    /// inside the box, returns `Some(0)`.
    pub fn intersects_ray(&self, ray: &Ray2<T>) -> Option<T> {
        let mut t_min = T::neg_infinity();
        let mut t_max = T::infinity();
        for i in 0..2 {
            let inv_d = T::one() / ray.direction[i];
            let mut t1 = (self.min[i] - ray.origin[i]) * inv_d;
            let mut t2 = (self.max[i] - ray.origin[i]) * inv_d;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            t_min = t_min.max(t1);
            t_max = t_max.min(t2);
        }
        if t_max < t_min || t_max < T::zero() {
            None
        } else if t_min < T::zero() {
            Some(T::zero())
        } else {
            Some(t_min)
        }
    }

    /// Time-of-impact in `[0, 1]` of `self` moving by `velocity` into static `other`,
    /// or `None` if they don't collide during the move. Returns `Some(0)` if
    /// already overlapping.
    pub fn sweep_aabb(&self, velocity: &Vector2<T>, other: &Self) -> Option<T> {
        if self.intersects_aabb(other) {
            return Some(T::zero());
        }
        if velocity[0] == T::zero() && velocity[1] == T::zero() {
            return None;
        }
        let half = self.half_extents();
        let expanded = Aabb2::new(other.min - half, other.max + half);
        let ray = Ray2::new(self.center(), *velocity);
        match expanded.intersects_ray(&ray) {
            Some(t) if t <= T::one() => Some(t),
            _ => None,
        }
    }
}

impl<T: Float + Copy> Aabb3<T> {
    pub fn new(min: Vector3<T>, max: Vector3<T>) -> Self {
        Aabb3 { min, max }
    }

    pub fn from_center_half_extents(center: Vector3<T>, half_extents: Vector3<T>) -> Self {
        Aabb3 {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    pub fn center(&self) -> Vector3<T> {
        let two = T::one() + T::one();
        (self.min + self.max) / two
    }

    pub fn half_extents(&self) -> Vector3<T> {
        let two = T::one() + T::one();
        (self.max - self.min) / two
    }

    pub fn size(&self) -> Vector3<T> {
        self.max - self.min
    }

    pub fn contains_point(&self, p: &Vector3<T>) -> bool {
        p[0] >= self.min[0]
            && p[0] <= self.max[0]
            && p[1] >= self.min[1]
            && p[1] <= self.max[1]
            && p[2] >= self.min[2]
            && p[2] <= self.max[2]
    }

    pub fn intersects_aabb(&self, other: &Self) -> bool {
        self.min[0] <= other.max[0]
            && self.max[0] >= other.min[0]
            && self.min[1] <= other.max[1]
            && self.max[1] >= other.min[1]
            && self.min[2] <= other.max[2]
            && self.max[2] >= other.min[2]
    }

    pub fn union(&self, other: &Self) -> Self {
        Aabb3 {
            min: Vector3::new(
                self.min[0].min(other.min[0]),
                self.min[1].min(other.min[1]),
                self.min[2].min(other.min[2]),
            ),
            max: Vector3::new(
                self.max[0].max(other.max[0]),
                self.max[1].max(other.max[1]),
                self.max[2].max(other.max[2]),
            ),
        }
    }

    /// The point on (or in) the box closest to `p`.
    pub fn closest_point(&self, p: &Vector3<T>) -> Vector3<T> {
        Vector3::new(
            p[0].max(self.min[0]).min(self.max[0]),
            p[1].max(self.min[1]).min(self.max[1]),
            p[2].max(self.min[2]).min(self.max[2]),
        )
    }

    pub fn intersects_sphere(&self, sphere: &Sphere<T>) -> bool {
        let closest = self.closest_point(&sphere.center);
        closest.distance_squared(&sphere.center) <= sphere.radius * sphere.radius
    }

    /// Slab-method ray/box intersection. Returns the entry distance along the
    /// ray, or `None` if the ray misses the box. If the ray origin is already
    /// inside the box, returns `Some(0)`.
    pub fn intersects_ray(&self, ray: &Ray3<T>) -> Option<T> {
        let mut t_min = T::neg_infinity();
        let mut t_max = T::infinity();
        for i in 0..3 {
            let inv_d = T::one() / ray.direction[i];
            let mut t1 = (self.min[i] - ray.origin[i]) * inv_d;
            let mut t2 = (self.max[i] - ray.origin[i]) * inv_d;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            t_min = t_min.max(t1);
            t_max = t_max.min(t2);
        }
        if t_max < t_min || t_max < T::zero() {
            None
        } else if t_min < T::zero() {
            Some(T::zero())
        } else {
            Some(t_min)
        }
    }

    /// Time-of-impact in `[0, 1]` of `self` moving by `velocity` into static `other`,
    /// or `None` if they don't collide during the move. Returns `Some(0)` if
    /// already overlapping.
    pub fn sweep_aabb(&self, velocity: &Vector3<T>, other: &Self) -> Option<T> {
        if self.intersects_aabb(other) {
            return Some(T::zero());
        }
        if velocity[0] == T::zero() && velocity[1] == T::zero() && velocity[2] == T::zero() {
            return None;
        }
        let half = self.half_extents();
        let expanded = Aabb3::new(other.min - half, other.max + half);
        let ray = Ray3::new(self.center(), *velocity);
        match expanded.intersects_ray(&ray) {
            Some(t) if t <= T::one() => Some(t),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sphere::{Circle, Sphere};

    const EPS: f32 = 1e-5;
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    #[test]
    fn test_center_half_extents_size() {
        let b = Aabb2::new(Vector2::new(0.0, 0.0), Vector2::new(4.0, 2.0));
        assert_eq!(b.center().as_slice(), [2.0, 1.0]);
        assert_eq!(b.half_extents().as_slice(), [2.0, 1.0]);
        assert_eq!(b.size().as_slice(), [4.0, 2.0]);

        let b2 = Aabb2::from_center_half_extents(Vector2::new(2.0, 1.0), Vector2::new(2.0, 1.0));
        assert_eq!(b2.min.as_slice(), [0.0, 0.0]);
        assert_eq!(b2.max.as_slice(), [4.0, 2.0]);
    }

    #[test]
    fn test_contains_point() {
        let b = Aabb2::new(Vector2::new(0.0, 0.0), Vector2::new(2.0, 2.0));
        assert!(b.contains_point(&Vector2::new(1.0, 1.0)));
        assert!(b.contains_point(&Vector2::new(0.0, 0.0))); // boundary inclusive
        assert!(!b.contains_point(&Vector2::new(3.0, 1.0)));
    }

    #[test]
    fn test_intersects_aabb() {
        let a = Aabb2::new(Vector2::new(0.0, 0.0), Vector2::new(2.0, 2.0));
        let overlapping = Aabb2::new(Vector2::new(1.0, 1.0), Vector2::new(3.0, 3.0));
        let touching = Aabb2::new(Vector2::new(2.0, 0.0), Vector2::new(4.0, 2.0));
        let separate = Aabb2::new(Vector2::new(5.0, 5.0), Vector2::new(6.0, 6.0));
        assert!(a.intersects_aabb(&overlapping));
        assert!(a.intersects_aabb(&touching)); // touching edges count as intersecting
        assert!(!a.intersects_aabb(&separate));
    }

    #[test]
    fn test_union() {
        let a = Aabb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0));
        let b = Aabb2::new(Vector2::new(-1.0, 2.0), Vector2::new(3.0, 4.0));
        let u = a.union(&b);
        assert_eq!(u.min.as_slice(), [-1.0, 0.0]);
        assert_eq!(u.max.as_slice(), [3.0, 4.0]);
    }

    #[test]
    fn test_closest_point() {
        let b = Aabb2::new(Vector2::new(0.0, 0.0), Vector2::new(2.0, 2.0));
        // Outside -> clamps to nearest edge.
        assert_eq!(
            b.closest_point(&Vector2::new(5.0, 1.0)).as_slice(),
            [2.0, 1.0]
        );
        // Inside -> unchanged.
        assert_eq!(
            b.closest_point(&Vector2::new(1.0, 1.0)).as_slice(),
            [1.0, 1.0]
        );
    }

    #[test]
    fn test_aabb2_intersects_circle() {
        let b = Aabb2::new(Vector2::new(0.0, 0.0), Vector2::new(2.0, 2.0));
        let touching = Circle::new(Vector2::new(3.0, 1.0), 1.0); // exactly touches edge x=2
        let overlapping = Circle::new(Vector2::new(3.0, 1.0), 1.5);
        let separate = Circle::new(Vector2::new(10.0, 10.0), 1.0);
        assert!(b.intersects_circle(&touching));
        assert!(b.intersects_circle(&overlapping));
        assert!(!b.intersects_circle(&separate));
    }

    #[test]
    fn test_aabb3_intersects_sphere() {
        let b = Aabb3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0));
        let overlapping = Sphere::new(Vector3::new(3.0, 1.0, 1.0), 1.5);
        let separate = Sphere::new(Vector3::new(10.0, 10.0, 10.0), 1.0);
        assert!(b.intersects_sphere(&overlapping));
        assert!(!b.intersects_sphere(&separate));
    }

    #[test]
    fn test_aabb3_intersects_ray_hit() {
        let b = Aabb3::new(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
        let ray = Ray3::new(Vector3::new(-5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let t = b.intersects_ray(&ray).expect("should hit");
        assert!(approx_eq(t, 4.0));
        let hit_point = ray.point_at(t);
        assert!(approx_eq(hit_point[0], -1.0));
    }

    #[test]
    fn test_aabb3_intersects_ray_miss() {
        let b = Aabb3::new(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
        // Parallel to the box, offset outside it.
        let ray = Ray3::new(Vector3::new(-5.0, 5.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        assert!(b.intersects_ray(&ray).is_none());
    }

    #[test]
    fn test_aabb3_intersects_ray_pointing_away() {
        let b = Aabb3::new(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
        let ray = Ray3::new(Vector3::new(-5.0, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0));
        assert!(b.intersects_ray(&ray).is_none());
    }

    #[test]
    fn test_aabb3_intersects_ray_origin_inside() {
        let b = Aabb3::new(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
        let ray = Ray3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let t = b
            .intersects_ray(&ray)
            .expect("origin inside should hit at t=0");
        assert!(approx_eq(t, 0.0));
    }

    #[test]
    fn test_sweep_aabb_already_overlapping() {
        let a = Aabb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0));
        let b = Aabb2::new(Vector2::new(0.5, 0.5), Vector2::new(1.5, 1.5));
        let t = a
            .sweep_aabb(&Vector2::new(1.0, 0.0), &b)
            .expect("already overlapping");
        assert!(approx_eq(t, 0.0));
    }

    #[test]
    fn test_sweep_aabb_hits() {
        // Moving box a from far left toward static box b.
        let a = Aabb2::new(Vector2::new(-5.0, 0.0), Vector2::new(-4.0, 1.0)); // width/height 1
        let b = Aabb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0));
        let velocity = Vector2::new(10.0, 0.0); // moves far enough to reach and pass b in one step
        let t = a.sweep_aabb(&velocity, &b).expect("should collide");
        // a's right edge (-4) must reach b's left edge (0): needs to travel 4 units in x,
        // out of 10 available -> t = 0.4
        assert!(approx_eq(t, 0.4));
    }

    #[test]
    fn test_sweep_aabb_misses() {
        let a = Aabb2::new(Vector2::new(-5.0, 0.0), Vector2::new(-4.0, 1.0));
        let b = Aabb2::new(Vector2::new(0.0, 5.0), Vector2::new(1.0, 6.0)); // out of the way
        let velocity = Vector2::new(10.0, 0.0);
        assert!(a.sweep_aabb(&velocity, &b).is_none());
    }

    #[test]
    fn test_sweep_aabb_not_enough_travel() {
        let a = Aabb2::new(Vector2::new(-5.0, 0.0), Vector2::new(-4.0, 1.0));
        let b = Aabb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0));
        let velocity = Vector2::new(1.0, 0.0); // not enough to reach b within this step
        assert!(a.sweep_aabb(&velocity, &b).is_none());
    }

    #[test]
    fn test_sweep_aabb_stationary_no_overlap() {
        let a = Aabb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0));
        let b = Aabb2::new(Vector2::new(5.0, 5.0), Vector2::new(6.0, 6.0));
        assert!(a.sweep_aabb(&Vector2::new(0.0, 0.0), &b).is_none());
    }
}

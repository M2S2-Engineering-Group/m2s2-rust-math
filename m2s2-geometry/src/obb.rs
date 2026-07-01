use m2s2_math::{Quaternion, Vector2, Vector2Ops, Vector3, Vector3Ops};
use num_traits::Float;

use crate::aabb::{Aabb2, Aabb3};
use crate::ray::{Ray2, Ray3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obb2<T> {
    pub center: Vector2<T>,
    pub half_extents: Vector2<T>,
    pub rotation: T,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obb3<T> {
    pub center: Vector3<T>,
    pub half_extents: Vector3<T>,
    pub orientation: Quaternion<T>,
}

pub type Obb2f32 = Obb2<f32>;
pub type Obb2f64 = Obb2<f64>;
pub type Obb3f32 = Obb3<f32>;
pub type Obb3f64 = Obb3<f64>;

impl<T: Float + Copy> Obb2<T> {
    pub fn new(center: Vector2<T>, half_extents: Vector2<T>, rotation: T) -> Self {
        Obb2 {
            center,
            half_extents,
            rotation,
        }
    }

    /// The box's two orthonormal local axes (local +X, local +Y) in world space.
    pub fn axes(&self) -> (Vector2<T>, Vector2<T>) {
        let (s, c) = self.rotation.sin_cos();
        let x_axis = Vector2::new(c, s);
        let y_axis = x_axis.perpendicular();
        (x_axis, y_axis)
    }

    pub fn contains_point(&self, p: &Vector2<T>) -> bool {
        let d = *p - self.center;
        let (ax, ay) = self.axes();
        d.dot(&ax).abs() <= self.half_extents.x() && d.dot(&ay).abs() <= self.half_extents.y()
    }

    fn projected_radius(
        half_extents: Vector2<T>,
        axes: (Vector2<T>, Vector2<T>),
        axis: Vector2<T>,
    ) -> T {
        half_extents.x() * axes.0.dot(&axis).abs() + half_extents.y() * axes.1.dot(&axis).abs()
    }

    /// SAT via the 4 candidate separating axes (each box's own 2 axes). In 2D
    /// this set is provably sufficient: an edge's outward normal is always one
    /// of its own box's 2 axes, and there is no third-dimension "edge cross
    /// product" axis the way there is in 3D SAT.
    pub fn intersects_obb(&self, other: &Self) -> bool {
        let center_delta = other.center - self.center;
        let axes_a = self.axes();
        let axes_b = other.axes();

        for axis in [axes_a.0, axes_a.1, axes_b.0, axes_b.1] {
            let ra = Self::projected_radius(self.half_extents, axes_a, axis);
            let rb = Self::projected_radius(other.half_extents, axes_b, axis);
            if center_delta.dot(&axis).abs() > ra + rb {
                return false; // separating axis found
            }
        }
        true
    }

    /// Treats the AABB as a zero-rotation OBB and reuses `intersects_obb`.
    pub fn intersects_aabb(&self, aabb: &Aabb2<T>) -> bool {
        let as_obb = Obb2::new(aabb.center(), aabb.half_extents(), T::zero());
        self.intersects_obb(&as_obb)
    }

    /// Transforms the ray into the box's local (axis-aligned) frame and reuses
    /// `Aabb2::intersects_ray`.
    pub fn intersects_ray(&self, ray: &Ray2<T>) -> Option<T> {
        let (ax, ay) = self.axes();
        let rel_origin = ray.origin - self.center;
        let local_origin = Vector2::new(rel_origin.dot(&ax), rel_origin.dot(&ay));
        let local_direction = Vector2::new(ray.direction.dot(&ax), ray.direction.dot(&ay));
        let local_box = Aabb2::new(-self.half_extents, self.half_extents);
        local_box.intersects_ray(&Ray2::new(local_origin, local_direction))
    }
}

impl<T: Float + Copy> Obb3<T> {
    pub fn new(center: Vector3<T>, half_extents: Vector3<T>, orientation: Quaternion<T>) -> Self {
        Obb3 {
            center,
            half_extents,
            orientation,
        }
    }

    /// The box's three orthonormal local axes (local +X, +Y, +Z) in world space.
    pub fn axes(&self) -> [Vector3<T>; 3] {
        [
            self.orientation
                .rotate_vector(Vector3::new(T::one(), T::zero(), T::zero())),
            self.orientation
                .rotate_vector(Vector3::new(T::zero(), T::one(), T::zero())),
            self.orientation
                .rotate_vector(Vector3::new(T::zero(), T::zero(), T::one())),
        ]
    }

    pub fn contains_point(&self, p: &Vector3<T>) -> bool {
        let d = *p - self.center;
        let axes = self.axes();
        let e = self.half_extents;
        d.dot(&axes[0]).abs() <= e.x()
            && d.dot(&axes[1]).abs() <= e.y()
            && d.dot(&axes[2]).abs() <= e.z()
    }

    /// Separating Axis Theorem test over all 15 candidate axes (3 face normals
    /// from each box, plus the 9 pairwise edge cross products) — the classical
    /// OBB-OBB test (Ericson, *Real-Time Collision Detection* SS4.4.1).
    pub fn intersects_obb(&self, other: &Self) -> bool {
        sat_obb3(
            self.center,
            self.half_extents,
            self.axes(),
            other.center,
            other.half_extents,
            other.axes(),
        )
    }

    /// Treats the AABB as an identity-orientation OBB and reuses `intersects_obb`.
    pub fn intersects_aabb(&self, aabb: &Aabb3<T>) -> bool {
        let as_obb = Obb3::new(aabb.center(), aabb.half_extents(), Quaternion::identity());
        self.intersects_obb(&as_obb)
    }

    /// Transforms the ray into the box's local (axis-aligned) frame and reuses
    /// `Aabb3::intersects_ray`.
    pub fn intersects_ray(&self, ray: &Ray3<T>) -> Option<T> {
        let axes = self.axes();
        let rel_origin = ray.origin - self.center;
        let local_origin = Vector3::new(
            rel_origin.dot(&axes[0]),
            rel_origin.dot(&axes[1]),
            rel_origin.dot(&axes[2]),
        );
        let local_direction = Vector3::new(
            ray.direction.dot(&axes[0]),
            ray.direction.dot(&axes[1]),
            ray.direction.dot(&axes[2]),
        );
        let local_box = Aabb3::new(-self.half_extents, self.half_extents);
        local_box.intersects_ray(&Ray3::new(local_origin, local_direction))
    }
}

fn sat_obb3<T: Float + Copy>(
    ca: Vector3<T>,
    ea: Vector3<T>,
    axa: [Vector3<T>; 3],
    cb: Vector3<T>,
    eb: Vector3<T>,
    axb: [Vector3<T>; 3],
) -> bool {
    let ea = [ea.x(), ea.y(), ea.z()];
    let eb = [eb.x(), eb.y(), eb.z()];

    // R[i][j] = dot(A's axis i, B's axis j): expresses B's axes in A's frame.
    let mut r = [[T::zero(); 3]; 3];
    for (i, axis_a) in axa.iter().enumerate() {
        for (j, axis_b) in axb.iter().enumerate() {
            r[i][j] = axis_a.dot(axis_b);
        }
    }

    // A small epsilon added to every |R| entry robustly handles near-parallel
    // edges: it prevents a genuinely-degenerate cross-product axis (~zero
    // vector) from ever being reported as a false separating axis, without an
    // explicit skip branch. Mirrors the tolerance pattern used in Quaternion::slerp.
    let eps = T::from(1e-6).unwrap_or_else(T::epsilon);
    let mut abs_r = [[T::zero(); 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            abs_r[i][j] = r[i][j].abs() + eps;
        }
    }

    // Translation vector, expressed in A's local frame.
    let t_world = cb - ca;
    let t = [
        t_world.dot(&axa[0]),
        t_world.dot(&axa[1]),
        t_world.dot(&axa[2]),
    ];

    // Test axes L = A0, A1, A2 (A's own face normals).
    for i in 0..3 {
        let ra = ea[i];
        let rb = eb[0] * abs_r[i][0] + eb[1] * abs_r[i][1] + eb[2] * abs_r[i][2];
        if t[i].abs() > ra + rb {
            return false;
        }
    }

    // Test axes L = B0, B1, B2 (B's own face normals).
    for (j, &rb) in eb.iter().enumerate() {
        let ra = ea[0] * abs_r[0][j] + ea[1] * abs_r[1][j] + ea[2] * abs_r[2][j];
        let proj = t[0] * r[0][j] + t[1] * r[1][j] + t[2] * r[2][j];
        if proj.abs() > ra + rb {
            return false;
        }
    }

    // Test the 9 axes L = Ai x Bj.
    macro_rules! test_cross_axis {
        ($ra:expr, $rb:expr, $proj:expr) => {
            if ($proj).abs() > ($ra) + ($rb) {
                return false;
            }
        };
    }
    test_cross_axis!(
        ea[1] * abs_r[2][0] + ea[2] * abs_r[1][0],
        eb[1] * abs_r[0][2] + eb[2] * abs_r[0][1],
        t[2] * r[1][0] - t[1] * r[2][0]
    ); // A0 x B0
    test_cross_axis!(
        ea[1] * abs_r[2][1] + ea[2] * abs_r[1][1],
        eb[0] * abs_r[0][2] + eb[2] * abs_r[0][0],
        t[2] * r[1][1] - t[1] * r[2][1]
    ); // A0 x B1
    test_cross_axis!(
        ea[1] * abs_r[2][2] + ea[2] * abs_r[1][2],
        eb[0] * abs_r[0][1] + eb[1] * abs_r[0][0],
        t[2] * r[1][2] - t[1] * r[2][2]
    ); // A0 x B2
    test_cross_axis!(
        ea[0] * abs_r[2][0] + ea[2] * abs_r[0][0],
        eb[1] * abs_r[1][2] + eb[2] * abs_r[1][1],
        t[0] * r[2][0] - t[2] * r[0][0]
    ); // A1 x B0
    test_cross_axis!(
        ea[0] * abs_r[2][1] + ea[2] * abs_r[0][1],
        eb[0] * abs_r[1][2] + eb[2] * abs_r[1][0],
        t[0] * r[2][1] - t[2] * r[0][1]
    ); // A1 x B1
    test_cross_axis!(
        ea[0] * abs_r[2][2] + ea[2] * abs_r[0][2],
        eb[0] * abs_r[1][1] + eb[1] * abs_r[1][0],
        t[0] * r[2][2] - t[2] * r[0][2]
    ); // A1 x B2
    test_cross_axis!(
        ea[0] * abs_r[1][0] + ea[1] * abs_r[0][0],
        eb[1] * abs_r[2][2] + eb[2] * abs_r[2][1],
        t[1] * r[0][0] - t[0] * r[1][0]
    ); // A2 x B0
    test_cross_axis!(
        ea[0] * abs_r[1][1] + ea[1] * abs_r[0][1],
        eb[0] * abs_r[2][2] + eb[2] * abs_r[2][0],
        t[1] * r[0][1] - t[0] * r[1][1]
    ); // A2 x B1
    test_cross_axis!(
        ea[0] * abs_r[1][2] + ea[1] * abs_r[0][2],
        eb[0] * abs_r[2][1] + eb[1] * abs_r[2][0],
        t[1] * r[0][2] - t[0] * r[1][2]
    ); // A2 x B2

    true // no separating axis found
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    const EPS: f32 = 1e-4;
    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPS
    }

    // --- Obb2 ---

    #[test]
    fn test_obb2_axes_identity_rotation() {
        let obb = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        let (ax, ay) = obb.axes();
        assert!(approx_eq(ax[0], 1.0) && approx_eq(ax[1], 0.0));
        assert!(approx_eq(ay[0], 0.0) && approx_eq(ay[1], 1.0));
    }

    #[test]
    fn test_obb2_axes_90_degrees() {
        let obb = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), PI / 2.0);
        let (ax, ay) = obb.axes();
        assert!(approx_eq(ax[0], 0.0) && approx_eq(ax[1], 1.0));
        assert!(approx_eq(ay[0], -1.0) && approx_eq(ay[1], 0.0));
    }

    #[test]
    fn test_obb2_contains_point_rotated() {
        let obb = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(2.0, 1.0), PI / 4.0);
        // Local-frame coords of (1.4, 1.4) under a 45 degree rotation are
        // approximately (1.98, 0.0), inside half_extents (2, 1).
        assert!(obb.contains_point(&Vector2::new(1.4, 1.4)));
        // Far along global X only, outside even accounting for rotation.
        assert!(!obb.contains_point(&Vector2::new(3.0, 0.0)));
    }

    #[test]
    fn test_obb2_intersects_obb_axis_aligned_separated() {
        let a = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        let b = Obb2::new(Vector2::new(5.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        assert!(!a.intersects_obb(&b));
    }

    #[test]
    fn test_obb2_intersects_obb_axis_aligned_touching() {
        let a = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        let b = Obb2::new(Vector2::new(2.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        assert!(a.intersects_obb(&b)); // touching at x=1, boundary inclusive
    }

    #[test]
    fn test_obb2_intersects_obb_overlapping() {
        let a = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        let b = Obb2::new(Vector2::new(1.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        assert!(a.intersects_obb(&b));
    }

    #[test]
    fn test_obb2_intersects_obb_rotated_overlapping() {
        let a = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        let b = Obb2::new(Vector2::new(1.2, 0.0), Vector2::new(1.0, 1.0), PI / 4.0);
        assert!(a.intersects_obb(&b));
    }

    #[test]
    fn test_obb2_intersects_obb_rotated_separated() {
        let a = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        let b = Obb2::new(Vector2::new(3.0, 0.0), Vector2::new(1.0, 1.0), PI / 4.0);
        assert!(!a.intersects_obb(&b));
    }

    #[test]
    fn test_obb2_intersects_aabb() {
        let obb = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        let overlapping = Aabb2::new(Vector2::new(0.5, 0.5), Vector2::new(2.0, 2.0));
        let separate = Aabb2::new(Vector2::new(10.0, 10.0), Vector2::new(11.0, 11.0));
        assert!(obb.intersects_aabb(&overlapping));
        assert!(!obb.intersects_aabb(&separate));
    }

    #[test]
    fn test_obb2_intersects_ray_rotated_hit() {
        let obb = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), PI / 4.0);
        let ray = Ray2::new(Vector2::new(-5.0, 0.0), Vector2::new(1.0, 0.0));
        assert!(obb.intersects_ray(&ray).is_some());
    }

    #[test]
    fn test_obb2_intersects_ray_rotated_miss() {
        let obb = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), PI / 4.0);
        let ray = Ray2::new(Vector2::new(-5.0, 5.0), Vector2::new(1.0, 0.0));
        assert!(obb.intersects_ray(&ray).is_none());
    }

    #[test]
    fn test_obb2_intersects_ray_axis_aligned_matches_aabb() {
        let obb = Obb2::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 1.0), 0.0);
        let aabb = Aabb2::new(Vector2::new(-1.0, -1.0), Vector2::new(1.0, 1.0));
        let ray = Ray2::new(Vector2::new(-5.0, 0.0), Vector2::new(1.0, 0.0));
        let t_obb = obb.intersects_ray(&ray).expect("should hit");
        let t_aabb = aabb.intersects_ray(&ray).expect("should hit");
        assert!(approx_eq(t_obb, t_aabb));
    }

    // --- Obb3 ---

    #[test]
    fn test_obb3_axes_identity_orientation() {
        let obb = Obb3::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Quaternion::identity(),
        );
        let axes = obb.axes();
        assert!(
            approx_eq(axes[0][0], 1.0) && approx_eq(axes[0][1], 0.0) && approx_eq(axes[0][2], 0.0)
        );
        assert!(
            approx_eq(axes[1][0], 0.0) && approx_eq(axes[1][1], 1.0) && approx_eq(axes[1][2], 0.0)
        );
        assert!(
            approx_eq(axes[2][0], 0.0) && approx_eq(axes[2][1], 0.0) && approx_eq(axes[2][2], 1.0)
        );
    }

    #[test]
    fn test_obb3_axes_after_rotation() {
        let q = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), PI / 2.0);
        let obb = Obb3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0), q);
        let axes = obb.axes();
        // Rotating +X by 90 degrees about +Z gives +Y.
        assert!(
            approx_eq(axes[0][0], 0.0) && approx_eq(axes[0][1], 1.0) && approx_eq(axes[0][2], 0.0)
        );
    }

    #[test]
    fn test_obb3_contains_point_rotated() {
        let q = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), PI / 4.0);
        let obb = Obb3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 1.0, 1.0), q);
        assert!(obb.contains_point(&Vector3::new(1.4, 1.4, 0.0)));
        assert!(!obb.contains_point(&Vector3::new(3.0, 0.0, 0.0)));
    }

    #[test]
    fn test_obb3_intersects_obb_axis_aligned_separated() {
        let a = Obb3::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Quaternion::identity(),
        );
        let b = Obb3::new(
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Quaternion::identity(),
        );
        assert!(!a.intersects_obb(&b));
    }

    #[test]
    fn test_obb3_intersects_obb_axis_aligned_touching() {
        let a = Obb3::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Quaternion::identity(),
        );
        let b = Obb3::new(
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Quaternion::identity(),
        );
        assert!(a.intersects_obb(&b));
    }

    #[test]
    fn test_obb3_intersects_obb_overlapping() {
        let a = Obb3::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Quaternion::identity(),
        );
        let b = Obb3::new(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Quaternion::identity(),
        );
        assert!(a.intersects_obb(&b));
    }

    #[test]
    fn test_obb3_intersects_obb_rotated_overlapping() {
        let q = Quaternion::from_axis_angle(Vector3::new(1.0, 1.0, 0.0), PI / 4.0);
        let a = Obb3::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Quaternion::identity(),
        );
        let b = Obb3::new(Vector3::new(0.5, 0.5, 0.0), Vector3::new(1.0, 1.0, 1.0), q);
        assert!(a.intersects_obb(&b));
    }

    /// The critical SAT edge case: two long "rod" boxes arranged so all 6
    /// face-normal axes report overlap, but exactly one of the 9 edge
    /// cross-product axes (A0 x B1) separates them. A face-only SAT
    /// implementation would incorrectly report this as overlapping.
    #[test]
    fn test_obb3_intersects_obb_edge_axis_only_separation() {
        let a = Obb3::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.5, 0.5),
            Quaternion::identity(),
        );
        let axis = Vector3::new(
            0.42080293486615905,
            -0.5919336873204334,
            -0.6874150128003188,
        );
        let q = Quaternion::from_axis_angle(axis, 1.1357651039198697);
        let separated = Obb3::new(
            Vector3::new(-0.2689317283797865, 1.049120329831768, -1.9915757865955572),
            Vector3::new(0.5, 4.0, 0.5),
            q,
        );
        assert!(!a.intersects_obb(&separated));

        // Same pair, moved close together: no axis (face or edge) separates them.
        let overlapping = Obb3::new(
            Vector3::new(0.05, 0.05, 0.05),
            Vector3::new(0.5, 4.0, 0.5),
            q,
        );
        assert!(a.intersects_obb(&overlapping));

        // Same pair, moved far apart: trivially separated by a face axis.
        let far = Obb3::new(
            Vector3::new(20.0, 20.0, 20.0),
            Vector3::new(0.5, 4.0, 0.5),
            q,
        );
        assert!(!a.intersects_obb(&far));
    }

    #[test]
    fn test_obb3_intersects_aabb() {
        let obb = Obb3::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Quaternion::identity(),
        );
        let overlapping = Aabb3::new(Vector3::new(0.5, 0.5, 0.5), Vector3::new(2.0, 2.0, 2.0));
        let separate = Aabb3::new(
            Vector3::new(10.0, 10.0, 10.0),
            Vector3::new(11.0, 11.0, 11.0),
        );
        assert!(obb.intersects_aabb(&overlapping));
        assert!(!obb.intersects_aabb(&separate));
    }

    #[test]
    fn test_obb3_intersects_ray_rotated_hit() {
        let q = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), PI / 4.0);
        let obb = Obb3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0), q);
        let ray = Ray3::new(Vector3::new(-5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        assert!(obb.intersects_ray(&ray).is_some());
    }

    #[test]
    fn test_obb3_intersects_ray_rotated_miss() {
        let q = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), PI / 4.0);
        let obb = Obb3::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0), q);
        let ray = Ray3::new(Vector3::new(-5.0, 5.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        assert!(obb.intersects_ray(&ray).is_none());
    }

    #[test]
    fn test_obb3_intersects_ray_axis_aligned_matches_aabb() {
        let obb = Obb3::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Quaternion::identity(),
        );
        let aabb = Aabb3::new(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
        let ray = Ray3::new(Vector3::new(-5.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let t_obb = obb.intersects_ray(&ray).expect("should hit");
        let t_aabb = aabb.intersects_ray(&ray).expect("should hit");
        assert!(approx_eq(t_obb, t_aabb));
    }
}

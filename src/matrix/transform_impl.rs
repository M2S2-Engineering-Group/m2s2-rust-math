use crate::matrix::transform_traits::{Transform2x2, Transform3x3, Transform4x4};
use crate::matrix::{Matrix2x2, Matrix3x3, Matrix4x4};
use crate::vector::vector_ops::{Vector2Ops, Vector3Ops};
use crate::vector::{Vector2, Vector3};
use num_traits::{One, Zero};

macro_rules! impl_transform_4x4 {
    ($matrix_type:ident, $float_type:ty) => {
        impl Transform4x4<$float_type> for $matrix_type<$float_type> {
            // --- Perspective ---

            fn perspective_rh_zo(
                fov_y_radians: $float_type,
                aspect_ratio: $float_type,
                near: $float_type,
                far: $float_type,
            ) -> Self {
                let f = <$float_type>::one()
                    / (fov_y_radians / (<$float_type>::one() + <$float_type>::one())).tan();
                Self::from_2d_array([
                    [
                        f / aspect_ratio,
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        f,
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        far / (near - far),
                        (near * far) / (near - far),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        -<$float_type>::one(),
                        <$float_type>::zero(),
                    ],
                ])
            }

            fn perspective_rh_no(
                fov_y_radians: $float_type,
                aspect_ratio: $float_type,
                near: $float_type,
                far: $float_type,
            ) -> Self {
                let f = <$float_type>::one()
                    / (fov_y_radians / (<$float_type>::one() + <$float_type>::one())).tan();
                let two = <$float_type>::one() + <$float_type>::one();
                Self::from_2d_array([
                    [
                        f / aspect_ratio,
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        f,
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        (far + near) / (near - far),
                        (two * far * near) / (near - far),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        -<$float_type>::one(),
                        <$float_type>::zero(),
                    ],
                ])
            }

            fn perspective_lh_zo(
                fov_y_radians: $float_type,
                aspect_ratio: $float_type,
                near: $float_type,
                far: $float_type,
            ) -> Self {
                let f = <$float_type>::one()
                    / (fov_y_radians / (<$float_type>::one() + <$float_type>::one())).tan();
                Self::from_2d_array([
                    [
                        f / aspect_ratio,
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        f,
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        far / (far - near),
                        -(far * near) / (far - near),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                        <$float_type>::zero(),
                    ],
                ])
            }

            fn perspective_lh_no(
                fov_y_radians: $float_type,
                aspect_ratio: $float_type,
                near: $float_type,
                far: $float_type,
            ) -> Self {
                let f = <$float_type>::one()
                    / (fov_y_radians / (<$float_type>::one() + <$float_type>::one())).tan();
                let two = <$float_type>::one() + <$float_type>::one();
                Self::from_2d_array([
                    [
                        f / aspect_ratio,
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        f,
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        (far + near) / (far - near),
                        -(two * far * near) / (far - near),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                        <$float_type>::zero(),
                    ],
                ])
            }

            // --- Orthographic ---

            fn ortho_rh_no(
                left: $float_type,
                right: $float_type,
                bottom: $float_type,
                top: $float_type,
                near: $float_type,
                far: $float_type,
            ) -> Self {
                let two = <$float_type>::one() + <$float_type>::one();
                Self::from_2d_array([
                    [
                        two / (right - left),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        -(right + left) / (right - left),
                    ],
                    [
                        <$float_type>::zero(),
                        two / (top - bottom),
                        <$float_type>::zero(),
                        -(top + bottom) / (top - bottom),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        -two / (far - near),
                        -(far + near) / (far - near),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn ortho_rh_zo(
                left: $float_type,
                right: $float_type,
                bottom: $float_type,
                top: $float_type,
                near: $float_type,
                far: $float_type,
            ) -> Self {
                let two = <$float_type>::one() + <$float_type>::one();
                Self::from_2d_array([
                    [
                        two / (right - left),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        -(right + left) / (right - left),
                    ],
                    [
                        <$float_type>::zero(),
                        two / (top - bottom),
                        <$float_type>::zero(),
                        -(top + bottom) / (top - bottom),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        -<$float_type>::one() / (far - near),
                        -near / (far - near),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn ortho_lh_zo(
                left: $float_type,
                right: $float_type,
                bottom: $float_type,
                top: $float_type,
                near: $float_type,
                far: $float_type,
            ) -> Self {
                let two = <$float_type>::one() + <$float_type>::one();
                Self::from_2d_array([
                    [
                        two / (right - left),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        -(right + left) / (right - left),
                    ],
                    [
                        <$float_type>::zero(),
                        two / (top - bottom),
                        <$float_type>::zero(),
                        -(top + bottom) / (top - bottom),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one() / (far - near),
                        -near / (far - near),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn ortho_lh_no(
                left: $float_type,
                right: $float_type,
                bottom: $float_type,
                top: $float_type,
                near: $float_type,
                far: $float_type,
            ) -> Self {
                let two = <$float_type>::one() + <$float_type>::one();
                Self::from_2d_array([
                    [
                        two / (right - left),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        -(right + left) / (right - left),
                    ],
                    [
                        <$float_type>::zero(),
                        two / (top - bottom),
                        <$float_type>::zero(),
                        -(top + bottom) / (top - bottom),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        two / (far - near),
                        -(far + near) / (far - near),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            // --- Look-at ---

            fn look_at_rh(
                eye: Vector3<$float_type>,
                target: Vector3<$float_type>,
                up: Vector3<$float_type>,
            ) -> Self {
                let forward = (target - eye).normalize();
                let right = forward.cross(&up).normalize();
                let up = right.cross(&forward);

                let zero = <$float_type>::zero();
                let one = <$float_type>::one();
                Self::from_2d_array([
                    [right.x(), right.y(), right.z(), -right.dot(&eye)],
                    [up.x(), up.y(), up.z(), -up.dot(&eye)],
                    [-forward.x(), -forward.y(), -forward.z(), forward.dot(&eye)],
                    [zero, zero, zero, one],
                ])
            }

            fn look_at_lh(
                eye: Vector3<$float_type>,
                target: Vector3<$float_type>,
                up: Vector3<$float_type>,
            ) -> Self {
                let forward = (target - eye).normalize();
                let right = up.cross(&forward).normalize();
                let up = forward.cross(&right);

                let zero = <$float_type>::zero();
                let one = <$float_type>::one();
                Self::from_2d_array([
                    [right.x(), right.y(), right.z(), -right.dot(&eye)],
                    [up.x(), up.y(), up.z(), -up.dot(&eye)],
                    [forward.x(), forward.y(), forward.z(), -forward.dot(&eye)],
                    [zero, zero, zero, one],
                ])
            }

            // --- Convention-independent ---

            fn translation(translation: Vector3<$float_type>) -> Self {
                Self::from_2d_array([
                    [
                        <$float_type>::one(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        translation.x(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::one(),
                        <$float_type>::zero(),
                        translation.y(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                        translation.z(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn uniform_scale(scale: $float_type) -> Self {
                Self::from_2d_array([
                    [
                        scale,
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        scale,
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        scale,
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn scale(scale: Vector3<$float_type>) -> Self {
                Self::from_2d_array([
                    [
                        scale.x(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        scale.y(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        scale.z(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn rotation_x(angle_radians: $float_type) -> Self {
                let cos_a = angle_radians.cos();
                let sin_a = angle_radians.sin();
                Self::from_2d_array([
                    [
                        <$float_type>::one(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [<$float_type>::zero(), cos_a, -sin_a, <$float_type>::zero()],
                    [<$float_type>::zero(), sin_a, cos_a, <$float_type>::zero()],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn rotation_y(angle_radians: $float_type) -> Self {
                let cos_a = angle_radians.cos();
                let sin_a = angle_radians.sin();
                Self::from_2d_array([
                    [cos_a, <$float_type>::zero(), sin_a, <$float_type>::zero()],
                    [
                        <$float_type>::zero(),
                        <$float_type>::one(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                    ],
                    [-sin_a, <$float_type>::zero(), cos_a, <$float_type>::zero()],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn rotation_z(angle_radians: $float_type) -> Self {
                let cos_a = angle_radians.cos();
                let sin_a = angle_radians.sin();
                Self::from_2d_array([
                    [cos_a, -sin_a, <$float_type>::zero(), <$float_type>::zero()],
                    [sin_a, cos_a, <$float_type>::zero(), <$float_type>::zero()],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn rotation_axis_angle(axis: Vector3<$float_type>, angle_radians: $float_type) -> Self {
                let axis = axis.normalize();
                let cos_a = angle_radians.cos();
                let sin_a = angle_radians.sin();
                let one_minus_cos = <$float_type>::one() - cos_a;
                let (x, y, z) = (axis.x(), axis.y(), axis.z());
                Self::from_2d_array([
                    [
                        cos_a + x * x * one_minus_cos,
                        x * y * one_minus_cos - z * sin_a,
                        x * z * one_minus_cos + y * sin_a,
                        <$float_type>::zero(),
                    ],
                    [
                        y * x * one_minus_cos + z * sin_a,
                        cos_a + y * y * one_minus_cos,
                        y * z * one_minus_cos - x * sin_a,
                        <$float_type>::zero(),
                    ],
                    [
                        z * x * one_minus_cos - y * sin_a,
                        z * y * one_minus_cos + x * sin_a,
                        cos_a + z * z * one_minus_cos,
                        <$float_type>::zero(),
                    ],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }
        }
    };
}

macro_rules! impl_transform_3x3 {
    ($matrix_type:ident, $float_type:ty) => {
        impl Transform3x3<$float_type> for $matrix_type<$float_type> {
            fn translation_2d(translation: Vector2<$float_type>) -> Self {
                Self::from_2d_array([
                    [<$float_type>::one(), <$float_type>::zero(), translation.x()],
                    [<$float_type>::zero(), <$float_type>::one(), translation.y()],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn rotation_2d(angle_radians: $float_type) -> Self {
                let cos_a = angle_radians.cos();
                let sin_a = angle_radians.sin();
                Self::from_2d_array([
                    [cos_a, -sin_a, <$float_type>::zero()],
                    [sin_a, cos_a, <$float_type>::zero()],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn scale_2d(scale: Vector2<$float_type>) -> Self {
                Self::from_2d_array([
                    [scale.x(), <$float_type>::zero(), <$float_type>::zero()],
                    [<$float_type>::zero(), scale.y(), <$float_type>::zero()],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }

            fn uniform_scale_2d(scale: $float_type) -> Self {
                Self::from_2d_array([
                    [scale, <$float_type>::zero(), <$float_type>::zero()],
                    [<$float_type>::zero(), scale, <$float_type>::zero()],
                    [
                        <$float_type>::zero(),
                        <$float_type>::zero(),
                        <$float_type>::one(),
                    ],
                ])
            }
        }
    };
}

macro_rules! impl_transform_2x2 {
    ($matrix_type:ident, $float_type:ty) => {
        impl Transform2x2<$float_type> for $matrix_type<$float_type> {
            fn rotation_2d(angle_radians: $float_type) -> Self {
                let cos_a = angle_radians.cos();
                let sin_a = angle_radians.sin();
                Self::from_2d_array([[cos_a, -sin_a], [sin_a, cos_a]])
            }
        }
    };
}

impl_transform_4x4!(Matrix4x4, f32);
impl_transform_4x4!(Matrix4x4, f64);

impl_transform_3x3!(Matrix3x3, f32);
impl_transform_3x3!(Matrix3x3, f64);

impl_transform_2x2!(Matrix2x2, f32);
impl_transform_2x2!(Matrix2x2, f64);

#[cfg(test)]
mod tests {
    use crate::matrix::transform_traits::{Transform2x2, Transform3x3, Transform4x4};
    use crate::matrix::{Matrix2x2f32, Matrix3x3f32, Matrix4x4f32};
    use crate::vector::{Vector2f32, Vector3f32, Vector4f32};
    use std::f32::consts::PI;

    const EPSILON: f32 = 1e-5;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    fn vec4_approx_eq(a: &Vector4f32, b: &Vector4f32) -> bool {
        approx_eq(a[0], b[0])
            && approx_eq(a[1], b[1])
            && approx_eq(a[2], b[2])
            && approx_eq(a[3], b[3])
    }

    // --- Rotation / scale / translation (convention-independent) ---

    #[test]
    fn test_rotation_x() {
        let mat = Matrix4x4f32::rotation_x(PI / 2.0);
        let result = mat * Vector4f32::new(0.0, 1.0, 0.0, 1.0);
        assert!(vec4_approx_eq(
            &result,
            &Vector4f32::new(0.0, 0.0, 1.0, 1.0)
        ));
    }

    #[test]
    fn test_rotation_y() {
        let mat = Matrix4x4f32::rotation_y(PI / 2.0);
        let result = mat * Vector4f32::new(1.0, 0.0, 0.0, 1.0);
        assert!(vec4_approx_eq(
            &result,
            &Vector4f32::new(0.0, 0.0, -1.0, 1.0)
        ));
    }

    #[test]
    fn test_rotation_z() {
        let mat = Matrix4x4f32::rotation_z(PI / 2.0);
        let result = mat * Vector4f32::new(1.0, 0.0, 0.0, 1.0);
        assert!(vec4_approx_eq(
            &result,
            &Vector4f32::new(0.0, 1.0, 0.0, 1.0)
        ));
    }

    #[test]
    fn test_rotation_axis_angle_matches_rotation_z() {
        let mat_aa = Matrix4x4f32::rotation_axis_angle(Vector3f32::new(0.0, 0.0, 1.0), PI / 2.0);
        let mat_z = Matrix4x4f32::rotation_z(PI / 2.0);
        for (a, b) in mat_aa.as_slice().iter().zip(mat_z.as_slice()) {
            assert!(approx_eq(*a, *b));
        }
    }

    #[test]
    fn test_rotation_axis_angle_x_axis() {
        let mat_aa = Matrix4x4f32::rotation_axis_angle(Vector3f32::new(1.0, 0.0, 0.0), PI / 2.0);
        let mat_x = Matrix4x4f32::rotation_x(PI / 2.0);
        for (a, b) in mat_aa.as_slice().iter().zip(mat_x.as_slice()) {
            assert!(approx_eq(*a, *b));
        }
    }

    #[test]
    fn test_translation() {
        let mat = Matrix4x4f32::translation(Vector3f32::new(1.0, 2.0, 3.0));
        let result = mat * Vector4f32::new(0.0, 0.0, 0.0, 1.0);
        assert!(vec4_approx_eq(
            &result,
            &Vector4f32::new(1.0, 2.0, 3.0, 1.0)
        ));
    }

    #[test]
    fn test_scale() {
        let mat = Matrix4x4f32::scale(Vector3f32::new(2.0, 3.0, 4.0));
        let result = mat * Vector4f32::new(1.0, 1.0, 1.0, 1.0);
        assert!(vec4_approx_eq(
            &result,
            &Vector4f32::new(2.0, 3.0, 4.0, 1.0)
        ));
    }

    #[test]
    fn test_uniform_scale() {
        let mat = Matrix4x4f32::uniform_scale(5.0);
        let result = mat * Vector4f32::new(1.0, 1.0, 1.0, 1.0);
        assert!(vec4_approx_eq(
            &result,
            &Vector4f32::new(5.0, 5.0, 5.0, 1.0)
        ));
    }

    // --- Look-at ---

    #[test]
    fn test_look_at_rh_origin_to_view() {
        // Camera at (0,0,5) RH. World origin should land at (0,0,-5) in view space.
        let view = Matrix4x4f32::look_at_rh(
            Vector3f32::new(0.0, 0.0, 5.0),
            Vector3f32::new(0.0, 0.0, 0.0),
            Vector3f32::new(0.0, 1.0, 0.0),
        );
        let result = view * Vector4f32::new(0.0, 0.0, 0.0, 1.0);
        assert!(vec4_approx_eq(
            &result,
            &Vector4f32::new(0.0, 0.0, -5.0, 1.0)
        ));
    }

    #[test]
    fn test_look_at_rh_eye_maps_to_origin() {
        let eye = Vector3f32::new(1.0, 2.0, 3.0);
        let view = Matrix4x4f32::look_at_rh(
            eye,
            Vector3f32::new(0.0, 0.0, 0.0),
            Vector3f32::new(0.0, 1.0, 0.0),
        );
        let result = view * Vector4f32::new(eye[0], eye[1], eye[2], 1.0);
        assert!(approx_eq(result[0], 0.0));
        assert!(approx_eq(result[1], 0.0));
        assert!(approx_eq(result[2], 0.0));
        assert!(approx_eq(result[3], 1.0));
    }

    #[test]
    fn test_look_at_lh_origin_to_view() {
        // Camera at (0,0,-5) LH looking toward +Z. World origin is 5 units forward = +5 view-Z.
        let view = Matrix4x4f32::look_at_lh(
            Vector3f32::new(0.0, 0.0, -5.0),
            Vector3f32::new(0.0, 0.0, 0.0),
            Vector3f32::new(0.0, 1.0, 0.0),
        );
        let result = view * Vector4f32::new(0.0, 0.0, 0.0, 1.0);
        assert!(vec4_approx_eq(
            &result,
            &Vector4f32::new(0.0, 0.0, 5.0, 1.0)
        ));
    }

    #[test]
    fn test_look_at_lh_eye_maps_to_origin() {
        let eye = Vector3f32::new(1.0, 2.0, 3.0);
        let view = Matrix4x4f32::look_at_lh(
            eye,
            Vector3f32::new(4.0, 5.0, 6.0),
            Vector3f32::new(0.0, 1.0, 0.0),
        );
        let result = view * Vector4f32::new(eye[0], eye[1], eye[2], 1.0);
        assert!(approx_eq(result[0], 0.0));
        assert!(approx_eq(result[1], 0.0));
        assert!(approx_eq(result[2], 0.0));
        assert!(approx_eq(result[3], 1.0));
    }

    // --- Perspective depth mapping ---
    // All tests: feed clip coords through perspective divide to get NDC z.

    #[test]
    fn test_perspective_rh_zo() {
        // RH + [0,1]: near → 0, far → 1  (Vulkan / Metal)
        let (near, far) = (0.1_f32, 100.0_f32);
        let mat = Matrix4x4f32::perspective_rh_zo(PI / 2.0, 1.0, near, far);
        let near_pt = mat * Vector4f32::new(0.0, 0.0, -near, 1.0);
        let far_pt = mat * Vector4f32::new(0.0, 0.0, -far, 1.0);
        assert!(approx_eq(near_pt[2] / near_pt[3], 0.0));
        assert!(approx_eq(far_pt[2] / far_pt[3], 1.0));
        assert!(near_pt[3] > 0.0);
    }

    #[test]
    fn test_perspective_rh_no() {
        // RH + [-1,1]: near → -1, far → 1  (OpenGL)
        let (near, far) = (0.1_f32, 100.0_f32);
        let mat = Matrix4x4f32::perspective_rh_no(PI / 2.0, 1.0, near, far);
        let near_pt = mat * Vector4f32::new(0.0, 0.0, -near, 1.0);
        let far_pt = mat * Vector4f32::new(0.0, 0.0, -far, 1.0);
        assert!(approx_eq(near_pt[2] / near_pt[3], -1.0));
        assert!(approx_eq(far_pt[2] / far_pt[3], 1.0));
        assert!(near_pt[3] > 0.0);
    }

    #[test]
    fn test_perspective_lh_zo() {
        // LH + [0,1]: near → 0, far → 1  (D3D)
        let (near, far) = (0.1_f32, 100.0_f32);
        let mat = Matrix4x4f32::perspective_lh_zo(PI / 2.0, 1.0, near, far);
        // LH: view z is positive forward
        let near_pt = mat * Vector4f32::new(0.0, 0.0, near, 1.0);
        let far_pt = mat * Vector4f32::new(0.0, 0.0, far, 1.0);
        assert!(approx_eq(near_pt[2] / near_pt[3], 0.0));
        assert!(approx_eq(far_pt[2] / far_pt[3], 1.0));
        assert!(near_pt[3] > 0.0);
    }

    #[test]
    fn test_perspective_lh_no() {
        // LH + [-1,1]: near → -1, far → 1
        let (near, far) = (0.1_f32, 100.0_f32);
        let mat = Matrix4x4f32::perspective_lh_no(PI / 2.0, 1.0, near, far);
        let near_pt = mat * Vector4f32::new(0.0, 0.0, near, 1.0);
        let far_pt = mat * Vector4f32::new(0.0, 0.0, far, 1.0);
        assert!(approx_eq(near_pt[2] / near_pt[3], -1.0));
        assert!(approx_eq(far_pt[2] / far_pt[3], 1.0));
        assert!(near_pt[3] > 0.0);
    }

    // --- Orthographic depth mapping ---

    #[test]
    fn test_ortho_rh_no() {
        // RH + [-1,1]: corners map to NDC ±1  (OpenGL)
        let (left, right) = (-2.0_f32, 2.0_f32);
        let (bottom, top) = (-2.0_f32, 2.0_f32);
        let (near, far) = (1.0_f32, 5.0_f32);
        let mat = Matrix4x4f32::ortho_rh_no(left, right, bottom, top, near, far);
        let v = mat * Vector4f32::new(right, top, -near, 1.0);
        assert!(approx_eq(v[0], 1.0));
        assert!(approx_eq(v[1], 1.0));
        assert!(approx_eq(v[2], -1.0));
        let v2 = mat * Vector4f32::new(left, bottom, -far, 1.0);
        assert!(approx_eq(v2[0], -1.0));
        assert!(approx_eq(v2[1], -1.0));
        assert!(approx_eq(v2[2], 1.0));
    }

    #[test]
    fn test_ortho_rh_zo() {
        // RH + [0,1]: near → 0, far → 1  (Vulkan / Metal)
        let (left, right) = (-2.0_f32, 2.0_f32);
        let (bottom, top) = (-2.0_f32, 2.0_f32);
        let (near, far) = (1.0_f32, 5.0_f32);
        let mat = Matrix4x4f32::ortho_rh_zo(left, right, bottom, top, near, far);
        let v_near = mat * Vector4f32::new(0.0, 0.0, -near, 1.0);
        let v_far = mat * Vector4f32::new(0.0, 0.0, -far, 1.0);
        assert!(approx_eq(v_near[2], 0.0));
        assert!(approx_eq(v_far[2], 1.0));
        assert!(approx_eq(v_near[3], 1.0));
    }

    #[test]
    fn test_ortho_lh_zo() {
        // LH + [0,1]: near → 0, far → 1  (D3D)
        let (left, right) = (-2.0_f32, 2.0_f32);
        let (bottom, top) = (-2.0_f32, 2.0_f32);
        let (near, far) = (1.0_f32, 5.0_f32);
        let mat = Matrix4x4f32::ortho_lh_zo(left, right, bottom, top, near, far);
        let v_near = mat * Vector4f32::new(0.0, 0.0, near, 1.0);
        let v_far = mat * Vector4f32::new(0.0, 0.0, far, 1.0);
        assert!(approx_eq(v_near[2], 0.0));
        assert!(approx_eq(v_far[2], 1.0));
        assert!(approx_eq(v_near[3], 1.0));
    }

    #[test]
    fn test_ortho_lh_no() {
        // LH + [-1,1]: near → -1, far → 1
        let (left, right) = (-2.0_f32, 2.0_f32);
        let (bottom, top) = (-2.0_f32, 2.0_f32);
        let (near, far) = (1.0_f32, 5.0_f32);
        let mat = Matrix4x4f32::ortho_lh_no(left, right, bottom, top, near, far);
        let v_near = mat * Vector4f32::new(0.0, 0.0, near, 1.0);
        let v_far = mat * Vector4f32::new(0.0, 0.0, far, 1.0);
        assert!(approx_eq(v_near[2], -1.0));
        assert!(approx_eq(v_far[2], 1.0));
    }

    // --- 2D transforms ---

    #[test]
    fn test_rotation_2d_matrix2x2() {
        let mat = Matrix2x2f32::rotation_2d(PI / 2.0);
        let result = mat * Vector2f32::new(1.0, 0.0);
        assert!(approx_eq(result[0], 0.0));
        assert!(approx_eq(result[1], 1.0));
    }

    #[test]
    fn test_translation_2d_matrix3x3() {
        let mat = Matrix3x3f32::translation_2d(Vector2f32::new(3.0, 4.0));
        let result = mat * Vector3f32::new(0.0, 0.0, 1.0);
        assert!(approx_eq(result[0], 3.0));
        assert!(approx_eq(result[1], 4.0));
        assert!(approx_eq(result[2], 1.0));
    }

    #[test]
    fn test_scale_2d() {
        let mat = Matrix3x3f32::scale_2d(Vector2f32::new(2.0, 3.0));
        let result = mat * Vector3f32::new(1.0, 1.0, 1.0);
        assert!(approx_eq(result[0], 2.0));
        assert!(approx_eq(result[1], 3.0));
        assert!(approx_eq(result[2], 1.0));
    }

    #[test]
    fn test_uniform_scale_2d() {
        let mat = Matrix3x3f32::uniform_scale_2d(4.0);
        let result = mat * Vector3f32::new(1.0, 1.0, 1.0);
        assert!(approx_eq(result[0], 4.0));
        assert!(approx_eq(result[1], 4.0));
        assert!(approx_eq(result[2], 1.0));
    }

    #[test]
    fn test_rotation_2d_matrix3x3() {
        let mat = Matrix3x3f32::rotation_2d(PI / 2.0);
        let result = mat * Vector3f32::new(1.0, 0.0, 1.0);
        assert!(approx_eq(result[0], 0.0));
        assert!(approx_eq(result[1], 1.0));
    }
}

mod matrix;
mod quaternion;
mod vector;

pub use crate::quaternion::{Quaternion, Quaternionf32, Quaternionf64};

pub use crate::matrix::{
    Matrix2x2f32, Matrix2x2f64, Matrix2x2i32, Matrix2x2i64, Matrix3x3f32, Matrix3x3f64,
    Matrix3x3i32, Matrix3x3i64, Matrix4x4f32, Matrix4x4f64, Matrix4x4i32, Matrix4x4i64,
};
pub use crate::vector::{
    Vector, Vector2, Vector2f32, Vector2f64, Vector2i32, Vector2i64, Vector3, Vector3f32,
    Vector3f64, Vector3i32, Vector3i64, Vector4, Vector4f32, Vector4f64, Vector4i32, Vector4i64,
};

// Export graphics traits for users who want to use them directly
pub use crate::matrix::transform_traits::{Transform2x2, Transform3x3, Transform4x4};
pub use crate::vector::vector_ops::{Vector2Ops, Vector3Ops, Vector4Ops};

#[cfg(test)]
mod tests {
    use crate::{Matrix2x2f32, Matrix2x2i32, Matrix4x4f32, Matrix4x4i32};
    use crate::{Vector2i32, Vector3f32, Vector3i32, Vector4f32, Vector4i32};
    // --- Vector Tests ---
    #[test]
    fn test_vector_new_i32() {
        let v = Vector3i32::from_slice(&[1, 2, 3]);
        assert_eq!(v.as_slice(), [1, 2, 3]);
        assert_eq!(v.dimension(), 3);

        let v2 = Vector2i32::new(10, 20);
        assert_eq!(v2.as_slice(), [10, 20]);
        assert_eq!(v2.dimension(), 2);

        let v3 = Vector3f32::new(1.1, 2.2, 3.3);
        assert_eq!(v3.as_slice(), [1.1, 2.2, 3.3]);
        assert_eq!(v3.dimension(), 3);
    }

    #[test]
    fn test_vector_from_slice() {
        let v_i32 = Vector3i32::from_slice(&[1, 2, 3]);
        assert_eq!(v_i32.as_slice(), [1, 2, 3]);

        let v_f32 = Vector4f32::from_slice(&[1.0, -2.5, 0.0, 100.0]);
        assert_eq!(v_f32.as_slice(), [1.0, -2.5, 0.0, 100.0]);
    }

    #[test]
    #[should_panic(expected = "Incorrect number of elements")]
    fn test_vector_from_slice_panic() {
        let _v = Vector3i32::from_slice(&[1, 2]); // Should panic
    }

    #[test]
    fn test_vector_get_set_index() {
        let mut v = Vector4i32::from_slice(&[1, 2, 3, 4]);
        assert_eq!(v.get(2), Some(3));
        assert_eq!(v[2], 3); // Test Index trait
        assert_eq!(v.get(5), None);

        v.set(1, 10);
        assert_eq!(v.as_slice(), [1, 10, 3, 4]);
        v[0] = 100; // Test IndexMut trait
        assert_eq!(v.as_slice(), [100, 10, 3, 4]);
    }

    #[test]
    fn test_vector_get_panic() {
        let v = Vector3i32::from_slice(&[1, 2, 3]);
        let _val = v.get(5);
        assert_eq!(_val, None)
    }

    #[test]
    #[should_panic(expected = "Vector index 5 out of bounds")]
    fn test_vector_set_panic() {
        let mut v = Vector3i32::from_slice(&[1, 2, 3]);
        v.set(5, 10); // Should panic
    }

    #[test]
    fn test_vector_add() {
        let v1_i = Vector2i32::new(1, 2);
        let v2_i = Vector2i32::new(3, 4);
        let v3_i = v1_i + v2_i;
        assert_eq!(v3_i.as_slice(), [4, 6]);

        let v1_f = Vector3f32::from_slice(&[1.0, -2.0, 3.5]);
        let v2_f = Vector3f32::from_slice(&[0.5, 1.0, -1.5]);
        let v3_f = v1_f + v2_f;
        assert_eq!(v3_f.as_slice(), [1.5, -1.0, 2.0]);
    }

    #[test]
    fn test_vector_sub() {
        let v1_i = Vector2i32::new(5, 8);
        let v2_i = Vector2i32::new(1, 3);
        let v3_i = v1_i - v2_i;
        assert_eq!(v3_i.as_slice(), [4, 5]);

        let v1_f = Vector3f32::from_slice(&[1.0, -2.0, 3.5]);
        let v2_f = Vector3f32::from_slice(&[0.5, 1.0, -1.5]);
        let v3_f = v1_f - v2_f;
        assert_eq!(v3_f.as_slice(), [0.5, -3.0, 5.0]);
    }

    #[test]
    fn test_vector_mul_scalar() {
        let v_i = Vector2i32::new(3, 4);
        let result_i = v_i * 2;
        assert_eq!(result_i.as_slice(), [6, 8]);

        let v_f = Vector3f32::from_slice(&[1.0, -2.0, 3.0]);
        let result_f = v_f * 0.5;
        assert_eq!(result_f.as_slice(), [0.5, -1.0, 1.5]);
    }

    #[test]
    fn test_vector_neg() {
        let v_i = Vector2i32::new(3, -4);
        let result_i = -v_i;
        assert_eq!(result_i.as_slice(), [-3, 4]);

        let v_f = Vector3f32::from_slice(&[1.0, -2.0, 0.0]);
        let result_f = -v_f;
        assert_eq!(result_f.as_slice(), [-1.0, 2.0, -0.0]);
    }

    #[test]
    fn test_vector_add_assign() {
        let mut v1 = Vector2i32::new(1, 2);
        let v2 = Vector2i32::new(3, 4);
        v1 += &v2; // Use with reference due to impl AddAssign<&Self>
        assert_eq!(v1.as_slice(), [4, 6]);
    }

    #[test]
    fn test_vector_sub_assign() {
        let mut v1 = Vector2i32::new(5, 5);
        let v2 = Vector2i32::new(1, 2);
        v1 -= &v2; // Use with reference due to impl SubAssign<&Self>
        assert_eq!(v1.as_slice(), [4, 3]);
    }

    #[test]
    fn test_vector2i_rotate_90() {
        let pivot = Vector2i32::new(10, 10);

        let p1 = Vector2i32::new(11, 10); // 1 unit right of pivot
        let rotated_cw = p1.rotate_90_cw(pivot); // Should be 1 unit down
        assert_eq!(rotated_cw, Vector2i32::new(10, 9)); // Assuming Y down

        let rotated_ccw = p1.rotate_90_ccw(pivot); // Should be 1 unit up
        assert_eq!(rotated_ccw, Vector2i32::new(10, 11)); // Assuming Y down

        let p2 = Vector2i32::new(10, 9); // 1 unit down of pivot
        let rotated_cw2 = p2.rotate_90_cw(pivot); // Should be 1 unit left
        assert_eq!(rotated_cw2, Vector2i32::new(9, 10));

        let rotated_ccw2 = p2.rotate_90_ccw(pivot); // Should be 1 unit right
        assert_eq!(rotated_ccw2, Vector2i32::new(11, 10));

        // Test rotation around origin (simple case)
        let origin = Vector2i32::new(0, 0);
        let p_origin = Vector2i32::new(1, 0);
        assert_eq!(p_origin.rotate_90_cw(origin), Vector2i32::new(0, -1)); // Y down (1,0) -> (0,1)
        assert_eq!(
            Vector2i32::new(0, 1).rotate_90_cw(origin),
            Vector2i32::new(1, 0)
        );
        assert_eq!(
            Vector2i32::new(-1, 0).rotate_90_cw(origin),
            Vector2i32::new(0, 1)
        );
        assert_eq!(
            Vector2i32::new(0, -1).rotate_90_cw(origin),
            Vector2i32::new(-1, 0)
        );
        assert_eq!(p_origin.rotate_90_ccw(origin), Vector2i32::new(0, 1)); // Y down (1,0) -> (0,-1)
    }

    // --- Matrix Tests ---
    // Test names use specific struct names (Matrix2x2, Matrix4x4 etc.)
    #[test]
    fn test_matrix4x4_identity() {
        let identity_mat = Matrix4x4f32::identity();
        assert_eq!(
            identity_mat.as_slice(),
            [
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0
            ]
        );
    }

    #[test]
    fn test_matrix2x2_identity() {
        let identity_mat = Matrix2x2i32::identity();
        assert_eq!(identity_mat.as_slice(), [1, 0, 0, 1]);
    }

    #[test]
    fn test_matrix4x4_from_slice() {
        let mat = Matrix4x4f32::from_slice(&[
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        ]);
        assert_eq!(
            mat.as_slice(),
            [
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0,
                16.0
            ]
        );
    }

    #[test]
    fn test_matrix2x2_from_2d_array() {
        let mat = Matrix2x2i32::from_2d_array([[1, 2], [3, 4]]);
        assert_eq!(mat.as_slice(), [1, 2, 3, 4]);
    }

    #[test]
    fn test_matrix4x4_get_set_index() {
        let mut mat = Matrix4x4f32::identity();
        assert_eq!(mat.get(0, 0), Some(1.0));
        assert_eq!(mat[1][1], 1.0); // Test Index trait
        assert_eq!(mat.get(4, 0), None); // Test out of bounds get

        mat.set(0, 0, 10.0);
        assert_eq!(mat[0][0], 10.0);
        mat[1][1] = 20.0; // Test IndexMut trait
        assert_eq!(mat[1][1], 20.0);
    }

    #[test]
    fn test_matrix4x4_add() {
        let mat1 = Matrix4x4i32::from_2d_array([
            [1, 2, 3, 4],
            [5, 6, 7, 8],
            [9, 10, 11, 12],
            [13, 14, 15, 16],
        ]);
        let mat2 =
            Matrix4x4i32::from_2d_array([[1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1]]);
        let result = mat1 + mat2; // Uses Add trait
        assert_eq!(
            result.as_slice(),
            [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]
        );
    }

    #[test]
    fn test_matrix2x2_add() {
        let mat1 = Matrix2x2f32::from_2d_array([[1.0, 2.0], [3.0, 4.0]]);
        let mat2 = Matrix2x2f32::from_2d_array([[5.0, 6.0], [7.0, 8.0]]);
        let result = mat1 + mat2; // Uses Add trait
        assert_eq!(result.as_slice(), [6.0, 8.0, 10.0, 12.0]);
    }

    #[test]
    fn test_matrix4x4_mul_scalar() {
        let mat = Matrix4x4f32::from_2d_array([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let result = mat * 2.0; // Uses Mul<T> trait
        assert_eq!(
            result.as_slice(),
            [
                2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0,
                30.0, 32.0
            ]
        );
    }

    #[test]
    fn test_matrix2x2_mul_scalar() {
        let mat = Matrix2x2i32::from_2d_array([[1, 2], [3, 4]]);
        let result = mat * 10; // Uses Mul<T> trait
        assert_eq!(result.as_slice(), [10, 20, 30, 40]);
    }

    #[test]
    fn test_matrix4x4_neg() {
        let mat = Matrix4x4i32::from_2d_array([
            [1, -2, 0, 5],
            [-3, 4, -1, 0],
            [0, 0, 0, 0],
            [10, -10, 10, -10],
        ]);
        let result = -mat; // Uses Neg trait
        assert_eq!(
            result.as_slice(),
            [-1, 2, 0, -5, 3, -4, 1, 0, 0, 0, 0, 0, -10, 10, -10, 10]
        );
    }

    #[test]
    fn test_matrix4x4_mul_vector4() {
        let mat_identity = Matrix4x4f32::identity();
        let vec = Vector4f32::new(5.0, 10.0, 2.0, 1.0);
        let result_identity = mat_identity * vec; // Uses Mul<Vector> trait
        assert_eq!(result_identity.as_slice(), [5.0, 10.0, 2.0, 1.0]);

        let rotation_mat = Matrix4x4f32::from_2d_array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0], // Rotate Y 90 deg CW around X axis (common in graphics)
        ]);
        let vec2 = Vector4f32::new(0.0, 1.0, 0.0, 1.0); // Point (0,1,0)
        let rotated_vec2 = rotation_mat * vec2; // Uses Mul<Vector> trait
        assert_eq!(rotated_vec2.as_slice(), [0.0, 0.0, 1.0, 1.0]); // Should be (0, 0, 1) in 3D coords
    }

    #[test]
    fn test_matrix2x2_mul_vector2() {
        let mat = Matrix2x2i32::from_2d_array([[0, -1], [1, 0]]); // 90 deg CW rotation (2D)
        let vec = Vector2i32::new(10, 0); // Point (10, 0)
        let rotated_vec = mat * vec; // Uses Mul<Vector> trait
        assert_eq!(rotated_vec.as_slice(), [0, 10]); // Should be (0, 10)
    }

    #[test]
    fn test_matrix4x4_mul_matrix4x4() {
        let mat_a = Matrix4x4f32::from_2d_array([
            // Translation (1, 2, 0) in 3D homogeneous
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let mat_b = Matrix4x4f32::from_2d_array([
            // Rotation 90 deg CW in 2D homogeneous (around Z)
            [0.0, -1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let result = mat_a * mat_b; // Apply rotation THEN translation (Uses Mul<Matrix> trait)
        // Expected: [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 0], [0, 0, 0, 1]]
        assert_eq!(
            result.as_slice(),
            [
                0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0
            ]
        );
    }

    #[test]
    fn test_matrix4x4_sub() {
        let mat1 =
            Matrix4x4i32::from_2d_array([[5, 5, 5, 5], [5, 5, 5, 5], [5, 5, 5, 5], [5, 5, 5, 5]]);
        let mat2 =
            Matrix4x4i32::from_2d_array([[1, 2, 3, 4], [1, 2, 3, 4], [1, 2, 3, 4], [1, 2, 3, 4]]);
        let result = mat1 - mat2;
        assert_eq!(
            result.as_slice(),
            [4, 3, 2, 1, 4, 3, 2, 1, 4, 3, 2, 1, 4, 3, 2, 1]
        );
    }

    #[test]
    fn test_matrix_transpose() {
        let mat = Matrix4x4i32::from_2d_array([
            [1, 2, 3, 4],
            [5, 6, 7, 8],
            [9, 10, 11, 12],
            [13, 14, 15, 16],
        ]);
        let t = mat.transpose();
        assert_eq!(
            t.as_slice(),
            [1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15, 4, 8, 12, 16]
        );
        // Double-transpose is identity
        assert_eq!(mat.transpose().transpose().as_slice(), mat.as_slice());
    }

    #[test]
    fn test_matrix2x2_sub() {
        let mat1 = Matrix2x2f32::from_2d_array([[5.0, 4.0], [3.0, 2.0]]);
        let mat2 = Matrix2x2f32::from_2d_array([[1.0, 1.0], [1.0, 1.0]]);
        let result = mat1 - mat2;
        assert_eq!(result.as_slice(), [4.0, 3.0, 2.0, 1.0]);
    }

    #[test]
    fn test_matrix2x2_mul_matrix2x2() {
        let mat1 = Matrix2x2i32::from_2d_array([[1, 2], [3, 4]]);
        let mat2 = Matrix2x2i32::from_2d_array([[5, 6], [7, 8]]);
        let result = mat1 * mat2; // Uses Mul<Matrix> trait
        // Expected: [[1*5+2*7, 1*6+2*8], [3*5+4*7, 3*6+4*8]] = [[5+14, 6+16], [15+28, 18+32]] = [[19, 22], [43, 50]]
        assert_eq!(result.as_slice(), [19, 22, 43, 50]);
    }
}

use std::ops::{Add, Mul, Sub};

use num_traits::{Float, Zero};

use crate::matrix::{Matrix2x2, Matrix3x3, Matrix4x4};

macro_rules! impl_matrix_trace {
    ($name:ident, $dim:expr) => {
        impl<T: Add<Output = T> + Zero + Copy> $name<T> {
            /// Sum of the diagonal elements.
            pub fn trace(&self) -> T {
                let mut sum = T::zero();
                for i in 0..$dim {
                    sum = sum + self.data[i * $dim + i];
                }
                sum
            }
        }
    };
}

impl_matrix_trace!(Matrix2x2, 2);
impl_matrix_trace!(Matrix3x3, 3);
impl_matrix_trace!(Matrix4x4, 4);

fn det3x3<T: Mul<Output = T> + Sub<Output = T> + Add<Output = T> + Copy>(m: [T; 9]) -> T {
    m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6])
}

impl<T: Mul<Output = T> + Sub<Output = T> + Copy> Matrix2x2<T> {
    /// Determinant. Works for both integer and float element types.
    pub fn determinant(&self) -> T {
        self.data[0] * self.data[3] - self.data[1] * self.data[2]
    }
}

impl<T: Mul<Output = T> + Sub<Output = T> + Add<Output = T> + Copy> Matrix3x3<T> {
    /// Determinant via cofactor expansion along the first row. Works for both
    /// integer and float element types.
    pub fn determinant(&self) -> T {
        det3x3(self.data)
    }
}

impl<T: Mul<Output = T> + Sub<Output = T> + Add<Output = T> + Copy> Matrix4x4<T> {
    /// Determinant via cofactor expansion along the first row, using 3x3 minors.
    /// Works for both integer and float element types.
    pub fn determinant(&self) -> T {
        let d = self.data;
        let minor = |skip_col: usize| -> [T; 9] {
            let mut m = [d[0]; 9];
            let mut idx = 0;
            for row in 1..4 {
                for col in 0..4 {
                    if col == skip_col {
                        continue;
                    }
                    m[idx] = d[row * 4 + col];
                    idx += 1;
                }
            }
            m
        };
        d[0] * det3x3(minor(0)) - d[1] * det3x3(minor(1)) + d[2] * det3x3(minor(2))
            - d[3] * det3x3(minor(3))
    }
}

macro_rules! impl_matrix_inverse {
    ($name:ident, $size:expr, $dim:expr) => {
        impl<T: Float + Copy> $name<T>
        where
            [(); $size]:,
        {
            /// Inverse via Gauss-Jordan elimination with partial pivoting.
            /// Returns `None` if the matrix is singular (or near-singular).
            pub fn inverse(&self) -> Option<Self> {
                let mut a = self.data;
                let mut inv = Self::identity().data;

                for col in 0..$dim {
                    let mut pivot_row = col;
                    let mut max_val = a[col * $dim + col].abs();
                    for r in (col + 1)..$dim {
                        let val = a[r * $dim + col].abs();
                        if val > max_val {
                            max_val = val;
                            pivot_row = r;
                        }
                    }
                    if max_val <= T::epsilon() {
                        return None;
                    }

                    if pivot_row != col {
                        for k in 0..$dim {
                            a.swap(col * $dim + k, pivot_row * $dim + k);
                            inv.swap(col * $dim + k, pivot_row * $dim + k);
                        }
                    }

                    let pivot = a[col * $dim + col];
                    for k in 0..$dim {
                        a[col * $dim + k] = a[col * $dim + k] / pivot;
                        inv[col * $dim + k] = inv[col * $dim + k] / pivot;
                    }

                    for r in 0..$dim {
                        if r == col {
                            continue;
                        }
                        let factor = a[r * $dim + col];
                        if factor != T::zero() {
                            for k in 0..$dim {
                                a[r * $dim + k] = a[r * $dim + k] - factor * a[col * $dim + k];
                                inv[r * $dim + k] =
                                    inv[r * $dim + k] - factor * inv[col * $dim + k];
                            }
                        }
                    }
                }

                Some(Self { data: inv })
            }
        }
    };
}

impl_matrix_inverse!(Matrix2x2, 4, 2);
impl_matrix_inverse!(Matrix3x3, 9, 3);
impl_matrix_inverse!(Matrix4x4, 16, 4);

#[cfg(test)]
mod tests {
    use crate::matrix::transform_traits::Transform4x4;
    use crate::matrix::{Matrix2x2f32, Matrix2x2i32, Matrix3x3f32, Matrix3x3i32, Matrix4x4f32};
    use crate::vector::Vector3f32;
    use std::f32::consts::PI;

    const EPSILON: f32 = 1e-4;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    fn mat_approx_eq<const N: usize>(a: &[f32], b: &[f32]) -> bool {
        a.len() == N && b.len() == N && a.iter().zip(b.iter()).all(|(x, y)| approx_eq(*x, *y))
    }

    // --- Trace ---

    #[test]
    fn test_trace_identity() {
        assert_eq!(Matrix4x4f32::identity().trace(), 4.0);
        assert_eq!(Matrix3x3f32::identity().trace(), 3.0);
        assert_eq!(Matrix2x2f32::identity().trace(), 2.0);
    }

    #[test]
    fn test_trace_diagonal() {
        let mat = Matrix3x3i32::from_2d_array([[1, 9, 9], [9, 2, 9], [9, 9, 3]]);
        assert_eq!(mat.trace(), 6);
    }

    // --- Determinant ---

    #[test]
    fn test_determinant_2x2() {
        let mat = Matrix2x2i32::from_2d_array([[3, 8], [4, 6]]);
        assert_eq!(mat.determinant(), 3 * 6 - 8 * 4); // -14
        assert_eq!(Matrix2x2f32::identity().determinant(), 1.0);
    }

    #[test]
    fn test_determinant_3x3() {
        let mat = Matrix3x3i32::from_2d_array([[6, 1, 1], [4, -2, 5], [2, 8, 7]]);
        // Hand-computed: 6*(-2*7-5*8) - 1*(4*7-5*2) + 1*(4*8-(-2)*2) = 6*(-54) - 1*18 + 1*36 = -324-18+36 = -306
        assert_eq!(mat.determinant(), -306);
    }

    #[test]
    fn test_determinant_3x3_singular() {
        // Duplicate row -> determinant 0.
        let mat = Matrix3x3f32::from_2d_array([[1.0, 2.0, 3.0], [1.0, 2.0, 3.0], [7.0, 8.0, 9.0]]);
        assert!(approx_eq(mat.determinant(), 0.0));
    }

    #[test]
    fn test_determinant_4x4_identity_and_scale() {
        assert_eq!(Matrix4x4f32::identity().determinant(), 1.0);
        let scale = Matrix4x4f32::from_2d_array([
            [2.0, 0.0, 0.0, 0.0],
            [0.0, 3.0, 0.0, 0.0],
            [0.0, 0.0, 4.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        assert!(approx_eq(scale.determinant(), 24.0));
    }

    #[test]
    fn test_determinant_4x4_singular() {
        let mat = Matrix4x4f32::from_2d_array([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0], // rows are linearly dependent (arithmetic progression)
        ]);
        assert!(approx_eq(mat.determinant(), 0.0));
    }

    // --- Inverse ---

    #[test]
    fn test_inverse_identity() {
        let inv = Matrix4x4f32::identity().inverse().unwrap();
        assert!(mat_approx_eq::<16>(
            inv.as_slice(),
            Matrix4x4f32::identity().as_slice()
        ));
    }

    #[test]
    fn test_inverse_2x2() {
        let mat = Matrix2x2f32::from_2d_array([[4.0, 7.0], [2.0, 6.0]]);
        let inv = mat.inverse().unwrap();
        // Known inverse: 1/10 * [[6, -7], [-2, 4]]
        assert!(mat_approx_eq::<4>(inv.as_slice(), &[0.6, -0.7, -0.2, 0.4]));
    }

    #[test]
    fn test_inverse_scale() {
        let mat = Matrix4x4f32::uniform_scale(2.0);
        let inv = mat.inverse().unwrap();
        let result = mat * inv;
        assert!(mat_approx_eq::<16>(
            result.as_slice(),
            Matrix4x4f32::identity().as_slice()
        ));
    }

    #[test]
    fn test_inverse_rotation_equals_transpose() {
        // Rotation matrices are orthogonal: inverse == transpose.
        let mat = Matrix4x4f32::rotation_z(PI / 3.0);
        let inv = mat.inverse().unwrap();
        assert!(mat_approx_eq::<16>(
            inv.as_slice(),
            mat.transpose().as_slice()
        ));
    }

    #[test]
    fn test_inverse_round_trip_general_matrix() {
        let mat = Matrix4x4f32::translation(Vector3f32::new(1.0, 2.0, 3.0))
            * Matrix4x4f32::rotation_y(0.4)
            * Matrix4x4f32::scale(Vector3f32::new(2.0, 1.0, 0.5));
        let inv = mat.inverse().expect("matrix should be invertible");

        let identity = Matrix4x4f32::identity();
        assert!(mat_approx_eq::<16>(
            (mat * inv).as_slice(),
            identity.as_slice()
        ));
        assert!(mat_approx_eq::<16>(
            (inv * mat).as_slice(),
            identity.as_slice()
        ));
    }

    #[test]
    fn test_inverse_singular_returns_none() {
        let zero = Matrix3x3f32::from_2d_array([[0.0; 3]; 3]);
        assert!(zero.inverse().is_none());

        let duplicate_rows =
            Matrix3x3f32::from_2d_array([[1.0, 2.0, 3.0], [1.0, 2.0, 3.0], [7.0, 8.0, 9.0]]);
        assert!(duplicate_rows.inverse().is_none());
    }

    #[test]
    fn test_inverse_3x3() {
        let mat = Matrix3x3f32::from_2d_array([[6.0, 1.0, 1.0], [4.0, -2.0, 5.0], [2.0, 8.0, 7.0]]);
        let inv = mat.inverse().unwrap();
        let result = mat * inv;
        assert!(mat_approx_eq::<9>(
            result.as_slice(),
            Matrix3x3f32::identity().as_slice()
        ));
    }
}

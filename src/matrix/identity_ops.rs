use num_traits::{One, Zero};
use std::mem::MaybeUninit;
use std::ptr;

use crate::matrix::{Matrix2x2, Matrix3x3, Matrix4x4};

macro_rules! impl_matrix_identity {
    ($name:ident, $size:expr, $dims:expr) => {
        impl<T: Zero + One + Copy> $name<T> {
            pub fn identity() -> Self {
                let mut result_data: MaybeUninit<[T; $size]> = MaybeUninit::uninit();
                let res_ptr = result_data.as_mut_ptr() as *mut T;
                for i in 0..$dims {
                    for j in 0..$dims {
                        unsafe {
                            ptr::write(
                                res_ptr.add(i * $dims + j),
                                if i == j { T::one() } else { T::zero() },
                            )
                        };
                    }
                }

                $name {
                    data: unsafe { result_data.assume_init() },
                }
            }
        }
    };
}

impl_matrix_identity!(Matrix2x2, 4, 2);
impl_matrix_identity!(Matrix3x3, 9, 3);
impl_matrix_identity!(Matrix4x4, 16, 4);

// `identity()` above can't be a `const fn` generically over `T: Zero + One`,
// since those are regular (non-const) trait methods. `IDENTITY` associated
// consts are only provided for the concrete numeric type aliases, where
// `0`/`1` literals are const-evaluable directly. Computed via a const-eval
// while loop rather than hand-written flat arrays, for the same reason
// `inverse()` uses Gauss-Jordan instead of a hand-derived formula: less
// surface area for a transcription mistake, especially at 4x4.
macro_rules! impl_matrix_identity_const {
    ($name:ident, $t:ty, $dim:expr, $size:expr) => {
        impl $name<$t> {
            pub const IDENTITY: Self = {
                let mut data = [0 as $t; $size];
                let mut i = 0;
                while i < $dim {
                    data[i * $dim + i] = 1 as $t;
                    i += 1;
                }
                $name { data }
            };
        }
    };
}

impl_matrix_identity_const!(Matrix2x2, i32, 2, 4);
impl_matrix_identity_const!(Matrix2x2, i64, 2, 4);
impl_matrix_identity_const!(Matrix2x2, f32, 2, 4);
impl_matrix_identity_const!(Matrix2x2, f64, 2, 4);
impl_matrix_identity_const!(Matrix3x3, i32, 3, 9);
impl_matrix_identity_const!(Matrix3x3, i64, 3, 9);
impl_matrix_identity_const!(Matrix3x3, f32, 3, 9);
impl_matrix_identity_const!(Matrix3x3, f64, 3, 9);
impl_matrix_identity_const!(Matrix4x4, i32, 4, 16);
impl_matrix_identity_const!(Matrix4x4, i64, 4, 16);
impl_matrix_identity_const!(Matrix4x4, f32, 4, 16);
impl_matrix_identity_const!(Matrix4x4, f64, 4, 16);

#[cfg(test)]
mod const_tests {
    use crate::matrix::{Matrix2x2f32, Matrix2x2i32, Matrix3x3f64, Matrix4x4f32};

    #[test]
    fn test_identity_const_matches_identity_fn() {
        assert_eq!(
            Matrix4x4f32::IDENTITY.as_slice(),
            Matrix4x4f32::identity().as_slice()
        );
        assert_eq!(
            Matrix3x3f64::IDENTITY.as_slice(),
            Matrix3x3f64::identity().as_slice()
        );
        assert_eq!(
            Matrix2x2i32::IDENTITY.as_slice(),
            Matrix2x2i32::identity().as_slice()
        );
    }

    #[test]
    fn test_identity_const_values() {
        assert_eq!(Matrix2x2f32::IDENTITY.as_slice(), [1.0, 0.0, 0.0, 1.0]);
        assert_eq!(Matrix2x2i32::IDENTITY.as_slice(), [1, 0, 0, 1]);
    }
}

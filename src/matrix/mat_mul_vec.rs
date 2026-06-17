use std::{
    mem::MaybeUninit,
    ops::{Add, Mul},
    ptr,
};

use num_traits::Zero;

use crate::vector::{Vector2, Vector3, Vector4};

use super::{Matrix2x2, Matrix3x3, Matrix4x4};

macro_rules! impl_matrix_mul_vector {
    ($name:ident, $vec_type:ident, $mat_rows:expr, $mat_cols:expr, $mat_size:expr, $vec_d:expr) => {
        //assert_eq!($mat_cols, $vec_d);
        //assert_eq!($mat_rows, $vec_d);
        impl<T: Add<Output = T> + Mul<Output = T> + Copy + Zero> Mul<$vec_type<T>> for $name<T>
        where
            [(); $mat_size]:,
            [(); $mat_rows]:,
            [(); $mat_cols]:,
        {
            type Output = $vec_type<T>;
            fn mul(self, rhs: $vec_type<T>) -> Self::Output {
                let mut res_data: MaybeUninit<[T; $vec_d]> = MaybeUninit::uninit();
                let res_ptr = res_data.as_mut_ptr() as *mut T;
                for i in 0..$mat_rows {
                    let mut sum = T::zero();
                    for j in 0..$mat_cols {
                        sum = sum + self.data[i * $mat_cols + j] * rhs[j]
                    }

                    unsafe { ptr::write(res_ptr.add(i), sum) };
                }

                $vec_type::from_slice(&unsafe { res_data.assume_init() })
            }
        }
    };
}

impl_matrix_mul_vector!(Matrix2x2, Vector2, 2, 2, 4, 2);
impl_matrix_mul_vector!(Matrix3x3, Vector3, 3, 3, 9, 3);
impl_matrix_mul_vector!(Matrix4x4, Vector4, 4, 4, 16, 4);

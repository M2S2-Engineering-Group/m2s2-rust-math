use std::{
    mem::MaybeUninit,
    ops::{Add, Mul},
    ptr,
};

use num_traits::Zero;

use super::{Matrix2x2, Matrix3x3, Matrix4x4};

macro_rules! impl_matrix_mul_matrix {
    ($lhs:ident, $rhs:ident, $res:ident, $size:expr, $rows:expr, $cols:expr) => {
        impl<T: Add<Output = T> + Mul<Output = T> + Copy + Zero> Mul<$rhs<T>> for $lhs<T>
        where
            [(); $size]:,
        {
            type Output = $res<T>;
            fn mul(self, rhs: $rhs<T>) -> Self::Output {
                let mut res_data: MaybeUninit<[T; $size]> = MaybeUninit::uninit();
                let res_ptr = res_data.as_mut_ptr() as *mut T;
                for i in 0..$rows {
                    for j in 0..$cols {
                        let mut sum = T::zero();
                        for k in 0..$cols {
                            sum = sum + self.data[i * $cols + k] * rhs.data[k * $cols + j];
                        }
                        unsafe { ptr::write(res_ptr.add(i * $cols + j), sum) };
                    }
                }
                $res {
                    data: unsafe { res_data.assume_init() },
                }
            }
        }
    };
}

impl_matrix_mul_matrix!(Matrix2x2, Matrix2x2, Matrix2x2, 4, 2, 2);
impl_matrix_mul_matrix!(Matrix3x3, Matrix3x3, Matrix3x3, 9, 3, 3);
impl_matrix_mul_matrix!(Matrix4x4, Matrix4x4, Matrix4x4, 16, 4, 4);

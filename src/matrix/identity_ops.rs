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

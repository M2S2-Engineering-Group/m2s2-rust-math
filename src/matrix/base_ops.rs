use std::ops::{Add, Mul, Neg, Sub};

use crate::matrix::{Matrix2x2, Matrix3x3, Matrix4x4};

macro_rules! impl_matrix_base_ops {
    ($name:ident, $size:expr) => {
        impl<T: Add<Output = T> + Copy> Add for $name<T>
        where
            [(); $size]:,
        {
            type Output = Self;
            fn add(self, rhs: Self) -> Self {
                self.map_binary(&rhs, |a, b| a + b)
            }
        }

        impl<T: Mul<Output = T> + Copy> Mul<T> for $name<T>
        where
            [(); $size]:,
        {
            type Output = Self;
            fn mul(self, scalar: T) -> Self {
                self.map_unary(|a| a * scalar)
            }
        }

        impl<T: Sub<Output = T> + Copy> Sub for $name<T>
        where
            [(); $size]:,
        {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self {
                self.map_binary(&rhs, |a, b| a - b)
            }
        }

        impl<T: Neg<Output = T> + Copy> Neg for $name<T>
        where
            [(); $size]:,
        {
            type Output = Self;
            fn neg(self) -> Self {
                self.map_unary(|a| -a)
            }
        }
    };
}

impl_matrix_base_ops!(Matrix2x2, 4);
impl_matrix_base_ops!(Matrix3x3, 9);
impl_matrix_base_ops!(Matrix4x4, 16);

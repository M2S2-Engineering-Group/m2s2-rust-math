use std::{
    clone::Clone,
    fmt::Debug,
    marker::Copy,
    mem::MaybeUninit,
    ops::{Index, IndexMut},
    ptr, slice,
};

mod base_ops;
mod identity_ops;
mod mat_mul_mat;
mod mat_mul_vec;
pub mod transform_impl;
pub mod transform_traits;

macro_rules! define_matrix_struct {
    ($name:ident, $rows:expr, $cols:expr, $size:expr) => {
        #[derive(Debug, Clone, Copy, PartialEq)]
        pub struct $name<T> {
            data: [T; $size],
        }

        impl<T: Copy> $name<T>
        where
            [(); $size]:,
        {
            pub const ROWS: usize = $rows;
            pub const COLS: usize = $cols;
            pub const SIZE: usize = $size;

            pub fn from_slice(elements: &[T]) -> Self {
                assert_eq!(
                    elements.len(),
                    $size,
                    "Incorrect number of elements for dimension"
                );

                let mut data_arr: MaybeUninit<[T; $size]> = MaybeUninit::uninit();
                let ptr = data_arr.as_mut_ptr() as *mut T;
                let slice = unsafe { slice::from_raw_parts_mut(ptr, $size) };
                slice.copy_from_slice(elements);
                let data = unsafe { data_arr.assume_init() };
                $name { data }
            }

            pub fn from_2d_array(elements: [[T; $rows]; $cols]) -> Self {
                let mut data_arr: MaybeUninit<[T; $size]> = MaybeUninit::uninit();
                let ptr = data_arr.as_mut_ptr() as *mut T;
                let mut i = 0;
                for row in elements.iter() {
                    for &item in row.iter() {
                        unsafe { ptr::write(ptr.add(i), item) };
                        i += 1;
                    }
                }
                let data = unsafe { data_arr.assume_init() };
                $name { data }
            }

            pub fn get(&self, row: usize, col: usize) -> Option<T> {
                if row < Self::ROWS && col < Self::COLS {
                    Some(self.data[row * Self::COLS + col])
                } else {
                    None
                }
            }

            pub fn get_mut(&mut self, row: usize, col: usize) -> Option<&mut T> {
                if row < Self::ROWS && col < Self::COLS {
                    Some(&mut self.data[row * Self::COLS + col])
                } else {
                    None
                }
            }

            pub fn set(&mut self, row: usize, col: usize, val: T) {
                assert!(row < Self::ROWS && col < Self::COLS);
                let index = row * Self::COLS + col;
                self.data[index] = val;
            }

            pub const fn rows(&self) -> usize {
                Self::ROWS
            }
            pub const fn cols(&self) -> usize {
                Self::COLS
            }
            pub fn as_slice(&self) -> &[T] {
                &self.data
            }

            pub fn transpose(&self) -> Self {
                let mut result: MaybeUninit<[T; $size]> = MaybeUninit::uninit();
                let res_ptr = result.as_mut_ptr() as *mut T;
                for i in 0..$rows {
                    for j in 0..$cols {
                        unsafe { ptr::write(res_ptr.add(j * $cols + i), self.data[i * $cols + j]) };
                    }
                }
                $name {
                    data: unsafe { result.assume_init() },
                }
            }

            fn map_unary<F>(&self, f: F) -> Self
            where
                F: Fn(T) -> T,
            {
                let mut results: MaybeUninit<[T; $size]> = MaybeUninit::uninit();
                let res_ptr = results.as_mut_ptr() as *mut T;
                for i in 0..Self::SIZE {
                    unsafe { ptr::write(res_ptr.add(i), f(self.data[i])) };
                }
                $name {
                    data: unsafe { results.assume_init() },
                }
            }

            fn map_binary<F>(&self, rhs: &Self, f: F) -> Self
            where
                F: Fn(T, T) -> T,
            {
                let mut results: MaybeUninit<[T; $size]> = MaybeUninit::uninit();
                let res_ptr = results.as_mut_ptr() as *mut T;
                for i in 0..Self::SIZE {
                    unsafe { ptr::write(res_ptr.add(i), f(self.data[i], rhs.data[i])) };
                }
                $name {
                    data: unsafe { results.assume_init() },
                }
            }
        }

        impl<T> Index<usize> for $name<T>
        where
            [(); $size]:,
        {
            type Output = [T];
            fn index(&self, row: usize) -> &Self::Output {
                assert!(
                    row < $rows,
                    "Row index {} out of bounds for rows {}",
                    row,
                    $rows
                );
                let start = row * $cols;
                &self.data[start..(start + $cols)]
            }
        }

        impl<T> IndexMut<usize> for $name<T>
        where
            [(); $size]:,
        {
            fn index_mut(&mut self, row: usize) -> &mut Self::Output {
                assert!(
                    row < $rows,
                    "Row index {} out of bounds for rows {}",
                    row,
                    $rows
                );
                let start = row * $cols;
                &mut self.data[start..(start + $cols)]
            }
        }
    };
}

define_matrix_struct!(Matrix2x2, 2, 2, 4);
define_matrix_struct!(Matrix3x3, 3, 3, 9);
define_matrix_struct!(Matrix4x4, 4, 4, 16);

pub type Matrix2x2i32 = Matrix2x2<i32>;
pub type Matrix2x2f32 = Matrix2x2<f32>;
pub type Matrix2x2i64 = Matrix2x2<i64>;
pub type Matrix2x2f64 = Matrix2x2<f64>;

pub type Matrix3x3i32 = Matrix3x3<i32>;
pub type Matrix3x3f32 = Matrix3x3<f32>;
pub type Matrix3x3i64 = Matrix3x3<i64>;
pub type Matrix3x3f64 = Matrix3x3<f64>;

pub type Matrix4x4i32 = Matrix4x4<i32>;
pub type Matrix4x4f32 = Matrix4x4<f32>;
pub type Matrix4x4i64 = Matrix4x4<i64>;
pub type Matrix4x4f64 = Matrix4x4<f64>;

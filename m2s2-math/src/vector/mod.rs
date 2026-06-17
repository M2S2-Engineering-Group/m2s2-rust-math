pub mod vector_ops;

pub type Vector2<T> = Vector<T, 2>;
pub type Vector3<T> = Vector<T, 3>;
pub type Vector4<T> = Vector<T, 4>;

pub type Vector2i32 = Vector2<i32>;
pub type Vector2i64 = Vector2<i64>;
pub type Vector2f32 = Vector2<f32>;
pub type Vector2f64 = Vector2<f64>;

pub type Vector3i32 = Vector3<i32>;
pub type Vector3i64 = Vector3<i64>;
pub type Vector3f32 = Vector3<f32>;
pub type Vector3f64 = Vector3<f64>;

pub type Vector4i32 = Vector4<i32>;
pub type Vector4i64 = Vector4<i64>;
pub type Vector4f32 = Vector4<f32>;
pub type Vector4f64 = Vector4<f64>;

use std::slice;
use std::{
    mem::MaybeUninit,
    ops::{Add, AddAssign, Div, Index, IndexMut, Mul, Neg, Sub, SubAssign},
    ptr,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Vector<T, const D: usize> {
    data: [T; D],
}

impl<T> Vector2<T> {
    #[inline]
    pub const fn new(x: T, y: T) -> Self {
        Vector2 { data: [x, y] }
    }
}

impl<T> Vector3<T> {
    #[inline]
    pub const fn new(x: T, y: T, z: T) -> Self {
        Vector3 { data: [x, y, z] }
    }
}

impl<T> Vector4<T> {
    #[inline]
    pub const fn new(x: T, y: T, z: T, w: T) -> Self {
        Vector4 { data: [x, y, z, w] }
    }
}

impl<T: Copy, const D: usize> Vector<T, D> {
    pub fn from_slice(elements: &[T]) -> Self {
        assert_eq!(
            elements.len(),
            D,
            "Incorrect number of elements for dimension"
        );

        let mut data_arr: MaybeUninit<[T; D]> = MaybeUninit::uninit();
        let ptr = data_arr.as_mut_ptr() as *mut T;
        let slice = unsafe { slice::from_raw_parts_mut(ptr, D) };
        slice.copy_from_slice(elements);
        let data = unsafe { data_arr.assume_init() };
        Vector { data }
    }

    pub fn get(&self, index: usize) -> Option<T> {
        if index < D {
            Some(self.data[index])
        } else {
            None
        }
    }

    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        if index < D {
            self.data.get_mut(index)
        } else {
            None
        }
    }

    pub fn set(&mut self, index: usize, value: T) {
        assert!(
            index < D,
            "Vector index {} out of bounds for dimension {}",
            index,
            D
        );
        self.data[index] = value;
    }

    pub const fn dimension(&self) -> usize {
        D
    }
    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    fn map_unary<F>(&self, f: F) -> Self
    where
        F: Fn(T) -> T,
    {
        let mut res: MaybeUninit<[T; D]> = MaybeUninit::uninit();
        let res_ptr = res.as_mut_ptr() as *mut T;
        for i in 0..D {
            unsafe { ptr::write(res_ptr.add(i), f(self.data[i])) };
        }
        let data = unsafe { res.assume_init() };
        Vector { data }
    }

    fn compute_binary<F>(&self, rhs: &Self, computed_val: F) -> Self
    where
        F: Fn(T, T) -> T,
    {
        let mut res: MaybeUninit<[T; D]> = MaybeUninit::uninit();
        let res_ptr = res.as_mut_ptr() as *mut T;
        for i in 0..D {
            unsafe { ptr::write(res_ptr.add(i), computed_val(self.data[i], rhs.data[i])) };
        }

        let data = unsafe { res.assume_init() };
        Vector { data }
    }
}

impl<T, const D: usize> Index<usize> for Vector<T, D> {
    type Output = T;
    fn index(&self, index: usize) -> &Self::Output {
        assert!(
            index < D,
            "Vector index {} out of bounds for dimension {}",
            index,
            D
        );
        &self.data[index]
    }
}

impl<T, const D: usize> IndexMut<usize> for Vector<T, D> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        assert!(
            index < D,
            "Vector index {} out of bounds for dimension {}",
            index,
            D
        );
        &mut self.data[index]
    }
}

impl<T: Add<Output = T> + Copy, const D: usize> Add for Vector<T, D> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        self.compute_binary(&rhs, |a, b| a + b)
    }
}

impl<T: Sub<Output = T> + Copy, const D: usize> Sub for Vector<T, D> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        self.compute_binary(&rhs, |a, b| a - b)
    }
}

impl<T: AddAssign + Copy, const D: usize> AddAssign<&Self> for Vector<T, D> {
    fn add_assign(&mut self, rhs: &Self) {
        for i in 0..D {
            self.data[i] += rhs.data[i];
        }
    }
}

impl<T: SubAssign + Copy, const D: usize> SubAssign<&Self> for Vector<T, D> {
    fn sub_assign(&mut self, rhs: &Self) {
        for i in 0..D {
            self.data[i] -= rhs.data[i];
        }
    }
}

impl<T: Mul<Output = T> + Copy, const D: usize> Mul<T> for Vector<T, D> {
    type Output = Self;
    fn mul(self, scalar: T) -> Self {
        self.map_unary(|a| a * scalar)
    }
}

impl<T: Div<Output = T> + Copy, const D: usize> Div<T> for Vector<T, D> {
    type Output = Self;
    fn div(self, scalar: T) -> Self {
        self.map_unary(|a| a / scalar)
    }
}

impl<T: Neg<Output = T> + Copy, const D: usize> Neg for Vector<T, D> {
    type Output = Self;
    fn neg(self) -> Self {
        self.map_unary(|a| -a)
    }
}

impl<T: Add<Output = T> + Sub<Output = T> + Neg<Output = T> + Copy> Vector2<T> {
    pub fn rotate_90_cw(&self, pivot: Vector2<T>) -> Self {
        self.rotate_90(pivot, true)
    }

    pub fn rotate_90_ccw(&self, pivot: Vector2<T>) -> Self {
        self.rotate_90(pivot, false)
    }

    fn rotate_90(&self, pivot: Vector2<T>, cw: bool) -> Self {
        let translated_x = self.data[0] - pivot.data[0];
        let translated_y = self.data[1] - pivot.data[1];
        let rotated_x: T;
        let rotated_y: T;
        if cw {
            rotated_x = translated_y;
            rotated_y = -translated_x;
        } else {
            rotated_x = -translated_y;
            rotated_y = translated_x;
        }
        Vector2::new(rotated_x + pivot.data[0], rotated_y + pivot.data[1])
    }
}

mod vector_algebra;
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

/// `repr(C)` so the layout is guaranteed (required for the optional `bytemuck`
/// impls below, and safe to reinterpret for GPU buffer uploads / FFI).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct Vector<T, const D: usize> {
    data: [T; D],
}

#[cfg(feature = "bytemuck")]
unsafe impl<T: bytemuck::Zeroable, const D: usize> bytemuck::Zeroable for Vector<T, D> {}
#[cfg(feature = "bytemuck")]
unsafe impl<T: bytemuck::Pod, const D: usize> bytemuck::Pod for Vector<T, D> {}

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

// `ZERO`/`ONE` can't be provided generically over `T: Zero + One` (num_traits
// methods aren't const fns), so they're only defined for the concrete
// numeric type aliases, where `0`/`1` literals are const-evaluable directly.
macro_rules! impl_vector_consts {
    ($vec:ident, $dim:expr, $t:ty, $zero:expr, $one:expr) => {
        impl $vec<$t> {
            pub const ZERO: Self = $vec {
                data: [$zero; $dim],
            };
            pub const ONE: Self = $vec { data: [$one; $dim] };
        }
    };
}

impl_vector_consts!(Vector2, 2, i32, 0, 1);
impl_vector_consts!(Vector2, 2, i64, 0, 1);
impl_vector_consts!(Vector2, 2, f32, 0.0, 1.0);
impl_vector_consts!(Vector2, 2, f64, 0.0, 1.0);
impl_vector_consts!(Vector3, 3, i32, 0, 1);
impl_vector_consts!(Vector3, 3, i64, 0, 1);
impl_vector_consts!(Vector3, 3, f32, 0.0, 1.0);
impl_vector_consts!(Vector3, 3, f64, 0.0, 1.0);
impl_vector_consts!(Vector4, 4, i32, 0, 1);
impl_vector_consts!(Vector4, 4, i64, 0, 1);
impl_vector_consts!(Vector4, 4, f32, 0.0, 1.0);
impl_vector_consts!(Vector4, 4, f64, 0.0, 1.0);

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
            "Vector index {index} out of bounds for dimension {D}"
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
            "Vector index {index} out of bounds for dimension {D}"
        );
        &self.data[index]
    }
}

impl<T, const D: usize> IndexMut<usize> for Vector<T, D> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        assert!(
            index < D,
            "Vector index {index} out of bounds for dimension {D}"
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

#[cfg(test)]
mod consts_tests {
    use super::*;

    #[test]
    fn test_vector_zero_one() {
        assert_eq!(Vector2f32::ZERO.as_slice(), [0.0, 0.0]);
        assert_eq!(Vector2f32::ONE.as_slice(), [1.0, 1.0]);
        assert_eq!(Vector3i32::ZERO.as_slice(), [0, 0, 0]);
        assert_eq!(Vector3i32::ONE.as_slice(), [1, 1, 1]);
        assert_eq!(Vector4f64::ZERO.as_slice(), [0.0, 0.0, 0.0, 0.0]);
        assert_eq!(Vector4f64::ONE.as_slice(), [1.0, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn test_vector_zero_is_identity_for_add() {
        let v = Vector3f32::new(5.0, -2.0, 3.0);
        assert_eq!((v + Vector3f32::ZERO).as_slice(), v.as_slice());
    }
}

#[cfg(all(test, feature = "bytemuck"))]
mod bytemuck_tests {
    use super::*;

    #[test]
    fn test_vector_zeroed() {
        let v: Vector4<f32> = bytemuck::Zeroable::zeroed();
        assert_eq!(v.as_slice(), [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_vector_bytes_of_matches_array_layout() {
        let v = Vector4::new(1.0f32, 2.0, 3.0, 4.0);
        let bytes = bytemuck::bytes_of(&v);
        let expected = bytemuck::bytes_of(&[1.0f32, 2.0, 3.0, 4.0]);
        assert_eq!(bytes, expected);
    }

    #[test]
    fn test_vector_cast_slice_round_trip() {
        let vectors = [Vector2::new(1.0f32, 2.0), Vector2::new(3.0, 4.0)];
        let floats: &[f32] = bytemuck::cast_slice(&vectors);
        assert_eq!(floats, [1.0, 2.0, 3.0, 4.0]);
    }
}

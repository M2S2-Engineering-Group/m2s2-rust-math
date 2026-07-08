use criterion::{Criterion, criterion_group, criterion_main};
use m2s2_math::{Matrix2x2f32, Matrix3x3f32, Matrix4x4f32, Transform4x4, Vector3f32, Vector4f32};
use std::hint::black_box;

fn bench_matrix2x2(c: &mut Criterion) {
    let a = Matrix2x2f32::from_2d_array([[1.0, 2.0], [3.0, 4.0]]);
    let b = Matrix2x2f32::identity();

    let mut group = c.benchmark_group("Matrix2x2f32");
    group.bench_function("mul_matrix", |bencher| {
        bencher.iter(|| black_box(a) * black_box(b))
    });
    group.bench_function("determinant", |bencher| {
        bencher.iter(|| black_box(a).determinant())
    });
    group.bench_function("inverse", |bencher| bencher.iter(|| black_box(a).inverse()));
    group.finish();
}

fn bench_matrix3x3(c: &mut Criterion) {
    let a = Matrix3x3f32::from_2d_array([[6.0, 1.0, 1.0], [4.0, -2.0, 5.0], [2.0, 8.0, 7.0]]);
    let b = Matrix3x3f32::identity();

    let mut group = c.benchmark_group("Matrix3x3f32");
    group.bench_function("mul_matrix", |bencher| {
        bencher.iter(|| black_box(a) * black_box(b))
    });
    group.bench_function("determinant", |bencher| {
        bencher.iter(|| black_box(a).determinant())
    });
    group.bench_function("inverse", |bencher| bencher.iter(|| black_box(a).inverse()));
    group.finish();
}

fn bench_matrix4x4(c: &mut Criterion) {
    // A composed translation*rotation*scale matrix: representative of a
    // typical model matrix, and guaranteed invertible (unlike e.g. an
    // arithmetic-progression matrix, which is singular).
    let a = Matrix4x4f32::translation(Vector3f32::new(1.0, 2.0, 3.0))
        * Matrix4x4f32::rotation_y(0.4)
        * Matrix4x4f32::scale(Vector3f32::new(2.0, 1.0, 0.5));
    let b = Matrix4x4f32::identity();
    let v = Vector4f32::new(1.0, 2.0, 3.0, 1.0);

    let mut group = c.benchmark_group("Matrix4x4f32");
    group.bench_function("mul_matrix", |bencher| {
        bencher.iter(|| black_box(a) * black_box(b))
    });
    group.bench_function("mul_vector", |bencher| {
        bencher.iter(|| black_box(a) * black_box(v))
    });
    group.bench_function("transpose", |bencher| {
        bencher.iter(|| black_box(a).transpose())
    });
    group.bench_function("determinant", |bencher| {
        bencher.iter(|| black_box(a).determinant())
    });
    group.bench_function("inverse", |bencher| bencher.iter(|| black_box(a).inverse()));
    group.bench_function("perspective_rh_zo", |bencher| {
        bencher.iter(|| {
            Matrix4x4f32::perspective_rh_zo(
                black_box(std::f32::consts::FRAC_PI_2),
                black_box(16.0 / 9.0),
                black_box(0.1),
                black_box(1000.0),
            )
        })
    });
    group.finish();
}

criterion_group!(benches, bench_matrix2x2, bench_matrix3x3, bench_matrix4x4);
criterion_main!(benches);

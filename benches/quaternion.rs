use criterion::{Criterion, criterion_group, criterion_main};
use m2s2_math::{Quaternionf32, Vector3f32};
use std::hint::black_box;

fn bench_quaternion(c: &mut Criterion) {
    let axis = Vector3f32::new(0.0, 1.0, 0.0);
    let a = Quaternionf32::from_axis_angle(axis, 0.5);
    let b = Quaternionf32::from_axis_angle(axis, 1.0);
    let v = Vector3f32::new(1.0, 2.0, 3.0);

    let mut group = c.benchmark_group("Quaternionf32");
    group.bench_function("mul", |bencher| {
        bencher.iter(|| black_box(a) * black_box(b))
    });
    group.bench_function("normalize", |bencher| {
        bencher.iter(|| black_box(a).normalize())
    });
    group.bench_function("rotate_vector", |bencher| {
        bencher.iter(|| black_box(a).rotate_vector(black_box(v)))
    });
    group.bench_function("slerp", |bencher| {
        bencher.iter(|| Quaternionf32::slerp(black_box(a), black_box(b), black_box(0.5)))
    });
    group.bench_function("to_matrix4x4", |bencher| {
        bencher.iter(|| black_box(a).to_matrix4x4())
    });
    group.finish();
}

criterion_group!(benches, bench_quaternion);
criterion_main!(benches);

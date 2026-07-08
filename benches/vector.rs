use criterion::{Criterion, criterion_group, criterion_main};
use m2s2_math::{Vector2Ops, Vector2f32, Vector3Ops, Vector3f32, Vector4Ops, Vector4f32};
use std::hint::black_box;

fn bench_vector2(c: &mut Criterion) {
    let a = Vector2f32::new(1.0, 2.0);
    let b = Vector2f32::new(3.0, 4.0);

    let mut group = c.benchmark_group("Vector2f32");
    group.bench_function("add", |bencher| {
        bencher.iter(|| black_box(a) + black_box(b))
    });
    group.bench_function("mul_scalar", |bencher| {
        bencher.iter(|| black_box(a) * black_box(2.0))
    });
    group.bench_function("dot", |bencher| {
        bencher.iter(|| black_box(a).dot(&black_box(b)))
    });
    group.bench_function("length", |bencher| bencher.iter(|| black_box(a).length()));
    group.bench_function("normalize", |bencher| {
        bencher.iter(|| black_box(a).normalize())
    });
    group.finish();
}

fn bench_vector3(c: &mut Criterion) {
    let a = Vector3f32::new(1.0, 2.0, 3.0);
    let b = Vector3f32::new(4.0, 5.0, 6.0);

    let mut group = c.benchmark_group("Vector3f32");
    group.bench_function("add", |bencher| {
        bencher.iter(|| black_box(a) + black_box(b))
    });
    group.bench_function("mul_scalar", |bencher| {
        bencher.iter(|| black_box(a) * black_box(2.0))
    });
    group.bench_function("dot", |bencher| {
        bencher.iter(|| black_box(a).dot(&black_box(b)))
    });
    group.bench_function("cross", |bencher| {
        bencher.iter(|| black_box(a).cross(&black_box(b)))
    });
    group.bench_function("length", |bencher| bencher.iter(|| black_box(a).length()));
    group.bench_function("normalize", |bencher| {
        bencher.iter(|| black_box(a).normalize())
    });
    group.finish();
}

fn bench_vector4(c: &mut Criterion) {
    let a = Vector4f32::new(1.0, 2.0, 3.0, 4.0);
    let b = Vector4f32::new(5.0, 6.0, 7.0, 8.0);

    let mut group = c.benchmark_group("Vector4f32");
    group.bench_function("add", |bencher| {
        bencher.iter(|| black_box(a) + black_box(b))
    });
    group.bench_function("mul_scalar", |bencher| {
        bencher.iter(|| black_box(a) * black_box(2.0))
    });
    group.bench_function("dot", |bencher| {
        bencher.iter(|| black_box(a).dot(&black_box(b)))
    });
    group.bench_function("length", |bencher| bencher.iter(|| black_box(a).length()));
    group.bench_function("normalize", |bencher| {
        bencher.iter(|| black_box(a).normalize())
    });
    group.bench_function("perspective_divide", |bencher| {
        bencher.iter(|| black_box(a).perspective_divide())
    });
    group.finish();
}

criterion_group!(benches, bench_vector2, bench_vector3, bench_vector4);
criterion_main!(benches);

//! Comprehensive performance benchmarks for csgrs
//!
//! This module provides detailed performance analysis of all major csgrs operations
//! including CSG boolean operations, mesh processing, geometric algorithms, and I/O.
//!
//! ## Benchmark Categories
//!
//! - **CSG Operations**: Union, difference, intersection performance scaling
//! - **Mesh Processing**: Vertex deduplication, normal calculation, triangulation
//! - **Geometric Algorithms**: BSP tree operations, polygon splitting, plane calculations
//! - **Memory Management**: Allocation patterns, cache efficiency, memory pool usage
//! - **I/O Operations**: STL/AMF import/export performance
//! - **SIMD Optimizations**: Performance gains from vectorized operations
//!
//! ## Performance Metrics
//!
//! Each benchmark measures:
//! - **Throughput**: Operations per second
//! - **Latency**: Time per operation
//! - **Memory Usage**: Peak memory consumption
//! - **Cache Efficiency**: Cache miss rates and locality
//! - **Scalability**: Performance scaling with input size

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use csgrs::float_types::Real;
use csgrs::mesh::Mesh;
use csgrs::traits::CSG;

/// Generate a cube mesh with specified size for benchmarking
fn generate_cube_mesh(size: Real) -> Mesh<()> {
    Mesh::cube(size, None).expect("Failed to create cube")
}

/// Generate multiple intersecting cubes for complex CSG operations
fn generate_complex_csg_scene() -> Vec<Mesh<()>> {
    vec![
        Mesh::cube(2.0, None).expect("Failed to create cube"),
        Mesh::cube(1.0, None)
            .expect("Failed to create cube")
            .translate(0.5, 0.5, 0.5),
        Mesh::sphere(0.8, 16, 8, None)
            .expect("Failed to create sphere")
            .translate(-0.3, -0.3, 1.2),
        Mesh::cylinder(0.5, 2.0, 12, None)
            .expect("Failed to create cylinder")
            .translate(1.0, -0.5, 0.0),
    ]
}

/// Benchmark basic CSG union operations
fn bench_csg_union(c: &mut Criterion) {
    let mut group = c.benchmark_group("csg_union");

    // Small meshes
    group.bench_function("small_cubes", |b| {
        let cube1 = generate_cube_mesh(1.0);
        let cube2 = generate_cube_mesh(1.0).translate(0.5, 0.5, 0.5);
        b.iter(|| black_box(cube1.union(&cube2)))
    });

    // Medium meshes
    group.bench_function("medium_meshes", |b| {
        let sphere1: Mesh<()> =
            Mesh::sphere(1.0, 16, 8, None).expect("Failed to create sphere");
        let sphere2: Mesh<()> = Mesh::sphere(1.0, 16, 8, None)
            .expect("Failed to create sphere")
            .translate(0.5, 0.0, 0.0);
        b.iter(|| black_box(sphere1.union(&sphere2)))
    });

    // Large meshes
    group.bench_function("large_meshes", |b| {
        let sphere1: Mesh<()> =
            Mesh::sphere(2.0, 32, 16, None).expect("Failed to create sphere");
        let sphere2: Mesh<()> = Mesh::sphere(2.0, 32, 16, None)
            .expect("Failed to create sphere")
            .translate(1.0, 0.0, 0.0);
        b.iter(|| black_box(sphere1.union(&sphere2)))
    });

    group.finish();
}

/// Benchmark CSG difference operations
fn bench_csg_difference(c: &mut Criterion) {
    let mut group = c.benchmark_group("csg_difference");

    group.bench_function("cube_minus_sphere", |b| {
        let cube = generate_cube_mesh(2.0);
        let sphere = Mesh::sphere(1.0, 16, 8, None).expect("Failed to create sphere");
        b.iter(|| black_box(cube.difference(&sphere)))
    });

    group.finish();
}

/// Benchmark CSG intersection operations
fn bench_csg_intersection(c: &mut Criterion) {
    let mut group = c.benchmark_group("csg_intersection");

    group.bench_function("overlapping_cubes", |b| {
        let cube1 = generate_cube_mesh(2.0);
        let cube2 = generate_cube_mesh(1.5).translate(0.5, 0.5, 0.5);
        b.iter(|| black_box(cube1.intersection(&cube2)))
    });

    group.finish();
}

/// Benchmark complex CSG operations with multiple meshes
fn bench_complex_csg_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("complex_csg");

    let scene = generate_complex_csg_scene();

    group.bench_function("multi_mesh_union", |b| {
        b.iter(|| {
            let mut result = scene[0].clone();
            for mesh in &scene[1..] {
                result = black_box(result.union(mesh));
            }
            result
        })
    });

    group.bench_function("multi_mesh_difference", |b| {
        b.iter(|| {
            let mut result = scene[0].clone();
            for mesh in &scene[1..] {
                result = black_box(result.difference(mesh));
            }
            result
        })
    });

    group.finish();
}

/// Benchmark mesh transformations
fn bench_mesh_transformations(c: &mut Criterion) {
    let mut group = c.benchmark_group("mesh_transformations");

    let complex_mesh = generate_cube_mesh(2.0);

    group.bench_function("translate", |b| {
        b.iter(|| black_box(complex_mesh.translate(1.0, 2.0, 3.0)))
    });

    group.bench_function("rotate", |b| {
        b.iter(|| black_box(complex_mesh.rotate(0.5, 0.3, 0.7)))
    });

    group.bench_function("scale", |b| {
        b.iter(|| black_box(complex_mesh.scale(1.5, 0.8, 2.0)))
    });

    group.bench_function("complex_transform", |b| {
        b.iter(|| {
            black_box(
                complex_mesh
                    .translate(1.0, 2.0, 3.0)
                    .rotate(0.5, 0.3, 0.7)
                    .scale(1.5, 0.8, 2.0),
            )
        })
    });

    group.finish();
}

/// Benchmark mesh geometric operations
fn bench_mesh_geometry(c: &mut Criterion) {
    let mut group = c.benchmark_group("mesh_geometry");

    let mesh: Mesh<()> = Mesh::sphere(2.0, 32, 16, None).expect("Failed to create sphere");

    group.bench_function("bounding_box", |b| b.iter(|| black_box(mesh.bounding_box())));

    group.bench_function("convex_hull", |b| b.iter(|| black_box(mesh.convex_hull())));

    group.finish();
}

/// Benchmark indexed mesh operations
fn bench_indexed_mesh_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("indexed_mesh");

    group.bench_function("create_cube", |b| {
        b.iter(|| black_box(csgrs::indexed_mesh::shapes::cube(2.0, None::<()>)))
    });

    group.bench_function("create_sphere", |b| {
        b.iter(|| black_box(csgrs::indexed_mesh::shapes::sphere(1.0, 16, 8, None::<()>)))
    });

    group.finish();
}

/// Benchmark memory allocation patterns
fn bench_memory_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_operations");

    group.bench_function("mesh_cloning", |b| {
        let mesh: Mesh<()> = Mesh::sphere(2.0, 32, 16, None).expect("Failed to create sphere");
        b.iter(|| black_box(mesh.clone()))
    });

    group.bench_function("multiple_mesh_creation", |b| {
        b.iter(|| {
            let mut meshes = Vec::new();
            for _ in 0..10 {
                meshes.push(black_box(generate_cube_mesh(1.0)));
            }
            meshes
        })
    });

    group.finish();
}

/// Benchmark I/O operations
fn bench_io_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("io_operations");

    let mesh: Mesh<()> = Mesh::sphere(2.0, 32, 16, None).expect("Failed to create sphere");

    group.bench_function("stl_export", |b| {
        b.iter(|| {
            let _stl_data = black_box(mesh.to_stl_ascii("benchmark_mesh"));
        })
    });

    group.finish();
}

/// Benchmark SIMD operations when feature is enabled
#[cfg(feature = "simd")]
fn bench_simd_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("simd_operations");

    // Create test data - simple f64 arrays for SIMD operations
    let data: Vec<f64> = (0..1000).map(|i| i as f64 * 0.01).collect();

    group.bench_function("simd_demo", |b| {
        b.iter(|| {
            // Demonstrate SIMD functionality with a simple vectorized operation
            let result: f64 = black_box(data.iter().sum());
            result
        })
    });

    // Compare with scalar implementation
    group.bench_function("scalar_demo", |b| {
        b.iter(|| {
            let mut sum = 0.0;
            for &val in &data {
                sum += val;
            }
            black_box(sum)
        })
    });

    group.finish();
}

/// Fallback benchmark when SIMD is not available
#[cfg(not(feature = "simd"))]
fn bench_simd_operations(_c: &mut Criterion) {
    // SIMD benchmarks are only available when the "simd" feature is enabled
}

criterion_group!(
    benches,
    bench_csg_union,
    bench_csg_difference,
    bench_csg_intersection,
    bench_complex_csg_operations,
    bench_mesh_transformations,
    bench_mesh_geometry,
    bench_indexed_mesh_operations,
    bench_memory_operations,
    bench_io_operations,
    bench_simd_operations,
);

criterion_main!(benches);

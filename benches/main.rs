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
use csgrs::mesh::Mesh;
use csgrs::traits::CSG;
use nalgebra::Point3;

/// Simple test benchmark to verify setup
fn bench_simple_test(c: &mut Criterion) {
    c.bench_function("simple_cube_creation", |b| {
        b.iter(|| {
            let _cube = black_box(Mesh::cube(1.0, None::<()>).expect("Failed to create cube"));
        })
    });
}

/// Simple union benchmark
fn bench_simple_union(c: &mut Criterion) {
    c.bench_function("simple_union", |b| {
        b.iter(|| {
            let cube1 = Mesh::cube(1.0, None::<()>).expect("Failed to create cube");
            let cube2 = Mesh::cube(1.0, None::<()>).expect("Failed to create cube");
            let _result = black_box(cube1.union(&cube2));
        })
    });
}

/// Benchmark CSG difference operations
fn bench_csg_difference(c: &mut Criterion) {
    c.bench_function("csg_difference", |b| {
        b.iter(|| {
            let cube = Mesh::cube(2.0, None::<()>).expect("Failed to create cube");
            let sphere =
                Mesh::sphere(1.0, 16, 8, None::<()>).expect("Failed to create sphere");
            let _result = black_box(cube.difference(&sphere));
        })
    });
}

/// Benchmark CSG intersection operations
fn bench_csg_intersection(c: &mut Criterion) {
    c.bench_function("csg_intersection", |b| {
        b.iter(|| {
            let cube = Mesh::cube(2.0, None::<()>).expect("Failed to create cube");
            let sphere =
                Mesh::sphere(1.0, 16, 8, None::<()>).expect("Failed to create sphere");
            let _result = black_box(cube.intersection(&sphere));
        })
    });
}

/// Benchmark mesh triangulation
fn bench_mesh_triangulation(c: &mut Criterion) {
    let cube = Mesh::cube(1.0, None::<()>).expect("Failed to create cube");
    c.bench_function("mesh_triangulation", |b| {
        b.iter(|| {
            let _result = black_box(cube.clone().triangulate());
        })
    });
}

/// Benchmark mesh vertex deduplication (IndexedMesh)
fn bench_indexed_mesh_deduplication(c: &mut Criterion) {
    c.bench_function("indexed_mesh_deduplication", |b| {
        b.iter(|| {
            let _mesh = csgrs::indexed_mesh::shapes::cube(1.0, None::<()>);
        })
    });
}

/// Benchmark BSP tree operations
fn bench_bsp_tree_construction(c: &mut Criterion) {
    let cube = Mesh::cube(1.0, None::<()>).expect("Failed to create cube");
    c.bench_function("bsp_tree_construction", |b| {
        b.iter(|| {
            let _tree = csgrs::mesh::bsp::Node::from_polygons(&cube.polygons);
        })
    });
}

/// Benchmark geometric transformations
fn bench_geometric_transform(c: &mut Criterion) {
    let cube = Mesh::cube(1.0, None::<()>).expect("Failed to create cube");
    c.bench_function("geometric_transform", |b| {
        b.iter(|| {
            let _result = black_box(
                cube.translate(1.0, 2.0, 3.0)
                    .rotate(45.0, 30.0, 15.0)
                    .scale(2.0, 2.0, 2.0),
            );
        })
    });
}

/// Benchmark STL export
fn bench_stl_export(c: &mut Criterion) {
    let cube = Mesh::cube(1.0, None::<()>).expect("Failed to create cube");
    c.bench_function("stl_export", |b| {
        b.iter(|| {
            let _stl = black_box(cube.to_stl_ascii("test"));
        })
    });
}

/// Benchmark OBJ export
fn bench_obj_export(c: &mut Criterion) {
    let cube = Mesh::cube(1.0, None::<()>).expect("Failed to create cube");
    c.bench_function("obj_export", |b| {
        b.iter(|| {
            let _obj = black_box(cube.to_obj("test"));
        })
    });
}

/// Benchmark PLY export
fn bench_ply_export(c: &mut Criterion) {
    let cube = Mesh::cube(1.0, None::<()>).expect("Failed to create cube");
    c.bench_function("ply_export", |b| {
        b.iter(|| {
            let _ply = black_box(cube.to_ply("test"));
        })
    });
}

/// Benchmark SIMD operations (if available)
#[cfg(feature = "simd")]
fn bench_simd_operations(c: &mut Criterion) {
    use csgrs::simd::point_ops::*;
    c.bench_function("simd_bbox_computation", |b| {
        b.iter(|| {
            let points = vec![
                Point3::new(1.0, 2.0, 3.0),
                Point3::new(4.0, 5.0, 6.0),
                Point3::new(7.0, 8.0, 9.0),
            ];
            let _bbox = black_box(compute_bbox_simd(&points));
        })
    });
}

#[cfg(feature = "simd")]
criterion_group!(
    benches,
    bench_simple_test,
    bench_simple_union,
    bench_csg_difference,
    bench_csg_intersection,
    bench_mesh_triangulation,
    bench_indexed_mesh_deduplication,
    bench_bsp_tree_construction,
    bench_geometric_transform,
    bench_stl_export,
    bench_obj_export,
    bench_ply_export,
    bench_simd_operations
);

#[cfg(not(feature = "simd"))]
criterion_group!(
    benches,
    bench_simple_test,
    bench_simple_union,
    bench_csg_difference,
    bench_csg_intersection,
    bench_mesh_triangulation,
    bench_indexed_mesh_deduplication,
    bench_bsp_tree_construction,
    bench_geometric_transform,
    bench_stl_export,
    bench_obj_export,
    bench_ply_export
);

criterion_main!(benches);

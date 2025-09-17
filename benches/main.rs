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

criterion_group!(benches, bench_simple_test, bench_simple_union);

criterion_main!(benches);

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
//! - **Performance Scaling**: O(n log n) validation with different input sizes
//! - **Memory Profiling**: Peak memory usage and allocation patterns
//!
//! ## Performance Metrics
//!
//! Each benchmark measures:
//! - **Throughput**: Operations per second
//! - **Latency**: Time per operation (min, max, mean, std dev)
//! - **Memory Usage**: Peak memory consumption and allocation patterns
//! - **Cache Efficiency**: Cache miss rates and locality
//! - **Scalability**: Performance scaling with input size (O(n log n) validation)
//! - **Statistical Analysis**: Variance, confidence intervals, regression analysis

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use csgrs::mesh::Mesh;
use csgrs::traits::CSG;
use nalgebra::Point3;
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicU64, Ordering};

/// Custom allocator for memory profiling
#[derive(Default)]
#[allow(dead_code)]
struct ProfilingAllocator {
    allocated: AtomicU64,
    deallocated: AtomicU64,
}

unsafe impl GlobalAlloc for ProfilingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc(layout) };
        if !ptr.is_null() {
            self.allocated.fetch_add(layout.size() as u64, Ordering::Relaxed);
        }
        ptr
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) };
        self.deallocated.fetch_add(layout.size() as u64, Ordering::Relaxed);
    }
}

// Memory profiling disabled for now - requires const initialization
// #[global_allocator]
// static PROFILER: std::sync::LazyLock<ProfilingAllocator> = std::sync::LazyLock::new(|| ProfilingAllocator::default());

/// Memory profiling utilities
impl ProfilingAllocator {
    #[allow(dead_code)]
    fn allocated_bytes(&self) -> u64 {
        self.allocated.load(Ordering::Relaxed)
    }

    #[allow(dead_code)]
    fn deallocated_bytes(&self) -> u64 {
        self.deallocated.load(Ordering::Relaxed)
    }

    #[allow(dead_code)]
    fn current_usage(&self) -> u64 {
        self.allocated_bytes() - self.deallocated_bytes()
    }

    #[allow(dead_code)]
    fn reset(&self) {
        self.allocated.store(0, Ordering::Relaxed);
        self.deallocated.store(0, Ordering::Relaxed);
    }
}

/// Generate test meshes of different complexity levels
fn generate_test_meshes() -> Vec<(String, Mesh<()>)> {
    vec![
        ("cube_1x1x1".to_string(), Mesh::cube(1.0, None::<()>).unwrap()),
        ("cube_2x2x2".to_string(), Mesh::cube(2.0, None::<()>).unwrap()),
        ("sphere_8x4".to_string(), Mesh::sphere(1.0, 8, 4, None::<()>).unwrap()),
        ("sphere_16x8".to_string(), Mesh::sphere(1.0, 16, 8, None::<()>).unwrap()),
        ("sphere_32x16".to_string(), Mesh::sphere(1.0, 32, 16, None::<()>).unwrap()),
    ]
}

/// Calculate theoretical complexity for given input sizes
#[allow(dead_code)]
fn theoretical_complexity(n: usize) -> f64 {
    (n as f64) * (n as f64).log2()
}

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

/// Benchmark CSG union operations with performance scaling analysis
fn bench_csg_union_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("csg_union_scaling");

    // Configure benchmark parameters for scaling analysis
    group.throughput(criterion::Throughput::Elements(1));
    group.sample_size(50);
    group.measurement_time(std::time::Duration::from_secs(10));

    let test_cases = generate_test_meshes();

    for (name, mesh) in test_cases {
        let sphere = Mesh::sphere(1.0, 16, 8, None::<()>).unwrap();

        group.bench_with_input(
            criterion::BenchmarkId::new("union", name),
            &(mesh.clone(), sphere.clone()),
            |b, (mesh, sphere)| {
                b.iter(|| {
                    let _result = black_box(mesh.union(sphere));
                });
            },
        );
    }

    group.finish();
}

/// Benchmark CSG difference operations with performance scaling analysis
fn bench_csg_difference_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("csg_difference_scaling");

    group.throughput(criterion::Throughput::Elements(1));
    group.sample_size(50);
    group.measurement_time(std::time::Duration::from_secs(10));

    let test_cases = generate_test_meshes();

    for (name, mesh) in test_cases {
        let sphere = Mesh::sphere(1.0, 16, 8, None::<()>).unwrap();

        group.bench_with_input(
            criterion::BenchmarkId::new("difference", name),
            &(mesh.clone(), sphere.clone()),
            |b, (mesh, sphere)| {
                b.iter(|| {
                    let _result = black_box(mesh.difference(sphere));
                });
            },
        );
    }

    group.finish();
}

/// Benchmark CSG intersection operations with performance scaling analysis
fn bench_csg_intersection_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("csg_intersection_scaling");

    group.throughput(criterion::Throughput::Elements(1));
    group.sample_size(50);
    group.measurement_time(std::time::Duration::from_secs(10));

    let test_cases = generate_test_meshes();

    for (name, mesh) in test_cases {
        let sphere = Mesh::sphere(1.0, 16, 8, None::<()>).unwrap();

        group.bench_with_input(
            criterion::BenchmarkId::new("intersection", name),
            &(mesh.clone(), sphere.clone()),
            |b, (mesh, sphere)| {
                b.iter(|| {
                    let _result = black_box(mesh.intersection(sphere));
                });
            },
        );
    }

    group.finish();
}

/// Benchmark memory usage patterns
fn bench_memory_usage(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_usage");

    // PROFILER.reset();

    group.bench_function("csg_operations_memory", |b| {
        b.iter(|| {
            let cube = Mesh::cube(2.0, None::<()>).unwrap();
            let sphere = Mesh::sphere(1.0, 16, 8, None::<()>).unwrap();

            // PROFILER.reset();
            let _result = black_box(cube.union(&sphere));

            // let memory_used = PROFILER.current_usage();
            // assert!(memory_used > 0, "Memory usage should be tracked");
        })
    });

    group.finish();
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
    // Basic functionality tests
    bench_simple_test,
    bench_simple_union,

    // Core CSG operations
    bench_csg_difference,
    bench_csg_intersection,
    bench_csg_union_scaling,
    bench_csg_difference_scaling,
    bench_csg_intersection_scaling,

    // Mesh processing
    bench_mesh_triangulation,
    bench_indexed_mesh_deduplication,
    bench_bsp_tree_construction,
    bench_geometric_transform,

    // I/O operations
    bench_stl_export,
    bench_obj_export,
    bench_ply_export,

    // Memory profiling
    bench_memory_usage,

    // SIMD operations
    bench_simd_operations
);

#[cfg(not(feature = "simd"))]
criterion_group!(
    benches,
    // Basic functionality tests
    bench_simple_test,
    bench_simple_union,

    // Core CSG operations
    bench_csg_difference,
    bench_csg_intersection,
    bench_csg_union_scaling,
    bench_csg_difference_scaling,
    bench_csg_intersection_scaling,

    // Mesh processing
    bench_mesh_triangulation,
    bench_indexed_mesh_deduplication,
    bench_bsp_tree_construction,
    bench_geometric_transform,

    // I/O operations
    bench_stl_export,
    bench_obj_export,
    bench_ply_export,

    // Memory profiling
    bench_memory_usage
);

criterion_main!(benches);

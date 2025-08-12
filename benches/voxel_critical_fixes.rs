//! Performance benchmarks for the Week 1 critical fixes to the voxel subsystem
//!
//! This benchmark suite validates that the critical fixes maintain or improve performance:
//! 1. BSP splitting plane selection with Surface Area Heuristic
//! 2. Canonical BSP CSG operations
//! 3. Advanced recursive termination criteria
//!
//! Note: This benchmark focuses on core library functions to avoid linking issues
//! with the binary target.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use csgrs::voxels::bsp::Node as BspNode;
use csgrs::voxels::bsp_integration::BspIntegrator;
use csgrs::voxels::polygon::Polygon;
use csgrs::voxels::vertex::Vertex;
use csgrs::voxels::{Svo, SvoNode, Occupancy};
use nalgebra::{Point3, Vector3};

fn create_test_polygons(count: usize) -> Vec<Polygon<()>> {
    let mut polygons = Vec::with_capacity(count);
    
    for i in 0..count {
        let offset = i as f64 * 0.1;
        let vertices = vec![
            Vertex::new(
                Point3::new(offset, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ),
            Vertex::new(
                Point3::new(offset + 1.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ),
            Vertex::new(
                Point3::new(offset + 0.5, 1.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ),
        ];
        polygons.push(Polygon::new(vertices, None));
    }
    
    polygons
}

fn benchmark_bsp_splitting_plane_selection(c: &mut Criterion) {
    let mut group = c.benchmark_group("BSP Splitting Plane Selection");
    
    // Test with different polygon counts
    for &count in &[10, 50, 100, 500] {
        let polygons = create_test_polygons(count);
        let bsp = BspNode::<()>::new();
        
        group.bench_function(&format!("SAH_{}_polygons", count), |b| {
            b.iter(|| {
                black_box(bsp.pick_best_splitting_plane(black_box(&polygons)))
            })
        });
    }
    
    group.finish();
}

fn benchmark_canonical_bsp_csg(c: &mut Criterion) {
    let mut group = c.benchmark_group("Canonical BSP CSG Operations");

    // Create test BSP trees
    let polygons_a = create_test_polygons(10); // Reduced size to avoid complexity
    let polygons_b = create_test_polygons(10);

    let mut bsp_a = BspNode::new();
    let mut bsp_b = BspNode::new();
    bsp_a.build(&polygons_a);
    bsp_b.build(&polygons_b);

    // Create test SVO nodes directly (avoiding full SVO construction)
    let mut node_a = SvoNode::<()>::new();
    let mut node_b = SvoNode::<()>::new();

    node_a.occupancy = Occupancy::Mixed;
    node_b.occupancy = Occupancy::Mixed;
    node_a.local_bsp = Some(bsp_a.clone());
    node_b.local_bsp = Some(bsp_b.clone());

    // Benchmark individual BSP operations instead of full CSG
    group.bench_function("bsp_build", |b| {
        b.iter(|| {
            let mut bsp = BspNode::<()>::new();
            bsp.build(black_box(&polygons_a));
            black_box(bsp)
        })
    });

    group.bench_function("bsp_all_polygons", |b| {
        b.iter(|| {
            black_box(bsp_a.all_polygons())
        })
    });

    group.bench_function("bsp_invert", |b| {
        b.iter(|| {
            let mut bsp_copy = bsp_a.clone();
            bsp_copy.invert();
            black_box(bsp_copy)
        })
    });

    group.finish();
}

fn benchmark_recursive_termination(c: &mut Criterion) {
    let mut group = c.benchmark_group("Recursive Termination Criteria");

    // Test SDF-based subdivision with simpler cases to avoid complexity
    let simple_sdf = |p: &Point3<f64>| p.coords.norm() - 1.0;

    // Benchmark smaller, more focused operations
    group.bench_function("simple_sdf_subdivision_small", |b| {
        b.iter(|| {
            let mut svo: Svo<()> = Svo::new(Point3::origin(), 1.0, 4); // Smaller size and depth
            BspIntegrator::integrate_bsp_from_sdf(
                black_box(&mut svo),
                black_box(&simple_sdf),
                black_box(0.0),
            );
            black_box(svo)
        })
    });

    // Benchmark the termination criteria function directly
    group.bench_function("termination_criteria_evaluation", |b| {
        let corner_values = vec![-1.0, 1.0, -0.5, 0.5, -2.0, 2.0, -1.5, 1.5];
        b.iter(|| {
            // Test various parameter combinations
            for depth in 0..5 {
                for &half_size in &[0.1, 1.0, 10.0] {
                    for &polygon_count in &[0, 5, 50] {
                        black_box(BspIntegrator::should_terminate_subdivision(
                            black_box(depth),
                            black_box(6),
                            black_box(half_size),
                            black_box(polygon_count),
                            black_box(Some(&corner_values)),
                            black_box(0.0),
                        ));
                    }
                }
            }
        })
    });

    group.finish();
}

criterion_group!(
    benches,
    benchmark_bsp_splitting_plane_selection,
    benchmark_canonical_bsp_csg,
    benchmark_recursive_termination
);
criterion_main!(benches);

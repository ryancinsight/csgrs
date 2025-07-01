//! Spatial Showcase: Intelligent Automated Structure Selection
//!
//! This showcase demonstrates the complete spatial data structures ecosystem with
//! intelligent automation that selects optimal structures based on data characteristics
//! and query patterns, eliminating the need for manual spatial data structure expertise.
//!
//! Run with: cargo run --example spatial_showcase

use csgrs::spatial::{
    SpatialStructureFactory, SpatialConfig, QueryType, SpatialStructureType,
    SpatialIndex, SpatialStructureSelector, traits::Aabb
};
use csgrs::geometry::{Polygon, Vertex};
use nalgebra::{Point3, Vector3};
use std::time::Instant;
use std::collections::HashMap;

/// Test scenario with expected optimal structure
#[derive(Debug, Clone)]
struct TestScenario {
    name: String,
    description: String,
    query_type: QueryType,
    expected_structure: SpatialStructureType,
    dataset_generator: fn(usize) -> Vec<Polygon<i32>>,
    dataset_size: usize,
    performance_metric: String,
}

/// Performance measurement results
#[derive(Debug, Clone)]
struct PerformanceResult {
    structure_name: String,
    construction_time_ms: f64,
    query_time_us: f64,
    memory_usage_kb: f64,
    polygon_count: usize,
    node_count: usize,
    performance_score: f64, // Higher is better
}

fn main() {
    println!("🚀 SPATIAL SHOWCASE: INTELLIGENT AUTOMATED STRUCTURE SELECTION");
    println!("{}", "=".repeat(80));
    println!();
    
    // Define test scenarios with expected optimal structures
    let scenarios = create_test_scenarios();
    
    // Phase 1: Demonstrate intelligent structure selection
    demonstrate_intelligent_selection(&scenarios);
    
    // Phase 2: Validate performance advantages
    validate_performance_advantages(&scenarios);
    
    // Phase 3: Show zero-configuration production usage
    demonstrate_production_usage(&scenarios);
    
    // Phase 4: CSG mesh integration demonstration
    demonstrate_csg_integration();
    
    // Phase 5: Multi-structure operation validation
    validate_cross_structure_operations();
    
    // Phase 6: Educational summary and recommendations
    provide_educational_summary();
    
    println!("\n🎯 SHOWCASE COMPLETE");
    println!("✅ Intelligent automation eliminates spatial data structure complexity");
    println!("✅ Developers achieve optimal performance through automated selection");
    println!("✅ Production-ready ecosystem with comprehensive error handling");
    println!("✅ Seamless integration with CSG mesh operations");
}

/// Create comprehensive test scenarios covering all spatial structures
fn create_test_scenarios() -> Vec<TestScenario> {
    vec![
        TestScenario {
            name: "CAD Boolean Operations".to_string(),
            description: "Mechanical parts requiring union/intersection operations".to_string(),
            query_type: QueryType::BooleanOperations,
            expected_structure: SpatialStructureType::Bsp,
            dataset_generator: create_cad_dataset,
            dataset_size: 30,
            performance_metric: "Boolean operation speed".to_string(),
        },
        
        TestScenario {
            name: "Game Engine Queries".to_string(),
            description: "Sparse objects requiring nearest neighbor searches".to_string(),
            query_type: QueryType::PointLocation,
            expected_structure: SpatialStructureType::Kdtree,
            dataset_generator: create_sparse_dataset,
            dataset_size: 40,
            performance_metric: "Nearest neighbor query speed".to_string(),
        },
        
        TestScenario {
            name: "3D Volume Rendering".to_string(),
            description: "Voxel data requiring frustum culling and LOD".to_string(),
            query_type: QueryType::VolumeQuery,
            expected_structure: SpatialStructureType::Octree,
            dataset_generator: create_voxel_dataset,
            dataset_size: 50,
            performance_metric: "Volume query and culling speed".to_string(),
        },
        
        TestScenario {
            name: "GIS Range Queries".to_string(),
            description: "Geographic data requiring spatial database operations".to_string(),
            query_type: QueryType::RangeQuery,
            expected_structure: SpatialStructureType::Rtree,
            dataset_generator: create_geographic_dataset,
            dataset_size: 80,
            performance_metric: "Range query speed".to_string(),
        },
        
        TestScenario {
            name: "Ray Tracing Scene".to_string(),
            description: "3D models requiring ray-object intersection".to_string(),
            query_type: QueryType::RayTracing,
            expected_structure: SpatialStructureType::Bvh,
            dataset_generator: create_rendering_dataset,
            dataset_size: 60,
            performance_metric: "Ray intersection speed".to_string(),
        },
        
        TestScenario {
            name: "Collision Detection".to_string(),
            description: "Dynamic objects requiring real-time collision queries".to_string(),
            query_type: QueryType::CollisionDetection,
            expected_structure: SpatialStructureType::Bvh,
            dataset_generator: create_dynamic_dataset,
            dataset_size: 70,
            performance_metric: "Collision detection speed".to_string(),
        },
    ]
}

/// CAD dataset with structured mechanical parts
fn create_cad_dataset(count: usize) -> Vec<Polygon<i32>> {
    let mut parts = Vec::new();
    for i in 0..count {
        let x = (i % 6) as f64 * 5.0;
        let y = (i / 6) as f64 * 5.0;
        
        // Create complex mechanical part shapes
        let vertices = if i % 3 == 0 {
            // Hexagonal parts
            vec![
                Vertex::new(Point3::new(x + 1.0, y, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 2.0, y + 0.5, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 2.0, y + 1.5, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 1.0, y + 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x, y + 1.5, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x, y + 0.5, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            ]
        } else {
            // Rectangular parts
            vec![
                Vertex::new(Point3::new(x, y, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 3.0, y, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 3.0, y + 1.5, 0.0), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x, y + 1.5, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            ]
        };
        
        parts.push(Polygon::new(vertices, Some(i as i32)));
    }
    parts
}

/// Sparse dataset for game engine scenarios
fn create_sparse_dataset(count: usize) -> Vec<Polygon<i32>> {
    let mut objects = Vec::new();
    for i in 0..count {
        // Pseudo-random sparse distribution
        let x = ((i as f64 * 17.0) % 200.0) - 100.0;
        let y = ((i as f64 * 23.0) % 200.0) - 100.0;
        let z = (i as f64 * 0.7) % 10.0;
        let size = 0.5 + (i as f64 * 0.3) % 2.0;
        
        let vertices = vec![
            Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + size, y, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + size/2.0, y + size, z), Vector3::new(0.0, 0.0, 1.0)),
        ];
        
        objects.push(Polygon::new(vertices, Some(i as i32)));
    }
    objects
}

/// Voxel dataset for 3D volume rendering
fn create_voxel_dataset(count: usize) -> Vec<Polygon<i32>> {
    let mut voxels = Vec::new();
    let chunk_size = 2.0;
    let grid_size = ((count as f64).cbrt() as usize).max(1);
    
    for i in 0..count {
        let x = (i % grid_size) as f64 * chunk_size;
        let y = ((i / grid_size) % grid_size) as f64 * chunk_size;
        let z = (i / (grid_size * grid_size)) as f64 * chunk_size;
        
        let vertices = vec![
            Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + chunk_size, y, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + chunk_size, y + chunk_size, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x, y + chunk_size, z), Vector3::new(0.0, 0.0, 1.0)),
        ];
        
        voxels.push(Polygon::new(vertices, Some(i as i32)));
    }
    voxels
}

/// Geographic dataset for GIS applications
fn create_geographic_dataset(count: usize) -> Vec<Polygon<i32>> {
    let mut features = Vec::new();
    let grid_size = (count as f64).sqrt() as usize;
    
    for i in 0..count {
        let x = (i % grid_size) as f64 * 10.0 + (i as f64 * 0.1) % 3.0;
        let y = (i / grid_size) as f64 * 10.0 + (i as f64 * 0.2) % 3.0;
        let width = 2.0 + (i as f64 * 0.3) % 4.0;
        let height = 2.0 + (i as f64 * 0.4) % 4.0;
        
        // Geographic features (buildings, parcels, etc.)
        let vertices = vec![
            Vertex::new(Point3::new(x, y, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + width, y, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + width, y + height, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x, y + height, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        
        features.push(Polygon::new(vertices, Some(i as i32)));
    }
    features
}

/// Rendering dataset for ray tracing
fn create_rendering_dataset(count: usize) -> Vec<Polygon<i32>> {
    let mut scene = Vec::new();
    
    for i in 0..count {
        let x = ((i as f64 * 13.0) % 50.0) - 25.0;
        let y = ((i as f64 * 17.0) % 50.0) - 25.0;
        let z = ((i as f64 * 7.0) % 20.0) - 10.0;
        
        let vertices = if i % 4 == 0 {
            // Large triangles for ray intersection
            vec![
                Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 4.0, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 2.0, y + 4.0, z), Vector3::new(0.0, 0.0, 1.0)),
            ]
        } else {
            // Detailed geometry
            vec![
                Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 2.0, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 1.0, y + 2.0, z), Vector3::new(0.0, 0.0, 1.0)),
            ]
        };
        
        scene.push(Polygon::new(vertices, Some(i as i32)));
    }
    scene
}

/// Dynamic dataset for collision detection
fn create_dynamic_dataset(count: usize) -> Vec<Polygon<i32>> {
    let mut objects = Vec::new();
    
    for i in 0..count {
        // Dynamic objects with varying sizes
        let x = ((i as f64 * 11.0) % 40.0) - 20.0;
        let y = ((i as f64 * 13.0) % 40.0) - 20.0;
        let z = ((i as f64 * 7.0) % 10.0) - 5.0;
        let size = 1.0 + (i as f64 * 0.5) % 3.0;
        
        let vertices = vec![
            Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + size, y, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + size, y + size, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x, y + size, z), Vector3::new(0.0, 0.0, 1.0)),
        ];
        
        objects.push(Polygon::new(vertices, Some(i as i32)));
    }
    objects
}

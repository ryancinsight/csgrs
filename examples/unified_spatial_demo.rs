//! Unified Spatial Data Structures Demo
//!
//! This comprehensive demo showcases the complete spatial data structures ecosystem,
//! demonstrating real-world applications, automatic structure selection, performance
//! characteristics, and integration features.
//!
//! Run with: cargo run --example unified_spatial_demo

use csgrs::spatial::{
    SpatialStructureFactory, SpatialConfig, QueryType, SpatialStructureType,
    SpatialIndex, SpatialStructureSelector, traits::Aabb
};
use csgrs::geometry::{Polygon, Vertex};
use nalgebra::{Point3, Vector3};
use std::time::Instant;
use std::collections::HashMap;

/// Performance metrics for structure comparison
#[derive(Debug, Clone)]
struct PerformanceMetrics {
    construction_time_ms: f64,
    memory_usage_bytes: usize,
    range_query_time_us: f64,
    point_query_time_us: f64,
    polygon_count: usize,
    node_count: usize,
    max_depth: usize,
}

/// Dataset characteristics for analysis
#[derive(Debug, Clone)]
struct DatasetInfo {
    name: String,
    polygon_count: usize,
    spatial_distribution: String,
    aspect_ratio: f64,
    density: String,
    recommended_structure: SpatialStructureType,
    use_case: String,
}

fn main() {
    println!("🏗️  UNIFIED SPATIAL DATA STRUCTURES ECOSYSTEM DEMO");
    println!("=" .repeat(80));
    println!();

    // Create diverse datasets for comprehensive testing
    let datasets = create_test_datasets();
    
    // Demonstrate automatic structure selection
    demonstrate_automatic_selection(&datasets);
    
    // Show real-world application scenarios
    demonstrate_real_world_scenarios(&datasets);
    
    // Performance benchmarking across all structures
    perform_comprehensive_benchmarks(&datasets);
    
    // Integration systems showcase
    demonstrate_integration_features(&datasets);
    
    // Educational decision guide
    provide_decision_guidance();
    
    println!("\n🎯 DEMO COMPLETE - Spatial ecosystem ready for production use!");
}

/// Create diverse datasets representing different real-world scenarios
fn create_test_datasets() -> Vec<(DatasetInfo, Vec<Polygon<i32>>)> {
    println!("📊 CREATING DIVERSE TEST DATASETS");
    println!("-".repeat(50));
    
    let mut datasets = Vec::new();
    
    // Dataset 1: Dense urban buildings (R-tree optimal)
    let urban_buildings = create_urban_buildings(50);
    datasets.push((
        DatasetInfo {
            name: "Urban Buildings".to_string(),
            polygon_count: urban_buildings.len(),
            spatial_distribution: "Clustered".to_string(),
            aspect_ratio: 1.2,
            density: "Dense".to_string(),
            recommended_structure: SpatialStructureType::Rtree,
            use_case: "GIS range queries, spatial databases".to_string(),
        },
        urban_buildings
    ));
    
    // Dataset 2: Sparse game objects (KD-tree optimal)
    let game_objects = create_game_objects(25);
    datasets.push((
        DatasetInfo {
            name: "Game Objects".to_string(),
            polygon_count: game_objects.len(),
            spatial_distribution: "Random".to_string(),
            aspect_ratio: 1.0,
            density: "Sparse".to_string(),
            recommended_structure: SpatialStructureType::Kdtree,
            use_case: "Nearest neighbor, collision detection".to_string(),
        },
        game_objects
    ));
    
    // Dataset 3: CAD mechanical parts (BSP optimal)
    let cad_parts = create_cad_parts(15);
    datasets.push((
        DatasetInfo {
            name: "CAD Parts".to_string(),
            polygon_count: cad_parts.len(),
            spatial_distribution: "Structured".to_string(),
            aspect_ratio: 2.5,
            density: "Medium".to_string(),
            recommended_structure: SpatialStructureType::Bsp,
            use_case: "Boolean operations, CSG modeling".to_string(),
        },
        cad_parts
    ));
    
    // Dataset 4: 3D voxel world (Octree optimal)
    let voxel_chunks = create_voxel_chunks(35);
    datasets.push((
        DatasetInfo {
            name: "Voxel World".to_string(),
            polygon_count: voxel_chunks.len(),
            spatial_distribution: "Hierarchical".to_string(),
            aspect_ratio: 1.0,
            density: "Volumetric".to_string(),
            recommended_structure: SpatialStructureType::Octree,
            use_case: "3D rendering, frustum culling".to_string(),
        },
        voxel_chunks
    ));
    
    for (info, _polygons) in &datasets {
        println!("✅ {}: {} polygons, {} distribution, {} density", 
                 info.name, info.polygon_count, info.spatial_distribution, info.density);
    }
    
    println!();
    datasets
}

/// Create urban building dataset (optimal for R-trees)
fn create_urban_buildings(count: usize) -> Vec<Polygon<i32>> {
    let mut buildings = Vec::new();
    let grid_size = (count as f64).sqrt() as usize;
    
    for i in 0..count {
        let x = (i % grid_size) as f64 * 10.0 + (i as f64 * 0.1) % 3.0;
        let y = (i / grid_size) as f64 * 10.0 + (i as f64 * 0.2) % 3.0;
        let width = 2.0 + (i as f64 * 0.3) % 4.0;
        let height = 2.0 + (i as f64 * 0.4) % 4.0;
        
        // Create rectangular building footprint
        let vertices = vec![
            Vertex::new(Point3::new(x, y, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + width, y, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + width, y + height, 0.0), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x, y + height, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        ];
        
        buildings.push(Polygon::new(vertices, Some(i as i32)));
    }
    
    buildings
}

/// Create game objects dataset (optimal for KD-trees)
fn create_game_objects(count: usize) -> Vec<Polygon<i32>> {
    let mut objects = Vec::new();
    
    for i in 0..count {
        // Pseudo-random positions across large area
        let x = ((i as f64 * 17.0) % 200.0) - 100.0;
        let y = ((i as f64 * 23.0) % 200.0) - 100.0;
        let z = (i as f64 * 0.7) % 10.0;
        let size = 0.5 + (i as f64 * 0.3) % 2.0;
        
        // Create triangular game object
        let vertices = vec![
            Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + size, y, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + size/2.0, y + size, z), Vector3::new(0.0, 0.0, 1.0)),
        ];
        
        objects.push(Polygon::new(vertices, Some(i as i32)));
    }
    
    objects
}

/// Create CAD parts dataset (optimal for BSP trees)
fn create_cad_parts(count: usize) -> Vec<Polygon<i32>> {
    let mut parts = Vec::new();
    
    for i in 0..count {
        let x = (i % 4) as f64 * 5.0;
        let y = (i / 4) as f64 * 5.0;
        let z = 0.0;
        
        // Create complex mechanical part shape
        let vertices = if i % 3 == 0 {
            // Hexagonal part
            vec![
                Vertex::new(Point3::new(x + 1.0, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 2.0, y + 0.5, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 2.0, y + 1.5, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 1.0, y + 2.0, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x, y + 1.5, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x, y + 0.5, z), Vector3::new(0.0, 0.0, 1.0)),
            ]
        } else {
            // Rectangular part
            vec![
                Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 3.0, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 3.0, y + 1.5, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x, y + 1.5, z), Vector3::new(0.0, 0.0, 1.0)),
            ]
        };
        
        parts.push(Polygon::new(vertices, Some(i as i32)));
    }
    
    parts
}

/// Create voxel chunks dataset (optimal for Octrees)
fn create_voxel_chunks(count: usize) -> Vec<Polygon<i32>> {
    let mut chunks = Vec::new();
    let chunk_size = 2.0;
    let grid_size = ((count as f64).cbrt() as usize).max(1);
    
    for i in 0..count {
        let x = (i % grid_size) as f64 * chunk_size;
        let y = ((i / grid_size) % grid_size) as f64 * chunk_size;
        let z = (i / (grid_size * grid_size)) as f64 * chunk_size;
        
        // Create cubic voxel chunk (simplified as quad)
        let vertices = vec![
            Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + chunk_size, y, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x + chunk_size, y + chunk_size, z), Vector3::new(0.0, 0.0, 1.0)),
            Vertex::new(Point3::new(x, y + chunk_size, z), Vector3::new(0.0, 0.0, 1.0)),
        ];
        
        chunks.push(Polygon::new(vertices, Some(i as i32)));
    }
    
    chunks
}

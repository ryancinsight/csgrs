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
    println!("{}", "=".repeat(80));
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
    println!("{}", "-".repeat(50));
    
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

    // Dataset 5: Ray tracing scene (BVH optimal)
    let ray_scene = create_ray_tracing_scene(40);
    datasets.push((
        DatasetInfo {
            name: "Ray Tracing Scene".to_string(),
            polygon_count: ray_scene.len(),
            spatial_distribution: "Mixed".to_string(),
            aspect_ratio: 1.5,
            density: "Moderate".to_string(),
            recommended_structure: SpatialStructureType::Bvh,
            use_case: "Ray tracing, rendering, collision detection".to_string(),
        },
        ray_scene
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

/// Create ray tracing scene dataset (optimal for BVH)
fn create_ray_tracing_scene(count: usize) -> Vec<Polygon<i32>> {
    let mut scene = Vec::new();

    for i in 0..count {
        // Create mixed geometry for ray tracing
        let x = ((i as f64 * 13.0) % 50.0) - 25.0;
        let y = ((i as f64 * 17.0) % 50.0) - 25.0;
        let z = ((i as f64 * 7.0) % 20.0) - 10.0;

        let vertices = if i % 4 == 0 {
            // Large triangles (good for ray intersection)
            vec![
                Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 4.0, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 2.0, y + 4.0, z), Vector3::new(0.0, 0.0, 1.0)),
            ]
        } else if i % 4 == 1 {
            // Quads (common in 3D models)
            vec![
                Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 2.0, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 2.0, y + 2.0, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x, y + 2.0, z), Vector3::new(0.0, 0.0, 1.0)),
            ]
        } else {
            // Small triangles (detailed geometry)
            vec![
                Vertex::new(Point3::new(x, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 1.0, y, z), Vector3::new(0.0, 0.0, 1.0)),
                Vertex::new(Point3::new(x + 0.5, y + 1.0, z), Vector3::new(0.0, 0.0, 1.0)),
            ]
        };

        scene.push(Polygon::new(vertices, Some(i as i32)));
    }

    scene
}

/// Demonstrate automatic structure selection system
fn demonstrate_automatic_selection(datasets: &[(DatasetInfo, Vec<Polygon<i32>>)]) {
    println!("🤖 AUTOMATIC STRUCTURE SELECTION SYSTEM");
    println!("{}", "-".repeat(50));

    for (info, polygons) in datasets {
        println!("\n📋 Dataset: {}", info.name);
        println!("   Polygons: {}, Distribution: {}, Density: {}",
                 info.polygon_count, info.spatial_distribution, info.density);

        // Analyze dataset characteristics
        let characteristics = SpatialStructureSelector::analyze_dataset(polygons);
        println!("   Analysis: aspect_ratio={:.2}, polygon_count={}, distribution={:?}",
                 characteristics.aspect_ratio, characteristics.polygon_count, characteristics.spatial_distribution);

        // Test different query types
        let query_types = vec![
            QueryType::RangeQuery,
            QueryType::PointLocation,
            QueryType::BooleanOperations,
            QueryType::VolumeQuery,
            QueryType::NearestNeighbor,
            QueryType::RayTracing,
            QueryType::CollisionDetection,
        ];

        for query_type in query_types {
            let recommended = SpatialStructureSelector::recommend_structure(&characteristics, query_type);
            let structure_name = match recommended {
                SpatialStructureType::Bsp => "BSP Tree",
                SpatialStructureType::Bvh => "BVH",
                SpatialStructureType::Kdtree => "KD-tree",
                SpatialStructureType::Octree => "Octree",
                SpatialStructureType::Rtree => "R-tree",
                SpatialStructureType::Hybrid => "Hybrid",
            };

            println!("   {:?} → {}", query_type, structure_name);
        }

        // Validate expected recommendation for primary use case
        let primary_query = match info.recommended_structure {
            SpatialStructureType::Rtree => QueryType::RangeQuery,
            SpatialStructureType::Kdtree => QueryType::PointLocation,
            SpatialStructureType::Bsp => QueryType::BooleanOperations,
            SpatialStructureType::Octree => QueryType::VolumeQuery,
            SpatialStructureType::Bvh => QueryType::RayTracing,
            _ => QueryType::RangeQuery,
        };

        let actual_recommendation = SpatialStructureSelector::recommend_structure(&characteristics, primary_query);
        let matches_expected = actual_recommendation == info.recommended_structure;

        println!("   ✅ Expected: {:?}, Got: {:?} {}",
                 info.recommended_structure, actual_recommendation,
                 if matches_expected { "✓" } else { "⚠️" });
    }
}

/// Demonstrate real-world application scenarios
fn demonstrate_real_world_scenarios(datasets: &[(DatasetInfo, Vec<Polygon<i32>>)]) {
    println!("\n🌍 REAL-WORLD APPLICATION SCENARIOS");
    println!("{}", "-".repeat(50));

    for (info, polygons) in datasets {
        println!("\n🎯 Scenario: {}", info.use_case);
        println!("   Dataset: {} ({} polygons)", info.name, info.polygon_count);

        // Create optimal structure for this scenario
        let structure = match info.recommended_structure {
            SpatialStructureType::Rtree => {
                println!("   🏗️  Creating R-tree for spatial database operations...");
                SpatialStructureFactory::create_rtree(polygons)
            },
            SpatialStructureType::Kdtree => {
                println!("   🏗️  Creating KD-tree for game engine queries...");
                SpatialStructureFactory::create_kdtree(polygons)
            },
            SpatialStructureType::Bsp => {
                println!("   🏗️  Creating BSP tree for CAD boolean operations...");
                SpatialStructureFactory::create_bsp(polygons)
            },
            SpatialStructureType::Octree => {
                println!("   🏗️  Creating Octree for 3D visualization...");
                SpatialStructureFactory::create_octree(polygons)
            },
            SpatialStructureType::Bvh => {
                println!("   🏗️  Creating BVH for ray tracing and collision detection...");
                SpatialStructureFactory::create_bvh(polygons)
            },
            _ => SpatialStructureFactory::create_optimal(polygons, QueryType::RangeQuery),
        };

        // Demonstrate typical operations for this scenario
        match info.recommended_structure {
            SpatialStructureType::Rtree => {
                // Spatial database: range queries
                let query_bounds = Aabb::new(
                    Point3::new(0.0, 0.0, -1.0),
                    Point3::new(20.0, 20.0, 1.0)
                );
                let results = structure.query_range(&query_bounds);
                println!("   📍 Range query (20x20 area): Found {} buildings", results.len());

                let larger_bounds = Aabb::new(
                    Point3::new(0.0, 0.0, -1.0),
                    Point3::new(50.0, 50.0, 1.0)
                );
                let larger_results = structure.query_range(&larger_bounds);
                println!("   📍 Larger range query (50x50 area): Found {} buildings", larger_results.len());
            },
            SpatialStructureType::Kdtree => {
                // Game engine: nearest neighbor
                let player_pos = Point3::new(0.0, 0.0, 5.0);
                if let Some(_nearest) = structure.nearest_neighbor(&player_pos) {
                    println!("   🎮 Found nearest enemy to player at ({:.1}, {:.1}, {:.1})",
                             player_pos.x, player_pos.y, player_pos.z);
                }

                // Collision detection area
                let collision_bounds = Aabb::new(
                    Point3::new(-10.0, -10.0, 0.0),
                    Point3::new(10.0, 10.0, 10.0)
                );
                let collision_objects = structure.query_range(&collision_bounds);
                println!("   💥 Collision detection: {} objects in range", collision_objects.len());
            },
            SpatialStructureType::Bsp => {
                // CAD: boolean operations simulation
                let stats = structure.statistics();
                println!("   🔧 CAD model loaded: {} parts in {} nodes (depth: {})",
                         stats.polygon_count, stats.node_count, stats.max_depth);

                // Simulate intersection query for boolean operations
                let work_area = Aabb::new(
                    Point3::new(0.0, 0.0, -0.5),
                    Point3::new(15.0, 10.0, 0.5)
                );
                let intersecting_parts = structure.query_range(&work_area);
                println!("   ⚙️  Boolean operation: {} parts intersect work area", intersecting_parts.len());
            },
            SpatialStructureType::Octree => {
                // 3D visualization: frustum culling
                let camera_frustum = Aabb::new(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(8.0, 8.0, 8.0)
                );
                let visible_chunks = structure.query_range(&camera_frustum);
                println!("   👁️  Frustum culling: {} chunks visible", visible_chunks.len());

                let stats = structure.statistics();
                println!("   🎮 3D world: {} chunks in octree (depth: {})",
                         stats.polygon_count, stats.max_depth);
            },
            SpatialStructureType::Bvh => {
                // Ray tracing: cast rays through scene
                use csgrs::spatial::traits::Ray;
                let ray = Ray {
                    origin: Point3::new(-30.0, 0.0, 0.0),
                    direction: Vector3::new(1.0, 0.0, 0.0),
                };
                let intersections = structure.ray_intersections(&ray);
                println!("   🌟 Ray casting: {} intersections found", intersections.len());

                // Collision detection simulation
                let collision_bounds = Aabb::new(
                    Point3::new(-5.0, -5.0, -5.0),
                    Point3::new(5.0, 5.0, 5.0)
                );
                let collision_objects = structure.query_range(&collision_bounds);
                println!("   💥 Collision detection: {} objects in collision zone", collision_objects.len());

                let stats = structure.statistics();
                println!("   🎬 Ray tracing scene: {} objects in BVH (depth: {})",
                         stats.polygon_count, stats.max_depth);
            },
            _ => {}
        }

        let stats = structure.statistics();
        println!("   📊 Structure stats: {} nodes, {:.1} KB memory",
                 stats.node_count, stats.memory_usage_bytes as f64 / 1024.0);
    }
}

/// Perform comprehensive performance benchmarks
fn perform_comprehensive_benchmarks(datasets: &[(DatasetInfo, Vec<Polygon<i32>>)]) {
    println!("\n⚡ COMPREHENSIVE PERFORMANCE BENCHMARKS");
    println!("{}", "-".repeat(50));

    for (info, polygons) in datasets {
        println!("\n📊 Benchmarking: {} ({} polygons)", info.name, info.polygon_count);

        let mut results = HashMap::new();

        // Benchmark all structure types
        let structure_types = vec![
            ("BSP", SpatialStructureType::Bsp),
            ("KD-tree", SpatialStructureType::Kdtree),
            ("Octree", SpatialStructureType::Octree),
            ("R-tree", SpatialStructureType::Rtree),
            ("BVH", SpatialStructureType::Bvh),
        ];

        for (name, structure_type) in structure_types {
            // Measure construction time
            let start = Instant::now();
            let structure = SpatialStructureFactory::create_structure_with_config(
                polygons,
                structure_type,
                &SpatialConfig::default()
            );
            let construction_time = start.elapsed().as_secs_f64() * 1000.0;

            let stats = structure.statistics();

            // Measure range query performance
            let query_bounds = Aabb::new(
                Point3::new(0.0, 0.0, -1.0),
                Point3::new(10.0, 10.0, 1.0)
            );

            let start = Instant::now();
            let _results = structure.query_range(&query_bounds);
            let range_query_time = start.elapsed().as_micros() as f64;

            // Measure point query performance
            let query_point = Point3::new(5.0, 5.0, 0.0);
            let start = Instant::now();
            let _contains = structure.contains_point(&query_point);
            let point_query_time = start.elapsed().as_micros() as f64;

            let metrics = PerformanceMetrics {
                construction_time_ms: construction_time,
                memory_usage_bytes: stats.memory_usage_bytes,
                range_query_time_us: range_query_time,
                point_query_time_us: point_query_time,
                polygon_count: stats.polygon_count,
                node_count: stats.node_count,
                max_depth: stats.max_depth,
            };

            results.insert(name, metrics);
        }

        // Display results table
        println!("   Structure    | Construct(ms) | Memory(KB) | Range(μs) | Point(μs) | Nodes | Depth");
        println!("   -------------|---------------|------------|-----------|-----------|-------|------");

        for (name, metrics) in &results {
            println!("   {:12} | {:13.2} | {:10.1} | {:9.1} | {:9.1} | {:5} | {:5}",
                     name,
                     metrics.construction_time_ms,
                     metrics.memory_usage_bytes as f64 / 1024.0,
                     metrics.range_query_time_us,
                     metrics.point_query_time_us,
                     metrics.node_count,
                     metrics.max_depth);
        }

        // Identify best performer for each metric
        let best_construction = results.iter().min_by(|a, b|
            a.1.construction_time_ms.partial_cmp(&b.1.construction_time_ms).unwrap()).unwrap();
        let best_memory = results.iter().min_by(|a, b|
            a.1.memory_usage_bytes.cmp(&b.1.memory_usage_bytes)).unwrap();
        let best_range = results.iter().min_by(|a, b|
            a.1.range_query_time_us.partial_cmp(&b.1.range_query_time_us).unwrap()).unwrap();
        let best_point = results.iter().min_by(|a, b|
            a.1.point_query_time_us.partial_cmp(&b.1.point_query_time_us).unwrap()).unwrap();

        println!("   🏆 Best performers:");
        println!("      Construction: {} ({:.2}ms)", best_construction.0, best_construction.1.construction_time_ms);
        println!("      Memory: {} ({:.1}KB)", best_memory.0, best_memory.1.memory_usage_bytes as f64 / 1024.0);
        println!("      Range Query: {} ({:.1}μs)", best_range.0, best_range.1.range_query_time_us);
        println!("      Point Query: {} ({:.1}μs)", best_point.0, best_point.1.point_query_time_us);

        // Validate expected best performer
        let expected_best = match info.recommended_structure {
            SpatialStructureType::Rtree => "R-tree",
            SpatialStructureType::Kdtree => "KD-tree",
            SpatialStructureType::Bsp => "BSP",
            SpatialStructureType::Octree => "Octree",
            _ => "Unknown",
        };

        println!("   📈 Expected optimal: {} for {}", expected_best, info.use_case);
    }
}

/// Demonstrate integration systems showcase
fn demonstrate_integration_features(datasets: &[(DatasetInfo, Vec<Polygon<i32>>)]) {
    println!("\n🔗 INTEGRATION SYSTEMS SHOWCASE");
    println!("{}", "-".repeat(50));

    // Take first dataset for integration demo
    if let Some((_info, polygons)) = datasets.first() {
        println!("\n🛠️  Unified Configuration System");

        // Demonstrate predefined configurations
        let configs = vec![
            ("Boolean Operations", SpatialConfig::for_boolean_operations()),
            ("Point Queries", SpatialConfig::for_point_queries()),
            ("Volume Queries", SpatialConfig::for_volume_queries()),
            ("Range Queries", SpatialConfig::for_range_queries()),
            ("Large Datasets", SpatialConfig::for_large_datasets()),
            ("Memory Efficiency", SpatialConfig::for_memory_efficiency()),
        ];

        for (name, config) in configs {
            match config.validate() {
                Ok(()) => println!("   ✅ {}: Configuration valid", name),
                Err(e) => println!("   ❌ {}: Configuration error - {}", name, e),
            }
        }

        println!("\n🚨 Error Handling System");

        // Demonstrate error handling
        let empty_polygons: Vec<Polygon<i32>> = vec![];

        // Test error handling for each structure type
        println!("   Testing error handling with empty datasets:");

        match SpatialStructureFactory::try_create_bsp(&empty_polygons) {
            Ok(_) => println!("   ⚠️  BSP: Unexpected success with empty data"),
            Err(e) => println!("   ✅ BSP: Proper error handling - {}", e),
        }

        match SpatialStructureFactory::try_create_kdtree(&empty_polygons) {
            Ok(_) => println!("   ⚠️  KD-tree: Unexpected success with empty data"),
            Err(e) => println!("   ✅ KD-tree: Proper error handling - {}", e),
        }

        match SpatialStructureFactory::try_create_octree(&empty_polygons) {
            Ok(_) => println!("   ⚠️  Octree: Unexpected success with empty data"),
            Err(e) => println!("   ✅ Octree: Proper error handling - {}", e),
        }

        match SpatialStructureFactory::try_create_rtree(&empty_polygons) {
            Ok(_) => println!("   ⚠️  R-tree: Unexpected success with empty data"),
            Err(e) => println!("   ✅ R-tree: Proper error handling - {}", e),
        }

        match SpatialStructureFactory::try_create_bvh(&empty_polygons) {
            Ok(_) => println!("   ⚠️  BVH: Unexpected success with empty data"),
            Err(e) => println!("   ✅ BVH: Proper error handling - {}", e),
        }

        // Test successful creation with error handling
        match SpatialStructureFactory::try_create_rtree(polygons) {
            Ok(structure) => {
                let stats = structure.statistics();
                println!("   ✅ Success: R-tree created with {} polygons", stats.polygon_count);
            },
            Err(e) => println!("   ❌ Unexpected error: {}", e),
        }

        println!("\n🧮 Shared Geometric Utilities");

        // Demonstrate shared utilities usage
        if let Some(polygon) = polygons.first() {
            let center = csgrs::spatial::utils::polygon_center(polygon);
            println!("   📍 Polygon center: ({:.2}, {:.2}, {:.2})", center.x, center.y, center.z);

            if let Some(bounds) = csgrs::spatial::utils::polygon_bounds(polygon) {
                let volume = csgrs::spatial::utils::bounds_volume(&bounds);
                println!("   📦 Polygon bounds volume: {:.3}", volume);
            }

            let p1 = Point3::new(0.0, 0.0, 0.0);
            let p2 = Point3::new(3.0, 4.0, 0.0);
            let distance = csgrs::spatial::utils::distance(&p1, &p2);
            println!("   📏 Distance calculation: {:.2}", distance);
        }

        println!("\n🎭 Polymorphic Collections");

        // Demonstrate polymorphic usage
        let structures: Vec<Box<dyn SpatialIndex<i32>>> = vec![
            SpatialStructureFactory::create_bsp(polygons),
            SpatialStructureFactory::create_kdtree(polygons),
            SpatialStructureFactory::create_octree(polygons),
            SpatialStructureFactory::create_rtree(polygons),
            SpatialStructureFactory::create_bvh(polygons),
        ];

        let structure_names = vec!["BSP", "KD-tree", "Octree", "R-tree", "BVH"];

        for (i, structure) in structures.iter().enumerate() {
            let stats = structure.statistics();
            let icon = match i {
                0 => "🌳", // BSP
                1 => "🔍", // KD-tree
                2 => "🧊", // Octree
                3 => "📊", // R-tree
                4 => "🎬", // BVH
                _ => "❓",
            };
            println!("   {} {}: {} polygons, {} nodes, {:.1}KB",
                     icon,
                     structure_names[i],
                     stats.polygon_count,
                     stats.node_count,
                     stats.memory_usage_bytes as f64 / 1024.0);
        }

        // Test uniform interface
        let query_bounds = Aabb::new(
            Point3::new(0.0, 0.0, -1.0),
            Point3::new(5.0, 5.0, 1.0)
        );

        println!("   🔍 Uniform query interface (5x5 area):");
        for (i, structure) in structures.iter().enumerate() {
            let results = structure.query_range(&query_bounds);
            println!("      {}: {} results", structure_names[i], results.len());
        }
    }
}

/// Provide educational decision guidance
fn provide_decision_guidance() {
    println!("\n📚 EDUCATIONAL DECISION GUIDE");
    println!("{}", "-".repeat(50));

    println!("\n🎯 WHEN TO USE EACH STRUCTURE:");

    println!("\n🌳 BSP Trees (Binary Space Partitioning):");
    println!("   ✅ BEST FOR:");
    println!("      • Boolean operations (union, intersection, difference)");
    println!("      • Constructive Solid Geometry (CSG)");
    println!("      • CAD applications with complex part interactions");
    println!("      • Rendering with depth sorting");
    println!("   ⚠️  AVOID WHEN:");
    println!("      • Primarily doing range queries");
    println!("      • Need frequent dynamic updates");
    println!("      • Working with point clouds");

    println!("\n🔍 KD-Trees (K-Dimensional Trees):");
    println!("   ✅ BEST FOR:");
    println!("      • Nearest neighbor searches");
    println!("      • Point location queries");
    println!("      • Game engine collision detection");
    println!("      • Sparse spatial data");
    println!("   ⚠️  AVOID WHEN:");
    println!("      • Data has high aspect ratios");
    println!("      • Primarily doing volume queries");
    println!("      • Need frequent range queries over large areas");

    println!("\n🧊 Octrees (Hierarchical Space Subdivision):");
    println!("   ✅ BEST FOR:");
    println!("      • 3D volume queries and frustum culling");
    println!("      • Voxel-based applications");
    println!("      • Level-of-detail (LOD) systems");
    println!("      • 3D rendering and visualization");
    println!("   ⚠️  AVOID WHEN:");
    println!("      • Working with 2D data");
    println!("      • Need precise geometric operations");
    println!("      • Data is primarily linear/planar");

    println!("\n📊 R-Trees (Rectangle Trees):");
    println!("   ✅ BEST FOR:");
    println!("      • Spatial databases and GIS applications");
    println!("      • Range queries over geographic data");
    println!("      • Dynamic spatial indexing");
    println!("      • Bounding box intersection queries");
    println!("   ⚠️  AVOID WHEN:");
    println!("      • Need exact geometric predicates");
    println!("      • Primarily doing point queries");
    println!("      • Working with very sparse data");

    println!("\n🎬 BVH (Bounding Volume Hierarchy):");
    println!("   ✅ BEST FOR:");
    println!("      • Ray tracing and rendering applications");
    println!("      • Real-time collision detection");
    println!("      • Game engines with dynamic objects");
    println!("      • Computer graphics and visualization");
    println!("   ⚠️  AVOID WHEN:");
    println!("      • Primarily doing range queries over static data");
    println!("      • Need spatial subdivision rather than object partitioning");
    println!("      • Working with very small datasets (<10 objects)");

    println!("\n🚀 PERFORMANCE OPTIMIZATION TIPS:");
    println!("   📈 For Speed:");
    println!("      • Use bulk loading for large datasets");
    println!("      • Choose structure based on query patterns");
    println!("      • Consider hybrid approaches for complex workflows");

    println!("   💾 For Memory:");
    println!("      • Use memory-efficient configurations");
    println!("      • Consider lower branching factors");
    println!("      • Share data between structures when possible");

    println!("   🔄 For Dynamic Data:");
    println!("      • R-trees excel at insertions/deletions");
    println!("      • Consider rebuilding periodically for other structures");
    println!("      • Use appropriate rebalancing strategies");

    println!("\n🎛️  CONFIGURATION RECOMMENDATIONS:");
    println!("   🏗️  Construction:");
    println!("      • SpatialConfig::for_boolean_operations() → BSP trees");
    println!("      • SpatialConfig::for_point_queries() → KD-trees");
    println!("      • SpatialConfig::for_volume_queries() → Octrees");
    println!("      • SpatialConfig::for_range_queries() → R-trees");
    println!("      • SpatialConfig::for_ray_tracing() → BVH");

    println!("   📊 Dataset Size:");
    println!("      • Small (<100 polygons): Any structure works");
    println!("      • Medium (100-1000): Choose based on query type");
    println!("      • Large (>1000): Use bulk loading and optimal structure");

    println!("   🎯 Query Patterns:");
    println!("      • 80%+ range queries → R-tree");
    println!("      • 80%+ point queries → KD-tree");
    println!("      • 80%+ boolean ops → BSP tree");
    println!("      • 80%+ volume queries → Octree");
    println!("      • 80%+ ray tracing → BVH");
    println!("      • Mixed patterns → Use SpatialStructureSelector");

    println!("\n💡 DECISION FLOWCHART:");
    println!("   1. Analyze your data characteristics");
    println!("   2. Identify primary query patterns");
    println!("   3. Consider performance requirements");
    println!("   4. Use SpatialStructureSelector::recommend_structure()");
    println!("   5. Benchmark with your actual data");
    println!("   6. Fine-tune configuration based on results");

    println!("\n🔧 INTEGRATION BEST PRACTICES:");
    println!("   • Always use try_create_*() methods in production");
    println!("   • Validate configurations before use");
    println!("   • Leverage shared utilities for consistency");
    println!("   • Use polymorphic collections for flexibility");
    println!("   • Monitor memory usage and performance metrics");
}

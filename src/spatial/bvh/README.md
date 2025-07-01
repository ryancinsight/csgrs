# BVH (Bounding Volume Hierarchy) Spatial Data Structure

BVHs are binary tree data structures specifically optimized for ray tracing, collision detection, and dynamic object management. They are particularly well-suited for:

- **Ray tracing and rendering** with optimal ray traversal performance
- **Real-time collision detection** with dynamic object updates
- **Game engines** requiring fast spatial queries with moving objects
- **Computer graphics** applications needing efficient ray-primitive intersection

## Algorithmic Background

BVHs organize spatial data by recursively partitioning objects into two groups, each enclosed by a bounding volume. Unlike spatial subdivision structures (Octrees, KD-trees), BVHs partition objects rather than space, making them ideal for dynamic scenes and ray tracing.

### Key Properties

- **Binary tree structure** - Each internal node has exactly two children for optimal ray traversal
- **Object partitioning** - Splits objects rather than space, better for dynamic scenes
- **Bounding volume hierarchy** - Each node contains the bounding volume of all its children
- **Ray tracing optimized** - Designed specifically for efficient ray-object intersection

### Time Complexity

- **Ray Traversal**: O(log n) average case for well-balanced trees
- **Construction**: O(n log n) with SAH, O(n log n) with median split
- **Dynamic Updates**: O(log n) for refit operations, O(n log n) for rebuilds
- **Collision Detection**: O(log n + k) where k is the number of intersections

## Basic Usage

### Creating a BVH

```rust
use csgrs::spatial::{bvh::Node, SpatialIndex};
use csgrs::geometry::{Polygon, Vertex};
use nalgebra::{Point3, Vector3};

// Create test polygons
let vertices = vec![
    Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
];
let polygon: Polygon<i32> = Polygon::new(vertices, None);
let polygons = vec![polygon];

// Build BVH from polygons
let bvh = Node::from_polygons(&polygons);
println!("Created BVH with {} polygons", bvh.all_polygons().len());
```

### Ray Tracing Operations

```rust
use csgrs::spatial::{bvh::Node, traits::Ray, SpatialIndex};

// Create BVH for ray tracing
let bvh = Node::from_polygons(&polygons);

// Cast ray through scene
let ray = Ray {
    origin: Point3::new(-1.0, 0.5, 0.0),
    direction: Vector3::new(1.0, 0.0, 0.0),
};

// Find all intersections
let intersections = bvh.ray_intersections(&ray);
println!("Ray intersected {} objects", intersections.len());

// Find closest intersection only
let closest = bvh.ray_traversal_optimized(&ray);
if let Some(hit) = closest.first() {
    println!("Closest hit at distance: {}", hit.distance);
}
```

### Dynamic Object Management

```rust
use csgrs::spatial::bvh::Node;

// Create empty BVH for dynamic scene
let mut bvh: Node<i32> = Node::new();

// Add objects dynamically
let vertices = vec![
    Vertex::new(Point3::new(2.0, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(3.0, 2.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(2.5, 3.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
];
let polygon: Polygon<i32> = Polygon::new(vertices, Some(42));

bvh.insert_object(polygon);
println!("BVH now contains {} polygons", bvh.polygon_count());

// Update after object movement
bvh.refit_bounding_volumes();

// Remove objects by metadata
let removed = bvh.remove_object_by_metadata(&Some(42));
println!("Removal successful: {}", removed);
```

## Construction Algorithms

BVHs support different construction algorithms with varying trade-offs:

### SAH (Surface Area Heuristic)

```rust
use csgrs::spatial::bvh::{BVHConfig, ConstructionAlgorithm};

// Optimal ray tracing performance
let sah_config = BVHConfig {
    construction_algorithm: ConstructionAlgorithm::SAH,
    max_polygons_per_leaf: 4,
    sah_traversal_cost: 1.0,
    sah_intersection_cost: 1.0,
    ..Default::default()
};

let bvh = Node::from_polygons_with_config(&polygons, &sah_config);
```

### Binned SAH

```rust
// Fast approximation of SAH for large datasets
let binned_config = BVHConfig {
    construction_algorithm: ConstructionAlgorithm::BinnedSAH,
    sah_bins: 32,
    ..Default::default()
};

let bvh = Node::from_polygons_with_config(&polygons, &binned_config);
```

### Median Split

```rust
// Fastest construction for dynamic scenes
let median_config = BVHConfig {
    construction_algorithm: ConstructionAlgorithm::Median,
    enable_incremental_updates: true,
    ..Default::default()
};

let bvh = Node::from_polygons_with_config(&polygons, &median_config);
```

### Spatial SAH

```rust
// Highest quality with primitive splitting
let spatial_config = BVHConfig {
    construction_algorithm: ConstructionAlgorithm::SpatialSAH,
    enable_spatial_splits: true,
    max_polygons_per_leaf: 2,
    ..Default::default()
};

let bvh = Node::from_polygons_with_config(&polygons, &spatial_config);
```

## Predefined Configurations

```rust
use csgrs::spatial::bvh::BVHConfig;

// Optimized for ray tracing performance
let ray_tracing = BVHConfig::for_ray_tracing();

// Optimized for dynamic scenes with moving objects
let dynamic_scenes = BVHConfig::for_dynamic_scenes();

// Highest quality ray tracing (slower construction)
let high_quality = BVHConfig::for_high_quality_rays();

// Fastest construction (lower quality)
let fast_construction = BVHConfig::for_fast_construction();
```

## Performance Characteristics

### Optimal Use Cases

- **Ray tracing and rendering** applications
- **Real-time collision detection** with dynamic objects
- **Game engines** with frequently moving objects
- **Computer graphics** requiring ray-primitive intersection
- **Virtual reality** applications needing low-latency spatial queries

### Performance Tuning

- **Construction Algorithm**: SAH for quality, Median for speed, Binned SAH for balance
- **Leaf Size**: Smaller leaves (1-4 polygons) improve ray performance but increase memory
- **SAH Parameters**: Adjust traversal/intersection costs based on actual hardware performance
- **Dynamic Updates**: Use refit for small movements, rebuild for major scene changes

### Benchmarks

Typical performance characteristics (varies by scene complexity):

- **Ray Traversal**: 5-50x faster than linear search depending on scene complexity
- **Construction**: SAH ~2x slower than median split, but 20-50% better ray performance
- **Memory Usage**: ~30-50 bytes per polygon plus tree overhead
- **Dynamic Updates**: Refit operations ~100x faster than full reconstruction

## Integration with Spatial System

### Polymorphic Usage

```rust
use csgrs::spatial::{SpatialStructureFactory, QueryType, SpatialIndex};

// Factory automatically selects BVH for ray tracing
let structure = SpatialStructureFactory::create_optimal(&polygons, QueryType::RayTracing);

// Explicit BVH creation
let bvh = SpatialStructureFactory::create_bvh(&polygons);

// Use polymorphically with other spatial structures
let structures: Vec<Box<dyn SpatialIndex<i32>>> = vec![
    SpatialStructureFactory::create_bsp(&polygons),
    SpatialStructureFactory::create_kdtree(&polygons), 
    SpatialStructureFactory::create_octree(&polygons),
    SpatialStructureFactory::create_rtree(&polygons),
    SpatialStructureFactory::create_bvh(&polygons),
];
```

### Configuration Integration

```rust
use csgrs::spatial::{SpatialConfig, SpatialStructureFactory};

// Use unified spatial configuration
let config = SpatialConfig::for_ray_tracing();
let bvh = SpatialStructureFactory::create_bvh_with_config(&polygons, &config);

// Error handling
match SpatialStructureFactory::try_create_bvh(&polygons) {
    Ok(bvh) => println!("BVH created successfully"),
    Err(e) => println!("Creation failed: {}", e),
}
```

## Implementation Details

### Tree Structure

- **Binary tree**: Each internal node has exactly two children for optimal ray traversal
- **Object partitioning**: Splits objects rather than space, better for dynamic scenes
- **Bounding volumes**: Automatically maintained and updated during operations
- **Cache-friendly layout**: Optimized memory layout for traversal performance

### Construction Algorithms

1. **SAH**: Minimizes expected ray traversal cost using surface area heuristic
2. **Binned SAH**: Fast approximation using spatial bins for large datasets
3. **Median Split**: O(n log n) construction with reasonable quality
4. **Spatial SAH**: Advanced algorithm allowing primitive splitting

### Ray Traversal Optimization

- **Stack-based traversal**: Avoids recursion overhead for better performance
- **Early termination**: Stops traversal when closer intersection found
- **Optimal child ordering**: Traverses children in ray direction order
- **SIMD hints**: Optimized ray-AABB intersection with vectorization hints

## See Also

- [Spatial Module Overview](../README.md)
- [BSP Trees](../bsp/README.md) - For Boolean operations
- [KD-trees](../kdtree/README.md) - For point queries
- [Octrees](../octree/README.md) - For volume queries
- [R-trees](../rtree/README.md) - For range queries
- [Shared Utilities](../utils.rs) - Common geometric operations

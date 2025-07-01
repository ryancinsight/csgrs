# KD-Tree (K-Dimensional Tree) Implementation

## Overview

This module provides a high-performance KD-tree implementation optimized for 3D geometric operations. KD-trees excel at spatial queries involving points and axis-aligned regions, making them ideal for nearest neighbor searches, range queries, and point location operations.

## Mathematical Foundation

### Core Concept

A KD-tree recursively partitions k-dimensional space using axis-aligned hyperplanes. In our 3D implementation, the tree alternates between splitting along the X, Y, and Z axes at each level. Each node represents a region of space, with polygons classified based on their centroid positions relative to the splitting plane.

### Algorithm Complexity

- **Construction**: O(n log n) average case, O(n²) worst case
- **Nearest Neighbor**: O(log n) average case, O(n) worst case
- **Range Query**: O(√n + k) where k is the number of results
- **Space**: O(n) for n polygons

### Splitting Strategy

The implementation supports two splitting strategies:

1. **Median Split**: Splits at the median coordinate value (balanced tree)
2. **Surface Area Heuristic (SAH)**: Minimizes expected query cost based on spatial distribution

## Module Structure

```
src/spatial/kdtree/
├── mod.rs           # Module declarations and re-exports
├── README.md        # This documentation
├── core.rs          # Core Node structure and basic operations
├── construction.rs  # Tree building algorithms
└── queries.rs       # Spatial query operations
```

## Core API

### Node Structure

```rust
pub struct Node<S: Clone> {
    pub axis: Option<Axis>,           // Splitting axis (X, Y, or Z)
    pub split_value: Option<Real>,    // Splitting coordinate value
    pub left: Option<Box<Node<S>>>,   // Left subtree (coord <= split_value)
    pub right: Option<Box<Node<S>>>,  // Right subtree (coord > split_value)
    pub polygons: Vec<Polygon<S>>,    // Polygons in this node
    pub bounds: Option<Aabb>,         // Bounding box of this subtree
}
```

### Basic Operations

#### Creating KD-Trees

```rust
use csgrs::spatial::kdtree::Node;
use csgrs::geometry::Polygon;

// Create empty tree
let mut kdtree = Node::new();

// Build from polygons (automatic method selection)
let kdtree = Node::from_polygons(&polygons);

// Build with custom configuration
let config = KdTreeConfig {
    max_depth: 15,
    max_polygons_per_leaf: 5,
    use_sah: true,
    ..Default::default()
};
let kdtree = Node::from_polygons_with_config(&polygons, &config);
```

#### Tree Information

```rust
// Get tree statistics
let stats = kdtree.statistics();
println!("Nodes: {}, Depth: {}, Memory: {} bytes", 
         stats.node_count, stats.max_depth, stats.memory_usage_bytes);

// Get all polygons
let all_polygons = kdtree.all_polygons();

// Get bounding box
if let Some(bounds) = kdtree.bounding_box() {
    println!("Bounds: {:?}", bounds);
}
```

## Spatial Query Operations

### Nearest Neighbor Search

Find the polygon closest to a given point:

```rust
use nalgebra::Point3;

let query_point = Point3::new(1.0, 2.0, 3.0);

// Automatic method selection
let nearest = kdtree.nearest_neighbor_auto(&query_point);

// Explicit sequential search
let nearest = kdtree.nearest_neighbor(&query_point);

// Explicit parallel search (requires "parallel" feature)
#[cfg(feature = "parallel")]
let nearest = kdtree.nearest_neighbor_parallel(&query_point);
```

**Performance Characteristics**:
- **Average Case**: O(log n) - excellent for sparse to moderately dense datasets
- **Worst Case**: O(n) - when tree becomes unbalanced
- **Parallel Threshold**: > 1,000 polygons for automatic parallel selection

### Range Queries

Find all polygons within a bounding box:

```rust
use csgrs::spatial::traits::Aabb;

let query_bounds = Aabb::new(
    Point3::new(0.0, 0.0, 0.0),
    Point3::new(10.0, 10.0, 10.0)
);

// Automatic method selection
let results = kdtree.range_query_auto(&query_bounds);

// Explicit sequential search
let results = kdtree.range_query(&query_bounds);

// Explicit parallel search (requires "parallel" feature)
#[cfg(feature = "parallel")]
let results = kdtree.range_query_parallel(&query_bounds);
```

**Performance Characteristics**:
- **Complexity**: O(√n + k) where k is the number of results
- **Optimal For**: Axis-aligned rectangular queries
- **Parallel Threshold**: > 1,000 polygons for automatic parallel selection

### Ray Intersection

Find all polygons intersected by a ray:

```rust
use csgrs::spatial::traits::Ray;
use nalgebra::Vector3;

let ray = Ray::new(
    Point3::new(0.0, 0.0, 0.0),
    Vector3::new(1.0, 0.0, 0.0).normalize()
);

let intersections = kdtree.ray_intersections(&ray);
for intersection in intersections {
    println!("Hit at distance: {}, point: {:?}", 
             intersection.distance, intersection.point);
}
```

## Performance Optimization

### Automatic Method Selection

The KD-tree implementation automatically chooses between sequential and parallel algorithms:

```rust
// These methods automatically select the best implementation
let nearest = kdtree.nearest_neighbor_auto(&point);
let range_results = kdtree.range_query_auto(&bounds);
```

**Selection Thresholds**:
- **Parallel Construction**: > 500 polygons
- **Parallel Queries**: > 1,000 polygons
- **SAH vs Median**: SAH enabled by default for better tree quality

### Memory Optimization

- **Lazy Bounds Calculation**: Bounding boxes computed only when needed
- **Efficient Partitioning**: In-place polygon partitioning during construction
- **Memory Pooling**: Reuse of temporary vectors during tree building

### Configuration Tuning

```rust
let config = KdTreeConfig {
    max_depth: 20,                    // Prevent excessive tree depth
    min_polygons_per_leaf: 1,         // Minimum polygons before stopping
    max_polygons_per_leaf: 10,        // Maximum polygons before splitting
    use_sah: true,                    // Use Surface Area Heuristic
};
```

## Usage Examples

### Point Location Queries

```rust
// Find polygons near a specific point
let query_point = Point3::new(5.0, 5.0, 5.0);
let search_radius = 2.0;

let search_bounds = Aabb::new(
    query_point - Vector3::new(search_radius, search_radius, search_radius),
    query_point + Vector3::new(search_radius, search_radius, search_radius)
);

let nearby_polygons = kdtree.range_query(&search_bounds);
```

### Spatial Clustering Analysis

```rust
// Find clusters of polygons
let mut clusters = Vec::new();
let cluster_radius = 1.0;

for polygon in &all_polygons {
    let center = Node::polygon_center(polygon);
    let search_bounds = Aabb::new(
        center - Vector3::new(cluster_radius, cluster_radius, cluster_radius),
        center + Vector3::new(cluster_radius, cluster_radius, cluster_radius)
    );
    
    let cluster_members = kdtree.range_query(&search_bounds);
    if cluster_members.len() > 3 {
        clusters.push(cluster_members);
    }
}
```

### Adaptive Level-of-Detail

```rust
// Select polygons based on distance from viewpoint
let viewpoint = Point3::new(0.0, 0.0, 10.0);
let detail_levels = vec![1.0, 5.0, 20.0]; // Distance thresholds

for &distance_threshold in &detail_levels {
    let search_bounds = Aabb::new(
        viewpoint - Vector3::new(distance_threshold, distance_threshold, distance_threshold),
        viewpoint + Vector3::new(distance_threshold, distance_threshold, distance_threshold)
    );
    
    let visible_polygons = kdtree.range_query(&search_bounds);
    // Render polygons at appropriate detail level
}
```

## Polymorphic Usage with SpatialIndex Trait

KD-trees implement the `SpatialIndex` trait, enabling polymorphic usage alongside BSP trees and Octrees:

```rust
use csgrs::spatial::{SpatialIndex, SpatialStructureFactory, QueryType};

// Create KD-tree as trait object
let kdtree: Box<dyn SpatialIndex<i32>> = SpatialStructureFactory::create_kdtree(&polygons);

// Use polymorphically with other structures
fn analyze_structure<S: Clone + Send + Sync + std::fmt::Debug>(
    structure: &dyn SpatialIndex<S>
) -> usize {
    structure.statistics().node_count
}

let node_count = analyze_structure(kdtree.as_ref());

// Automatic optimal selection for point queries
let optimal_structure = SpatialStructureFactory::create_optimal(&polygons, QueryType::PointLocation);
// Will likely choose KD-tree for point-based operations
```

This enables writing generic spatial algorithms that work with any spatial structure type.

### Configuration Examples

```rust
use csgrs::spatial::{SpatialConfig, SpatialStructureFactory};
use csgrs::geometry::{Polygon, Vertex};
use nalgebra::{Point3, Vector3};

// Create test data
let vertices = vec![
    Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
];
let polygon: Polygon<i32> = Polygon::new(vertices, None);
let polygons = vec![polygon];

// Use predefined configuration optimized for point queries
let point_config = SpatialConfig::for_point_queries();
let kdtree = SpatialStructureFactory::create_kdtree_with_config(&polygons, &point_config);

// Customize configuration for specific needs
let mut custom_config = SpatialConfig::for_point_queries();
custom_config.max_depth = 15;
custom_config.max_polygons_per_leaf = 3;
custom_config.use_sah = true;

let custom_kdtree = SpatialStructureFactory::create_kdtree_with_config(&polygons, &custom_config);

// Both trees work with the same interface
assert_eq!(kdtree.all_polygons().len(), 1);
assert_eq!(custom_kdtree.all_polygons().len(), 1);
```

## Best Practices

### When to Use KD-Trees

**Optimal For**:
- Point location queries
- Nearest neighbor searches
- Range queries with axis-aligned bounding boxes
- Datasets with uniform to moderate spatial clustering
- Applications requiring frequent spatial queries

**Less Optimal For**:
- Boolean operations (use BSP trees instead)
- Highly clustered or sparse datasets (consider Octrees)
- Non-axis-aligned queries
- Datasets with extreme aspect ratios

### Performance Tips

1. **Tree Configuration**:
   - Use SAH for better tree quality
   - Adjust `max_polygons_per_leaf` based on query patterns
   - Limit `max_depth` to prevent excessive memory usage

2. **Query Optimization**:
   - Use automatic method selection for best performance
   - Batch similar queries when possible
   - Consider caching results for repeated queries

3. **Memory Management**:
   - Build trees once and reuse for multiple queries
   - Use parallel construction for large datasets
   - Monitor memory usage with `statistics()` method

### Integration with Other Spatial Structures

```rust
// Use KD-tree for preprocessing, BSP for Boolean operations
let kdtree = Node::from_polygons(&all_polygons);
let nearby = kdtree.range_query(&query_bounds);

// Use BSP for precise Boolean operations on filtered set
let bsp = csgrs::spatial::bsp::Node::from_polygons(&nearby);
let result = bsp.clip_polygons(&other_polygons);
```

## Validation and Debugging

```rust
// Validate tree structure
if let Err(error) = kdtree.validate() {
    eprintln!("Tree validation failed: {}", error);
}

// Optimize tree after construction
kdtree.optimize();

// Get detailed statistics
let stats = kdtree.statistics();
println!("Tree efficiency: {:.2}%", 
         (stats.polygon_count as f64 / stats.node_count as f64) * 100.0);
```

## See Also

- [`../bsp/README.md`](../bsp/README.md) - BSP tree documentation
- [`../README.md`](../README.md) - Spatial module overview
- [`../../ARCHITECTURE.md`](../../ARCHITECTURE.md) - Overall architecture
- [Wikipedia: k-d tree](https://en.wikipedia.org/wiki/K-d_tree)

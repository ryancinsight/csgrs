//! # Sparse Voxel Utility Functions
//!
//! This module provides utility functions and statistics for sparse voxel operations.

use crate::float_types::Real;

/// Memory statistics for sparse voxel octrees
#[derive(Debug, Clone)]
pub struct VoxelMemoryStats {
    /// Total number of nodes in the octree
    pub node_count: usize,
    /// Number of occupied leaf nodes
    pub occupied_leaves: usize,
    /// Total number of nodes (for compatibility)
    pub total_nodes: usize,
    /// Estimated memory usage in bytes
    pub memory_usage_bytes: usize,
    /// Compression ratio (None if no compression enabled)
    pub compression_ratio: Option<Real>,
}

impl VoxelMemoryStats {
    /// Create new memory statistics
    pub const fn new(
        node_count: usize,
        occupied_leaves: usize,
        total_nodes: usize,
        compression_ratio: Option<Real>,
    ) -> Self {
        let memory_usage_bytes =
            total_nodes * std::mem::size_of::<crate::voxels::octree::SparseVoxelNode<()>>();

        Self {
            node_count,
            occupied_leaves,
            total_nodes,
            memory_usage_bytes,
            compression_ratio,
        }
    }

    /// Calculate compression savings as a percentage
    pub fn compression_savings_percent(&self) -> Option<Real> {
        self.compression_ratio
            .map(|ratio| (1.0 - 1.0 / ratio) * 100.0)
    }
}

/// Triangle utilities for voxelization operations
pub mod triangle_utils {
    use crate::float_types::Real;
    use nalgebra::Point3;

    /// Calculate the bounding box of a triangle
    pub fn triangle_bounding_box(
        triangle: &[Point3<Real>; 3],
    ) -> crate::float_types::parry3d::bounding_volume::Aabb {
        use crate::float_types::parry3d::bounding_volume::Aabb;

        let mut min = triangle[0].coords;
        let mut max = triangle[0].coords;

        for vertex in triangle.iter().skip(1) {
            min = min.inf(&vertex.coords);
            max = max.sup(&vertex.coords);
        }

        Aabb::new(Point3::from(min), Point3::from(max))
    }

    /// Test if a point lies inside a triangle using barycentric coordinates
    pub fn point_in_triangle(point: &Point3<Real>, triangle: &[Point3<Real>; 3]) -> bool {
        // First check if point is in the same plane as the triangle
        let plane_normal = (triangle[1] - triangle[0]).cross(&(triangle[2] - triangle[0]));
        let plane_d = -plane_normal.dot(&triangle[0].coords);

        let distance_to_plane = plane_normal.dot(&point.coords) + plane_d;
        if distance_to_plane.abs() > 1e-10 {
            return false; // Point is not in the triangle plane
        }

        // Project to XY plane and use barycentric coordinates
        let p = point.coords.xy();
        let a = triangle[0].coords.xy();
        let b = triangle[1].coords.xy();
        let c = triangle[2].coords.xy();

        let area = 0.5 * ((b - a).perp(&(c - a)));
        if area.abs() < 1e-10 {
            return false; // Degenerate triangle
        }

        // Barycentric coordinates
        let bary_a = 0.5 * ((b - p).perp(&(c - p))) / area;
        let bary_b = 0.5 * ((c - p).perp(&(a - p))) / area;
        let bary_c = 1.0 - bary_a - bary_b;

        (0.0..=1.0).contains(&bary_a)
            && (0.0..=1.0).contains(&bary_b)
            && (0.0..=1.0).contains(&bary_c)
    }
}

/// Performance benchmarking utilities
pub mod benchmark_utils {
    use crate::float_types::Real;
    use crate::voxels::octree::SparseVoxelOctree;
    use nalgebra::Point3;
    use std::fmt::Debug;
    use std::time::{Duration, Instant};

    /// Benchmark results for voxel operations
    #[derive(Debug, Clone)]
    pub struct VoxelBenchmarkResults {
        /// Operation name
        pub operation: String,
        /// Execution time
        pub duration: Duration,
        /// Number of operations performed
        pub operations_count: usize,
        /// Operations per second
        pub ops_per_second: Real,
        /// Memory usage in bytes
        pub memory_usage: usize,
    }

    impl VoxelBenchmarkResults {
        /// Create new benchmark results
        pub const fn new(
            operation: String,
            duration: Duration,
            operations_count: usize,
            memory_usage: usize,
        ) -> Self {
            let ops_per_second = if duration.as_secs_f64() > 0.0 {
                operations_count as Real / duration.as_secs_f64()
            } else {
                0.0
            };

            Self {
                operation,
                duration,
                operations_count,
                ops_per_second,
                memory_usage,
            }
        }

        /// Format duration as human-readable string
        pub fn duration_ms(&self) -> Real {
            self.duration.as_secs_f64() * 1000.0
        }
    }

    impl<S: Clone + Debug + Send + Sync + std::hash::Hash + std::cmp::PartialEq>
        SparseVoxelOctree<S>
    {
        /// Benchmark access performance for a set of query points
        pub fn benchmark_access_performance(
            &self,
            query_points: &[Point3<Real>],
        ) -> VoxelBenchmarkResults {
            let start = Instant::now();

            let mut _hits = 0;
            for point in query_points {
                if let Some(true) = self.get_voxel(point) {
                    _hits += 1;
                }
            }

            let duration = start.elapsed();
            let memory_stats = self.memory_stats();

            VoxelBenchmarkResults::new(
                "voxel_access".to_string(),
                duration,
                query_points.len(),
                memory_stats.memory_usage_bytes,
            )
        }

        /// Benchmark CSG performance with another octree
        pub fn benchmark_csg_performance(
            &self,
            other: &Self,
            operation: crate::voxels::operations::VoxelCsgOp,
        ) -> VoxelBenchmarkResults {
            let start = Instant::now();

            let _result = self.csg_operation(other, operation);

            let duration = start.elapsed();
            let memory_stats = self.memory_stats();

            VoxelBenchmarkResults::new(
                format!("csg_{:?}", operation),
                duration,
                1,
                memory_stats.memory_usage_bytes,
            )
        }

        /// Benchmark mesh conversion performance
        pub fn benchmark_mesh_conversion(&self) -> VoxelBenchmarkResults {
            let start = Instant::now();

            let _mesh = self.to_mesh();

            let duration = start.elapsed();
            let memory_stats = self.memory_stats();

            VoxelBenchmarkResults::new(
                "mesh_conversion".to_string(),
                duration,
                1,
                memory_stats.memory_usage_bytes,
            )
        }

        /// Generate a test octree with specified size and sparsity
        pub fn generate_test_octree(size: usize, sparsity: f64) -> Self {
            Self::generate_test_octree_custom_size(
                size,
                sparsity,
                Point3::new(0.0, 0.0, 0.0),
                10.0,
                4,
            )
        }

        /// Generate a test octree with custom parameters
        pub fn generate_test_octree_custom_size(
            size: usize,
            sparsity: f64,
            origin: Point3<Real>,
            octree_size: Real,
            max_depth: usize,
        ) -> Self {
            let mut octree = SparseVoxelOctree::new(origin, octree_size, max_depth, None);

            // Generate a grid of test points
            let step = octree_size / size as Real;
            let mut _point_count = 0;

            for x in 0..size {
                for y in 0..size {
                    for z in 0..size {
                        // Use a simple hash-like function to determine if voxel should be occupied
                        let hash = (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
                        let occupied = (hash as f64 / u32::MAX as f64) < sparsity;

                        if occupied {
                            let point = Point3::new(
                                origin.x + x as Real * step,
                                origin.y + y as Real * step,
                                origin.z + z as Real * step,
                            );
                            octree.set_voxel(&point, true, None);
                            _point_count += 1;
                        }
                    }
                }
            }

            octree
        }

        /// Run comprehensive benchmark suite
        pub fn run_comprehensive_benchmark(&self) -> Vec<VoxelBenchmarkResults> {
            let mut results = Vec::new();

            // Generate query points for access benchmark
            let query_points: Vec<Point3<Real>> = (0..1000)
                .map(|i| {
                    let t = i as Real * 0.01;
                    Point3::new(
                        self.origin.x + t * self.size,
                        self.origin.y + t * self.size,
                        self.origin.z + t * self.size,
                    )
                })
                .collect();

            // Access performance benchmark
            results.push(self.benchmark_access_performance(&query_points));

            // Mesh conversion benchmark
            results.push(self.benchmark_mesh_conversion());

            // CSG benchmarks (need another octree)
            let other = Self::generate_test_octree(8, 0.3);
            results.push(self.benchmark_csg_performance(
                &other,
                crate::voxels::operations::VoxelCsgOp::Union,
            ));
            results.push(self.benchmark_csg_performance(
                &other,
                crate::voxels::operations::VoxelCsgOp::Intersection,
            ));

            results
        }
    }

    /// Performance comparison results
    #[derive(Debug, Clone)]
    pub struct PerformanceComparison {
        /// Fastest operation name
        pub fastest_operation: String,
        /// Slowest operation name
        pub slowest_operation: String,
        /// Average performance improvement percentage
        pub average_improvement: Real,
    }

    impl PerformanceComparison {
        /// Create new performance comparison
        pub const fn new(
            fastest_operation: String,
            slowest_operation: String,
            average_improvement: Real,
        ) -> Self {
            Self {
                fastest_operation,
                slowest_operation,
                average_improvement,
            }
        }
    }

    /// Performance comparison utilities
    pub mod comparison {
        use super::{PerformanceComparison, VoxelBenchmarkResults};

        /// Compare benchmark results across different configurations
        pub fn compare_configurations(
            _results: &[Vec<VoxelBenchmarkResults>],
        ) -> PerformanceComparison {
            // Implementation for comparing benchmark results
            // This would analyze performance differences between configurations
            PerformanceComparison {
                fastest_operation: "placeholder".to_string(),
                slowest_operation: "placeholder".to_string(),
                average_improvement: 0.0,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_memory_stats_creation() {
        let stats = VoxelMemoryStats::new(100, 50, 100, Some(2.0));

        assert_eq!(stats.node_count, 100);
        assert_eq!(stats.occupied_leaves, 50);
        assert_eq!(stats.compression_ratio, Some(2.0));
        assert!(stats.memory_usage_bytes > 0);
        assert_eq!(stats.compression_savings_percent(), Some(50.0));
    }

    #[test]
    fn test_triangle_bounding_box() {
        use triangle_utils::*;

        let triangle = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        ];

        let bbox = triangle_bounding_box(&triangle);

        assert!((bbox.mins.x - 0.0).abs() < 1e-10);
        assert!((bbox.mins.y - 0.0).abs() < 1e-10);
        assert!((bbox.maxs.x - 2.0).abs() < 1e-10);
        assert!((bbox.maxs.y - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_point_in_triangle() {
        use triangle_utils::*;

        let triangle = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        ];

        // Test points inside triangle
        assert!(point_in_triangle(&Point3::new(1.0, 0.5, 0.0), &triangle));
        assert!(point_in_triangle(&Point3::new(1.0, 1.0, 0.0), &triangle));

        // Test points outside triangle
        assert!(!point_in_triangle(&Point3::new(3.0, 3.0, 0.0), &triangle));
        assert!(!point_in_triangle(&Point3::new(-1.0, -1.0, 0.0), &triangle));

        // Test point at wrong Z level
        assert!(!point_in_triangle(&Point3::new(1.0, 1.0, 1.0), &triangle));
    }
}

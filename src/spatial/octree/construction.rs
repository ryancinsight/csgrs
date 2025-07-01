//! Octree construction algorithms and adaptive refinement strategies

use super::core::{Node, Octant};
use crate::geometry::Polygon;
use crate::core::float_types::Real;
use crate::spatial::traits::Aabb;
use nalgebra::Point3;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::join;

/// Configuration for Octree construction
#[derive(Debug, Clone)]
pub struct OctreeConfig {
    /// Maximum depth of the tree
    pub max_depth: usize,
    /// Minimum number of polygons per leaf node
    pub min_polygons_per_leaf: usize,
    /// Maximum number of polygons per leaf node before subdividing
    pub max_polygons_per_leaf: usize,
    /// Minimum volume for subdivision (prevents excessive subdivision of small regions)
    pub min_subdivision_volume: Real,
    /// Whether to use adaptive refinement based on polygon density
    pub adaptive_refinement: bool,
}

impl Default for OctreeConfig {
    fn default() -> Self {
        Self {
            max_depth: 8,
            min_polygons_per_leaf: 1,
            max_polygons_per_leaf: 8,
            min_subdivision_volume: 0.001,
            adaptive_refinement: true,
        }
    }
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Build an Octree from polygons using sequential construction
    pub fn from_polygons_sequential(polygons: &[Polygon<S>]) -> Self {
        if polygons.is_empty() {
            return Self::new();
        }

        // Calculate bounding box for all polygons
        let bounds = Aabb::from_polygons(polygons)
            .unwrap_or_else(|| Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0)));

        let config = OctreeConfig::default();
        Self::build_recursive(polygons, bounds, 0, &config)
    }

    /// Build an Octree from polygons using parallel construction
    #[cfg(feature = "parallel")]
    pub fn from_polygons_parallel(polygons: &[Polygon<S>]) -> Self {
        if polygons.is_empty() {
            return Self::new();
        }

        // Calculate bounding box for all polygons
        let bounds = Aabb::from_polygons(polygons)
            .unwrap_or_else(|| Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0)));

        let config = OctreeConfig::default();
        Self::build_recursive_parallel(polygons, bounds, 0, &config)
    }

    /// Build an Octree with custom configuration
    pub fn from_polygons_with_config(polygons: &[Polygon<S>], config: &OctreeConfig) -> Self {
        if polygons.is_empty() {
            return Self::new();
        }

        let bounds = Aabb::from_polygons(polygons)
            .unwrap_or_else(|| Aabb::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0)));

        Self::build_recursive(polygons, bounds, 0, config)
    }

    /// Recursive tree building algorithm
    fn build_recursive(
        polygons: &[Polygon<S>],
        bounds: Aabb,
        depth: usize,
        config: &OctreeConfig,
    ) -> Self {
        let mut node = Self::new_with_bounds(bounds.clone());
        node.level = depth;

        // Base cases for recursion termination
        if polygons.is_empty() {
            return node;
        }

        if depth >= config.max_depth
            || polygons.len() <= config.max_polygons_per_leaf
            || bounds.volume() < config.min_subdivision_volume {
            // Create leaf node
            node.polygons = polygons.to_vec();
            return node;
        }

        // Subdivide the space into 8 octants
        let mut octant_polygons: [Vec<Polygon<S>>; 8] = [
            Vec::new(), Vec::new(), Vec::new(), Vec::new(),
            Vec::new(), Vec::new(), Vec::new(), Vec::new(),
        ];

        // Distribute polygons to octants
        for polygon in polygons {
            let octants = node.polygon_octants(polygon);
            for octant in octants {
                octant_polygons[octant as usize].push(polygon.clone());
            }
        }

        // Check if subdivision is beneficial
        let non_empty_octants = octant_polygons.iter().filter(|v| !v.is_empty()).count();
        if non_empty_octants <= 1 {
            // Subdivision doesn't help, create leaf node
            node.polygons = polygons.to_vec();
            return node;
        }

        // Create child nodes for non-empty octants
        node.is_subdivided = true;
        for octant in Octant::all() {
            let octant_index = octant as usize;
            if !octant_polygons[octant_index].is_empty() {
                let child_bounds = octant.bounds(&bounds);
                let child_node = Self::build_recursive(
                    &octant_polygons[octant_index],
                    child_bounds,
                    depth + 1,
                    config,
                );
                node.set_child(octant, child_node);
            }
        }

        node
    }

    /// Parallel recursive tree building algorithm
    #[cfg(feature = "parallel")]
    fn build_recursive_parallel(
        polygons: &[Polygon<S>],
        bounds: Aabb,
        depth: usize,
        config: &OctreeConfig,
    ) -> Self {
        let mut node = Self::new_with_bounds(bounds.clone());
        node.level = depth;

        // Base cases for recursion termination
        if polygons.is_empty() {
            return node;
        }

        if depth >= config.max_depth
            || polygons.len() <= config.max_polygons_per_leaf
            || bounds.volume() < config.min_subdivision_volume {
            // Create leaf node
            node.polygons = polygons.to_vec();
            return node;
        }

        // Subdivide the space into 8 octants
        let mut octant_polygons: [Vec<Polygon<S>>; 8] = [
            Vec::new(), Vec::new(), Vec::new(), Vec::new(),
            Vec::new(), Vec::new(), Vec::new(), Vec::new(),
        ];

        // Distribute polygons to octants
        for polygon in polygons {
            let octants = node.polygon_octants(polygon);
            for octant in octants {
                octant_polygons[octant as usize].push(polygon.clone());
            }
        }

        // Check if subdivision is beneficial
        let non_empty_octants = octant_polygons.iter().filter(|v| !v.is_empty()).count();
        if non_empty_octants <= 1 {
            // Subdivision doesn't help, create leaf node
            node.polygons = polygons.to_vec();
            return node;
        }

        // Create child nodes for non-empty octants
        node.is_subdivided = true;

        // For large datasets, build children in parallel
        if polygons.len() > 50 {
            // Build children in parallel groups
            let children_data: Vec<_> = Octant::all()
                .iter()
                .map(|&octant| {
                    let octant_index = octant as usize;
                    let child_bounds = octant.bounds(&bounds);
                    (octant, octant_polygons[octant_index].clone(), child_bounds)
                })
                .collect();

            // Process in parallel batches
            let mut child_results = Vec::new();
            for chunk in children_data.chunks(2) {
                if chunk.len() == 2 {
                    let (child1, child2) = join(
                        || {
                            let (octant, polys, bounds) = &chunk[0];
                            if !polys.is_empty() {
                                Some((*octant, Self::build_recursive_parallel(polys, bounds.clone(), depth + 1, config)))
                            } else {
                                None
                            }
                        },
                        || {
                            let (octant, polys, bounds) = &chunk[1];
                            if !polys.is_empty() {
                                Some((*octant, Self::build_recursive_parallel(polys, bounds.clone(), depth + 1, config)))
                            } else {
                                None
                            }
                        },
                    );
                    if let Some((octant, child)) = child1 {
                        child_results.push((octant, child));
                    }
                    if let Some((octant, child)) = child2 {
                        child_results.push((octant, child));
                    }
                } else {
                    let (octant, polys, bounds) = &chunk[0];
                    if !polys.is_empty() {
                        let child = Self::build_recursive_parallel(polys, bounds.clone(), depth + 1, config);
                        child_results.push((*octant, child));
                    }
                }
            }

            // Set the children
            for (octant, child) in child_results {
                node.set_child(octant, child);
            }
        } else {
            // Use sequential for smaller datasets
            for octant in Octant::all() {
                let octant_index = octant as usize;
                if !octant_polygons[octant_index].is_empty() {
                    let child_bounds = octant.bounds(&bounds);
                    let child_node = Self::build_recursive(
                        &octant_polygons[octant_index],
                        child_bounds,
                        depth + 1,
                        config,
                    );
                    node.set_child(octant, child_node);
                }
            }
        }

        node
    }

    /// Adaptive refinement based on polygon density
    pub fn adaptive_refine(&mut self, config: &OctreeConfig) {
        if !config.adaptive_refinement || self.is_subdivided {
            return;
        }

        // Calculate polygon density
        let density = self.polygons.len() as Real / self.bounds.volume();
        let adaptive_threshold = config.max_polygons_per_leaf as Real * 0.5;

        if density > adaptive_threshold && self.level < config.max_depth {
            // Subdivide this node
            let polygons = std::mem::take(&mut self.polygons);
            let bounds = self.bounds.clone();
            let depth = self.level;

            *self = Self::build_recursive(&polygons, bounds, depth, config);
        }

        // Recursively refine children
        for child in &mut self.children {
            if let Some(child_node) = child {
                child_node.adaptive_refine(config);
            }
        }
    }

    /// Optimize the tree structure after construction
    pub fn optimize(&mut self) {
        // Post-construction optimization could include:
        // - Merging sparse child nodes
        // - Rebalancing based on query patterns
        // - Adjusting subdivision thresholds

        // For now, just recursively optimize children
        for child in &mut self.children {
            if let Some(child_node) = child {
                child_node.optimize();
            }
        }
    }

    /// Validate the tree structure for debugging
    pub fn validate(&self) -> Result<(), String> {
        // Check that all polygons in children are within the parent bounds
        for (i, child) in self.children.iter().enumerate() {
            if let Some(child_node) = child {
                let octant = match i {
                    0 => Octant::BottomFrontLeft,
                    1 => Octant::BottomFrontRight,
                    2 => Octant::BottomBackLeft,
                    3 => Octant::BottomBackRight,
                    4 => Octant::TopFrontLeft,
                    5 => Octant::TopFrontRight,
                    6 => Octant::TopBackLeft,
                    7 => Octant::TopBackRight,
                    _ => unreachable!(),
                };

                let expected_bounds = octant.bounds(&self.bounds);
                if child_node.bounds.min != expected_bounds.min || child_node.bounds.max != expected_bounds.max {
                    return Err(format!(
                        "Child octant {} has incorrect bounds: expected {:?}, got {:?}",
                        i, expected_bounds, child_node.bounds
                    ));
                }

                // Recursively validate children
                child_node.validate()?;
            }
        }

        Ok(())
    }

    /// Get statistics about subdivision efficiency
    pub fn subdivision_stats(&self) -> SubdivisionStats {
        let mut stats = SubdivisionStats::default();
        self.collect_subdivision_stats(&mut stats);
        stats
    }

    fn collect_subdivision_stats(&self, stats: &mut SubdivisionStats) {
        if self.is_leaf() {
            stats.leaf_nodes += 1;
            stats.total_polygons_in_leaves += self.polygons.len();
            stats.max_polygons_per_leaf = stats.max_polygons_per_leaf.max(self.polygons.len());
            stats.min_polygons_per_leaf = stats.min_polygons_per_leaf.min(self.polygons.len());
        } else {
            stats.internal_nodes += 1;
            let active_children = self.children.iter().filter(|c| c.is_some()).count();
            stats.total_active_children += active_children;
            stats.max_children_per_node = stats.max_children_per_node.max(active_children);
        }

        for child in &self.children {
            if let Some(child_node) = child {
                child_node.collect_subdivision_stats(stats);
            }
        }
    }
}

/// Statistics about octree subdivision
#[derive(Debug, Clone, Default)]
pub struct SubdivisionStats {
    pub leaf_nodes: usize,
    pub internal_nodes: usize,
    pub total_polygons_in_leaves: usize,
    pub max_polygons_per_leaf: usize,
    pub min_polygons_per_leaf: usize,
    pub total_active_children: usize,
    pub max_children_per_node: usize,
}

impl SubdivisionStats {
    pub fn average_polygons_per_leaf(&self) -> f64 {
        if self.leaf_nodes > 0 {
            self.total_polygons_in_leaves as f64 / self.leaf_nodes as f64
        } else {
            0.0
        }
    }

    pub fn average_children_per_internal_node(&self) -> f64 {
        if self.internal_nodes > 0 {
            self.total_active_children as f64 / self.internal_nodes as f64
        } else {
            0.0
        }
    }
}

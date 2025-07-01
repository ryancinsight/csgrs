//! KD-tree construction algorithms and optimization strategies

use super::core::{Axis, Node};
use crate::geometry::Polygon;
use crate::core::float_types::Real;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::join;

/// Configuration for KD-tree construction
#[derive(Debug, Clone)]
pub struct KdTreeConfig {
    /// Maximum depth of the tree
    pub max_depth: usize,
    /// Minimum number of polygons per leaf node
    pub min_polygons_per_leaf: usize,
    /// Maximum number of polygons per leaf node before splitting
    pub max_polygons_per_leaf: usize,
    /// Whether to use Surface Area Heuristic for splitting
    pub use_sah: bool,
}

impl Default for KdTreeConfig {
    fn default() -> Self {
        Self {
            max_depth: 20,
            min_polygons_per_leaf: 1,
            max_polygons_per_leaf: 10,
            use_sah: true,
        }
    }
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Build a KD-tree from polygons using sequential construction
    pub fn from_polygons_sequential(polygons: &[Polygon<S>]) -> Self {
        let config = KdTreeConfig::default();
        Self::build_recursive(polygons, Axis::X, 0, &config)
    }

    /// Build a KD-tree from polygons using parallel construction
    #[cfg(feature = "parallel")]
    pub fn from_polygons_parallel(polygons: &[Polygon<S>]) -> Self {
        let config = KdTreeConfig::default();
        Self::build_recursive_parallel(polygons, Axis::X, 0, &config)
    }

    /// Build a KD-tree with custom configuration
    pub fn from_polygons_with_config(polygons: &[Polygon<S>], config: &KdTreeConfig) -> Self {
        Self::build_recursive(polygons, Axis::X, 0, config)
    }

    /// Recursive tree building algorithm
    fn build_recursive(
        polygons: &[Polygon<S>],
        axis: Axis,
        depth: usize,
        config: &KdTreeConfig,
    ) -> Self {
        let mut node = Self::new();

        // Base cases for recursion termination
        if polygons.is_empty() {
            return node;
        }

        if depth >= config.max_depth || polygons.len() <= config.max_polygons_per_leaf {
            // Create leaf node
            node.polygons = polygons.to_vec();
            node.update_bounds();
            return node;
        }

        // Find the best split
        let split_value = if config.use_sah {
            Self::find_best_split_sah(polygons, axis)
        } else {
            Self::find_median_split(polygons, axis)
        };

        // Partition polygons
        let (left_polygons, right_polygons) = Self::partition_polygons(polygons, axis, split_value);

        // Avoid infinite recursion if split doesn't separate polygons well
        if left_polygons.is_empty() || right_polygons.is_empty() {
            node.polygons = polygons.to_vec();
            node.update_bounds();
            return node;
        }

        // Set node properties
        node.axis = Some(axis);
        node.split_value = Some(split_value);

        // Build child nodes
        let next_axis = axis.next();
        node.left = Some(Box::new(Self::build_recursive(&left_polygons, next_axis, depth + 1, config)));
        node.right = Some(Box::new(Self::build_recursive(&right_polygons, next_axis, depth + 1, config)));

        node.update_bounds();
        node
    }

    /// Parallel recursive tree building algorithm
    #[cfg(feature = "parallel")]
    fn build_recursive_parallel(
        polygons: &[Polygon<S>],
        axis: Axis,
        depth: usize,
        config: &KdTreeConfig,
    ) -> Self {
        let mut node = Self::new();

        // Base cases for recursion termination
        if polygons.is_empty() {
            return node;
        }

        if depth >= config.max_depth || polygons.len() <= config.max_polygons_per_leaf {
            // Create leaf node
            node.polygons = polygons.to_vec();
            node.update_bounds();
            return node;
        }

        // Find the best split
        let split_value = if config.use_sah {
            Self::find_best_split_sah(polygons, axis)
        } else {
            Self::find_median_split(polygons, axis)
        };

        // Partition polygons
        let (left_polygons, right_polygons) = Self::partition_polygons(polygons, axis, split_value);

        // Avoid infinite recursion if split doesn't separate polygons well
        if left_polygons.is_empty() || right_polygons.is_empty() {
            node.polygons = polygons.to_vec();
            node.update_bounds();
            return node;
        }

        // Set node properties
        node.axis = Some(axis);
        node.split_value = Some(split_value);

        // Build child nodes in parallel for large datasets
        let next_axis = axis.next();
        if polygons.len() > 100 {
            let (left_node, right_node) = join(
                || Self::build_recursive_parallel(&left_polygons, next_axis, depth + 1, config),
                || Self::build_recursive_parallel(&right_polygons, next_axis, depth + 1, config),
            );
            node.left = Some(Box::new(left_node));
            node.right = Some(Box::new(right_node));
        } else {
            // Use sequential for smaller datasets to avoid overhead
            node.left = Some(Box::new(Self::build_recursive(&left_polygons, next_axis, depth + 1, config)));
            node.right = Some(Box::new(Self::build_recursive(&right_polygons, next_axis, depth + 1, config)));
        }

        node.update_bounds();
        node
    }

    /// Find the median split value along an axis
    fn find_median_split(polygons: &[Polygon<S>], axis: Axis) -> Real {
        let mut coordinates: Vec<Real> = polygons
            .iter()
            .map(|poly| {
                let center = Self::polygon_center(poly);
                axis.coordinate(&center)
            })
            .collect();

        coordinates.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        
        let mid_index = coordinates.len() / 2;
        if coordinates.len() % 2 == 0 && mid_index > 0 {
            (coordinates[mid_index - 1] + coordinates[mid_index]) * 0.5
        } else {
            coordinates[mid_index]
        }
    }

    /// Find the best split using Surface Area Heuristic (SAH)
    fn find_best_split_sah(polygons: &[Polygon<S>], axis: Axis) -> Real {
        if polygons.len() < 4 {
            return Self::find_median_split(polygons, axis);
        }

        // Get sorted coordinates
        let mut coordinates: Vec<Real> = polygons
            .iter()
            .map(|poly| {
                let center = Self::polygon_center(poly);
                axis.coordinate(&center)
            })
            .collect();

        coordinates.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let total_polygons = polygons.len() as Real;
        let mut best_split = coordinates[coordinates.len() / 2];
        let mut best_cost = Real::INFINITY;

        // Sample potential split positions
        let sample_count = (coordinates.len() / 4).max(3).min(20);
        for i in 1..sample_count {
            let index = (i * coordinates.len()) / sample_count;
            let split_candidate = coordinates[index];

            // Count polygons on each side
            let left_count = coordinates.iter().filter(|&&coord| coord <= split_candidate).count() as Real;
            let right_count = total_polygons - left_count;

            // Simple SAH cost: weighted by polygon count
            // In a full implementation, this would consider surface area
            let cost = left_count / total_polygons + right_count / total_polygons;

            if cost < best_cost {
                best_cost = cost;
                best_split = split_candidate;
            }
        }

        best_split
    }

    /// Partition polygons based on split value
    fn partition_polygons(
        polygons: &[Polygon<S>],
        axis: Axis,
        split_value: Real,
    ) -> (Vec<Polygon<S>>, Vec<Polygon<S>>) {
        let mut left_polygons = Vec::new();
        let mut right_polygons = Vec::new();

        for polygon in polygons {
            let center = Self::polygon_center(polygon);
            let coordinate = axis.coordinate(&center);

            if coordinate <= split_value {
                left_polygons.push(polygon.clone());
            } else {
                right_polygons.push(polygon.clone());
            }
        }

        (left_polygons, right_polygons)
    }

    /// Optimize the tree structure after construction
    pub fn optimize(&mut self) {
        // Post-construction optimization could include:
        // - Rebalancing uneven subtrees
        // - Merging small adjacent leaf nodes
        // - Adjusting split positions for better performance
        
        // For now, just recursively optimize children
        if let Some(ref mut left) = self.left {
            left.optimize();
        }
        if let Some(ref mut right) = self.right {
            right.optimize();
        }
    }

    /// Validate the tree structure for debugging
    pub fn validate(&self) -> Result<(), String> {
        // Check that split values are consistent
        if let (Some(axis), Some(split_value)) = (self.axis, self.split_value) {
            // Validate left child
            if let Some(ref left) = self.left {
                for polygon in &left.polygons {
                    let center = Self::polygon_center(polygon);
                    let coordinate = axis.coordinate(&center);
                    if coordinate > split_value {
                        return Err(format!(
                            "Left child contains polygon with coordinate {} > split_value {}",
                            coordinate, split_value
                        ));
                    }
                }
                left.validate()?;
            }

            // Validate right child
            if let Some(ref right) = self.right {
                for polygon in &right.polygons {
                    let center = Self::polygon_center(polygon);
                    let coordinate = axis.coordinate(&center);
                    if coordinate <= split_value {
                        return Err(format!(
                            "Right child contains polygon with coordinate {} <= split_value {}",
                            coordinate, split_value
                        ));
                    }
                }
                right.validate()?;
            }
        }

        Ok(())
    }
}

//! Core BVH data structures and basic operations
//!
//! This module defines the fundamental BVH node structure and provides
//! basic tree operations like creation, traversal, and statistics.

use crate::geometry::Polygon;
use crate::spatial::traits::{Aabb, SpatialIndex, SpatialStatistics, Ray, Intersection};
use crate::core::float_types::Real;
use super::config::BVHConfig;
use std::fmt::Debug;
use nalgebra::Point3;

/// BVH node representing either an internal node or leaf node
///
/// BVHs are binary trees where each node contains a bounding volume that
/// encompasses all its children. Leaf nodes contain actual polygons, while
/// internal nodes contain exactly two child nodes for optimal ray traversal.
///
/// # Examples
///
/// ## Creating an Empty BVH
///
/// ```rust
/// use csgrs::spatial::bvh::Node;
///
/// let bvh: Node<i32> = Node::new();
/// assert!(bvh.is_empty());
/// assert_eq!(bvh.polygon_count(), 0);
/// ```
///
/// ## Building from Polygons
///
/// ```rust
/// use csgrs::spatial::{bvh::Node, SpatialIndex};
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
/// let bvh = Node::from_polygons(&vec![polygon]);
/// assert_eq!(bvh.all_polygons().len(), 1);
/// ```
#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    /// Axis-aligned bounding box for this node
    pub bounding_volume: Option<Aabb>,
    
    /// Child nodes (for internal nodes) - exactly 2 children for binary tree
    pub children: Option<(Box<Node<S>>, Box<Node<S>>)>,
    
    /// Stored polygons (for leaf nodes)
    pub polygons: Vec<Polygon<S>>,
    
    /// Whether this is a leaf node
    pub is_leaf: bool,
    
    /// Splitting axis (0=X, 1=Y, 2=Z) for debugging/analysis
    pub split_axis: Option<usize>,
    
    /// Cached surface area for SAH calculations
    pub surface_area: Real,
    
    /// Configuration for tree operations
    pub config: BVHConfig,
    
    /// Tree level (0 for leaves, increases toward root)
    pub level: usize,
    
    /// Number of refit operations performed (for quality tracking)
    pub refit_count: usize,
}

impl<S: Clone> Node<S> {
    /// Create a new empty BVH node
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::Node;
    ///
    /// let bvh: Node<i32> = Node::new();
    /// assert!(bvh.is_leaf);
    /// assert_eq!(bvh.level, 0);
    /// assert!(bvh.is_empty());
    /// ```
    pub fn new() -> Self {
        Self::with_config(BVHConfig::default())
    }
    
    /// Create a new BVH node with custom configuration
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::{Node, BVHConfig};
    ///
    /// let config = BVHConfig::for_ray_tracing();
    /// let bvh: Node<i32> = Node::with_config(config);
    /// assert!(bvh.is_leaf);
    /// ```
    pub fn with_config(config: BVHConfig) -> Self {
        Self {
            bounding_volume: None,
            children: None,
            polygons: Vec::new(),
            is_leaf: true,
            split_axis: None,
            surface_area: 0.0,
            config,
            level: 0,
            refit_count: 0,
        }
    }
    
    /// Build BVH from a collection of polygons
    ///
    /// Uses the default configuration with binned SAH construction.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::{bvh::Node, SpatialIndex};
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, None);
    /// let polygons = vec![polygon];
    ///
    /// let bvh = Node::from_polygons(&polygons);
    /// assert_eq!(bvh.all_polygons().len(), 1);
    /// ```
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        Self::from_polygons_with_config(polygons, &BVHConfig::default())
    }
    
    /// Build BVH from polygons with custom configuration
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::{bvh::{Node, BVHConfig}, SpatialIndex};
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, None);
    /// let polygons = vec![polygon];
    ///
    /// let config = BVHConfig::for_ray_tracing();
    /// let bvh = Node::from_polygons_with_config(&polygons, &config);
    /// assert_eq!(bvh.all_polygons().len(), 1);
    /// ```
    pub fn from_polygons_with_config(polygons: &[Polygon<S>], config: &BVHConfig) -> Self {
        if polygons.is_empty() {
            return Self::with_config(config.clone());
        }
        
        // Use construction algorithms from construction.rs
        Self::build_bvh(polygons, config)
    }
    
    /// Check if the BVH is empty
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::Node;
    ///
    /// let bvh: Node<i32> = Node::new();
    /// assert!(bvh.is_empty());
    /// ```
    pub fn is_empty(&self) -> bool {
        self.polygons.is_empty() && self.children.is_none()
    }
    
    /// Get the total number of polygons in the BVH
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::Node;
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, None);
    /// let bvh = Node::from_polygons(&vec![polygon]);
    /// assert_eq!(bvh.polygon_count(), 1);
    /// ```
    pub fn polygon_count(&self) -> usize {
        if self.is_leaf {
            self.polygons.len()
        } else if let Some((ref left, ref right)) = self.children {
            left.polygon_count() + right.polygon_count()
        } else {
            0
        }
    }
    
    /// Get the depth of the BVH
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::Node;
    ///
    /// let bvh: Node<i32> = Node::new();
    /// assert_eq!(bvh.depth(), 1); // Single node has depth 1
    /// ```
    pub fn depth(&self) -> usize {
        if self.is_leaf {
            1
        } else if let Some((ref left, ref right)) = self.children {
            1 + left.depth().max(right.depth())
        } else {
            1
        }
    }
    
    /// Get the total number of nodes in the BVH
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::Node;
    ///
    /// let bvh: Node<i32> = Node::new();
    /// assert_eq!(bvh.node_count(), 1);
    /// ```
    pub fn node_count(&self) -> usize {
        if self.is_leaf {
            1
        } else if let Some((ref left, ref right)) = self.children {
            1 + left.node_count() + right.node_count()
        } else {
            1
        }
    }
    
    /// Calculate approximate memory usage in bytes
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::Node;
    ///
    /// let bvh: Node<i32> = Node::new();
    /// let memory_usage = bvh.memory_usage();
    /// assert!(memory_usage > 0);
    /// ```
    pub fn memory_usage(&self) -> usize {
        let base_size = std::mem::size_of::<Self>();
        let children_size = if let Some((ref left, ref right)) = self.children {
            left.memory_usage() + right.memory_usage()
        } else {
            0
        };
        let polygons_size = self.polygons.len() * std::mem::size_of::<Polygon<S>>();
        
        base_size + children_size + polygons_size
    }
    
    /// Update the bounding volume to encompass all children or polygons
    pub fn update_bounding_volume(&mut self) {
        if self.is_leaf {
            if self.polygons.is_empty() {
                self.bounding_volume = None;
                self.surface_area = 0.0;
            } else {
                let bounds: Vec<Aabb> = self.polygons.iter()
                    .filter_map(|p| crate::spatial::utils::polygon_bounds(p))
                    .collect();
                if !bounds.is_empty() {
                    let union_bounds = crate::spatial::utils::bounds_union(&bounds);
                    self.surface_area = self.calculate_surface_area(&union_bounds);
                    self.bounding_volume = Some(union_bounds);
                }
            }
        } else if let Some((ref left, ref right)) = self.children {
            match (left.bounding_volume.as_ref(), right.bounding_volume.as_ref()) {
                (Some(left_bounds), Some(right_bounds)) => {
                    let union_bounds = crate::spatial::utils::merge_bounds(left_bounds, right_bounds);
                    self.surface_area = self.calculate_surface_area(&union_bounds);
                    self.bounding_volume = Some(union_bounds);
                },
                (Some(bounds), None) | (None, Some(bounds)) => {
                    self.surface_area = self.calculate_surface_area(bounds);
                    self.bounding_volume = Some(bounds.clone());
                },
                (None, None) => {
                    self.bounding_volume = None;
                    self.surface_area = 0.0;
                }
            }
        }
    }
    
    /// Calculate surface area of a bounding box for SAH
    fn calculate_surface_area(&self, bounds: &Aabb) -> Real {
        let size = bounds.max - bounds.min;
        2.0 * (size.x * size.y + size.y * size.z + size.z * size.x)
    }
    
    /// Placeholder for BVH construction - implemented in construction.rs
    fn build_bvh(polygons: &[Polygon<S>], config: &BVHConfig) -> Self {
        // For now, create a simple leaf node
        let mut node = Self::with_config(config.clone());
        node.polygons = polygons.to_vec();
        node.update_bounding_volume();
        node
    }
    
    /// Insert an object into the BVH (dynamic operation)
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::bvh::Node;
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let mut bvh: Node<i32> = Node::new();
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, None);
    ///
    /// bvh.insert_object(polygon);
    /// assert_eq!(bvh.polygon_count(), 1);
    /// ```
    pub fn insert_object(&mut self, polygon: Polygon<S>) {
        if self.is_leaf {
            self.polygons.push(polygon);
            self.update_bounding_volume();
        } else {
            // For now, simple insertion - will be enhanced in operations.rs
            self.polygons.push(polygon);
            self.is_leaf = true;
            self.children = None;
            self.update_bounding_volume();
        }
    }
}

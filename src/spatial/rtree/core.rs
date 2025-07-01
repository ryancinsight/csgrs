//! Core R-tree data structures and basic operations
//!
//! This module defines the fundamental R-tree node structure and provides
//! basic tree operations like creation, traversal, and statistics.

use crate::geometry::Polygon;
use crate::spatial::traits::{Aabb, SpatialIndex, SpatialStatistics};
use crate::core::float_types::Real;
use super::config::RTreeConfig;
use std::fmt::Debug;
use nalgebra::Point3;

/// R-tree node representing either an internal node or leaf node
///
/// R-trees are balanced trees where each node contains a bounding box that
/// encompasses all its children. Leaf nodes contain actual polygons, while
/// internal nodes contain child nodes.
///
/// # Examples
///
/// ## Creating an Empty R-tree
///
/// ```rust
/// use csgrs::spatial::rtree::Node;
///
/// let rtree: Node<i32> = Node::new();
/// assert!(rtree.is_empty());
/// assert_eq!(rtree.polygon_count(), 0);
/// ```
///
/// ## Building from Polygons
///
/// ```rust
/// use csgrs::spatial::rtree::Node;
/// use csgrs::geometry::{Polygon, Vertex};
/// use nalgebra::{Point3, Vector3};
///
/// let vertices = vec![
///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
/// ];
/// let polygon: Polygon<i32> = Polygon::new(vertices, None);
/// let rtree = Node::from_polygons(&vec![polygon]);
/// assert_eq!(rtree.polygon_count(), 1);
/// ```
#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    /// Minimum bounding rectangle for this node
    pub bounding_box: Option<Aabb>,
    
    /// Child nodes (for internal nodes)
    pub children: Vec<Box<Node<S>>>,
    
    /// Stored polygons (for leaf nodes)
    pub polygons: Vec<Polygon<S>>,
    
    /// Whether this is a leaf node
    pub is_leaf: bool,
    
    /// Tree level (0 for leaves, increases toward root)
    pub level: usize,
    
    /// Configuration for tree operations
    config: RTreeConfig,
}

impl<S: Clone> Node<S> {
    /// Create a new empty R-tree node
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::Node;
    ///
    /// let rtree: Node<i32> = Node::new();
    /// assert!(rtree.is_leaf);
    /// assert_eq!(rtree.level, 0);
    /// assert!(rtree.is_empty());
    /// ```
    pub fn new() -> Self {
        Self::with_config(RTreeConfig::default())
    }
    
    /// Create a new R-tree node with custom configuration
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::{Node, RTreeConfig};
    ///
    /// let config = RTreeConfig::for_range_queries();
    /// let rtree: Node<i32> = Node::with_config(config);
    /// assert!(rtree.is_leaf);
    /// ```
    pub fn with_config(config: RTreeConfig) -> Self {
        Self {
            bounding_box: None,
            children: Vec::new(),
            polygons: Vec::new(),
            is_leaf: true,
            level: 0,
            config,
        }
    }
    
    /// Build R-tree from a collection of polygons
    ///
    /// Uses bulk loading if polygon count exceeds threshold, otherwise
    /// uses incremental insertion.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::Node;
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
    /// let rtree = Node::from_polygons(&polygons);
    /// assert_eq!(rtree.all_polygons().len(), 1);
    /// ```
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        Self::from_polygons_with_config(polygons, &RTreeConfig::default())
    }
    
    /// Build R-tree from polygons with custom configuration
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::{Node, RTreeConfig};
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
    /// let config = RTreeConfig::for_range_queries();
    /// let rtree = Node::from_polygons_with_config(&polygons, &config);
    /// assert_eq!(rtree.all_polygons().len(), 1);
    /// ```
    pub fn from_polygons_with_config(polygons: &[Polygon<S>], config: &RTreeConfig) -> Self {
        if polygons.is_empty() {
            return Self::with_config(config.clone());
        }
        
        if polygons.len() >= config.bulk_load_threshold {
            // Use bulk loading for large datasets
            Self::bulk_load(polygons, config)
        } else {
            // Use incremental insertion for small datasets
            let mut tree = Self::with_config(config.clone());
            for polygon in polygons {
                tree.insert(polygon.clone());
            }
            tree
        }
    }
    
    /// Check if the tree is empty
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::Node;
    ///
    /// let rtree: Node<i32> = Node::new();
    /// assert!(rtree.is_empty());
    /// ```
    pub fn is_empty(&self) -> bool {
        self.polygons.is_empty() && self.children.is_empty()
    }
    
    /// Get the total number of polygons in the tree
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::Node;
    /// use csgrs::geometry::{Polygon, Vertex};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let vertices = vec![
    ///     Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    ///     Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
    /// ];
    /// let polygon: Polygon<i32> = Polygon::new(vertices, None);
    /// let rtree = Node::from_polygons(&vec![polygon]);
    /// assert_eq!(rtree.polygon_count(), 1);
    /// ```
    pub fn polygon_count(&self) -> usize {
        if self.is_leaf {
            self.polygons.len()
        } else {
            self.children.iter().map(|child| child.polygon_count()).sum()
        }
    }
    
    /// Get the depth of the tree
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::Node;
    ///
    /// let rtree: Node<i32> = Node::new();
    /// assert_eq!(rtree.depth(), 1); // Single node has depth 1
    /// ```
    pub fn depth(&self) -> usize {
        if self.is_leaf {
            1
        } else {
            1 + self.children.iter().map(|child| child.depth()).max().unwrap_or(0)
        }
    }
    
    /// Get the total number of nodes in the tree
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::Node;
    ///
    /// let rtree: Node<i32> = Node::new();
    /// assert_eq!(rtree.node_count(), 1);
    /// ```
    pub fn node_count(&self) -> usize {
        1 + self.children.iter().map(|child| child.node_count()).sum::<usize>()
    }
    
    /// Calculate approximate memory usage in bytes
    ///
    /// # Examples
    ///
    /// ```rust
    /// use csgrs::spatial::rtree::Node;
    ///
    /// let rtree: Node<i32> = Node::new();
    /// let memory_usage = rtree.memory_usage();
    /// assert!(memory_usage > 0);
    /// ```
    pub fn memory_usage(&self) -> usize {
        let base_size = std::mem::size_of::<Self>();
        let children_size = self.children.iter().map(|child| child.memory_usage()).sum::<usize>();
        let polygons_size = self.polygons.len() * std::mem::size_of::<Polygon<S>>();
        
        base_size + children_size + polygons_size
    }
    
    /// Update the bounding box to encompass all children or polygons
    fn update_bounding_box(&mut self) {
        if self.is_leaf {
            if self.polygons.is_empty() {
                self.bounding_box = None;
            } else {
                let bounds: Vec<Aabb> = self.polygons.iter()
                    .filter_map(|p| crate::spatial::utils::polygon_bounds(p))
                    .collect();
                if !bounds.is_empty() {
                    self.bounding_box = Some(crate::spatial::utils::bounds_union(&bounds));
                }
            }
        } else {
            if self.children.is_empty() {
                self.bounding_box = None;
            } else {
                let bounds: Vec<Aabb> = self.children.iter()
                    .filter_map(|child| child.bounding_box.clone())
                    .collect();
                if !bounds.is_empty() {
                    self.bounding_box = Some(crate::spatial::utils::bounds_union(&bounds));
                }
            }
        }
    }
    
    /// Placeholder for bulk loading implementation
    fn bulk_load(polygons: &[Polygon<S>], config: &RTreeConfig) -> Self {
        // For now, fall back to incremental insertion
        // TODO: Implement STR (Sort-Tile-Recursive) bulk loading
        let mut tree = Self::with_config(config.clone());
        for polygon in polygons {
            tree.insert(polygon.clone());
        }
        tree
    }
}

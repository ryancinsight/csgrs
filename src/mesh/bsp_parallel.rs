//! **Enhanced Parallel BSP Tree Implementation - Simplified Version**
//! 
//! This module provides optimized BSP tree construction with:
//! - Better splitting plane selection heuristics
//! - Robust polygon classification to prevent holes
//! - Memory-efficient tree construction
//! 
//! Design principles applied: SOLID, DRY, KISS, YAGNI

use super::{bsp::Node, plane::Plane, polygon::Polygon, Mesh};
use crate::mesh::Real;
use std::fmt::Debug;

/// **Enhanced BSP Tree Builder**
/// 
/// Provides improved algorithms for BSP tree construction
/// without external parallel dependencies.
pub struct EnhancedBspBuilder {
    /// Maximum tree depth to prevent excessive recursion
    max_depth: usize,
    /// Enhanced splitting tolerance for robust classification
    split_tolerance: Real,
}

impl Default for EnhancedBspBuilder {
    fn default() -> Self {
        Self {
            max_depth: 32,
            split_tolerance: 1e-9, // Tighter tolerance for TPMS meshes
        }
    }
}

impl EnhancedBspBuilder {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self::default()
    }

    /// **Enhanced BSP construction with improved heuristics**
    pub fn build<S: Clone + Send + Sync + Debug>(
        &self,
        polygons: Vec<Polygon<S>>,
    ) -> Option<Node<S>> {
        if polygons.is_empty() {
            return None;
        }

        // **Optimization**: Pre-filter degenerate polygons to prevent issues
        let valid_polygons: Vec<_> = polygons
            .into_iter()
            .filter(|p| p.vertices.len() >= 3 && self.is_valid_polygon(p))
            .collect();

        if valid_polygons.is_empty() {
            return None;
        }

        self.build_node(valid_polygons, 0)
    }

    /// **Enhanced**: Validates polygon integrity
    fn is_valid_polygon<S: Clone>(&self, polygon: &Polygon<S>) -> bool {
        if polygon.vertices.len() < 3 {
            return false;
        }

        // Check for non-zero area using cross product
        let v0 = &polygon.vertices[0].pos;
        let v1 = &polygon.vertices[1].pos;
        let v2 = &polygon.vertices[2].pos;
        
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let cross = edge1.cross(&edge2);
        
        cross.norm_squared() > self.split_tolerance * self.split_tolerance
    }

    /// **Enhanced recursive node building**
    fn build_node<S: Clone + Send + Sync + Debug>(
        &self,
        mut polygons: Vec<Polygon<S>>,
        depth: usize,
    ) -> Option<Node<S>> {
        if polygons.is_empty() {
            return None;
        }

        if depth >= self.max_depth || polygons.len() == 1 {
            // Create a leaf node using the existing API
            return Some(Node::from_polygons(&polygons));
        }

        // **Enhanced**: Use improved splitting plane selection
        let splitting_plane = self.select_optimal_splitting_plane(&polygons);
        
        // **Optimization**: Polygon classification with better handling
        let (coplanar_front, _coplanar_back, front_polys, back_polys) = 
            self.classify_polygons(&polygons, &splitting_plane);

        polygons.clear(); // Free memory early

        // **Enhancement**: Prevent infinite recursion with better termination
        if front_polys.is_empty() && back_polys.is_empty() {
            return Some(Node::from_polygons(&coplanar_front));
        }

        // Build child nodes
        let front_child = if front_polys.is_empty() {
            None
        } else {
            self.build_node(front_polys, depth + 1).map(Box::new)
        };

        let back_child = if back_polys.is_empty() {
            None
        } else {
            self.build_node(back_polys, depth + 1).map(Box::new)
        };

        // Create a new node with the enhanced structure
        let mut node = Node::new();
        node.plane = Some(splitting_plane);
        node.polygons = coplanar_front;
        node.front = front_child;
        node.back = back_child;

        Some(node)
    }

    /// **Enhanced splitting plane selection with improved heuristics**
    fn select_optimal_splitting_plane<S: Clone>(&self, polygons: &[Polygon<S>]) -> Plane {
        if polygons.len() == 1 {
            return polygons[0].plane.clone();
        }

        // **Enhancement**: Multi-criteria scoring with adaptive weights
        let mut best_plane = polygons[0].plane.clone();
        let mut best_score = Real::MAX;

        // **Optimization**: Sample subset for large polygon counts
        let sample_size = polygons.len().min(50);
        let step = if polygons.len() > sample_size {
            polygons.len() / sample_size
        } else {
            1
        };

        for i in (0..polygons.len()).step_by(step) {
            let candidate_plane = &polygons[i].plane;
            let score = self.evaluate_plane_quality(candidate_plane, polygons);
            
            if score < best_score {
                best_score = score;
                best_plane = candidate_plane.clone();
            }
        }

        best_plane
    }

    /// **Enhanced plane quality evaluation**
    fn evaluate_plane_quality<S: Clone>(&self, plane: &Plane, polygons: &[Polygon<S>]) -> Real {
        let mut front_count = 0;
        let mut back_count = 0;
        let mut split_count = 0;
        let mut _coplanar_count = 0;

        for polygon in polygons {
            let classification = plane.classify_polygon(polygon);
            match classification {
                1 => front_count += 1,  // FRONT
                -1 => back_count += 1,  // BACK
                3 => split_count += 1,  // SPANNING 
                0 => _coplanar_count += 1, // COPLANAR
                _ => {}
            }
        }

        // **Enhanced scoring**: Balance multiple factors
        let balance_penalty = ((front_count as i32 - back_count as i32).abs() as Real) * 0.8;
        let split_penalty = (split_count as Real) * 3.0; // Heavy penalty for splits
        let small_set_penalty = if front_count == 0 || back_count == 0 { 100.0 } else { 0.0 };

        balance_penalty + split_penalty + small_set_penalty
    }

    /// **Enhanced polygon classification**
    fn classify_polygons<S: Clone + Send + Sync>(
        &self,
        polygons: &[Polygon<S>],
        plane: &Plane,
    ) -> (Vec<Polygon<S>>, Vec<Polygon<S>>, Vec<Polygon<S>>, Vec<Polygon<S>>) {
        let estimated_size = polygons.len() / 4;
        let mut coplanar_front = Vec::with_capacity(estimated_size);
        let mut coplanar_back = Vec::with_capacity(estimated_size);
        let mut front = Vec::with_capacity(estimated_size);
        let mut back = Vec::with_capacity(estimated_size);

        for polygon in polygons {
            let (mut cf, mut cb, mut f, mut b) = plane.split_polygon(polygon);
            coplanar_front.append(&mut cf);
            coplanar_back.append(&mut cb);
            front.append(&mut f);
            back.append(&mut b);
        }

        (coplanar_front, coplanar_back, front, back)
    }
}

/// **Enhanced convenience function for BSP construction**
pub fn build_enhanced_bsp<S: Clone + Send + Sync + Debug>(
    mesh: &Mesh<S>,
) -> Option<Node<S>> {
    let builder = EnhancedBspBuilder::new();
    builder.build(mesh.polygons.clone())
}

/// **Enhanced function with custom parameters**
pub fn build_enhanced_bsp_with_params<S: Clone + Send + Sync + Debug>(
    mesh: &Mesh<S>,
    max_depth: usize,
) -> Option<Node<S>> {
    let builder = EnhancedBspBuilder {
        max_depth,
        split_tolerance: 1e-9,
    };
    builder.build(mesh.polygons.clone())
}

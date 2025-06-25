//! Core BSP tree node structure and basic operations

use crate::geometry::{Plane, Polygon};
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::join;

/// A [BSP](https://en.wikipedia.org/wiki/Binary_space_partitioning) tree node, containing polygons plus optional front/back subtrees
#[derive(Debug, Clone)]
pub struct Node<S: Clone> {
    /// Splitting plane for this node *or* **None** for a leaf that
    /// only stores polygons.
    pub plane: Option<Plane>,

    /// Polygons in *front* half‑spaces.
    pub front: Option<Box<Node<S>>>,

    /// Polygons in *back* half‑spaces.
    pub back: Option<Box<Node<S>>>,

    /// Polygons that lie *exactly* on `plane`
    /// (after the node has been built).
    pub polygons: Vec<Polygon<S>>,
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// Create a new empty BSP node
    pub fn new() -> Self {
        Self {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
        }
    }

    /// Invert all polygons in the BSP tree
    pub fn invert(&mut self) {
        // Flip all polygons and plane in this node
        self.polygons.iter_mut().for_each(|p| p.flip());
        if let Some(ref mut plane) = self.plane {
            plane.flip();
        }

        // Recursively invert children - parallel when available and both exist
        #[cfg(feature = "parallel")]
        match (&mut self.front, &mut self.back) {
            (Some(front_node), Some(back_node)) => {
                join(|| front_node.invert(), || back_node.invert());
            },
            (Some(front_node), None) => front_node.invert(),
            (None, Some(back_node)) => back_node.invert(),
            (None, None) => {},
        }

        #[cfg(not(feature = "parallel"))]
        {
            if let Some(ref mut front) = self.front {
                front.invert();
            }
            if let Some(ref mut back) = self.back {
                back.invert();
            }
        }

        std::mem::swap(&mut self.front, &mut self.back);
    }

    /// Return all polygons in this BSP tree using an iterative approach.
    pub fn all_polygons(&self) -> Vec<Polygon<S>> {
        let mut result = Vec::new();
        let mut stack = vec![self];

        while let Some(node) = stack.pop() {
            result.extend_from_slice(&node.polygons);

            // Use iterator to add child nodes more efficiently
            stack.extend(
                [&node.front, &node.back]
                    .iter()
                    .filter_map(|child| child.as_ref().map(|boxed| boxed.as_ref())),
            );
        }
        result
    }
}

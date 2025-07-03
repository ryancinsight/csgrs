//! **Spatial Configuration Types (The Skeleton)**
//!
//! This module defines configuration types and query specifications for spatial operations,
//! following Cathedral Engineering principles where configuration represents part of the
//! "skeleton" that provides structural foundation.

use crate::core::float_types::Real;

/// **Types of spatial queries that can be performed**
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum QueryType {
    PointLocation,
    RangeQuery,
    NearestNeighbor,
    RayIntersection,
    BooleanOperations,
}

/// **Spatial distribution patterns of data**
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpatialDistribution {
    Uniform,
    Clustered,
    Linear,
    Sparse,
    Dense,
}

/// **Types of spatial data structures**
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpatialStructureType {
    BSP,
    KDTree,
    Octree,
    BVH,
    RTree,
}

/// **Configuration for spatial data structures**
#[derive(Debug, Clone)]
pub struct SpatialConfig {
    pub max_depth: usize,
    pub max_polygons_per_leaf: usize,
    pub split_threshold: Real,
    pub structure_type: Option<SpatialStructureType>,
    pub optimize_for: QueryType,
}

impl Default for SpatialConfig {
    fn default() -> Self {
        Self {
            max_depth: 20,
            max_polygons_per_leaf: 10,
            split_threshold: 0.5,
            structure_type: None,
            optimize_for: QueryType::RangeQuery,
        }
    }
}

impl SpatialConfig {
    /// **Configuration optimized for point location queries**
    pub fn for_point_queries() -> Self {
        Self {
            max_depth: 25,
            max_polygons_per_leaf: 5,
            split_threshold: 0.5,
            structure_type: Some(SpatialStructureType::KDTree),
            optimize_for: QueryType::PointLocation,
        }
    }

    /// **Configuration optimized for volume queries**
    pub fn for_volume_queries() -> Self {
        Self {
            max_depth: 15,
            max_polygons_per_leaf: 20,
            split_threshold: 0.4,
            structure_type: Some(SpatialStructureType::Octree),
            optimize_for: QueryType::RangeQuery,
        }
    }

    /// **Configuration optimized for boolean operations**
    pub fn for_boolean_operations() -> Self {
        Self {
            max_depth: 30,
            max_polygons_per_leaf: 1,
            split_threshold: 0.5,
            structure_type: Some(SpatialStructureType::BSP),
            optimize_for: QueryType::BooleanOperations,
        }
    }
}

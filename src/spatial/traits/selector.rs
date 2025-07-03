//! **Spatial Structure Selector (The Mind)**
//!
//! This module provides automatic structure selection algorithms,
//! following Cathedral Engineering principles where selectors represent the "mind"
//! that performs intelligent decision-making.

use crate::geometry::Polygon;
use super::config::{SpatialConfig, QueryType, SpatialStructureType};
use std::fmt::Debug;

/// **Automatic spatial structure selector**
pub struct SpatialStructureSelector;

impl SpatialStructureSelector {
    /// **Recommend the optimal structure for given data and query type**
    pub fn recommend_structure<S: Clone + Debug + Send + Sync>(
        _polygons: &[Polygon<S>],
        query_type: QueryType
    ) -> SpatialStructureType {
        match query_type {
            QueryType::PointLocation => SpatialStructureType::KDTree,
            QueryType::RangeQuery => SpatialStructureType::Octree,
            QueryType::NearestNeighbor => SpatialStructureType::KDTree,
            QueryType::RayIntersection => SpatialStructureType::BVH,
            QueryType::BooleanOperations => SpatialStructureType::BSP,
        }
    }

    /// **Create optimal configuration for given data and query type**
    pub fn optimal_config<S: Clone + Debug + Send + Sync>(
        polygons: &[Polygon<S>],
        query_type: QueryType
    ) -> SpatialConfig {
        let structure_type = Self::recommend_structure(polygons, query_type);
        
        let mut config = match query_type {
            QueryType::PointLocation => SpatialConfig::for_point_queries(),
            QueryType::RangeQuery => SpatialConfig::for_volume_queries(),
            QueryType::BooleanOperations => SpatialConfig::for_boolean_operations(),
            _ => SpatialConfig::default(),
        };
        
        config.structure_type = Some(structure_type);
        config
    }
}

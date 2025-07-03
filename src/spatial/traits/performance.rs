//! **Performance Analysis Types (The Skeleton)**
//!
//! This module defines performance analysis types and ratings for spatial structures,
//! following Cathedral Engineering principles where performance types represent part of the
//! "skeleton" that provides structural foundation for performance analysis.

use super::config::{QueryType, SpatialStructureType};
use std::collections::HashMap;

/// **Performance rating scale**
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PerformanceRating {
    Excellent = 5,
    Good = 4,
    Average = 3,
    Poor = 2,
    VeryPoor = 1,
}

impl PerformanceRating {
    /// **Convert rating to descriptive string**
    pub fn as_str(&self) -> &'static str {
        match self {
            PerformanceRating::Excellent => "Excellent",
            PerformanceRating::Good => "Good",
            PerformanceRating::Average => "Average",
            PerformanceRating::Poor => "Poor",
            PerformanceRating::VeryPoor => "Very Poor",
        }
    }

    /// **Convert rating to numeric score**
    pub fn score(&self) -> u8 {
        *self as u8
    }
}

/// **Performance characteristics of a spatial structure**
#[derive(Debug, Clone)]
pub struct StructurePerformance {
    pub construction_speed: PerformanceRating,
    pub query_speed: PerformanceRating,
    pub memory_efficiency: PerformanceRating,
    pub update_performance: PerformanceRating,
    pub scalability: PerformanceRating,
}

impl StructurePerformance {
    /// **Calculate overall performance score**
    pub fn overall_score(&self) -> f64 {
        let total = self.construction_speed.score() as f64 +
                   self.query_speed.score() as f64 +
                   self.memory_efficiency.score() as f64 +
                   self.update_performance.score() as f64 +
                   self.scalability.score() as f64;
        total / 5.0
    }

    /// **Get overall performance rating**
    pub fn overall_rating(&self) -> PerformanceRating {
        let score = self.overall_score();
        if score >= 4.5 {
            PerformanceRating::Excellent
        } else if score >= 3.5 {
            PerformanceRating::Good
        } else if score >= 2.5 {
            PerformanceRating::Average
        } else if score >= 1.5 {
            PerformanceRating::Poor
        } else {
            PerformanceRating::VeryPoor
        }
    }
}

/// **Performance matrix for different spatial structures and query types**
#[derive(Debug, Clone)]
pub struct PerformanceMatrix {
    matrix: HashMap<(SpatialStructureType, QueryType), StructurePerformance>,
}

impl Default for PerformanceMatrix {
    fn default() -> Self {
        let mut matrix = HashMap::new();
        
        // KD-Tree performance characteristics
        matrix.insert(
            (SpatialStructureType::KDTree, QueryType::PointLocation),
            StructurePerformance {
                construction_speed: PerformanceRating::Good,
                query_speed: PerformanceRating::Excellent,
                memory_efficiency: PerformanceRating::Good,
                update_performance: PerformanceRating::Poor,
                scalability: PerformanceRating::Good,
            }
        );

        matrix.insert(
            (SpatialStructureType::KDTree, QueryType::RangeQuery),
            StructurePerformance {
                construction_speed: PerformanceRating::Good,
                query_speed: PerformanceRating::Good,
                memory_efficiency: PerformanceRating::Good,
                update_performance: PerformanceRating::Poor,
                scalability: PerformanceRating::Good,
            }
        );

        // Octree performance characteristics
        matrix.insert(
            (SpatialStructureType::Octree, QueryType::RangeQuery),
            StructurePerformance {
                construction_speed: PerformanceRating::Average,
                query_speed: PerformanceRating::Good,
                memory_efficiency: PerformanceRating::Average,
                update_performance: PerformanceRating::Good,
                scalability: PerformanceRating::Excellent,
            }
        );

        // BSP Tree performance characteristics
        matrix.insert(
            (SpatialStructureType::BSP, QueryType::BooleanOperations),
            StructurePerformance {
                construction_speed: PerformanceRating::Poor,
                query_speed: PerformanceRating::Excellent,
                memory_efficiency: PerformanceRating::Poor,
                update_performance: PerformanceRating::VeryPoor,
                scalability: PerformanceRating::Average,
            }
        );

        // BVH performance characteristics
        matrix.insert(
            (SpatialStructureType::BVH, QueryType::RayIntersection),
            StructurePerformance {
                construction_speed: PerformanceRating::Good,
                query_speed: PerformanceRating::Excellent,
                memory_efficiency: PerformanceRating::Good,
                update_performance: PerformanceRating::Average,
                scalability: PerformanceRating::Good,
            }
        );

        Self { matrix }
    }
}

impl PerformanceMatrix {
    /// **Get performance characteristics for a structure-query combination**
    pub fn get_performance(
        &self,
        structure: SpatialStructureType,
        query_type: QueryType
    ) -> Option<&StructurePerformance> {
        self.matrix.get(&(structure, query_type))
    }

    /// **Find the best structure for a specific query type**
    pub fn best_structure_for_query(&self, query_type: QueryType) -> Option<SpatialStructureType> {
        self.matrix
            .iter()
            .filter(|((_, qt), _)| *qt == query_type)
            .max_by(|(_, perf1), (_, perf2)| {
                perf1.overall_score().partial_cmp(&perf2.overall_score()).unwrap()
            })
            .map(|((structure, _), _)| *structure)
    }

    /// **Get all structures ranked by performance for a query type**
    pub fn ranked_structures_for_query(&self, query_type: QueryType) -> Vec<(SpatialStructureType, f64)> {
        let mut results: Vec<_> = self.matrix
            .iter()
            .filter(|((_, qt), _)| *qt == query_type)
            .map(|((structure, _), perf)| (*structure, perf.overall_score()))
            .collect();
        
        results.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        results
    }

    /// **Add or update performance data**
    pub fn set_performance(
        &mut self,
        structure: SpatialStructureType,
        query_type: QueryType,
        performance: StructurePerformance
    ) {
        self.matrix.insert((structure, query_type), performance);
    }
}

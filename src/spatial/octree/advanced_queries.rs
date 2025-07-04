//! **Advanced Spatial Query Operations for Octree**
//!
//! This module provides advanced spatial query operations using iterator patterns
//! with conditional processing, parallel optimization, and aggregation functions.

use super::Node;
use crate::geometry::Polygon;
use crate::spatial::traits::geometry::Aabb;
use std::fmt::Debug;

impl<S: Clone + Send + Sync + Debug> Node<S> {
    /// **Advanced spatial query with conditional processing**
    ///
    /// Performs spatial queries with skip_while() and take_while() patterns
    /// for conditional processing based on spatial criteria.
    pub fn conditional_spatial_query<F, G>(
        &self,
        query_bounds: &Aabb,
        skip_condition: F,
        take_condition: G,
        max_results: usize,
    ) -> Vec<&Polygon<S>>
    where
        F: Fn(&Polygon<S>) -> bool + Copy,
        G: Fn(&Polygon<S>) -> bool + Copy,
    {
        let mut results = Vec::new();
        self.conditional_spatial_recursive(query_bounds, skip_condition, take_condition, max_results, &mut results);
        results
    }

    /// **Recursive implementation of conditional spatial query**
    fn conditional_spatial_recursive<'a, F, G>(
        &'a self,
        query_bounds: &Aabb,
        skip_condition: F,
        take_condition: G,
        max_results: usize,
        results: &mut Vec<&'a Polygon<S>>,
    )
    where
        F: Fn(&Polygon<S>) -> bool + Copy,
        G: Fn(&Polygon<S>) -> bool + Copy,
    {
        // Early termination if we have enough results
        if results.len() >= max_results {
            return;
        }

        // Check if this node's bounds intersect with query bounds
        if !self.bounds.intersects(query_bounds) {
            return;
        }

        // Advanced iterator patterns with conditional processing
        let filtered_polygons: Vec<&Polygon<S>> = self.polygons
            .iter()
            .filter(|polygon| self.polygon_intersects_bounds(polygon, query_bounds))
            .skip_while(|polygon| skip_condition(polygon))
            .take_while(|polygon| take_condition(polygon) && results.len() < max_results)
            .collect();

        results.extend(filtered_polygons);

        // Recursively search children with early termination
        for child in &self.children {
            if results.len() >= max_results {
                break;
            }
            if let Some(child_node) = child {
                child_node.conditional_spatial_recursive(
                    query_bounds,
                    skip_condition,
                    take_condition,
                    max_results,
                    results,
                );
            }
        }
    }

    /// **Batch spatial processing with parallel optimization**
    ///
    /// Processes multiple spatial queries in batch with intelligent
    /// parallel processing based on dataset size.
    #[cfg(feature = "parallel")]
    pub fn batch_spatial_queries<F>(&self, queries: &[Aabb], processor: F) -> Vec<Vec<&Polygon<S>>>
    where
        F: Fn(&Polygon<S>) -> bool + Send + Sync + Copy,
    {
        if queries.len() > 100 {
            use rayon::prelude::*;
            
            // Use parallel processing for large query batches
            queries
                .par_iter()
                .map(|query_bounds| {
                    self.volume_query(query_bounds)
                        .into_iter()
                        .filter(|polygon| processor(polygon))
                        .collect()
                })
                .collect()
        } else {
            // Sequential processing for smaller batches
            queries
                .iter()
                .map(|query_bounds| {
                    self.volume_query(query_bounds)
                        .into_iter()
                        .filter(|polygon| processor(polygon))
                        .collect()
                })
                .collect()
        }
    }

    /// **Spatial aggregation with fold() patterns**
    ///
    /// Performs spatial aggregation operations using advanced iterator patterns
    /// with fold() and reduce() for statistical analysis.
    #[allow(unused_variables)] // reducer is used in parallel feature
    pub fn spatial_aggregation<T, F, R>(&self, query_bounds: &Aabb, initial: T, folder: F, reducer: R) -> T
    where
        T: Clone + Send + Sync,
        F: Fn(T, &Polygon<S>) -> T + Send + Sync + Copy,
        R: Fn(T, T) -> T + Send + Sync + Copy,
    {
        let polygons = self.volume_query(query_bounds);
        
        #[cfg(feature = "parallel")]
        {
            if polygons.len() > 1000 {
                use rayon::prelude::*;
                
                // Use parallel fold and reduce for large datasets
                return polygons
                    .par_iter()
                    .fold(|| initial.clone(), |acc, polygon| folder(acc, polygon))
                    .reduce(|| initial.clone(), |a, b| reducer(a, b));
            }
        }
        
        // Sequential fold for smaller datasets
        polygons
            .iter()
            .fold(initial, |acc, polygon| folder(acc, polygon))
    }
}

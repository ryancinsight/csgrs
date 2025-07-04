//! Non-parallel BSP tree operations (clip, build, slice)

use super::core::Node;
use crate::core::float_types::{EPSILON, Real};
use crate::geometry::{BACK, COPLANAR, FRONT, Plane, Polygon, SPANNING, Vertex};
use std::fmt::Debug;

/// Configuration for BSP tree construction
#[derive(Debug, Clone)]
pub struct BspConfig {
    /// Maximum depth of the tree
    pub max_depth: usize,
    /// Minimum number of polygons per leaf node
    pub min_polygons_per_leaf: usize,
    /// Maximum number of polygons per leaf node before splitting
    pub max_polygons_per_leaf: usize,
    /// Balance factor for plane selection (0.0 to 1.0)
    pub balance_factor: Real,
    /// Spanning factor for plane selection (0.0 to 1.0)
    pub spanning_factor: Real,
}

impl Default for BspConfig {
    fn default() -> Self {
        Self {
            max_depth: 25,
            min_polygons_per_leaf: 1,
            max_polygons_per_leaf: 8,
            balance_factor: 0.5,
            spanning_factor: 0.5,
        }
    }
}

impl<S: Clone + Send + Sync + Debug> Node<S> {
    pub(super) fn pick_best_splitting_plane(&self, polygons: &[Polygon<S>]) -> Plane {
        const K_SPANS: Real = 8.0; // Weight for spanning polygons
        const K_BALANCE: Real = 1.0; // Weight for front/back balance

        let mut best_plane = polygons[0].plane.clone();
        let mut best_score = Real::MAX;

        // Take a sample of polygons as candidate planes
        let sample_size = polygons.len().min(20);
        for p in polygons.iter().take(sample_size) {
            let plane = &p.plane;
            let mut num_front = 0;
            let mut num_back = 0;
            let mut num_spanning = 0;

            for poly in polygons {
                match plane.classify_polygon(poly) {
                    COPLANAR => {}, // Not counted for balance
                    FRONT => num_front += 1,
                    BACK => num_back += 1,
                    SPANNING => num_spanning += 1,
                    _ => num_spanning += 1, // Treat any other combination as spanning
                }
            }

            let score = K_SPANS * num_spanning as Real
                + K_BALANCE * ((num_front - num_back) as Real).abs();

            if score < best_score {
                best_score = score;
                best_plane = plane.clone();
            }
        }
        best_plane
    }

    /// Build a BSP tree from the given polygons
    /// **Mathematical Foundation**: Recursively partition 3D space using hyperplanes,
    /// classifying polygons as FRONT, BACK, COPLANAR, or SPANNING relative to the splitting plane.
    /// **Algorithm**: O(n log n) expected time for balanced trees, O(n²) worst case.
    pub fn build(&mut self, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        // Choose the best splitting plane using a heuristic if not already set.
        if self.plane.is_none() {
            self.plane = Some(self.pick_best_splitting_plane(polygons));
        }
        let plane = self.plane.as_ref().unwrap();

        // Pre-allocate with estimated capacity for better performance
        let mut front = Vec::with_capacity(polygons.len() / 2);
        let mut back = Vec::with_capacity(polygons.len() / 2);

        // Optimized polygon classification using iterator pattern
        // **Mathematical Theorem**: Each polygon is classified relative to the splitting plane
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                plane.split_polygon(polygon);

            // Extend collections efficiently with iterator chains
            self.polygons.extend(coplanar_front);
            self.polygons.extend(coplanar_back);
            front.append(&mut front_parts);
            back.append(&mut back_parts);
        }

        // Build child nodes using lazy initialization pattern for memory efficiency
        if !front.is_empty() {
            self.front
                .get_or_insert_with(|| Box::new(Node::new()))
                .build(&front);
        }

        if !back.is_empty() {
            self.back
                .get_or_insert_with(|| Box::new(Node::new()))
                .build(&back);
        }
    }

    /// Creates a new BSP node from polygons using the appropriate build method
    ///
    /// This function automatically chooses between parallel and non-parallel implementations
    /// based on feature availability and input size.
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        #[cfg(feature = "parallel")]
        {
            if polygons.len() > 100 {
                // Use parallel for larger datasets
                let mut node = Self::new();
                node.build_parallel(polygons);
                return node;
            }
        }

        // Default to non-parallel implementation
        let mut node = Self::new();
        node.build(polygons);
        node
    }

    /// Clips polygons using the appropriate method based on feature availability
    pub fn clip_polygons_auto(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        #[cfg(feature = "parallel")]
        {
            if polygons.len() > 50 {
                return self.clip_polygons_parallel(polygons);
            }
        }

        self.clip_polygons(polygons)
    }

    /// Clips to another BSP tree using the appropriate method
    pub fn clip_to_auto(&mut self, bsp: &Node<S>) {
        #[cfg(feature = "parallel")]
        {
            if self.polygons.len() > 50 {
                self.clip_to_parallel(bsp);
                return;
            }
        }

        self.clip_to(bsp);
    }

    /// Slices the BSP tree using the appropriate method
    pub fn slice_auto(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        #[cfg(feature = "parallel")]
        {
            if self.polygons.len() > 50 {
                return self.slice_parallel(slicing_plane);
            }
        }

        self.slice(slicing_plane)
    }

    /// Recursively remove all polygons in `polygons` that are inside this BSP tree
    /// **Mathematical Foundation**: Uses plane classification to determine polygon visibility.
    /// Polygons entirely in BACK half-space are clipped (removed).
    /// **Algorithm**: O(n log d) where n is polygon count, d is tree depth.
    pub fn clip_polygons(&self, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        if self.plane.is_none() {
            return polygons.to_vec();
        }

        let plane = self.plane.as_ref().unwrap();

        // Pre-allocate for better performance
        let mut front_polys = Vec::with_capacity(polygons.len());
        let mut back_polys = Vec::with_capacity(polygons.len());

        // Optimized polygon splitting with parallel iterator patterns for large datasets
        #[cfg(feature = "parallel")]
        let (all_front_parts, all_back_parts): (Vec<Vec<Polygon<S>>>, Vec<Vec<Polygon<S>>>) = {
            if polygons.len() > 100 {
                use rayon::prelude::*;
                polygons
                    .par_iter()
                    .map(|polygon| {
                        let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                            plane.split_polygon(polygon);

                        // Efficient coplanar polygon classification using iterator chain
                        let (coplanar_front_parts, coplanar_back_parts): (Vec<_>, Vec<_>) = coplanar_front
                            .into_iter()
                            .chain(coplanar_back.into_iter())
                            .partition(|coplanar_poly| plane.orient_plane(&coplanar_poly.plane) == FRONT);

                        front_parts.extend(coplanar_front_parts);
                        back_parts.extend(coplanar_back_parts);

                        (front_parts, back_parts)
                    })
                    .unzip()
            } else {
                polygons
                    .iter()
                    .map(|polygon| {
                        let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                            plane.split_polygon(polygon);

                        // Efficient coplanar polygon classification using iterator chain
                        let (coplanar_front_parts, coplanar_back_parts): (Vec<_>, Vec<_>) = coplanar_front
                            .into_iter()
                            .chain(coplanar_back.into_iter())
                            .partition(|coplanar_poly| plane.orient_plane(&coplanar_poly.plane) == FRONT);

                        front_parts.extend(coplanar_front_parts);
                        back_parts.extend(coplanar_back_parts);

                        (front_parts, back_parts)
                    })
                    .unzip()
            }
        };

        #[cfg(not(feature = "parallel"))]
        let (all_front_parts, all_back_parts): (Vec<Vec<Polygon<S>>>, Vec<Vec<Polygon<S>>>) = polygons
            .iter()
            .map(|polygon| {
                let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                    plane.split_polygon(polygon);

                // Efficient coplanar polygon classification using iterator chain
                let (coplanar_front_parts, coplanar_back_parts): (Vec<_>, Vec<_>) = coplanar_front
                    .into_iter()
                    .chain(coplanar_back.into_iter())
                    .partition(|coplanar_poly| plane.orient_plane(&coplanar_poly.plane) == FRONT);

                front_parts.extend(coplanar_front_parts);
                back_parts.extend(coplanar_back_parts);

                (front_parts, back_parts)
            })
            .unzip();

        // Flatten the collected parts
        front_polys.extend(all_front_parts.into_iter().flatten());
        back_polys.extend(all_back_parts.into_iter().flatten());

        // Recursively clip with optimized pattern
        let mut result = if let Some(front_node) = &self.front {
            front_node.clip_polygons(&front_polys)
        } else {
            front_polys
        };

        if let Some(back_node) = &self.back {
            result.extend(back_node.clip_polygons(&back_polys));
        }

        result
    }

    /// Remove all polygons in this BSP tree that are inside the other BSP tree
    pub fn clip_to(&mut self, bsp: &Node<S>) {
        self.polygons = bsp.clip_polygons(&self.polygons);
        if let Some(ref mut front) = self.front {
            front.clip_to(bsp);
        }
        if let Some(ref mut back) = self.back {
            back.clip_to(bsp);
        }
    }

    /// Slices this BSP node with `slicing_plane`, returning:
    /// - All polygons that are coplanar with the plane (within EPSILON),
    /// - A list of line‐segment intersections (each a [Vertex; 2]) from polygons that span the plane.
    pub fn slice(&self, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let mut coplanar_polygons = Vec::new();
        let mut intersection_edges = Vec::new();
        self.slice_recursive(slicing_plane, &mut coplanar_polygons, &mut intersection_edges);
        (coplanar_polygons, intersection_edges)
    }

    /// Recursive helper for slice operation
    fn slice_recursive(
        &self,
        slicing_plane: &Plane,
        coplanar_polygons: &mut Vec<Polygon<S>>,
        intersection_edges: &mut Vec<[Vertex; 2]>,
    ) {
        for poly in &self.polygons {
            let vcount = poly.vertices.len();
            if vcount < 2 {
                continue; // degenerate polygon => skip
            }

            // Use iterator chain to compute vertex types more efficiently
            let types: Vec<_> = poly
                .vertices
                .iter()
                .map(|vertex| slicing_plane.orient_point(&vertex.pos))
                .collect();

            let polygon_type = types.iter().fold(0, |acc, &vertex_type| acc | vertex_type);

            // Based on the combined classification of its vertices:
            match polygon_type {
                COPLANAR => {
                    // The entire polygon is in the plane, so push it to the coplanar list.
                    coplanar_polygons.push(poly.clone());
                },

                FRONT | BACK => {
                    // Entirely on one side => no intersection. We skip it.
                },

                SPANNING => {
                    // Use iterator chain to collect intersection points more efficiently
                    let crossing_points: Vec<_> = (0..vcount)
                        .filter_map(|i| {
                            let j = (i + 1) % vcount;
                            let ti = types[i];
                            let tj = types[j];
                            let vi = &poly.vertices[i];
                            let vj = &poly.vertices[j];

                            if (ti | tj) == SPANNING {
                                let denom = slicing_plane.normal().dot(&(vj.pos - vi.pos));
                                if denom.abs() > EPSILON {
                                    let intersection = (slicing_plane.offset()
                                        - slicing_plane.normal().dot(&vi.pos.coords))
                                        / denom;
                                    Some(vi.interpolate(vj, intersection))
                                } else {
                                    None
                                }
                            } else {
                                None
                            }
                        })
                        .collect();

                    // Convert crossing points to intersection edges
                    intersection_edges.extend(
                        crossing_points
                            .chunks_exact(2)
                            .map(|chunk| [chunk[0].clone(), chunk[1].clone()]),
                    );
                },
                _ => {},
            }
        }

        if let Some(front) = &self.front {
            front.slice_recursive(slicing_plane, coplanar_polygons, intersection_edges);
        }
        if let Some(back) = &self.back {
            back.slice_recursive(slicing_plane, coplanar_polygons, intersection_edges);
        }
    }

    /// Build BSP tree from polygons using custom configuration
    pub fn from_polygons_with_config(polygons: &[Polygon<S>], config: &BspConfig) -> Self {
        let mut node = Self::new();
        node.build_with_config(polygons, config);
        node
    }

    /// Build BSP tree using custom configuration
    pub fn build_with_config(&mut self, polygons: &[Polygon<S>], config: &BspConfig) {
        self.build_recursive_with_config(polygons, 0, config);
    }

    /// Recursive build with configuration
    fn build_recursive_with_config(&mut self, polygons: &[Polygon<S>], depth: usize, config: &BspConfig) {
        if polygons.is_empty() {
            return;
        }

        // Check termination conditions
        if depth >= config.max_depth || polygons.len() <= config.max_polygons_per_leaf {
            self.polygons = polygons.to_vec();
            return;
        }

        // Pick splitting plane using configuration factors
        self.plane = Some(self.pick_best_splitting_plane_with_config(polygons, config));

        let mut front_polys = Vec::new();
        let mut back_polys = Vec::new();

        for polygon in polygons {
            match self.plane.as_ref().unwrap().classify_polygon(polygon) {
                COPLANAR => self.polygons.push(polygon.clone()),
                FRONT => front_polys.push(polygon.clone()),
                BACK => back_polys.push(polygon.clone()),
                SPANNING => {
                    let (mut coplanar_front, mut coplanar_back, mut front_parts, mut back_parts) =
                        self.plane.as_ref().unwrap().split_polygon(polygon);
                    self.polygons.append(&mut coplanar_front);
                    self.polygons.append(&mut coplanar_back);
                    front_polys.append(&mut front_parts);
                    back_polys.append(&mut back_parts);
                },
                _ => {} // Handle any other values
            }
        }

        // Recursively build children
        if !front_polys.is_empty() {
            let mut front_node = Node::new();
            front_node.build_recursive_with_config(&front_polys, depth + 1, config);
            self.front = Some(Box::new(front_node));
        }

        if !back_polys.is_empty() {
            let mut back_node = Node::new();
            back_node.build_recursive_with_config(&back_polys, depth + 1, config);
            self.back = Some(Box::new(back_node));
        }
    }

    /// Pick best splitting plane using configuration factors
    fn pick_best_splitting_plane_with_config(&self, polygons: &[Polygon<S>], config: &BspConfig) -> Plane {
        // Use advanced iterator patterns with parallel processing for plane evaluation
        #[cfg(feature = "parallel")]
        let best_plane = {
            if polygons.len() > 50 {
                use rayon::prelude::*;
                polygons
                    .par_iter()
                    .map(|polygon| {
                        let plane = &polygon.plane;

                        // Use fold() to accumulate classification counts
                        let (front_count, back_count, spanning_count) = polygons
                            .par_iter()
                            .map(|test_polygon| plane.classify_polygon(test_polygon))
                            .fold(
                                || (0u32, 0u32, 0u32),
                                |(front, back, spanning), classification| match classification {
                                    FRONT => (front + 1, back, spanning),
                                    BACK => (front, back + 1, spanning),
                                    SPANNING => (front, back, spanning + 1),
                                    _ => (front, back, spanning),
                                }
                            )
                            .reduce(
                                || (0u32, 0u32, 0u32),
                                |(f1, b1, s1), (f2, b2, s2)| (f1 + f2, b1 + b2, s1 + s2)
                            );

                        // Calculate score using configuration factors
                        let balance = (front_count as Real - back_count as Real).abs();
                        let spanning = spanning_count as Real;
                        let score = config.balance_factor * balance + config.spanning_factor * spanning;

                        (score, plane.clone())
                    })
                    .min_by(|(score1, _), (score2, _)| {
                        score1.partial_cmp(score2).unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .map(|(_, plane)| plane)
                    .unwrap_or_else(|| polygons[0].plane.clone())
            } else {
                polygons
                    .iter()
                    .map(|polygon| {
                        let plane = &polygon.plane;

                        // Use fold() to accumulate classification counts
                        let (front_count, back_count, spanning_count) = polygons
                            .iter()
                            .map(|test_polygon| plane.classify_polygon(test_polygon))
                            .fold((0u32, 0u32, 0u32), |(front, back, spanning), classification| {
                                match classification {
                                    FRONT => (front + 1, back, spanning),
                                    BACK => (front, back + 1, spanning),
                                    SPANNING => (front, back, spanning + 1),
                                    _ => (front, back, spanning),
                                }
                            });

                        // Calculate score using configuration factors
                        let balance = (front_count as Real - back_count as Real).abs();
                        let spanning = spanning_count as Real;
                        let score = config.balance_factor * balance + config.spanning_factor * spanning;

                        (score, plane.clone())
                    })
                    .min_by(|(score1, _), (score2, _)| {
                        score1.partial_cmp(score2).unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .map(|(_, plane)| plane)
                    .unwrap_or_else(|| polygons[0].plane.clone())
            }
        };

        #[cfg(not(feature = "parallel"))]
        let best_plane = polygons
            .iter()
            .map(|polygon| {
                let plane = &polygon.plane;

                // Use fold() to accumulate classification counts
                let (front_count, back_count, spanning_count) = polygons
                    .iter()
                    .map(|test_polygon| plane.classify_polygon(test_polygon))
                    .fold((0u32, 0u32, 0u32), |(front, back, spanning), classification| {
                        match classification {
                            FRONT => (front + 1, back, spanning),
                            BACK => (front, back + 1, spanning),
                            SPANNING => (front, back, spanning + 1),
                            _ => (front, back, spanning),
                        }
                    });

                // Calculate score using configuration factors
                let balance = (front_count as Real - back_count as Real).abs();
                let spanning = spanning_count as Real;
                let score = config.balance_factor * balance + config.spanning_factor * spanning;

                (score, plane.clone())
            })
            .min_by(|(score1, _), (score2, _)| {
                score1.partial_cmp(score2).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(_, plane)| plane)
            .unwrap_or_else(|| polygons[0].plane.clone());

        best_plane
    }

    /// **Advanced BSP spatial query with early termination**
    ///
    /// Performs spatial queries on BSP tree using advanced iterator patterns
    /// with early termination for optimal performance.
    pub fn spatial_query_with_early_termination<F>(&self, predicate: F) -> Option<&Polygon<S>>
    where
        F: Fn(&Polygon<S>) -> bool + Copy,
    {
        // Use find() for immediate early termination
        self.polygons
            .iter()
            .find(|polygon| predicate(polygon))
            .or_else(|| {
                // Search front child first, then back child with early termination
                self.front
                    .as_ref()
                    .and_then(|child| child.spatial_query_with_early_termination(predicate))
                    .or_else(|| {
                        self.back
                            .as_ref()
                            .and_then(|child| child.spatial_query_with_early_termination(predicate))
                    })
            })
    }

    /// **Conditional BSP traversal with skip_while() and take_while()**
    ///
    /// Traverses BSP tree with conditional processing based on spatial criteria.
    pub fn conditional_traversal<F, G>(
        &self,
        skip_condition: F,
        take_condition: G,
        max_results: usize,
    ) -> Vec<&Polygon<S>>
    where
        F: Fn(&Polygon<S>) -> bool + Copy,
        G: Fn(&Polygon<S>) -> bool + Copy,
    {
        let mut results = Vec::new();
        self.conditional_traversal_recursive(skip_condition, take_condition, max_results, &mut results);
        results
    }

    /// **Recursive implementation of conditional BSP traversal**
    fn conditional_traversal_recursive<'a, F, G>(
        &'a self,
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

        // Process polygons in this node with advanced iterator patterns
        let filtered_polygons: Vec<&Polygon<S>> = self.polygons
            .iter()
            .skip_while(|polygon| skip_condition(polygon))
            .take_while(|polygon| take_condition(polygon) && results.len() < max_results)
            .collect();

        results.extend(filtered_polygons);

        // Recursively traverse children with early termination
        if results.len() < max_results {
            if let Some(ref front_child) = self.front {
                front_child.conditional_traversal_recursive(
                    skip_condition,
                    take_condition,
                    max_results,
                    results,
                );
            }
        }

        if results.len() < max_results {
            if let Some(ref back_child) = self.back {
                back_child.conditional_traversal_recursive(
                    skip_condition,
                    take_condition,
                    max_results,
                    results,
                );
            }
        }
    }

    /// **Parallel BSP spatial filtering with intelligent thresholds**
    ///
    /// Performs parallel spatial filtering for large BSP trees with
    /// automatic threshold-based optimization.
    #[cfg(feature = "parallel")]
    pub fn parallel_bsp_filter<F>(&self, predicate: F) -> usize
    where
        F: Fn(&Polygon<S>) -> bool + Send + Sync + Copy,
    {
        let all_polygons = self.all_polygons();

        if all_polygons.len() > 500 {
            use rayon::prelude::*;

            // Use parallel iterator for large datasets - return count instead of references
            all_polygons
                .par_iter()
                .filter(|polygon| predicate(polygon))
                .count()
        } else {
            // Use sequential iterator for smaller datasets
            all_polygons
                .iter()
                .filter(|polygon| predicate(polygon))
                .count()
        }
    }
}

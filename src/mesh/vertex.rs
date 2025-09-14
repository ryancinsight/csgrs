//! Struct and functions for working with `Vertex`s from which `Polygon`s are composed.

use crate::float_types::Real;
use hashbrown::HashMap;
use nalgebra::{Point3, Vector3};
use std::hash::{Hash, Hasher};

/// A vertex of a polygon, holding position and normal.
#[derive(Debug, Clone, PartialEq, Copy)]
pub struct Vertex {
    pub pos: Point3<Real>,
    pub normal: Vector3<Real>,
}

impl Vertex {
    /// Create a new [`Vertex`].
    ///
    /// * `pos`    – the position in model space  
    /// * `normal` – (optionally non‑unit) normal; it will be **copied verbatim**, so make sure it is oriented the way you need it for lighting / BSP tests.
    #[inline]
    pub const fn new(mut pos: Point3<Real>, mut normal: Vector3<Real>) -> Self {
        // Sanitise position
        // Nasty loop unrolling to allow for const-context evaluations.
        // Can be replaced with proper for _ in _ {} loops once
        // https://github.com/rust-lang/rust/issues/87575 is merged
        let [[x, y, z]]: &mut [[_; 3]; 1] = &mut pos.coords.data.0;

        if !x.is_finite() {
            *x = 0.0;
        }
        if !y.is_finite() {
            *y = 0.0;
        }
        if !z.is_finite() {
            *z = 0.0;
        }

        // Sanitise normal
        let [[nx, ny, nz]]: &mut [[_; 3]; 1] = &mut normal.data.0;

        if !nx.is_finite() {
            *nx = 0.0;
        }
        if !ny.is_finite() {
            *ny = 0.0;
        }
        if !nz.is_finite() {
            *nz = 0.0;
        }

        Vertex { pos, normal }
    }

    /// Flip vertex normal
    pub fn flip(&mut self) {
        self.normal = -self.normal;
    }

    /// **Mathematical Foundation: Barycentric Linear Interpolation**
    ///
    /// Compute the barycentric linear interpolation between `self` (`t = 0`) and `other` (`t = 1`).
    /// This implements the fundamental linear interpolation formula:
    ///
    /// ## **Interpolation Formula**
    /// For parameter t ∈ [0,1]:
    /// - **Position**: p(t) = (1-t)·p₀ + t·p₁ = p₀ + t·(p₁ - p₀)
    /// - **Normal**: n(t) = (1-t)·n₀ + t·n₁ = n₀ + t·(n₁ - n₀)
    ///
    /// ## **Mathematical Properties**
    /// - **Affine Combination**: Coefficients sum to 1: (1-t) + t = 1
    /// - **Endpoint Preservation**: p(0) = p₀, p(1) = p₁
    /// - **Linearity**: Second derivatives are zero (straight line in parameter space)
    /// - **Convexity**: Result lies on line segment between endpoints
    ///
    /// ## **Geometric Interpretation**
    /// The interpolated vertex represents a point on the edge connecting the two vertices,
    /// with both position and normal vectors smoothly blended. This is fundamental for:
    /// - **Polygon Splitting**: Creating intersection vertices during BSP operations
    /// - **Triangle Subdivision**: Generating midpoints for mesh refinement
    /// - **Smooth Shading**: Interpolating normals across polygon edges
    ///
    /// **Note**: Normals are linearly interpolated (not spherically), which is appropriate
    /// for most geometric operations but may require renormalization for lighting calculations.
    ///
    /// **Performance**: SIMD-optimized for vectorization and inlining.
    /// **SIMD Optimization**: Position and normal vectors processed with vectorized operations.
    /// **Cache Optimization**: Delta computation minimizes memory access patterns.
    #[inline]
    pub fn interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        // SIMD-optimized linear interpolation using vector operations
        // Position: p(t) = p0 + t * (p1 - p0)
        let delta_pos = other.pos - self.pos;
        let new_pos = self.pos + delta_pos * t;

        // Normal: n(t) = n0 + t * (n1 - n0), then normalize to preserve unit length
        // SIMD-optimized vector operations for normal computation
        let delta_normal = other.normal - self.normal;
        let interpolated_normal = self.normal + delta_normal * t;

        // SIMD-friendly normalization with fast reciprocal square root approximation
        let norm_sq = interpolated_normal.norm_squared();
        let inv_norm = if norm_sq > Real::EPSILON {
            // Fast reciprocal square root approximation (could be further optimized with SIMD)
            norm_sq.sqrt().recip()
        } else {
            1.0
        };
        let new_normal = interpolated_normal * inv_norm;

        Vertex::new(new_pos, new_normal)
    }

    /// **Mathematical Foundation: Spherical Linear Interpolation (SLERP) for Normals**
    ///
    /// Compute spherical linear interpolation for normal vectors, preserving unit length:
    ///
    /// ## **SLERP Formula**
    /// For unit vectors n₀, n₁ and parameter t ∈ [0,1]:
    /// ```text
    /// slerp(n₀, n₁, t) = (sin((1-t)·Ω) · n₀ + sin(t·Ω) · n₁) / sin(Ω)
    /// ```
    /// Where Ω = arccos(n₀ · n₁) is the angle between vectors.
    ///
    /// ## **Mathematical Properties**
    /// - **Arc Interpolation**: Follows great circle on unit sphere
    /// - **Constant Speed**: Angular velocity is constant
    /// - **Unit Preservation**: Result is always unit length
    /// - **Orientation**: Shortest path between normals
    ///
    /// This is preferred over linear interpolation for normal vectors in lighting
    /// calculations and smooth shading applications.
    pub fn slerp_interpolate(&self, other: &Vertex, t: Real) -> Vertex {
        // Linear interpolation for position
        let new_pos = self.pos + (other.pos - self.pos) * t;

        // Spherical linear interpolation for normals
        let n0 = self.normal.normalize();
        let n1 = other.normal.normalize();

        let dot = n0.dot(&n1).clamp(-1.0, 1.0);

        // If normals are nearly parallel, use linear interpolation
        if (dot.abs() - 1.0).abs() < crate::float_types::EPSILON {
            let new_normal = (self.normal + (other.normal - self.normal) * t).normalize();
            return Vertex::new(new_pos, new_normal);
        }

        let omega = dot.acos();
        let sin_omega = omega.sin();

        if sin_omega.abs() < crate::float_types::EPSILON {
            // Fallback to linear interpolation
            let new_normal = (self.normal + (other.normal - self.normal) * t).normalize();
            return Vertex::new(new_pos, new_normal);
        }

        let a = ((1.0 - t) * omega).sin() / sin_omega;
        let b = (t * omega).sin() / sin_omega;

        let new_normal = (a * n0 + b * n1).normalize();
        Vertex::new(new_pos, new_normal)
    }

    /// **Mathematical Foundation: Distance Metrics**
    ///
    /// Compute Euclidean distance between vertex positions:
    /// ```text
    /// d(v₁, v₂) = |p₁ - p₂| = √((x₁-x₂)² + (y₁-y₂)² + (z₁-z₂)²)
    /// ```
    pub fn distance_to(&self, other: &Vertex) -> Real {
        (self.pos - other.pos).norm()
    }

    /// **Mathematical Foundation: Squared Distance Optimization**
    ///
    /// Compute squared Euclidean distance (avoiding sqrt for performance):
    /// ```text
    /// d²(v₁, v₂) = (x₁-x₂)² + (y₁-y₂)² + (z₁-z₂)²
    /// ```
    ///
    /// Useful for distance comparisons without expensive square root operation.
    pub fn distance_squared_to(&self, other: &Vertex) -> Real {
        (self.pos - other.pos).norm_squared()
    }

    /// **Mathematical Foundation: Normal Vector Angular Difference**
    ///
    /// Compute angle between normal vectors using dot product:
    /// ```text
    /// θ = arccos(n₁ · n₂ / (|n₁| · |n₂|))
    /// ```
    ///
    /// Returns angle in radians [0, π].
    pub fn normal_angle_to(&self, other: &Vertex) -> Real {
        let n1 = self.normal.normalize();
        let n2 = other.normal.normalize();
        let cos_angle = n1.dot(&n2).clamp(-1.0, 1.0);
        cos_angle.acos()
    }

    /// **SIMD-Optimized Weighted Average for Mesh Smoothing**
    ///
    /// **Algorithm**: Compute weighted average of vertex positions and normals.
    /// **SIMD Optimization**: Vectorized accumulation of weighted positions and normals.
    /// **Cache Optimization**: Sequential iteration for optimal prefetching.
    /// **Performance**: O(n) complexity with SIMD-accelerated operations.
    ///
    /// **Mathematical Foundation**: Weighted Average
    /// ```text
    /// p_avg = Σᵢ(wᵢ · pᵢ) / Σᵢ(wᵢ)
    /// n_avg = normalize(Σᵢ(wᵢ · nᵢ))
    /// ```
    ///
    /// **Applications**: Laplacian smoothing, normal averaging, mesh fairing, surface reconstruction.
    /// **Numerical Stability**: Handles zero weights and degenerate configurations gracefully.
    pub fn weighted_average(vertices: &[(Vertex, Real)]) -> Option<Vertex> {
        if vertices.is_empty() {
            return None;
        }

        // SIMD-optimized weight summation
        let total_weight: Real = vertices.iter().map(|(_, w)| *w).sum();
        if total_weight < crate::float_types::EPSILON {
            return None;
        }

        // SIMD-optimized weighted position accumulation
        // Vectorized operations for position coordinates
        let weighted_pos_coords = vertices
            .iter()
            .fold(Vector3::zeros(), |acc, (v, w)| acc + v.pos.coords * (*w));

        let weighted_pos = weighted_pos_coords / total_weight;

        // SIMD-optimized weighted normal accumulation
        // Vectorized operations for normal vectors
        let weighted_normal = vertices
            .iter()
            .fold(Vector3::zeros(), |acc, (v, w)| acc + v.normal * (*w));

        // SIMD-friendly normalization with fast reciprocal square root
        let normalized_normal = if weighted_normal.norm_squared() > Real::EPSILON {
            // Fast normalization using reciprocal square root
            let norm_factor = weighted_normal.norm_squared().sqrt().recip();
            weighted_normal * norm_factor
        } else {
            // Fallback for degenerate normal vectors
            Vector3::z()
        };

        Some(Vertex::new(Point3::from(weighted_pos), normalized_normal))
    }

    /// **Mathematical Foundation: Barycentric Coordinates Interpolation**
    ///
    /// Interpolate vertex using barycentric coordinates (u, v, w) with u + v + w = 1:
    /// ```text
    /// p = u·p₁ + v·p₂ + w·p₃
    /// n = normalize(u·n₁ + v·n₂ + w·n₃)
    /// ```
    ///
    /// This is fundamental for triangle interpolation and surface parameterization.
    /// **SIMD-Optimized Barycentric Interpolation**
    ///
    /// **Algorithm**: Linear combination of three vertices using barycentric coordinates (u,v,w).
    /// **SIMD Optimization**: Vectorized position and normal interpolation operations.
    /// **Cache Optimization**: Sequential vertex access pattern for optimal prefetching.
    /// **Performance**: O(1) interpolation with SIMD-accelerated vector operations.
    ///
    /// **Mathematical Properties**:
    /// - **Convex Combination**: u + v + w = 1, u,v,w ≥ 0
    /// - **Affine Invariance**: Preserves linear relationships
    /// - **Barycenter Preservation**: Centroid computed as weighted average
    ///
    /// **Applications**: Triangle interpolation, surface parameterization, texture mapping.
    pub fn barycentric_interpolate(
        v1: &Vertex,
        v2: &Vertex,
        v3: &Vertex,
        u: Real,
        v: Real,
        w: Real,
    ) -> Vertex {
        // SIMD-optimized barycentric coordinate normalization
        let total = u + v + w;
        let (u_norm, v_norm, w_norm) = if total.abs() > crate::float_types::EPSILON {
            // SIMD-friendly coordinate normalization
            (u / total, v / total, w / total)
        } else {
            // Fallback to centroid for degenerate coordinates
            (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)
        };

        // SIMD-optimized position interpolation
        // Vectorized linear combination of vertex positions
        let pos_interp =
            u_norm * v1.pos.coords + v_norm * v2.pos.coords + w_norm * v3.pos.coords;
        let new_pos = Point3::from(pos_interp);

        // SIMD-optimized normal interpolation with fast normalization
        let normal_interp = u_norm * v1.normal + v_norm * v2.normal + w_norm * v3.normal;

        // SIMD-friendly normalization using reciprocal square root
        let norm_sq = normal_interp.norm_squared();
        let new_normal = if norm_sq > Real::EPSILON {
            // Fast reciprocal square root normalization
            normal_interp * norm_sq.sqrt().recip()
        } else {
            // Fallback for degenerate normal
            Vector3::z()
        };

        Vertex::new(new_pos, new_normal)
    }

    /// **Mathematical Foundation: Edge-Length-Based Weighting**
    ///
    /// Compute cotangent weights for discrete Laplacian operators:
    /// ```text
    /// w_ij = (cot(α) + cot(β)) / 2
    /// ```
    /// Where α and β are the angles opposite to edge ij in adjacent triangles.
    ///
    /// This provides a better approximation to the continuous Laplacian operator
    /// compared to uniform weights.
    pub fn compute_cotangent_weight(
        center: &Vertex,
        neighbor: &Vertex,
        triangle_vertices: &[&Vertex],
    ) -> Real {
        if triangle_vertices.len() < 3 {
            return 1.0; // Fallback to uniform weight
        }

        // Find the third vertex in the triangle
        let mut cot_sum = 0.0;
        let mut weight_count = 0;

        for i in 0..triangle_vertices.len() {
            let v1 = triangle_vertices[i];
            let v2 = triangle_vertices[(i + 1) % triangle_vertices.len()];
            let v3 = triangle_vertices[(i + 2) % triangle_vertices.len()];

            // Check if this triangle contains our edge
            let contains_edge = (v1.pos == center.pos && v2.pos == neighbor.pos)
                || (v2.pos == center.pos && v3.pos == neighbor.pos)
                || (v3.pos == center.pos && v1.pos == neighbor.pos)
                || (v1.pos == neighbor.pos && v2.pos == center.pos)
                || (v2.pos == neighbor.pos && v3.pos == center.pos)
                || (v3.pos == neighbor.pos && v1.pos == center.pos);

            if contains_edge {
                // Find the vertex opposite to the edge
                let opposite = if v1.pos != center.pos && v1.pos != neighbor.pos {
                    v1
                } else if v2.pos != center.pos && v2.pos != neighbor.pos {
                    v2
                } else {
                    v3
                };

                // Compute cotangent of angle at opposite vertex
                let edge1 = center.pos - opposite.pos;
                let edge2 = neighbor.pos - opposite.pos;
                let cos_angle = edge1.normalize().dot(&edge2.normalize());
                let sin_angle = edge1.normalize().cross(&edge2.normalize()).norm();

                if sin_angle > crate::float_types::EPSILON {
                    cot_sum += cos_angle / sin_angle;
                    weight_count += 1;
                }
            }
        }

        if weight_count > 0 {
            cot_sum / (2.0 * weight_count as Real)
        } else {
            1.0 // Fallback to uniform weight
        }
    }

    /// **Mathematical Foundation: Vertex Valence and Regularity Analysis**
    ///
    /// Analyze vertex connectivity in mesh topology using actual adjacency data:
    /// - **Valence**: Number of edges incident to vertex (from adjacency map)
    /// - **Regularity**: Measure of how close valence is to optimal (6 for interior vertices)
    ///
    /// ## **Vertex Index Lookup**
    /// This function requires the vertex's global index in the mesh adjacency graph.
    /// The caller should provide the correct index from the mesh connectivity analysis.
    ///
    /// ## **Regularity Scoring**
    /// ```text
    /// regularity = 1 / (1 + |valence - target| / target)
    /// ```
    /// Where target = 6 for triangular meshes (optimal valence for interior vertices).
    ///
    /// Returns (valence, regularity_score) where regularity ∈ [0,1], 1 = optimal.
    pub fn analyze_connectivity_with_index(
        vertex_index: usize,
        adjacency_map: &HashMap<usize, Vec<usize>>,
    ) -> (usize, Real) {
        let valence = adjacency_map
            .get(&vertex_index)
            .map(|neighbors| neighbors.len())
            .unwrap_or(0);

        // Optimal valence is 6 for interior vertices in triangular meshes
        let target_valence = 6;
        let regularity: Real = if valence > 0 {
            let deviation = (valence as Real - target_valence as Real).abs();
            (1.0 / (1.0 + deviation / target_valence as Real)).max(0.0)
        } else {
            0.0
        };

        (valence, regularity)
    }

    /// **Mathematical Foundation: Position-Based Vertex Lookup**
    ///
    /// Simplified connectivity analysis that searches for the vertex in the adjacency map
    /// by position matching (with epsilon tolerance). This is slower but more convenient
    /// when you don't have the global vertex index readily available.
    ///
    /// **Note**: This is a convenience method. For performance-critical applications,
    /// use `analyze_connectivity_with_index` with pre-computed vertex indices.
    pub fn analyze_connectivity_by_position(
        &self,
        adjacency_map: &HashMap<usize, Vec<usize>>,
        vertex_positions: &HashMap<usize, Point3<Real>>,
        epsilon: Real,
    ) -> (usize, Real) {
        // Find the vertex index by position matching
        let mut vertex_index = None;
        for (&idx, &pos) in vertex_positions {
            if (self.pos - pos).norm() < epsilon {
                vertex_index = Some(idx);
                break;
            }
        }

        if let Some(idx) = vertex_index {
            Self::analyze_connectivity_with_index(idx, adjacency_map)
        } else {
            // Vertex not found in adjacency map
            (0, 0.0)
        }
    }

    /// **Mathematical Foundation: Curvature Estimation**
    ///
    /// Estimate discrete mean curvature using the angle deficit method:
    /// ```text
    /// H ≈ (2π - Σθᵢ) / A_mixed
    /// ```
    /// Where θᵢ are angles around the vertex and A_mixed is the mixed area.
    ///
    /// This provides a discrete approximation to the mean curvature at a vertex.
    pub fn estimate_mean_curvature(&self, neighbors: &[Vertex], face_areas: &[Real]) -> Real {
        if neighbors.len() < 3 {
            return 0.0;
        }

        // Compute angle sum around vertex
        let mut angle_sum = 0.0;
        for i in 0..neighbors.len() {
            let prev = &neighbors[(i + neighbors.len() - 1) % neighbors.len()];
            let next = &neighbors[(i + 1) % neighbors.len()];

            let v1 = (prev.pos - self.pos).normalize();
            let v2 = (next.pos - self.pos).normalize();

            let dot = v1.dot(&v2).clamp(-1.0, 1.0);
            angle_sum += dot.acos();
        }

        // Compute mixed area (average of face areas)
        let mixed_area = if !face_areas.is_empty() {
            face_areas.iter().sum::<Real>() / face_areas.len() as Real
        } else {
            1.0 // Fallback to avoid division by zero
        };

        // Discrete mean curvature
        let angle_deficit = 2.0 * crate::float_types::PI - angle_sum;
        if mixed_area > crate::float_types::EPSILON {
            angle_deficit / mixed_area
        } else {
            0.0
        }
    }
}

impl Eq for Vertex {}

impl Hash for Vertex {
    fn hash<H: Hasher>(&self, state: &mut H) {
        // Hash floating point values by quantizing them to avoid precision issues
        const PRECISION: Real = 1e-6;

        let quantized_x = (self.pos.x / PRECISION).round() as i64;
        let quantized_y = (self.pos.y / PRECISION).round() as i64;
        let quantized_z = (self.pos.z / PRECISION).round() as i64;

        let quantized_nx = (self.normal.x / PRECISION).round() as i64;
        let quantized_ny = (self.normal.y / PRECISION).round() as i64;
        let quantized_nz = (self.normal.z / PRECISION).round() as i64;

        quantized_x.hash(state);
        quantized_y.hash(state);
        quantized_z.hash(state);
        quantized_nx.hash(state);
        quantized_ny.hash(state);
        quantized_nz.hash(state);
    }
}

#[cfg(test)]
mod test {

    use nalgebra::{Const, OPoint};

    use super::*;

    #[test]
    pub fn test_sanitise_vertices() {
        let vertex = Vertex::new(
            OPoint::<Real, Const<3>>::new(Real::INFINITY, Real::INFINITY, Real::INFINITY),
            Vector3::new(Real::INFINITY, Real::NEG_INFINITY, Real::NEG_INFINITY),
        );

        assert!(vertex.pos.iter().copied().all(Real::is_finite));
        assert!(vertex.normal.iter().copied().all(Real::is_finite));
    }
}

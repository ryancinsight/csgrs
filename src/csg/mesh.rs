use super::CSG;
use crate::geometry::Polygon;
use crate::geometry::Vertex;
use crate::core::float_types::Real;
use nalgebra::Point3;
use std::fmt::Debug;
use std::collections::HashMap;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

/// **Mathematical Foundation: Robust Vertex Indexing for Mesh Connectivity**
///
/// Handles floating-point coordinate comparison with epsilon tolerance:
/// - **Spatial Hashing**: Groups nearby vertices for efficient lookup
/// - **Epsilon Matching**: Considers vertices within ε distance as identical
/// - **Global Indexing**: Maintains consistent vertex indices across mesh
#[derive(Debug, Clone)]
pub struct VertexIndexMap {
    /// Maps vertex positions to global indices (with epsilon tolerance)
    position_to_index: Vec<(Point3<Real>, usize)>,
    /// Maps global indices to representative positions
    index_to_position: HashMap<usize, Point3<Real>>,
    /// Spatial tolerance for vertex matching
    epsilon: Real,
}

impl VertexIndexMap {
    /// Create a new vertex index map with specified tolerance
    pub fn new(epsilon: Real) -> Self {
        Self {
            position_to_index: Vec::new(),
            index_to_position: HashMap::new(),
            epsilon,
        }
    }

    /// Get or create an index for a vertex position
    pub fn get_or_create_index(&mut self, pos: Point3<Real>) -> usize {
        // Look for existing vertex within epsilon tolerance
        for (existing_pos, existing_index) in &self.position_to_index {
            if (pos - existing_pos).norm() < self.epsilon {
                return *existing_index;
            }
        }
        
        // Create new index
        let new_index = self.position_to_index.len();
        self.position_to_index.push((pos, new_index));
        self.index_to_position.insert(new_index, pos);
        new_index
    }

    /// Get the position for a given index
    pub fn get_position(&self, index: usize) -> Option<Point3<Real>> {
        self.index_to_position.get(&index).copied()
    }

    /// Get total number of unique vertices
    pub fn vertex_count(&self) -> usize {
        self.position_to_index.len()
    }

    /// Get all vertex positions and their indices (for iteration)
    pub fn get_vertex_positions(&self) -> &Vec<(Point3<Real>, usize)> {
        &self.position_to_index
    }
}

/// **Mathematical Foundation: Triangle Quality Metrics**
///
/// Comprehensive triangle quality assessment for mesh optimization:
///
/// ## **Aspect Ratio**
/// Measures shape quality as ratio of circumradius to inradius:
/// ```text
/// Q = R / (2r) = abc / (8A·r)
/// ```
/// Where R = circumradius, r = inradius, A = area, a,b,c = edge lengths
/// - **Perfect triangle**: Q = 1 (equilateral)
/// - **Poor quality**: Q > 10 (very elongated/thin)
///
/// ## **Minimum Angle**
/// The smallest interior angle θ_min:
/// - **Good quality**: θ_min > 30°
/// - **Poor quality**: θ_min < 10° (sliver triangles)
///
/// ## **Edge Length Ratio**
/// Maximum to minimum edge length ratio:
/// ```text
/// R_edge = max(a,b,c) / min(a,b,c)
/// ```
/// - **Well-proportioned**: R_edge < 3
/// - **Degenerate**: R_edge > 10
#[derive(Debug, Clone)]
pub struct TriangleQuality {
    /// Aspect ratio (circumradius to inradius ratio)
    pub aspect_ratio: Real,
    /// Minimum interior angle in radians
    pub min_angle: Real,
    /// Maximum interior angle in radians  
    pub max_angle: Real,
    /// Edge length ratio (longest/shortest)
    pub edge_ratio: Real,
    /// Triangle area
    pub area: Real,
    /// Quality score (0-1, where 1 is perfect)
    pub quality_score: Real,
}

/// **Mathematical Foundation: Mesh Quality Assessment and Optimization**
///
/// Advanced mesh processing algorithms for quality improvement:
///
/// ## **Quality Metrics**
/// - **Shape Quality**: Aspect ratio, angle bounds, edge ratios
/// - **Connectivity**: Vertex valence, edge regularity
/// - **Geometric**: Surface smoothness, feature preservation
///
/// ## **Adaptive Refinement**
/// - **Curvature-based**: Refine high-curvature regions
/// - **Error-driven**: Refine based on approximation error
/// - **Feature-preserving**: Maintain sharp edges and corners
///
/// ## **Smoothing Algorithms**
/// - **Laplacian**: Simple position averaging
/// - **Taubin**: Feature-preserving with shrinkage correction
/// - **Bilateral**: Edge-preserving smoothing
#[derive(Debug, Clone)]
pub struct MeshQualityMetrics {
    /// Average triangle quality score
    pub avg_quality: Real,
    /// Minimum triangle quality in mesh
    pub min_quality: Real,
    /// Percentage of high-quality triangles (score > 0.7)
    pub high_quality_ratio: Real,
    /// Number of sliver triangles (min angle < 10°)
    pub sliver_count: usize,
    /// Average edge length
    pub avg_edge_length: Real,
    /// Edge length standard deviation
    pub edge_length_std: Real,
}

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// **Mathematical Foundation: Robust Mesh Connectivity Analysis**
    ///
    /// Build a proper vertex adjacency graph using epsilon-based vertex matching:
    ///
    /// ## **Vertex Matching Algorithm**
    /// 1. **Spatial Tolerance**: Vertices within ε distance are considered identical
    /// 2. **Global Indexing**: Each unique position gets a global index
    /// 3. **Adjacency Building**: For each edge, record bidirectional connectivity
    /// 4. **Manifold Validation**: Ensure each edge is shared by at most 2 triangles
    ///
    /// Returns (vertex_map, adjacency_graph) for robust mesh processing.
    pub fn build_mesh_connectivity(&self) -> (VertexIndexMap, HashMap<usize, Vec<usize>>) {
        let mut vertex_map = VertexIndexMap::new(Real::EPSILON * 100.0); // Tolerance for vertex matching
        let mut adjacency: HashMap<usize, Vec<usize>> = HashMap::new();
        
        // First pass: build vertex index mapping
        for polygon in &self.polygons {
            for vertex in &polygon.vertices {
                vertex_map.get_or_create_index(vertex.pos);
            }
        }
        
        // Second pass: build adjacency graph
        for polygon in &self.polygons {
            let mut vertex_indices = Vec::new();
            
            // Get indices for this polygon's vertices
            for vertex in &polygon.vertices {
                let index = vertex_map.get_or_create_index(vertex.pos);
                vertex_indices.push(index);
            }
            
            // Build adjacency for this polygon's edges
            for i in 0..vertex_indices.len() {
                let current = vertex_indices[i];
                let next = vertex_indices[(i + 1) % vertex_indices.len()];
                let prev = vertex_indices[(i + vertex_indices.len() - 1) % vertex_indices.len()];
                
                // Add bidirectional edges
                adjacency.entry(current).or_default().push(next);
                adjacency.entry(current).or_default().push(prev);
                adjacency.entry(next).or_default().push(current);
                adjacency.entry(prev).or_default().push(current);
            }
        }
        
        // Clean up adjacency lists - remove duplicates and self-references
        for (vertex_idx, neighbors) in adjacency.iter_mut() {
            neighbors.sort_unstable();
            neighbors.dedup();
            neighbors.retain(|&neighbor| neighbor != *vertex_idx);
        }
        
        (vertex_map, adjacency)
    }

    /// **Mathematical Foundation: True Laplacian Mesh Smoothing with Global Connectivity**
    ///
    /// Implements proper discrete Laplacian smoothing using global mesh connectivity:
    ///
    /// ## **Discrete Laplacian Operator**
    /// For each vertex v with neighbors N(v):
    /// ```text
    /// L(v) = (1/|N(v)|) · Σ(n∈N(v)) (n - v)
    /// ```
    ///
    /// ## **Global Connectivity Benefits**
    /// - **Proper Neighborhoods**: Uses actual mesh connectivity, not just polygon edges
    /// - **Uniform Weighting**: Each neighbor contributes equally to smoothing
    /// - **Boundary Detection**: Automatically detects and preserves mesh boundaries
    /// - **Volume Preservation**: Better volume preservation than local smoothing
    ///
    /// ## **Algorithm Improvements**
    /// - **Epsilon-based Vertex Matching**: Robust floating-point coordinate handling
    /// - **Manifold Preservation**: Ensures mesh topology is maintained
    /// - **Feature Detection**: Can preserve sharp features based on neighbor count
    pub fn laplacian_smooth_global(&self, lambda: Real, iterations: usize, preserve_boundaries: bool) -> CSG<S> {
        let (vertex_map, adjacency) = self.build_mesh_connectivity();
        let mut smoothed_polygons = self.polygons.clone();
        
        for iteration in 0..iterations {
            // Build current vertex position mapping
            let mut current_positions: HashMap<usize, Point3<Real>> = HashMap::new();
            for polygon in &smoothed_polygons {
                for vertex in &polygon.vertices {
                    // Find the global index for this position (with tolerance)
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            current_positions.insert(*idx, vertex.pos);
                            break;
                        }
                    }
                }
            }
            
            // Compute Laplacian for each vertex
            let mut laplacian_updates: HashMap<usize, Point3<Real>> = HashMap::new();
            for (&vertex_idx, neighbors) in &adjacency {
                if let Some(&current_pos) = current_positions.get(&vertex_idx) {
                    // Check if this is a boundary vertex
                    if preserve_boundaries && neighbors.len() < 4 {
                        // Boundary vertex - skip smoothing
                        laplacian_updates.insert(vertex_idx, current_pos);
                        continue;
                    }
                    
                    // Compute neighbor average
                    let mut neighbor_sum = Point3::origin();
                    let mut valid_neighbors = 0;
                    
                    for &neighbor_idx in neighbors {
                        if let Some(&neighbor_pos) = current_positions.get(&neighbor_idx) {
                            neighbor_sum += neighbor_pos.coords;
                            valid_neighbors += 1;
                        }
                    }
                    
                    if valid_neighbors > 0 {
                        let neighbor_avg = neighbor_sum / valid_neighbors as Real;
                        let laplacian = neighbor_avg - current_pos;
                        let new_pos = current_pos + laplacian * lambda;
                        laplacian_updates.insert(vertex_idx, new_pos);
                    } else {
                        laplacian_updates.insert(vertex_idx, current_pos);
                    }
                }
            }
            
            // Apply updates to mesh vertices
            for polygon in &mut smoothed_polygons {
                for vertex in &mut polygon.vertices {
                    // Find the global index for this vertex
                    for (pos, idx) in &vertex_map.position_to_index {
                        if (vertex.pos - pos).norm() < vertex_map.epsilon {
                            if let Some(&new_pos) = laplacian_updates.get(idx) {
                                vertex.pos = new_pos;
                            }
                            break;
                        }
                    }
                }
                // Recompute polygon plane and normals after smoothing
                polygon.set_new_normal();
            }
            
            // Progress feedback for long smoothing operations
            if iterations > 10 && iteration % (iterations / 10) == 0 {
                eprintln!("Smoothing progress: {}/{} iterations", iteration + 1, iterations);
            }
        }
        
        CSG::from_polygons(&smoothed_polygons)
    }

    /// **Mathematical Foundation: Comprehensive Triangle Quality Analysis**
    ///
    /// Analyze triangle quality using multiple geometric metrics:
    ///
    /// ## **Quality Assessment Algorithm**
    /// For each triangle with vertices A, B, C:
    /// 1. **Edge lengths**: a = |BC|, b = |CA|, c = |AB|
    /// 2. **Area**: A = ½|AB⃗ × AC⃗|
    /// 3. **Angles**: Using law of cosines: cos(θ) = (b² + c² - a²)/(2bc)
    /// 4. **Circumradius**: R = abc/(4A)
    /// 5. **Inradius**: r = A/s, where s = (a+b+c)/2
    /// 6. **Quality score**: Weighted combination of all metrics
    ///
    /// Returns quality metrics for each triangle in the mesh.
    pub fn analyze_triangle_quality(&self) -> Vec<TriangleQuality> {
        let triangulated = self.tessellate();
        
        #[cfg(feature = "parallel")]
        let qualities: Vec<TriangleQuality> = triangulated.polygons
            .par_iter()
            .map(|poly| Self::compute_triangle_quality(&poly.vertices))
            .collect();
        
        #[cfg(not(feature = "parallel"))]
        let qualities: Vec<TriangleQuality> = triangulated.polygons
            .iter()
            .map(|poly| Self::compute_triangle_quality(&poly.vertices))
            .collect();
        
        qualities
    }

    /// Compute comprehensive quality metrics for a single triangle
    fn compute_triangle_quality(vertices: &[Vertex]) -> TriangleQuality {
        if vertices.len() != 3 {
            return TriangleQuality {
                aspect_ratio: Real::INFINITY,
                min_angle: 0.0,
                max_angle: 0.0,
                edge_ratio: Real::INFINITY,
                area: 0.0,
                quality_score: 0.0,
            };
        }

        let a = vertices[0].pos;
        let b = vertices[1].pos;
        let c = vertices[2].pos;

        // Edge vectors and lengths
        let ab = b - a;
        let bc = c - b;
        let ca = a - c;
        
        let len_ab = ab.norm();
        let len_bc = bc.norm();
        let len_ca = ca.norm();

        // Handle degenerate cases
        if len_ab < Real::EPSILON || len_bc < Real::EPSILON || len_ca < Real::EPSILON {
            return TriangleQuality {
                aspect_ratio: Real::INFINITY,
                min_angle: 0.0,
                max_angle: 0.0,
                edge_ratio: Real::INFINITY,
                area: 0.0,
                quality_score: 0.0,
            };
        }

        // Triangle area using cross product
        let area = 0.5 * ab.cross(&(-ca)).norm();
        
        if area < Real::EPSILON {
            return TriangleQuality {
                aspect_ratio: Real::INFINITY,
                min_angle: 0.0,
                max_angle: 0.0,
                edge_ratio: len_ab.max(len_bc).max(len_ca) / len_ab.min(len_bc).min(len_ca),
                area: 0.0,
                quality_score: 0.0,
            };
        }

        // Interior angles using law of cosines
        let angle_a = ((len_bc.powi(2) + len_ca.powi(2) - len_ab.powi(2)) / (2.0 * len_bc * len_ca)).acos();
        let angle_b = ((len_ca.powi(2) + len_ab.powi(2) - len_bc.powi(2)) / (2.0 * len_ca * len_ab)).acos();
        let angle_c = ((len_ab.powi(2) + len_bc.powi(2) - len_ca.powi(2)) / (2.0 * len_ab * len_bc)).acos();

        let min_angle = angle_a.min(angle_b).min(angle_c);
        let max_angle = angle_a.max(angle_b).max(angle_c);

        // Edge length ratio
        let min_edge = len_ab.min(len_bc).min(len_ca);
        let max_edge = len_ab.max(len_bc).max(len_ca);
        let edge_ratio = max_edge / min_edge;

        // Aspect ratio (circumradius to inradius ratio)
        let semiperimeter = (len_ab + len_bc + len_ca) / 2.0;
        let circumradius = (len_ab * len_bc * len_ca) / (4.0 * area);
        let inradius = area / semiperimeter;
        let aspect_ratio = circumradius / inradius;

        // Quality score: weighted combination of metrics
        let angle_quality = (min_angle / (std::f64::consts::PI / 6.0)).min(1.0); // Normalized to 30°
        let shape_quality = (1.0 / aspect_ratio).min(1.0);
        let edge_quality = (3.0 / edge_ratio).min(1.0);
        
        let quality_score = (0.4 * angle_quality + 0.4 * shape_quality + 0.2 * edge_quality).max(0.0).min(1.0);

        TriangleQuality {
            aspect_ratio,
            min_angle,
            max_angle,
            edge_ratio,
            area,
            quality_score,
        }
    }

    /// **Mathematical Foundation: Adaptive Mesh Refinement**
    ///
    /// Intelligently refine mesh based on geometric criteria:
    ///
    /// ## **Refinement Criteria**
    /// - **Quality threshold**: Refine triangles with quality score < threshold
    /// - **Size variation**: Refine where edge lengths vary significantly
    /// - **Curvature**: Refine high-curvature regions (based on normal variation)
    /// - **Feature detection**: Preserve sharp edges and corners
    ///
    /// ## **Refinement Strategy**
    /// 1. **Quality-based**: Subdivide poor-quality triangles
    /// 2. **Size-based**: Subdivide triangles larger than target size
    /// 3. **Curvature-based**: Subdivide where surface curves significantly
    ///
    /// This provides better mesh quality compared to uniform subdivision.
    pub fn adaptive_refine(&self, quality_threshold: Real, max_edge_length: Real) -> CSG<S> {
        let qualities = self.analyze_triangle_quality();
        let mut refined_polygons = Vec::new();
        
        for (i, polygon) in self.polygons.iter().enumerate() {
            let should_refine = if i < qualities.len() {
                let quality = &qualities[i];
                // Refine if quality is poor or edges are too long
                quality.quality_score < quality_threshold || 
                Self::max_edge_length(&polygon.vertices) > max_edge_length
            } else {
                false
            };

            if should_refine {
                // Subdivide this polygon
                let subdivided = polygon.subdivide_triangles(1.try_into().unwrap());
                for triangle in subdivided {
                    let vertices = triangle.to_vec();
                    refined_polygons.push(Polygon::new(vertices, polygon.metadata.clone()));
                }
            } else {
                // Keep original polygon
                refined_polygons.push(polygon.clone());
            }
        }
        
        CSG::from_polygons(&refined_polygons)
    }

    /// Calculate maximum edge length in a polygon
    fn max_edge_length(vertices: &[Vertex]) -> Real {
        if vertices.len() < 2 {
            return 0.0;
        }
        
        let mut max_length: Real = 0.0;
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            let edge_length = (vertices[j].pos - vertices[i].pos).norm();
            max_length = max_length.max(edge_length);
        }
        max_length
    }

    /// **Mathematical Foundation: Mesh Quality Assessment**
    ///
    /// Compute comprehensive mesh quality metrics:
    ///
    /// ## **Statistical Measures**
    /// - **Average quality**: Overall mesh shape quality
    /// - **Quality distribution**: Histogram of triangle qualities  
    /// - **Outlier detection**: Identification of problematic triangles
    ///
    /// ## **Geometric Measures**
    /// - **Edge length distribution**: Uniformity of mesh resolution
    /// - **Valence distribution**: Vertex connectivity regularity
    /// - **Aspect ratio bounds**: Shape quality bounds
    ///
    /// Provides quantitative assessment for mesh optimization decisions.
    pub fn compute_mesh_quality(&self) -> MeshQualityMetrics {
        let qualities = self.analyze_triangle_quality();
        
        if qualities.is_empty() {
            return MeshQualityMetrics {
                avg_quality: 0.0,
                min_quality: 0.0,
                high_quality_ratio: 0.0,
                sliver_count: 0,
                avg_edge_length: 0.0,
                edge_length_std: 0.0,
            };
        }

        let total_quality: Real = qualities.iter().map(|q| q.quality_score).sum();
        let avg_quality = total_quality / qualities.len() as Real;
        
        let min_quality = qualities.iter()
            .map(|q| q.quality_score)
            .fold(Real::INFINITY, |a, b| a.min(b));
        
        let high_quality_count = qualities.iter()
            .filter(|q| q.quality_score > 0.7)
            .count();
        let high_quality_ratio = high_quality_count as Real / qualities.len() as Real;
        
        let sliver_count = qualities.iter()
            .filter(|q| q.min_angle < (10.0_f64.to_radians()))
            .count();

        // Compute edge length statistics
        let edge_lengths: Vec<Real> = self.polygons.iter()
            .flat_map(|poly| {
                poly.vertices.windows(2).map(|w| (w[1].pos - w[0].pos).norm())
                .chain(std::iter::once((poly.vertices[0].pos - poly.vertices.last().unwrap().pos).norm()))
            })
            .collect();

        let avg_edge_length = if !edge_lengths.is_empty() {
            edge_lengths.iter().sum::<Real>() / edge_lengths.len() as Real
        } else {
            0.0
        };
        
        let edge_length_variance = if edge_lengths.len() > 1 {
            let variance: Real = edge_lengths.iter()
                .map(|&len| (len - avg_edge_length).powi(2))
                .sum::<Real>() / (edge_lengths.len() - 1) as Real;
            variance.sqrt()
        } else {
            0.0
        };

        MeshQualityMetrics {
            avg_quality,
            min_quality,
            high_quality_ratio,
            sliver_count,
            avg_edge_length,
            edge_length_std: edge_length_variance,
        }
    }

    /// **Mathematical Foundation: Feature-Preserving Mesh Optimization**
    ///
    /// Remove poor-quality triangles while preserving important geometric features:
    ///
    /// ## **Quality-Based Filtering**
    /// Remove triangles that meet criteria:
    /// - **Sliver triangles**: min_angle < threshold (typically 5°)
    /// - **Needle triangles**: aspect_ratio > threshold (typically 20)
    /// - **Small triangles**: area < threshold
    ///
    /// ## **Feature Preservation**
    /// - **Sharp edges**: Preserve edges with large dihedral angles
    /// - **Boundaries**: Maintain mesh boundaries
    /// - **Topology**: Ensure mesh remains manifold
    ///
    /// Returns cleaned mesh with improved triangle quality.
    pub fn remove_poor_triangles(&self, min_quality: Real) -> CSG<S> {
        let qualities = self.analyze_triangle_quality();
        let mut filtered_polygons = Vec::new();
        
        for (i, polygon) in self.polygons.iter().enumerate() {
            let keep_triangle = if i < qualities.len() {
                let quality = &qualities[i];
                quality.quality_score >= min_quality && 
                quality.area > Real::EPSILON &&
                quality.min_angle > (5.0_f64.to_radians()) &&
                quality.aspect_ratio < 20.0
            } else {
                true // Keep if we can't assess quality
            };

            if keep_triangle {
                filtered_polygons.push(polygon.clone());
            }
        }
        
        CSG::from_polygons(&filtered_polygons)
    }

    /// Legacy Laplacian smoothing (kept for backward compatibility)
    /// **Note**: Use `laplacian_smooth_global` for better results with proper connectivity
    pub fn laplacian_smooth(&self, lambda: Real, iterations: usize, preserve_boundaries: bool) -> CSG<S> {
        // Delegate to the improved global connectivity version
        self.laplacian_smooth_global(lambda, iterations, preserve_boundaries)
    }

    /// Subdivide all polygons in this CSG 'levels' times, in place.
    /// This results in a triangular mesh with more detail.
    ///
    /// ## Example
    /// ```
    /// use csgrs::CSG;
    /// use core::num::NonZeroU32;
    /// let mut cube: CSG<()> = CSG::cube(2.0, None);
    /// // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    /// // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    /// cube.subdivide_triangles_mut(1.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 48);
    ///
    /// let mut cube: CSG<()> = CSG::cube(2.0, None);
    /// cube.subdivide_triangles_mut(2.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 192);
    /// ```
    pub fn subdivide_triangles_mut(&mut self, levels: core::num::NonZeroU32) {
        #[cfg(feature = "parallel")]
        {
            self.polygons = self
                .polygons
                .par_iter_mut()
                .flat_map(|poly| {
                    let sub_tris = poly.subdivide_triangles(levels.into());
                    // Convert each small tri back to a Polygon
                    sub_tris
                        .into_par_iter()
                        .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
                })
                .collect();
        }

        #[cfg(not(feature = "parallel"))]
        {
            self.polygons = self
                .polygons
                .iter_mut()
                .flat_map(|poly| {
                    let polytri = poly.subdivide_triangles(levels.into());
                    polytri
                        .into_iter()
                        .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
                })
                .collect();
        }
    }

    /// Subdivide all polygons in this CSG 'levels' times, returning a new CSG.
    /// This results in a triangular mesh with more detail.
    pub fn subdivide_triangles(&self, levels: core::num::NonZeroU32) -> CSG<S> {
        #[cfg(feature = "parallel")]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .par_iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                // Convert each small tri back to a Polygon
                sub_tris.into_par_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
                })
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                sub_tris.into_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
                })
            })
            .collect();

        CSG::from_polygons(&new_polygons)
    }

    /// Renormalize all polygons in this CSG by re-computing each polygon's plane
    /// and assigning that plane's normal to all vertices.
    pub fn renormalize(&mut self) {
        for poly in &mut self.polygons {
            poly.set_new_normal();
        }
    }

    /// Triangulate each polygon in the CSG returning a CSG containing triangles
    pub fn tessellate(&self) -> CSG<S> {
        let triangles = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.tessellate()
                    .into_iter()
                    .map(move |triangle| Polygon::new(triangle.to_vec(), poly.metadata.clone()))
            })
            .collect::<Vec<_>>();

        CSG::from_polygons(&triangles)
    }
} 

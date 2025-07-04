use crate::core::float_types::Real;
use crate::csg::CSG;
use crate::geometry::Vertex;
use std::fmt::Debug;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

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
        let qualities: Vec<TriangleQuality> = triangulated
            .polygons
            .par_iter()
            .map(|poly| Self::compute_triangle_quality(&poly.vertices))
            .collect();

        #[cfg(not(feature = "parallel"))]
        let qualities: Vec<TriangleQuality> = triangulated
            .polygons
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

        // Edge vectors and lengths using iterator patterns
        let edges = [b - a, c - b, a - c];
        let edge_lengths: Vec<Real> = edges.iter().map(|edge| edge.norm()).collect();
        let [len_ab, len_bc, len_ca] = [edge_lengths[0], edge_lengths[1], edge_lengths[2]];

        // Handle degenerate cases using iterator patterns
        if edge_lengths.iter().any(|&len| len < Real::EPSILON) {
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
        let area = 0.5 * edges[0].cross(&(-edges[2])).norm();

        if area < Real::EPSILON {
            return TriangleQuality {
                aspect_ratio: Real::INFINITY,
                min_angle: 0.0,
                max_angle: 0.0,
                edge_ratio: edge_lengths.iter().fold(0.0 as Real, |a, &b| a.max(b)) / edge_lengths.iter().fold(Real::INFINITY, |a, &b| a.min(b)),
                area: 0.0,
                quality_score: 0.0,
            };
        }

        // Interior angles using law of cosines
        let angle_a = ((len_bc.powi(2) + len_ca.powi(2) - len_ab.powi(2))
            / (2.0 * len_bc * len_ca))
            .acos();
        let angle_b = ((len_ca.powi(2) + len_ab.powi(2) - len_bc.powi(2))
            / (2.0 * len_ca * len_ab))
            .acos();
        let angle_c = ((len_ab.powi(2) + len_bc.powi(2) - len_ca.powi(2))
            / (2.0 * len_ab * len_bc))
            .acos();

        // Calculate min/max angles using iterator patterns
        let angles = [angle_a, angle_b, angle_c];
        let min_angle = angles.iter().fold(Real::INFINITY, |a, &b| a.min(b));
        let max_angle = angles.iter().fold(0.0 as Real, |a, &b| a.max(b));

        // Edge length ratio using iterator patterns
        let min_edge = edge_lengths.iter().fold(Real::INFINITY, |a, &b| a.min(b));
        let max_edge = edge_lengths.iter().fold(0.0 as Real, |a, &b| a.max(b));
        let edge_ratio = max_edge / min_edge;

        // Aspect ratio (circumradius to inradius ratio)
        let semiperimeter = edge_lengths.iter().sum::<Real>() / 2.0;
        let circumradius = (len_ab * len_bc * len_ca) / (4.0 * area);
        let inradius = area / semiperimeter;
        let aspect_ratio = circumradius / inradius;

        // Quality score: weighted combination of metrics
        let angle_quality = (min_angle / (std::f64::consts::PI / 6.0)).min(1.0); // Normalized to 30°
        let shape_quality = (1.0 / aspect_ratio).min(1.0);
        let edge_quality = (3.0 / edge_ratio).min(1.0);

        let quality_score = (0.4 * angle_quality + 0.4 * shape_quality + 0.2 * edge_quality)
            .max(0.0)
            .min(1.0);

        TriangleQuality {
            aspect_ratio,
            min_angle,
            max_angle,
            edge_ratio,
            area,
            quality_score,
        }
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

        let min_quality = qualities
            .iter()
            .map(|q| q.quality_score)
            .fold(Real::INFINITY, |a, b| a.min(b));

        // Use partition() for efficient quality classification
        let (high_quality_triangles, _low_quality_triangles): (Vec<_>, Vec<_>) = qualities
            .iter()
            .partition(|q| q.quality_score > 0.7);

        let high_quality_ratio = high_quality_triangles.len() as Real / qualities.len() as Real;

        // Use filter and count for sliver detection
        let sliver_count = qualities
            .iter()
            .filter(|q| q.min_angle < (10.0_f64.to_radians()))
            .count();

        // Compute edge length statistics with parallel processing for large meshes
        #[cfg(feature = "parallel")]
        let edge_lengths: Vec<Real> = {
            if self.polygons.len() > 1000 {
                use rayon::prelude::*;
                // Use parallel map to collect edge lengths per polygon, then flatten
                self.polygons
                    .par_iter()
                    .map(|poly| {
                        // Use iterator chain for edge length computation
                        poly.vertices
                            .windows(2)
                            .map(|w| (w[1].pos - w[0].pos).norm())
                            .chain(std::iter::once(
                                (poly.vertices[0].pos - poly.vertices.last().unwrap().pos).norm()
                            ))
                            .collect::<Vec<_>>()
                    })
                    .flatten()
                    .collect()
            } else {
                self.polygons
                    .iter()
                    .flat_map(|poly| {
                        poly.vertices
                            .windows(2)
                            .map(|w| (w[1].pos - w[0].pos).norm())
                            .chain(std::iter::once(
                                (poly.vertices[0].pos - poly.vertices.last().unwrap().pos).norm(),
                            ))
                    })
                    .collect()
            }
        };

        #[cfg(not(feature = "parallel"))]
        let edge_lengths: Vec<Real> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.vertices
                    .windows(2)
                    .map(|w| (w[1].pos - w[0].pos).norm())
                    .chain(std::iter::once(
                        (poly.vertices[0].pos - poly.vertices.last().unwrap().pos).norm(),
                    ))
            })
            .collect();

        let avg_edge_length = if !edge_lengths.is_empty() {
            edge_lengths.iter().sum::<Real>() / edge_lengths.len() as Real
        } else {
            0.0
        };

        let edge_length_variance = if edge_lengths.len() > 1 {
            let variance: Real = edge_lengths
                .iter()
                .map(|&len| (len - avg_edge_length).powi(2))
                .sum::<Real>()
                / (edge_lengths.len() - 1) as Real;
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

    /// **Progressive Quality Analysis with scan()**
    ///
    /// Uses scan() for stateful quality tracking across mesh regions,
    /// providing progressive quality degradation analysis.
    pub fn analyze_progressive_quality(&self) -> Vec<Real> {
        let qualities = self.analyze_triangle_quality();

        // Use scan() for running quality average computation
        qualities
            .iter()
            .enumerate()
            .scan(0.0, |running_sum, (i, quality)| {
                *running_sum += quality.quality_score;
                let running_avg = *running_sum / (i + 1) as Real;
                Some(running_avg)
            })
            .collect()
    }

    /// **Quality Distribution Analysis with group_by patterns**
    ///
    /// Groups triangles by quality ranges for distribution analysis.
    pub fn analyze_quality_distribution(&self) -> std::collections::HashMap<String, usize> {
        let qualities = self.analyze_triangle_quality();

        // Use iterator patterns to group by quality ranges
        let mut distribution = std::collections::HashMap::new();

        qualities
            .iter()
            .map(|q| {
                match q.quality_score {
                    score if score >= 0.9 => "Excellent",
                    score if score >= 0.7 => "Good",
                    score if score >= 0.5 => "Fair",
                    score if score >= 0.3 => "Poor",
                    _ => "Critical",
                }
            })
            .for_each(|category| {
                *distribution.entry(category.to_string()).or_insert(0) += 1;
            });

        distribution
    }
}
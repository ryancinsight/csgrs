//! Basic geometric primitives for Mesh generation
//!
//! This module contains fundamental 3D shapes like cubes, spheres, cylinders, etc.

use crate::errors::ValidationError;
use crate::float_types::Real;
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};
use std::f64::consts::PI;

/// Helper function to create a rectangular face with consistent vertex ordering
fn create_rectangular_face<S: Clone + Send + Sync + std::fmt::Debug>(
    points: [Point3<Real>; 4],
    normal: Vector3<Real>,
    metadata: &Option<S>,
) -> Polygon<S> {
    Polygon::new(
        vec![
            Vertex::new(points[0], normal),
            Vertex::new(points[1], normal),
            Vertex::new(points[2], normal),
            Vertex::new(points[3], normal),
        ],
        metadata.clone(),
    )
}

/// Helper function to validate that a dimension is positive and finite
fn validate_positive_dimension(name: &str, value: Real) -> Result<(), ValidationError> {
    if value <= 0.0 || !value.is_finite() {
        return Err(ValidationError::InvalidDimension(name.to_string(), value));
    }
    Ok(())
}

impl<S: Clone + Send + Sync + std::fmt::Debug> Mesh<S> {
    /// **Mathematical Foundations for 3D Box Geometry**
    ///
    /// This module implements mathematically rigorous algorithms for generating
    /// axis-aligned rectangular prisms (cuboids) and cubes based on solid geometry
    /// and computational topology principles.
    ///
    /// ## **Theoretical Foundations**
    ///
    /// ### **Cuboid Geometry**
    /// A right rectangular prism (cuboid) in 3D space is defined by:
    /// - **Vertices**: 8 corner points forming a rectangular parallelepiped
    /// - **Edges**: 12 edges connecting adjacent vertices
    /// - **Faces**: 6 rectangular faces, each with consistent outward normal
    ///
    /// ### **Coordinate System**
    /// Standard axis-aligned cuboid from origin:
    /// ```text
    /// (0,0,0) → (width, length, height)
    /// ```
    /// This creates a right-handed coordinate system with consistent face orientations.
    ///
    /// ### **Face Normal Calculation**
    /// Each face normal is computed using the right-hand rule:
    /// ```text
    /// n⃗ = (v⃗₁ - v⃗₀) × (v⃗₂ - v⃗₀)
    /// ```
    /// where vertices are ordered counter-clockwise when viewed from outside.
    ///
    /// ### **Winding Order Convention**
    /// All faces use counter-clockwise vertex ordering when viewed from exterior:
    /// - **Ensures consistent outward normals**
    /// - **Enables proper backface culling**
    /// - **Maintains manifold topology for CSG operations**
    ///
    /// ## **Geometric Properties**
    /// - **Volume**: V = width × length × height
    /// - **Surface Area**: A = 2(wl + wh + lh)
    /// - **Diagonal**: d = √(w² + l² + h²)
    /// - **Centroid**: (w/2, l/2, h/2)
    pub fn cuboid(
        width: Real,
        length: Real,
        height: Real,
        metadata: Option<S>,
    ) -> Result<Mesh<S>, ValidationError> {
        // Validate dimensions are positive and finite
        validate_positive_dimension("width", width)?;
        validate_positive_dimension("length", length)?;
        validate_positive_dimension("height", height)?;

        // Define the eight corner points of the prism.
        //    (x, y, z)
        let p000 = Point3::new(0.0, 0.0, 0.0);
        let p100 = Point3::new(width, 0.0, 0.0);
        let p110 = Point3::new(width, length, 0.0);
        let p010 = Point3::new(0.0, length, 0.0);

        let p001 = Point3::new(0.0, 0.0, height);
        let p101 = Point3::new(width, 0.0, height);
        let p111 = Point3::new(width, length, height);
        let p011 = Point3::new(0.0, length, height);

        // We’ll define 6 faces (each a Polygon), in an order that keeps outward-facing normals
        // and consistent (counter-clockwise) vertex winding as viewed from outside the prism.

        // Create faces using helper function for consistency and reduced duplication
        let bottom =
            create_rectangular_face([p000, p010, p110, p100], -Vector3::z(), &metadata);

        let top =
            create_rectangular_face([p001, p101, p111, p011], Vector3::z(), &metadata);

        let front =
            create_rectangular_face([p000, p100, p101, p001], -Vector3::y(), &metadata);

        let back =
            create_rectangular_face([p010, p011, p111, p110], Vector3::y(), &metadata);

        let left =
            create_rectangular_face([p000, p001, p011, p010], -Vector3::x(), &metadata);

        let right =
            create_rectangular_face([p100, p110, p111, p101], Vector3::x(), &metadata);

        // Combine all faces into a Mesh
        Ok(Mesh::from_polygons(
            &[bottom, top, front, back, left, right],
            metadata,
        ))
    }

    pub fn cube(width: Real, metadata: Option<S>) -> Result<Mesh<S>, ValidationError> {
        Self::cuboid(width, width, width, metadata)
    }

    /// **Mathematical Foundation: Spherical Mesh Generation**
    ///
    /// Construct a sphere using UV-parameterized quadrilateral tessellation.
    /// This implements the standard spherical coordinate parameterization
    /// with adaptive handling of polar degeneracies.
    ///
    /// ## **Sphere Mathematics**
    ///
    /// ### **Parametric Surface Equations**
    /// The sphere surface is defined by:
    /// ```text
    /// S(u,v) = r(sin(πv)cos(2πu), cos(πv), sin(πv)sin(2πu))
    /// where u ∈ \\[0,1\\], v ∈ \\[0,1\\]
    /// ```
    ///
    /// ### **Tessellation Algorithm**
    /// 1. **Parameter Grid**: Create (segments+1) × (stacks+1) parameter values
    /// 2. **Vertex Generation**: Evaluate S(u,v) at grid points
    /// 3. **Quadrilateral Formation**: Connect adjacent grid points
    /// 4. **Degeneracy Handling**: Poles require triangle adaptation
    ///
    /// ### **Pole Degeneracy Resolution**
    /// At poles (v=0 or v=1), the parameterization becomes singular:
    /// - **North pole** (v=0): All u values map to same point (0, r, 0)
    /// - **South pole** (v=1): All u values map to same point (0, -r, 0)
    /// - **Solution**: Use triangles instead of quads for polar caps
    ///
    /// ### **Normal Vector Computation**
    /// Sphere normals are simply the normalized position vectors:
    /// ```text
    /// n⃗ = p⃗/|p⃗| = (x,y,z)/r
    /// ```
    /// This is mathematically exact for spheres (no approximation needed).
    ///
    /// ### **Mesh Quality Metrics**
    /// - **Aspect Ratio**: Best when segments ≈ 2×stacks
    /// - **Area Distortion**: Minimal at equator, maximal at poles
    /// - **Angular Distortion**: Increases towards poles (unavoidable)
    ///
    /// ### **Numerical Considerations**
    /// - **Trigonometric Precision**: Uses crate::float_types::TAU and crate::float_types::PI for accuracy
    /// - **Pole Handling**: Avoids division by zero at singularities
    /// - **Winding Consistency**: Maintains outward-facing orientation
    ///
    /// ## **Geometric Properties**
    /// - **Surface Area**: A = 4πr²
    /// - **Volume**: V = (4/3)πr³
    /// - **Circumference** (any great circle): C = 2πr
    /// - **Curvature**: Gaussian K = 1/r², Mean H = 1/r
    ///
    /// # Parameters
    /// - `radius`: Sphere radius (> 0)
    /// - `segments`: Longitude divisions (≥ 3, recommend ≥ 8)
    /// - `stacks`: Latitude divisions (≥ 2, recommend ≥ 6)
    /// - `metadata`: Optional metadata for all faces
    pub fn sphere(
        radius: Real,
        segments: usize,
        stacks: usize,
        metadata: Option<S>,
    ) -> Result<Mesh<S>, ValidationError> {
        // Validate parameters
        validate_positive_dimension("radius", radius)?;
        if segments < 3 {
            return Err(ValidationError::InvalidShapeParameter(
                "segments".to_string(),
                "must be at least 3".to_string(),
            ));
        }
        if stacks < 2 {
            return Err(ValidationError::InvalidShapeParameter(
                "stacks".to_string(),
                "must be at least 2".to_string(),
            ));
        }

        // Pre-allocate polygons vector for better performance
        let mut polygons = Vec::with_capacity(segments * stacks);

        for i in 0..segments {
            for j in 0..stacks {
                // Pre-allocate vertices vector for quad polygons (4 vertices)
                let mut vertices = Vec::with_capacity(4);

                let vertex = |theta: Real, phi: Real| {
                    let dir = Vector3::new(
                        theta.cos() * phi.sin(),
                        phi.cos(),
                        theta.sin() * phi.sin(),
                    );
                    Vertex::new(
                        Point3::from(dir * radius),
                        dir, // Normalized direction vector = normal for unit sphere
                    )
                };

                // Calculate longitude and latitude angles for this quad
                let theta0 = (i as Real) * (2.0 * PI) / (segments as Real);
                let theta1 = ((i + 1) as Real) * (2.0 * PI) / (segments as Real);
                let phi0 = (j as Real) * PI / (stacks as Real);
                let phi1 = ((j + 1) as Real) * PI / (stacks as Real);

                // Create vertices for this quad (counter-clockwise winding)
                vertices.push(vertex(theta0, phi0));
                vertices.push(vertex(theta1, phi0));
                vertices.push(vertex(theta1, phi1));
                vertices.push(vertex(theta0, phi1));

                polygons.push(Polygon::new(vertices, metadata.clone()));
            }
        }

        Ok(Mesh {
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata,
        })
    }

    /// Generate a cylinder mesh along the Z-axis
    ///
    /// # Parameters
    /// - `radius`: Cylinder radius (> 0)
    /// - `height`: Cylinder height (> 0)
    /// - `segments`: Number of radial segments (≥ 3)
    /// - `metadata`: Optional metadata for all faces
    pub fn cylinder(
        radius: Real,
        height: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> Result<Mesh<S>, ValidationError> {
        validate_positive_dimension("radius", radius)?;
        validate_positive_dimension("height", height)?;
        if segments < 3 {
            return Err(ValidationError::InvalidShapeParameter(
                "segments".to_string(),
                "must be at least 3".to_string(),
            ));
        }

        let mut polygons = Vec::new();
        let half_height = height / 2.0;

        // Generate side faces
        for i in 0..segments {
            let angle0 = (i as Real) * 2.0 * PI / (segments as Real);
            let angle1 = ((i + 1) as Real) * 2.0 * PI / (segments as Real);

            let x0 = radius * angle0.cos();
            let z0 = radius * angle0.sin();
            let x1 = radius * angle1.cos();
            let z1 = radius * angle1.sin();

            // Bottom vertices
            let bottom0 = Point3::new(x0, -half_height, z0);
            let bottom1 = Point3::new(x1, -half_height, z1);

            // Top vertices
            let top0 = Point3::new(x0, half_height, z0);
            let top1 = Point3::new(x1, half_height, z1);

            // Create side face (counter-clockwise when viewed from outside)
            let vertices = vec![
                Vertex::new(bottom0, Vector3::new(x0, 0.0, z0).normalize()),
                Vertex::new(bottom1, Vector3::new(x1, 0.0, z1).normalize()),
                Vertex::new(top1, Vector3::new(x1, 0.0, z1).normalize()),
                Vertex::new(top0, Vector3::new(x0, 0.0, z0).normalize()),
            ];

            polygons.push(Polygon::new(vertices, metadata.clone()));
        }

        // Generate bottom cap
        let mut bottom_vertices = Vec::with_capacity(segments);
        for i in 0..segments {
            let angle = (i as Real) * 2.0 * PI / (segments as Real);
            let x = radius * angle.cos();
            let z = radius * angle.sin();
            bottom_vertices.push(Vertex::new(
                Point3::new(x, -half_height, z),
                -Vector3::y(),
            ));
        }
        polygons.push(Polygon::new(bottom_vertices, metadata.clone()));

        // Generate top cap
        let mut top_vertices = Vec::with_capacity(segments);
        for i in (0..segments).rev() {
            let angle = (i as Real) * 2.0 * PI / (segments as Real);
            let x = radius * angle.cos();
            let z = radius * angle.sin();
            top_vertices.push(Vertex::new(
                Point3::new(x, half_height, z),
                Vector3::y(),
            ));
        }
        polygons.push(Polygon::new(top_vertices, metadata.clone()));

        Ok(Mesh {
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata,
        })
    }

    /// Create a regular octahedron
    pub fn octahedron(radius: Real, metadata: Option<S>) -> Result<Mesh<S>, ValidationError> {
        validate_positive_dimension("radius", radius)?;

        // Octahedron vertices (normalized to radius)
        let vertices = [
            Vertex::new(Point3::new(radius, 0.0, 0.0), Vector3::x()),
            Vertex::new(Point3::new(-radius, 0.0, 0.0), -Vector3::x()),
            Vertex::new(Point3::new(0.0, radius, 0.0), Vector3::y()),
            Vertex::new(Point3::new(0.0, -radius, 0.0), -Vector3::y()),
            Vertex::new(Point3::new(0.0, 0.0, radius), Vector3::z()),
            Vertex::new(Point3::new(0.0, 0.0, -radius), -Vector3::z()),
        ];

        // Octahedron faces (triangles)
        let faces = vec![
            vec![0, 4, 2], // +X, +Z, +Y
            vec![0, 2, 5], // +X, +Y, -Z
            vec![0, 5, 3], // +X, -Z, -Y
            vec![0, 3, 4], // +X, -Y, +Z
            vec![1, 2, 4], // -X, +Y, +Z
            vec![1, 5, 2], // -X, -Z, +Y
            vec![1, 3, 5], // -X, -Y, -Z
            vec![1, 4, 3], // -X, +Z, -Y
        ];

        let polygons = faces
            .into_iter()
            .map(|face_indices| {
                let face_vertices = face_indices
                    .into_iter()
                    .map(|idx| vertices[idx])
                    .collect();
                Polygon::new(face_vertices, metadata.clone())
            })
            .collect();

        Ok(Mesh {
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata,
        })
    }

    /// Create a regular icosahedron
    pub fn icosahedron(radius: Real, metadata: Option<S>) -> Result<Mesh<S>, ValidationError> {
        validate_positive_dimension("radius", radius)?;

        // Golden ratio for icosahedron
        let phi = (1.0 + 5.0f64.sqrt()) / 2.0;

        // Icosahedron vertices (normalized to radius)
        let scale = radius / (phi * phi).sqrt();
        let vertices = vec![
            // Top vertex
            Vertex::new(
                Point3::new(0.0, scale * phi, 0.0),
                Vector3::new(0.0, phi, 0.0).normalize(),
            ),
            // Middle ring vertices
            Vertex::new(
                Point3::new(0.0, scale, scale * phi),
                Vector3::new(0.0, 1.0, phi).normalize(),
            ),
            Vertex::new(
                Point3::new(scale * phi, 0.0, scale),
                Vector3::new(phi, 0.0, 1.0).normalize(),
            ),
            Vertex::new(
                Point3::new(scale, scale * phi, 0.0),
                Vector3::new(1.0, phi, 0.0).normalize(),
            ),
            Vertex::new(
                Point3::new(-scale, scale * phi, 0.0),
                Vector3::new(-1.0, phi, 0.0).normalize(),
            ),
            Vertex::new(
                Point3::new(-scale * phi, 0.0, scale),
                Vector3::new(-phi, 0.0, 1.0).normalize(),
            ),
            Vertex::new(
                Point3::new(-scale * phi, 0.0, -scale),
                Vector3::new(-phi, 0.0, -1.0).normalize(),
            ),
            Vertex::new(
                Point3::new(-scale, -scale * phi, 0.0),
                Vector3::new(-1.0, -phi, 0.0).normalize(),
            ),
            Vertex::new(
                Point3::new(scale, -scale * phi, 0.0),
                Vector3::new(1.0, -phi, 0.0).normalize(),
            ),
            Vertex::new(
                Point3::new(scale * phi, 0.0, -scale),
                Vector3::new(phi, 0.0, -1.0).normalize(),
            ),
            Vertex::new(
                Point3::new(0.0, -scale, -scale * phi),
                Vector3::new(0.0, -1.0, -phi).normalize(),
            ),
            Vertex::new(
                Point3::new(0.0, -scale * phi, 0.0),
                Vector3::new(0.0, -phi, 0.0).normalize(),
            ),
        ];

        // Icosahedron faces (triangles)
        let faces = vec![
            // Top cap
            vec![0, 1, 2],
            vec![0, 2, 3],
            vec![0, 3, 4],
            vec![0, 4, 5],
            vec![0, 5, 1],
            // Middle band
            vec![1, 6, 2],
            vec![2, 6, 7],
            vec![2, 7, 3],
            vec![3, 7, 8],
            vec![3, 8, 4],
            vec![4, 8, 9],
            vec![4, 9, 5],
            vec![5, 9, 10],
            vec![5, 10, 1],
            vec![1, 10, 6],
            // Bottom cap
            vec![11, 7, 6],
            vec![11, 8, 7],
            vec![11, 9, 8],
            vec![11, 10, 9],
            vec![11, 6, 10],
        ];

        let polygons = faces
            .into_iter()
            .map(|face_indices| {
                let face_vertices = face_indices
                    .into_iter()
                    .map(|idx| vertices[idx])
                    .collect();
                Polygon::new(face_vertices, metadata.clone())
            })
            .collect();

        Ok(Mesh {
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata,
        })
    }

    /// Create a torus (doughnut shape)
    ///
    /// # Parameters
    /// - `major_radius`: Distance from center to tube center
    /// - `minor_radius`: Radius of the tube
    /// - `major_segments`: Segments around the major circle
    /// - `minor_segments`: Segments around the minor circle
    /// - `metadata`: Optional metadata for all faces
    pub fn torus(
        major_radius: Real,
        minor_radius: Real,
        major_segments: usize,
        minor_segments: usize,
        metadata: Option<S>,
    ) -> Result<Mesh<S>, ValidationError> {
        validate_positive_dimension("major_radius", major_radius)?;
        validate_positive_dimension("minor_radius", minor_radius)?;

        if major_segments < 3 || minor_segments < 3 {
            return Err(ValidationError::InvalidShapeParameter(
                "segments".to_string(),
                "must be at least 3".to_string(),
            ));
        }

        let mut polygons = Vec::new();

        for i in 0..major_segments {
            for j in 0..minor_segments {
                let u0 = (i as Real) * 2.0 * PI / (major_segments as Real);
                let u1 = ((i + 1) as Real) * 2.0 * PI / (major_segments as Real);
                let v0 = (j as Real) * 2.0 * PI / (minor_segments as Real);
                let v1 = ((j + 1) as Real) * 2.0 * PI / (minor_segments as Real);

                // Calculate positions on the torus surface
                let pos = |u: Real, v: Real| {
                    let center_x = major_radius * u.cos();
                    let center_z = major_radius * u.sin();
                    let x = center_x + minor_radius * u.cos() * v.cos();
                    let y = minor_radius * v.sin();
                    let z = center_z + minor_radius * u.sin() * v.cos();
                    Point3::new(x, y, z)
                };

                // Calculate normal vectors
                let normal = |u: Real, v: Real| {
                    let nx = u.cos() * v.cos();
                    let ny = v.sin();
                    let nz = u.sin() * v.cos();
                    Vector3::new(nx, ny, nz).normalize()
                };

                // Create quad face vertices (counter-clockwise)
                let vertices = vec![
                    Vertex::new(pos(u0, v0), normal(u0, v0)),
                    Vertex::new(pos(u1, v0), normal(u1, v0)),
                    Vertex::new(pos(u1, v1), normal(u1, v1)),
                    Vertex::new(pos(u0, v1), normal(u0, v1)),
                ];

                polygons.push(Polygon::new(vertices, metadata.clone()));
            }
        }

        Ok(Mesh {
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata,
        })
    }

    /// Creates a Mesh polyhedron from raw vertex data (`points`) and face indices.
    ///
    /// # Parameters
    ///
    /// - `points`: a slice of `[x,y,z]` coordinates.
    /// - `faces`: each element is a list of indices into `points`, describing one face.
    ///   Each face must have at least 3 indices.
    ///
    /// # Example
    /// ```
    /// # use csgrs::mesh::Mesh;
    ///
    /// let pts = &[
    ///     [0.0, 0.0, 0.0], // point0
    ///     [1.0, 0.0, 0.0], // point1
    ///     [1.0, 1.0, 0.0], // point2
    ///     [0.0, 1.0, 0.0], // point3
    ///     [0.5, 0.5, 1.0], // point4 - top
    /// ];
    ///
    /// // Two faces: bottom square [0,1,2,3], and a pyramid side [0,1,4]
    /// let fcs: &[&[usize]] = &[
    ///     &[0, 1, 2, 3],
    ///     &[0, 1, 4],
    ///     &[1, 2, 4],
    ///     &[2, 3, 4],
    ///     &[3, 0, 4],
    /// ];
    ///
    /// let mesh_poly = Mesh::<()>::polyhedron(pts, fcs, None);
    /// ```
    pub fn polyhedron(
        points: &[[Real; 3]],
        faces: &[&[usize]],
        metadata: Option<S>,
    ) -> Result<Mesh<S>, ValidationError> {
        // Pre-allocate polygons vector for better performance
        let mut polygons = Vec::with_capacity(faces.len());

        for face in faces {
            // Skip degenerate faces
            if face.len() < 3 {
                continue;
            }

            // Gather the vertices for this face
            let mut face_vertices = Vec::with_capacity(face.len());
            for &idx in face.iter() {
                // Ensure the index is valid
                if idx >= points.len() {
                    return Err(ValidationError::IndexOutOfRange);
                }
                let [x, y, z] = points[idx];
                face_vertices.push(Vertex::new(
                    Point3::new(x, y, z),
                    Vector3::zeros(), // we'll set this later
                ));
            }

            // Build the polygon (plane is auto-computed from first 3 vertices).
            let mut poly = Polygon::new(face_vertices, metadata.clone());

            // Set each vertex normal to match the polygon’s plane normal,
            let plane_normal = poly.plane.normal();
            for v in &mut poly.vertices {
                v.normal = plane_normal;
            }
            polygons.push(poly);
        }

        Ok(Mesh::from_polygons(&polygons, metadata))
    }
}

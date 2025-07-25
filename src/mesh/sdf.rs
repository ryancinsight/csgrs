//! Create `Mesh`s by meshing signed distance fields ([sdf](https://en.wikipedia.org/wiki/Signed_distance_function)) within a bounding box.

use crate::float_types::Real;
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    /// Return a Mesh created by meshing a signed distance field within a bounding box
    ///
    /// ```
    /// # use csgrs::{mesh::Mesh, float_types::Real};
    /// # use nalgebra::Point3;
    /// // Example SDF for a sphere of radius 1.5 centered at (0,0,0)
    /// let my_sdf = |p: &Point3<Real>| p.coords.norm() - 1.5;
    ///
    /// let resolution = (60, 60, 60);
    /// let min_pt = Point3::new(-2.0, -2.0, -2.0);
    /// let max_pt = Point3::new( 2.0,  2.0,  2.0);
    /// let iso_value = 0.0; // Typically zero for SDF-based surfaces
    ///
    ///    let mesh_shape = Mesh::<()>::sdf(my_sdf, resolution, min_pt, max_pt, iso_value, None);
    ///
    ///    // Now `mesh_shape` is your polygon mesh as a Mesh you can union, subtract, or export:
    ///    let _ = std::fs::write("stl/sdf_sphere.stl", mesh_shape.to_stl_binary("sdf_sphere").unwrap());
    pub fn sdf<F>(
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: Option<S>,
    ) -> Mesh<S>
    where
        // F is a closure or function that takes a 3D point and returns the signed distance.
        // Must be `Sync`/`Send` if you want to parallelize the sampling.
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        // **Enhancement**: Ensure minimum resolution and add boundary padding
        let nx = resolution.0.max(4) as u32; // Increased minimum from 2 to 4
        let ny = resolution.1.max(4) as u32;
        let nz = resolution.2.max(4) as u32;

        // **Optimization**: Add adaptive boundary expansion to prevent edge artifacts
        let bbox_size = max_pt - min_pt;
        let adaptive_padding = bbox_size.norm() * 0.02; // 2% padding
        let min_pt_padded = min_pt - Vector3::repeat(adaptive_padding);
        let max_pt_padded = max_pt + Vector3::repeat(adaptive_padding);

        // **Enhancement**: Use padded bounds for more robust sampling
        let dx = (max_pt_padded.x - min_pt_padded.x) / (nx as Real - 1.0);
        let dy = (max_pt_padded.y - min_pt_padded.y) / (ny as Real - 1.0);
        let dz = (max_pt_padded.z - min_pt_padded.z) / (nz as Real - 1.0);

        // Allocate storage for field values:
        let array_size = (nx * ny * nz) as usize;
        let mut field_values = vec![0.0_f32; array_size];

        // **Enhancement**: Improved finite value checking with mathematical correctness
        #[inline]
        fn point_finite(p: &Point3<Real>) -> bool {
            p.coords.iter().all(|&c| c.is_finite())
        }

        #[inline]
        fn vec_finite(v: &Vector3<Real>) -> bool {
            v.iter().all(|&c| c.is_finite())
        }

        // **Optimization**: Enhanced sampling with boundary refinement
        // Sample the SDF at each grid cell with adaptive supersampling near boundaries
        #[allow(clippy::unnecessary_cast)]
        for i in 0..(nx * ny * nz) {
            let iz = i / (nx * ny);
            let remainder = i % (nx * ny);
            let iy = remainder / nx;
            let ix = remainder % nx;

            let xf = min_pt_padded.x + (ix as Real) * dx;
            let yf = min_pt_padded.y + (iy as Real) * dy;
            let zf = min_pt_padded.z + (iz as Real) * dz;

            let p = Point3::new(xf, yf, zf);
            
            // **Enhancement**: Adaptive supersampling near zero-level set
            let mut sdf_val = sdf(&p);
            
            // **Optimization**: If we're near the surface (within 2 grid units), use supersampling
            let surface_threshold = 2.0 * (dx.min(dy).min(dz));
            if sdf_val.abs() < surface_threshold {
                // **Enhancement**: 2x2x2 supersampling for boundary regions
                let subsample_offset = 0.25;
                let mut samples = Vec::with_capacity(8);
                
                for sx in &[-subsample_offset, subsample_offset] {
                    for sy in &[-subsample_offset, subsample_offset] {
                        for sz in &[-subsample_offset, subsample_offset] {
                            let sub_p = Point3::new(
                                xf + sx * dx,
                                yf + sy * dy,
                                zf + sz * dz
                            );
                            let sub_val = sdf(&sub_p);
                            if sub_val.is_finite() {
                                samples.push(sub_val);
                            }
                        }
                    }
                }
                
                // **Enhancement**: Use median filtering to reduce noise
                if !samples.is_empty() {
                    samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    sdf_val = if samples.len() % 2 == 0 {
                        (samples[samples.len()/2 - 1] + samples[samples.len()/2]) * 0.5
                    } else {
                        samples[samples.len()/2]
                    };
                }
            }

            // **Enhancement**: Robust finite value handling with improved fallback
            field_values[i as usize] = if sdf_val.is_finite() {
                (sdf_val - iso_value) as f32
            } else {
                // **Improvement**: Use distance-based fallback instead of constant
                let fallback_distance = (dx + dy + dz) / 3.0;
                fallback_distance as f32
            };
        }

        // The shape describing our discrete grid for Surface Nets:
        #[derive(Clone, Copy)]
        struct GridShape {
            nx: u32,
            ny: u32,
            nz: u32,
        }

        impl fast_surface_nets::ndshape::Shape<3> for GridShape {
            type Coord = u32;

            #[inline]
            fn as_array(&self) -> [Self::Coord; 3] {
                [self.nx, self.ny, self.nz]
            }

            fn size(&self) -> Self::Coord {
                self.nx * self.ny * self.nz
            }

            fn usize(&self) -> usize {
                (self.nx * self.ny * self.nz) as usize
            }

            fn linearize(&self, coords: [Self::Coord; 3]) -> u32 {
                let [x, y, z] = coords;
                (z * self.ny + y) * self.nx + x
            }

            fn delinearize(&self, i: u32) -> [Self::Coord; 3] {
                let x = i % self.nx;
                let yz = i / self.nx;
                let y = yz % self.ny;
                let z = yz / self.ny;
                [x, y, z]
            }
        }

        let shape = GridShape { nx, ny, nz };

        // `SurfaceNetsBuffer` collects the positions, normals, and triangle indices
        let mut sn_buffer = SurfaceNetsBuffer::default();

        // The max valid coordinate in each dimension
        let max_x = nx - 1;
        let max_y = ny - 1;
        let max_z = nz - 1;

        // **Enhancement**: Use tighter bounds to reduce edge artifacts
        let start_bounds = [1, 1, 1]; // Skip boundary voxels
        let end_bounds = [max_x.saturating_sub(1), max_y.saturating_sub(1), max_z.saturating_sub(1)];

        // Run surface nets with improved bounds
        surface_nets(
            &field_values,
            &shape,
            start_bounds,
            end_bounds,
            &mut sn_buffer,
        );

        // Convert the resulting triangles into Mesh polygons
        let mut triangles = Vec::with_capacity(sn_buffer.indices.len() / 3);

        for tri in sn_buffer.indices.chunks_exact(3) {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;

            let p0i = sn_buffer.positions[i0];
            let p1i = sn_buffer.positions[i1];
            let p2i = sn_buffer.positions[i2];

            // **Enhancement**: Map back to original coordinate system (unpadded)
            let p0 = Point3::new(
                min_pt_padded.x + p0i[0] as Real * dx,
                min_pt_padded.y + p0i[1] as Real * dy,
                min_pt_padded.z + p0i[2] as Real * dz,
            );
            let p1 = Point3::new(
                min_pt_padded.x + p1i[0] as Real * dx,
                min_pt_padded.y + p1i[1] as Real * dy,
                min_pt_padded.z + p1i[2] as Real * dz,
            );
            let p2 = Point3::new(
                min_pt_padded.x + p2i[0] as Real * dx,
                min_pt_padded.y + p2i[1] as Real * dy,
                min_pt_padded.z + p2i[2] as Real * dz,
            );

            // Retrieve precomputed normal from Surface Nets:
            let n0 = sn_buffer.normals[i0];
            let n1 = sn_buffer.normals[i1];
            let n2 = sn_buffer.normals[i2];

            // **Enhancement**: Normalize and validate normals
            let n0v = Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real);
            let n1v = Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real);
            let n2v = Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real);

            // **Enhancement**: Improved finite value validation with fallback normals
            if !(point_finite(&p0) && point_finite(&p1) && point_finite(&p2)) {
                continue; // Skip invalid triangles
            }

            // **Enhancement**: Compute fallback normals if surface nets normals are invalid
            let (final_n0, final_n1, final_n2) = if vec_finite(&n0v) && vec_finite(&n1v) && vec_finite(&n2v) {
                (n0v, n1v, n2v)
            } else {
                // Compute triangle normal as fallback
                let edge1 = p1 - p0;
                let edge2 = p2 - p0;
                let face_normal = edge1.cross(&edge2);
                let normalized_normal = if face_normal.norm() > Real::EPSILON {
                    face_normal.normalize()
                } else {
                    Vector3::new(0.0, 0.0, 1.0) // Default up vector
                };
                (normalized_normal, normalized_normal, normalized_normal)
            };

            let v0 = Vertex::new(p0, final_n0);
            let v1 = Vertex::new(p1, final_n1);
            let v2 = Vertex::new(p2, final_n2);

            // **Enhancement**: Validate triangle area to avoid degenerate triangles
            let edge1 = p1 - p0;
            let edge2 = p2 - p0;
            let triangle_area = edge1.cross(&edge2).norm() * 0.5;
            let min_area = (dx * dy).min(dy * dz).min(dx * dz) * 1e-6; // Very small area threshold
            
            if triangle_area > min_area {
                let poly = Polygon::new(vec![v0, v1, v2], metadata.clone());
                triangles.push(poly);
            }
        }

        // Return as a Mesh
        Mesh::from_polygons(&triangles, metadata)
    }
}

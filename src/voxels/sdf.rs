//! SDF meshing for voxel pipelines with SVO integration
//!
//! This module provides SDF meshing that populates SVO with embedded BSP
//! at surface-crossing cells for precise surface representation.

use crate::float_types::Real;
use crate::voxels::polygon::Polygon;
use crate::voxels::vertex::Vertex;
use crate::voxels::csg::Voxels;
use crate::voxels::bsp_integration::BspIntegrator;
use crate::voxels::Svo;
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    /// Create Voxels from SDF using SVO with embedded BSP at surface-crossing cells
    pub fn sdf<F>(
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: Option<S>,
    ) -> Voxels<S>
    where
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        // Create SVO with appropriate bounds and resolution
        let center = Point3::new(
            (min_pt.x + max_pt.x) * 0.5,
            (min_pt.y + max_pt.y) * 0.5,
            (min_pt.z + max_pt.z) * 0.5,
        );
        let half = ((max_pt.x - min_pt.x) * 0.5)
            .max((max_pt.y - min_pt.y) * 0.5)
            .max((max_pt.z - min_pt.z) * 0.5);

        // Determine appropriate SVO depth based on resolution
        let max_res = resolution.0.max(resolution.1).max(resolution.2);
        let svo_depth = (max_res as f32).log2().ceil() as u8;

        let mut svo = Svo::new(center, half, svo_depth);

        // Integrate BSP at surface-crossing cells
        BspIntegrator::integrate_bsp_from_sdf(&mut svo, &sdf, iso_value);

        // Also generate surface polygons using Surface Nets for compatibility
        let surface_polygons = Self::extract_surface_nets(&sdf, resolution, min_pt, max_pt, iso_value, metadata.clone());

        // Create Voxels with both SVO and surface polygons
        let mut voxels = Self::from_svo(svo, metadata);

        // Cache the surface polygons for immediate use
        voxels.set_surface_cache(surface_polygons);

        voxels
    }

    /// Extract surface polygons using Surface Nets (for compatibility and immediate surface access)
    fn extract_surface_nets<F>(
        sdf: &F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: Option<S>,
    ) -> Vec<Polygon<S>>
    where
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        let nx = resolution.0.max(2) as u32;
        let ny = resolution.1.max(2) as u32;
        let nz = resolution.2.max(2) as u32;

        let dx = (max_pt.x - min_pt.x) / (nx as Real - 1.0);
        let dy = (max_pt.y - min_pt.y) / (ny as Real - 1.0);
        let dz = (max_pt.z - min_pt.z) / (nz as Real - 1.0);

        let array_size = (nx * ny * nz) as usize;
        let mut field_values = vec![0.0_f32; array_size];

        #[inline]
        fn point_finite(p: &Point3<Real>) -> bool { p.coords.iter().all(|&c| c.is_finite()) }

        #[allow(clippy::unnecessary_cast)]
        for i in 0..(nx * ny * nz) {
            let iz = i / (nx * ny);
            let remainder = i % (nx * ny);
            let iy = remainder / nx;
            let ix = remainder % nx;

            let xf = min_pt.x + (ix as Real) * dx;
            let yf = min_pt.y + (iy as Real) * dy;
            let zf = min_pt.z + (iz as Real) * dz;

            let p = Point3::new(xf, yf, zf);
            let sdf_val = sdf(&p);
            field_values[i as usize] = if sdf_val.is_finite() { (sdf_val - iso_value) as f32 } else { 1e10_f32 };
        }

        #[derive(Clone, Copy)]
        struct GridShape { nx: u32, ny: u32, nz: u32 }
        impl fast_surface_nets::ndshape::Shape<3> for GridShape {
            type Coord = u32;
            #[inline] fn as_array(&self) -> [Self::Coord; 3] { [self.nx, self.ny, self.nz] }
            fn size(&self) -> Self::Coord { self.nx * self.ny * self.nz }
            fn usize(&self) -> usize { (self.nx * self.ny * self.nz) as usize }
            fn linearize(&self, c: [Self::Coord; 3]) -> u32 { (c[2] * self.ny + c[1]) * self.nx + c[0] }
            fn delinearize(&self, i: u32) -> [Self::Coord; 3] {
                let x = i % self.nx; let yz = i / self.nx; let y = yz % self.ny; let z = yz / self.ny; [x,y,z]
            }
        }

        let mut sn_buffer = SurfaceNetsBuffer::default();
        let shape = GridShape { nx, ny, nz };
        let max_x = nx - 1; let max_y = ny - 1; let max_z = nz - 1;
        surface_nets(&field_values, &shape, [0,0,0], [max_x, max_y, max_z], &mut sn_buffer);

        // Convert triangles into polygons
        let mut triangles = Vec::with_capacity(sn_buffer.indices.len() / 3);
        for tri in sn_buffer.indices.chunks_exact(3) {
            let i0 = tri[0] as usize; let i1 = tri[1] as usize; let i2 = tri[2] as usize;
            let p0i = sn_buffer.positions[i0]; let p1i = sn_buffer.positions[i1]; let p2i = sn_buffer.positions[i2];
            let p0 = Point3::new(min_pt.x + p0i[0] as Real * dx, min_pt.y + p0i[1] as Real * dy, min_pt.z + p0i[2] as Real * dz);
            let p1 = Point3::new(min_pt.x + p1i[0] as Real * dx, min_pt.y + p1i[1] as Real * dy, min_pt.z + p1i[2] as Real * dz);
            let p2 = Point3::new(min_pt.x + p2i[0] as Real * dx, min_pt.y + p2i[1] as Real * dy, min_pt.z + p2i[2] as Real * dz);
            if !(point_finite(&p0) && point_finite(&p1) && point_finite(&p2)) { continue; }
            let n0 = sn_buffer.normals[i0]; let n1 = sn_buffer.normals[i1]; let n2 = sn_buffer.normals[i2];
            let v0 = Vertex::new(p0, Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real));
            let v1 = Vertex::new(p1, Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real));
            let v2 = Vertex::new(p2, Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real));
            triangles.push(Polygon::new(vec![v0, v1, v2], metadata.clone()));
        }

        triangles
    }
}


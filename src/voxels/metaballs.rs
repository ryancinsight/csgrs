//! Metaballs for Voxels using fast-surface-nets, mirroring mesh::metaballs

use crate::float_types::{EPSILON, Real};
use crate::voxels::csg::Voxels;
use crate::voxels::polygon::Polygon;
use crate::voxels::vertex::Vertex;
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

#[derive(Debug, Clone)]
pub struct MetaBall { pub center: Point3<Real>, pub radius: Real }
impl MetaBall { pub const fn new(center: Point3<Real>, radius: Real) -> Self { Self { center, radius } }
    pub fn influence(&self, p: &Point3<Real>) -> Real { let d2 = (p - self.center).norm_squared(); let thresh = self.radius * self.radius * 1000.0; if d2 > thresh { return 0.0; } let denom = d2 + EPSILON; (self.radius * self.radius) / denom }
}

fn scalar_field_metaballs(balls: &[MetaBall], p: &Point3<Real>) -> Real { balls.iter().map(|b| b.influence(p)).sum() }

impl<S: Clone + Debug + Send + Sync> Voxels<S> {
    pub fn metaballs(balls: &[MetaBall], resolution: (usize, usize, usize), iso_value: Real, padding: Real, metadata: Option<S>) -> Voxels<S> {
        if balls.is_empty() { return Voxels::new(); }
        let (min_pt, max_pt) = balls.iter().fold((Point3::new(Real::MAX, Real::MAX, Real::MAX), Point3::new(-Real::MAX, -Real::MAX, -Real::MAX)), |(mut min_p, mut max_p), mb| { let r = mb.radius + padding; min_p.x = min_p.x.min(mb.center.x - r); min_p.y = min_p.y.min(mb.center.y - r); min_p.z = min_p.z.min(mb.center.z - r); max_p.x = max_p.x.max(mb.center.x + r); max_p.y = max_p.y.max(mb.center.y + r); max_p.z = max_p.z.max(mb.center.z + r); (min_p, max_p) });
        let nx = resolution.0.max(2) as u32; let ny = resolution.1.max(2) as u32; let nz = resolution.2.max(2) as u32;
        let dx = (max_pt.x - min_pt.x) / (nx as Real - 1.0); let dy = (max_pt.y - min_pt.y) / (ny as Real - 1.0); let dz = (max_pt.z - min_pt.z) / (nz as Real - 1.0);
        let array_size = (nx * ny * nz) as usize; let mut field_values = vec![0.0f32; array_size];
        let index_3d = |ix: u32, iy: u32, iz: u32| -> usize { (iz * ny + iy) as usize * (nx as usize) + ix as usize };
        for iz in 0..nz { let zf = min_pt.z + (iz as Real) * dz; for iy in 0..ny { let yf = min_pt.y + (iy as Real) * dy; for ix in 0..nx { let xf = min_pt.x + (ix as Real) * dx; let p = Point3::new(xf, yf, zf); let val = scalar_field_metaballs(balls, &p) - iso_value; field_values[index_3d(ix, iy, iz)] = val as f32; }}}
        #[derive(Clone, Copy)] struct GridShape { nx: u32, ny: u32, nz: u32 }
        impl fast_surface_nets::ndshape::Shape<3> for GridShape { type Coord = u32; #[inline] fn as_array(&self) -> [Self::Coord; 3] { [self.nx, self.ny, self.nz] } fn size(&self) -> Self::Coord { self.nx * self.ny * self.nz } fn usize(&self) -> usize { (self.nx * self.ny * self.nz) as usize } fn linearize(&self, c: [Self::Coord;3]) -> u32 { (c[2]*self.ny + c[1])*self.nx + c[0] } fn delinearize(&self, i:u32)->[Self::Coord;3]{ let x=i%self.nx; let yz=i/self.nx; let y=yz%self.ny; let z=yz/self.ny; [x,y,z] } }
        let shape = GridShape { nx, ny, nz }; let mut sn_buffer = SurfaceNetsBuffer::default(); let (max_x, max_y, max_z) = (nx-1, ny-1, nz-1);
        surface_nets(&field_values, &shape, [0,0,0], [max_x, max_y, max_z], &mut sn_buffer);
        let mut triangles = Vec::with_capacity(sn_buffer.indices.len()/3);
        for tri in sn_buffer.indices.chunks_exact(3) { let i0=tri[0] as usize; let i1=tri[1] as usize; let i2=tri[2] as usize; let p0i=sn_buffer.positions[i0]; let p1i=sn_buffer.positions[i1]; let p2i=sn_buffer.positions[i2]; let p0=Point3::new(min_pt.x+p0i[0] as Real*dx, min_pt.y+p0i[1] as Real*dy, min_pt.z+p0i[2] as Real*dz); let p1=Point3::new(min_pt.x+p1i[0] as Real*dx, min_pt.y+p1i[1] as Real*dy, min_pt.z+p1i[2] as Real*dz); let p2=Point3::new(min_pt.x+p2i[0] as Real*dx, min_pt.y+p2i[1] as Real*dy, min_pt.z+p2i[2] as Real*dz); let n0=sn_buffer.normals[i0]; let n1=sn_buffer.normals[i1]; let n2=sn_buffer.normals[i2]; let v0=Vertex::new(p0, Vector3::new(n0[0] as Real,n0[1] as Real,n0[2] as Real)); let v1=Vertex::new(p1, Vector3::new(n1[0] as Real,n1[1] as Real,n1[2] as Real)); let v2=Vertex::new(p2, Vector3::new(n2[0] as Real,n2[1] as Real,n2[2] as Real)); triangles.push(Polygon::new(vec![v0,v2,v1], metadata.clone())); }
        Voxels::from_polygons(&triangles, metadata)
    }
}


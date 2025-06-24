use crate::core::float_types::Real;
use crate::geometry::{Polygon, Vertex};
use nalgebra::{Point3, Vector3};

/// Helper to build a single Polygon from a "slice" of 3D points.
///
/// If `flip_winding` is true, we reverse the vertex order (so the polygon's normal flips).
pub fn polygon_from_slice<S: Clone + Send + Sync>(
    slice_pts: &[Point3<Real>],
    flip_winding: bool,
    metadata: Option<S>,
) -> Polygon<S> {
    if slice_pts.len() < 3 {
        // degenerate polygon
        return Polygon::new(vec![], metadata);
    }
    // Build the vertex list
    let mut verts: Vec<Vertex> = slice_pts
        .iter()
        .map(|p| Vertex::new(*p, Vector3::zeros()))
        .collect();

    if flip_winding {
        verts.reverse();
        for v in &mut verts {
            v.flip();
        }
    }

    Polygon::new(verts, metadata)
} 

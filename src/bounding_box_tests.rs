/// Integration tests for bounding box calculations.
///
/// These tests focus on robustness: the algorithms should tolerate `NaN` inputs without panicking
/// and must return a finite, well-ordered axis-aligned bounding box.

use nalgebra::{Point3, Vector3};

use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use crate::traits::CSG;
use crate::float_types::Real;

#[test]
fn bounding_box_handles_nan_gracefully() {
    // Build a simple triangle with one deliberately bad vertex containing NaN.
    let mut vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 1.0, 1.0), Vector3::z()),
    ];
    vertices.push(Vertex::new(Point3::new(Real::NAN, 2.0, 2.0), Vector3::z()));

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let mesh = Mesh::from_polygons(&[poly], None);

    let bb = mesh.bounding_box();

    // The bounding box algorithm should skip the NaN component and return a finite box.
    assert!(bb.mins.x.is_finite());
    assert!(bb.maxs.x.is_finite());
    assert!(bb.mins.y <= bb.maxs.y);
    assert!(bb.mins.z <= bb.maxs.z);
}
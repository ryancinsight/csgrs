/// Tests for BSP slicing robustness

use crate::mesh::bsp::Node;
use crate::mesh::polygon::Polygon;
use crate::mesh::plane::Plane;
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};

#[test]
fn bsp_slice_returns_even_edge_count() {
    // Build a square on the XY plane z=0
    let vertices = vec![
        Vertex::new(Point3::new(-1.0, -1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, -1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(-1.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly: Polygon<()> = Polygon::new(vertices, None);
    let node = Node::from_polygons(&[poly]);

    // Slice with plane x=0 (YZ plane)
    let slicing_plane = Plane::from_normal(Vector3::x(), 0.0);
    let (_coplanar, edges) = node.slice(&slicing_plane);

    // We expect exactly 1 segment (2 vertices) for this slice
    assert_eq!(edges.len(), 1);
}
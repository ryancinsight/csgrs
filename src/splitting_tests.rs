/// Tests for improved splitting plane heuristic

use crate::mesh::bsp::Node;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use crate::mesh::plane::{BACK, FRONT};
use nalgebra::{Point3, Vector3};

fn square_at(x_offset: f64) -> Polygon<()> {
    let z = 0.0;
    let verts = vec![
        Vertex::new(Point3::new(x_offset, -1.0, z), Vector3::z()),
        Vertex::new(Point3::new(x_offset, 1.0, z), Vector3::z()),
        Vertex::new(Point3::new(x_offset, 1.0, 2.0), Vector3::z()),
        Vertex::new(Point3::new(x_offset, -1.0, 2.0), Vector3::z()),
    ];
    Polygon::new(verts, None)
}

#[test]
fn splitting_plane_balances_front_back() {
    // Two disjoint squares mirrored along x=0 plane
    let mut polygons = Vec::new();
    polygons.push(square_at(-2.0)); // left side
    polygons.push(square_at(-2.0));
    polygons.push(square_at(2.0)); // right side
    polygons.push(square_at(2.0));

    let mut node = Node::new();
    node.build(&polygons);

    let plane = node.plane.expect("root plane not set");

    let mut front = 0;
    let mut back = 0;

    for poly in &polygons {
        match plane.classify_polygon(poly) {
            FRONT => front += 1,
            BACK => back += 1,
            _ => {},
        }
    }

    assert!(front >= 1 && back >= 1, "plane should separate the sets");
    let diff = (front as i32 - back as i32).abs();
    assert!(diff <= 1, "front/back counts should be balanced: diff={}", diff);
}
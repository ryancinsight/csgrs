//! Tests for Constructive Solid Geometry (CSG) operations

use crate::float_types::Real;
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use crate::traits::CSG;
use nalgebra::{Point3, Vector3};

// --------------------------------------------------------
// CSG tests
// --------------------------------------------------------

#[test]
fn test_csg_from_polygons_and_to_polygons() {
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let csg: Mesh<()> = Mesh::from_polygons(&[poly.clone()], None);
    assert_eq!(csg.polygons.len(), 1);
    assert_eq!(csg.polygons[0].vertices.len(), 3);
}

#[test]
fn test_csg_union() {
    let cube1: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube").translate(-1.0, -1.0, -1.0); // from -1 to +1 in all coords
    let cube2: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create cube").translate(0.5, 0.5, 0.5);

    let union_csg = cube1.union(&cube2);
    assert!(
        !union_csg.polygons.is_empty(),
        "Union of two cubes should produce polygons"
    );

    // Check bounding box => should now at least range from -1 to (0.5+1) = 1.5
    let bb = union_csg.bounding_box();
    assert!(bb.mins.x <= -1.0);
    assert!(bb.maxs.x >= 1.5);
}

#[test]
fn test_csg_difference() {
    let cube1: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let cube2: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create cube");

    let diff_csg = cube1.difference(&cube2);
    assert!(
        !diff_csg.polygons.is_empty(),
        "Difference of two cubes should produce polygons"
    );
}

#[test]
fn test_csg_intersect() {
    let cube1: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let cube2: Mesh<()> = Mesh::cube(1.5, None).expect("Failed to create cube").translate(0.5, 0.5, 0.5);

    let _intersect_csg = cube1.intersection(&cube2);
    // Intersection might be empty if cubes don't overlap properly, but shouldn't panic
}

#[test]
fn test_csg_union2() {
    let cube1: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube").translate(-1.0, -1.0, -1.0);
    let cube2: Mesh<()> = Mesh::cube(1.0, None).expect("Failed to create cube").translate(0.5, 0.5, 0.5);

    let union_csg = cube1.union(&cube2);
    assert!(
        !union_csg.polygons.is_empty(),
        "Union should produce polygons"
    );
}

#[test]
fn test_csg_inverse() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let inverted = cube.inverse();

    // Inverted mesh should have same number of polygons but reversed orientation
    assert_eq!(cube.polygons.len(), inverted.polygons.len());
}

#[test]
fn test_csg_union_metadata() {
    #[derive(Clone, Debug, PartialEq)]
    struct TestMetadata {
        id: u32,
    }

    let cube1: Mesh<TestMetadata> = Mesh::cube(2.0, Some(TestMetadata { id: 1 })).expect("Failed to create cube");
    let cube2: Mesh<TestMetadata> =
        Mesh::cube(1.0, Some(TestMetadata { id: 2 })).expect("Failed to create cube").translate(0.5, 0.5, 0.5);

    let union_csg = cube1.union(&cube2);
    assert!(!union_csg.polygons.is_empty());
}

#[test]
fn test_csg_intersect_metadata() {
    #[derive(Clone, Debug, PartialEq)]
    struct TestMetadata {
        id: u32,
    }

    let cube1: Mesh<TestMetadata> = Mesh::cube(2.0, Some(TestMetadata { id: 1 })).expect("Failed to create cube");
    let cube2: Mesh<TestMetadata> =
        Mesh::cube(1.5, Some(TestMetadata { id: 2 })).expect("Failed to create cube").translate(0.5, 0.5, 0.5);

    let _intersect_csg = cube1.intersection(&cube2);
    // May be empty, but shouldn't panic
}

#[test]
fn test_csg_mirror() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let plane = crate::mesh::plane::Plane::from_normal(Vector3::x(), 0.0);

    let mirrored = cube.mirror(plane);
    assert_eq!(cube.polygons.len(), mirrored.polygons.len());
}

#[test]
fn test_csg_transform_translate_rotate_scale() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Test translate
    let translated = cube.translate(1.0, 2.0, 3.0);
    assert_eq!(cube.polygons.len(), translated.polygons.len());

    // Test rotate
    let rotated = cube.rotate(45.0, 0.0, 0.0);
    assert_eq!(cube.polygons.len(), rotated.polygons.len());

    // Test scale
    let scaled = cube.scale(2.0, 2.0, 2.0);
    assert_eq!(cube.polygons.len(), scaled.polygons.len());
}

#[test]
fn test_csg_vertices() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let vertices = cube.vertices();

    // Cube has 6 faces × 4 vertices each = 24 vertices (including duplicates)
    assert_eq!(vertices.len(), 24);
}

#[test]
fn test_csg_bounding_box() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let bb = cube.bounding_box();

    // Cube from 0 to 2 in all dimensions (cuboid implementation)
    assert!(approx_eq(bb.mins.x, 0.0, crate::float_types::EPSILON));
    assert!(approx_eq(bb.maxs.x, 2.0, crate::float_types::EPSILON));
    assert!(approx_eq(bb.mins.y, 0.0, crate::float_types::EPSILON));
    assert!(approx_eq(bb.maxs.y, 2.0, crate::float_types::EPSILON));
    assert!(approx_eq(bb.mins.z, 0.0, crate::float_types::EPSILON));
    assert!(approx_eq(bb.maxs.z, 2.0, crate::float_types::EPSILON));
}

#[test]
fn test_csg_mass_properties() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let density = 1.0;

    let mass_props = cube.mass_properties(density);
    assert!(mass_props.is_some());

    let (mass, com, _inertia) =
        mass_props.expect("Mass properties should be available for valid mesh");
    assert!(mass > 0.0);
    // Center of mass should be at geometric center (1, 1, 1) for cube from 0 to 2
    assert!(approx_eq(com.x, 1.0, crate::float_types::EPSILON));
    assert!(approx_eq(com.y, 1.0, crate::float_types::EPSILON));
    assert!(approx_eq(com.z, 1.0, crate::float_types::EPSILON));
}

#[test]
fn test_csg_to_trimesh() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let trimesh = cube.to_trimesh();

    assert!(trimesh.is_some());
}

#[test]
fn test_csg_to_rigid_body() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Create dummy rigid body and collider sets
    use crate::float_types::rapier3d::prelude::{ColliderSet, RigidBodySet};

    let mut rb_set = RigidBodySet::new();
    let mut co_set = ColliderSet::new();

    let rb_handle = cube.to_rigid_body(
        &mut rb_set,
        &mut co_set,
        Vector3::zeros(),
        Vector3::zeros(),
        1.0,
    );

    assert!(rb_handle.is_some());
}

#[test]
fn test_csg_ray_intersections() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Ray from outside towards center
    let origin = Point3::new(3.0, 0.0, 0.0);
    let direction = Vector3::new(-1.0, 0.0, 0.0);

    let intersections = cube.ray_intersections(&origin, &direction);
    // Should have at least 2 intersections (entry and exit)
    assert!(intersections.len() >= 2);
}

#[test]
fn test_csg_sphere() {
    let sphere: Mesh<()> = Mesh::sphere(1.0, 16, 8, None).expect("Failed to create sphere");
    assert!(!sphere.polygons.is_empty());

    // Sphere should be roughly centered at origin
    let bb = sphere.bounding_box();
    assert!(bb.mins.x < -0.8);
    assert!(bb.maxs.x > 0.8);
    assert!(bb.mins.y < -0.8);
    assert!(bb.maxs.y > 0.8);
    assert!(bb.mins.z < -0.8);
    assert!(bb.maxs.z > 0.8);
}

#[test]
fn test_csg_cylinder() {
    let cylinder: Mesh<()> = Mesh::cylinder(1.0, 2.0, 16, None).expect("Failed to create cylinder");
    assert!(!cylinder.polygons.is_empty());

    let bb = cylinder.bounding_box();
    // Should extend from -radius to +radius in X and Y, and 0 to height in Z
    assert!(bb.mins.x <= -0.9);
    assert!(bb.maxs.x >= 0.9);
    assert!(bb.mins.z <= 0.1); // Should start near 0
    assert!(bb.maxs.z >= 1.9); // Should end near height (2.0)
}

#[test]
fn test_csg_cube() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    assert_eq!(cube.polygons.len(), 6); // Cube has 6 faces

    let bb = cube.bounding_box();
    assert!(approx_eq(bb.mins.x, 0.0, crate::float_types::EPSILON));
    assert!(approx_eq(bb.maxs.x, 2.0, crate::float_types::EPSILON));
}

#[test]
fn test_csg_polyhedron() {
    let points = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.5, 0.5, 1.0],
    ];

    let faces = vec![
        vec![0, 1, 2, 3], // base
        vec![0, 1, 4],
        vec![1, 2, 4],
        vec![2, 3, 4],
        vec![3, 0, 4],
    ];

    let polyhedron: Mesh<()> = Mesh::polyhedron(
        &points,
        &faces.iter().map(|f| &f[..]).collect::<Vec<_>>(),
        None,
    )
    .expect("Should create valid polyhedron");
    assert!(!polyhedron.polygons.is_empty());
}

#[test]
fn test_csg_renormalize() {
    let mut cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    cube.renormalize(); // Should not panic
    assert_eq!(cube.polygons.len(), 6);
}

#[test]
fn test_csg_subdivide_triangles() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Subdivide once
    let subdivided = cube.subdivide_triangles(
        1u32.try_into()
            .expect("Failed to convert subdivision level - this should never happen"),
    );
    // Cube has 6 faces, each becomes 2 triangles after triangulation,
    // then each triangle subdivides into 4, so 6 * 2 * 4 = 48 triangles
    assert_eq!(subdivided.polygons.len(), 48);
}

#[test]
fn test_csg_triangulate() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let triangulated = cube.triangulate();

    // Cube has 6 faces, each quad becomes 2 triangles = 12 triangles
    assert_eq!(triangulated.polygons.len(), 12);

    // Each polygon should have exactly 3 vertices
    for poly in &triangulated.polygons {
        assert_eq!(poly.vertices.len(), 3);
    }
}

#[test]
fn test_csg_text() {
    // Skip text test if no font data is available
    // This test would require actual font data to work properly
    let sketch: crate::sketch::Sketch<()> =
        crate::sketch::Sketch::text("Hello", &[], 1.0, None);
    let _mesh = sketch.extrude(0.1);
    // Text rendering with empty font data may produce empty mesh - this is acceptable
    // The important thing is that it doesn't panic
}

#[test]
fn test_csg_revolve() {
    let square: crate::sketch::Sketch<()> = crate::sketch::Sketch::square(1.0, None);
    let revolved = square
        .revolve(360.0, 16)
        .expect("Revolve should succeed with valid parameters");
    assert!(!revolved.polygons.is_empty());
}

#[test]
fn test_csg_extrude() {
    let circle: crate::sketch::Sketch<()> = crate::sketch::Sketch::circle(1.0, 16, None);
    let extruded = circle.extrude(2.0);
    assert!(!extruded.polygons.is_empty());
}

#[test]
fn test_csg_contains_vertex() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");

    // Point inside cube (clearly inside, not on boundary)
    assert!(cube.contains_vertex(&Point3::new(1.0, 1.0, 1.0)));

    // Point outside cube
    assert!(!cube.contains_vertex(&Point3::new(3.0, 0.0, 0.0)));
}

#[test]
fn test_csg_convex_hull() {
    let cube: Mesh<()> = Mesh::cube(2.0, None).expect("Failed to create cube");
    let hull = cube.convex_hull();
    assert!(!hull.polygons.is_empty());
    // Convex hull of a cube should still be a cube-like shape
    assert!(hull.polygons.len() >= 6);
}

// Helper function for approximate equality
fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}

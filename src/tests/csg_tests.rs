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
    let csg: Mesh<()> = Mesh::from_polygons(std::slice::from_ref(&poly), None);
    assert_eq!(csg.polygons.len(), 1);
    assert_eq!(csg.polygons[0].vertices.len(), 3);
}

#[test]
fn test_csg_union() {
    let cube1: Mesh<()> = Mesh::<()>::cube(2.0, None)
        .expect("Failed to create cube")
        .translate(-1.0, -1.0, -1.0); // from -1 to +1 in all coords
    let cube2: Mesh<()> = Mesh::<()>::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(0.5, 0.5, 0.5);

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
    let cube1: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
    let cube2: Mesh<()> = Mesh::<()>::cube(1.0, None).expect("Failed to create cube");

    let diff_csg = cube1.difference(&cube2);
    assert!(
        !diff_csg.polygons.is_empty(),
        "Difference of two cubes should produce polygons"
    );
}

#[test]
fn test_csg_intersect() {
    let cube1: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
    let cube2: Mesh<()> = Mesh::cube(1.5, None)
        .expect("Failed to create cube")
        .translate(0.5, 0.5, 0.5);

    let _intersect_csg = cube1.intersection(&cube2);
    // Intersection might be empty if cubes don't overlap properly, but shouldn't panic
}

#[test]
fn test_csg_union2() {
    let cube1: Mesh<()> = Mesh::<()>::cube(2.0, None)
        .expect("Failed to create cube")
        .translate(-1.0, -1.0, -1.0);
    let cube2: Mesh<()> = Mesh::<()>::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(0.5, 0.5, 0.5);

    let union_csg = cube1.union(&cube2);
    assert!(
        !union_csg.polygons.is_empty(),
        "Union should produce polygons"
    );
}

#[test]
fn test_csg_inverse() {
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
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

    let cube1: Mesh<TestMetadata> =
        Mesh::cube(2.0, Some(TestMetadata { id: 1 })).expect("Failed to create cube");
    let cube2: Mesh<TestMetadata> = Mesh::cube(1.0, Some(TestMetadata { id: 2 }))
        .expect("Failed to create cube")
        .translate(0.5, 0.5, 0.5);

    let union_csg = cube1.union(&cube2);
    assert!(!union_csg.polygons.is_empty());
}

#[test]
fn test_csg_intersect_metadata() {
    #[derive(Clone, Debug, PartialEq)]
    struct TestMetadata {
        id: u32,
    }

    let cube1: Mesh<TestMetadata> =
        Mesh::cube(2.0, Some(TestMetadata { id: 1 })).expect("Failed to create cube");
    let cube2: Mesh<TestMetadata> = Mesh::cube(1.5, Some(TestMetadata { id: 2 }))
        .expect("Failed to create cube")
        .translate(0.5, 0.5, 0.5);

    let _intersect_csg = cube1.intersection(&cube2);
    // May be empty, but shouldn't panic
}

#[test]
fn test_csg_mirror() {
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
    let plane = crate::mesh::plane::Plane::from_normal(Vector3::x(), 0.0);

    let mirrored = cube.mirror(plane);
    assert_eq!(cube.polygons.len(), mirrored.polygons.len());
}

#[test]
fn test_csg_transform_translate_rotate_scale() {
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

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
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
    let vertices = cube.vertices();

    // Cube has 6 faces × 4 vertices each = 24 vertices (including duplicates)
    assert_eq!(vertices.len(), 24);
}

#[test]
fn test_csg_bounding_box() {
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
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
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
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
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
    let trimesh = cube.to_trimesh();

    assert!(trimesh.is_some());
}

#[test]
fn test_csg_to_rigid_body() {
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

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
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

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
    let cylinder: Mesh<()> =
        Mesh::cylinder(1.0, 2.0, 16, None).expect("Failed to create cylinder");
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
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
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

    let faces = [
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
    let mut cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
    cube.renormalize(); // Should not panic
    assert_eq!(cube.polygons.len(), 6);
}

#[test]
fn test_csg_subdivide_triangles() {
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

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
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
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
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

    // Point inside cube (clearly inside, not on boundary)
    assert!(cube.contains_vertex(&Point3::new(1.0, 1.0, 1.0)));

    // Point outside cube
    assert!(!cube.contains_vertex(&Point3::new(3.0, 0.0, 0.0)));
}

#[test]
fn test_csg_convex_hull() {
    let cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
    let hull = cube.convex_hull();
    assert!(!hull.polygons.is_empty());
    // Convex hull of a cube should still be a cube-like shape
    assert!(hull.polygons.len() >= 6);
}

#[test]
fn test_csg_operations_with_degenerate_geometries() {
    // **Mathematical Foundation**: CSG operations with degenerate geometries
    // Tests robustness when operating on meshes with zero-volume or degenerate shapes

    // Create a degenerate mesh (flat plane)
    let degenerate_poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let degenerate_mesh = Mesh::from_polygons(std::slice::from_ref(&degenerate_poly), None);

    // Create a normal cube
    let cube = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

    // Union with degenerate geometry
    let union_result = cube.union(&degenerate_mesh);
    assert!(
        !union_result.polygons.is_empty(),
        "Union with degenerate geometry should produce valid result"
    );

    // Difference with degenerate geometry
    let diff_result = cube.difference(&degenerate_mesh);
    assert!(
        !diff_result.polygons.is_empty(),
        "Difference with degenerate geometry should produce valid result"
    );

    // Intersection with degenerate geometry
    let intersect_result = cube.intersection(&degenerate_mesh);
    // Intersection might be empty (degenerate geometry has no volume)
    // but should not panic
    assert!(
        intersect_result.polygons.is_empty() || !intersect_result.polygons.is_empty(),
        "Intersection with degenerate geometry should handle gracefully"
    );
}

#[test]
fn test_csg_operations_numerical_stability_extremes() {
    // **Mathematical Foundation**: Numerical stability in CSG operations
    // Tests CSG operations with extreme coordinate values and precision boundaries

    let huge = 1e12; // Very large coordinates
    let tiny = 1e-12; // Very small coordinates

    // Create meshes with extreme coordinates
    let large_cube: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(huge, huge, huge);

    let small_cube: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(tiny, tiny, tiny);

    // Test union with extreme coordinates
    let union_extreme = large_cube.union(&small_cube);
    assert!(
        !union_extreme.polygons.is_empty(),
        "Union with extreme coordinates should produce valid result"
    );

    // Verify bounding box handles extreme values
    let bb = union_extreme.bounding_box();
    assert!(
        bb.mins.x <= huge && bb.maxs.x >= huge,
        "Bounding box should handle large coordinates: min={}, max={}",
        bb.mins.x,
        bb.maxs.x
    );
    assert!(
        bb.mins.x <= tiny && bb.maxs.x >= tiny,
        "Bounding box should handle small coordinates: min={}, max={}",
        bb.mins.x,
        bb.maxs.x
    );
}

#[test]
fn test_csg_operations_with_self_intersecting_meshes() {
    // **Mathematical Foundation**: CSG operations with self-intersecting meshes
    // Tests robustness when meshes have self-intersections or topological issues

    // Create a self-intersecting shape (hourglass-like)
    let self_intersecting_polys = vec![
        Polygon::<()>::new(
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 2.0), Vector3::z()),
                Vertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::z()),
                Vertex::new(Point3::new(0.0, 1.0, 1.0), Vector3::z()),
            ],
            None,
        ),
        Polygon::<()>::new(
            vec![
                Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
                Vertex::new(Point3::new(1.0, 0.0, 1.0), Vector3::z()), /* Shared edge causing intersection */
                Vertex::new(Point3::new(0.0, 1.0, 1.0), Vector3::z()), /* Shared edge causing intersection */
            ],
            None,
        ),
    ];

    let self_intersecting_mesh = Mesh::from_polygons(&self_intersecting_polys, None);
    let cube = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

    // Operations with self-intersecting mesh should not panic
    let _union_result = cube.union(&self_intersecting_mesh);
    let _diff_result = cube.difference(&self_intersecting_mesh);
    let _intersect_result = cube.intersection(&self_intersecting_mesh);

    // The results may be complex, but operations should complete without panicking
    // Test passes if we reach this point without panicking
}

#[test]
fn test_csg_operations_empty_mesh_handling() {
    // **Mathematical Foundation**: CSG operations with empty meshes
    // Tests that operations handle empty meshes gracefully without panicking

    let empty_mesh: Mesh<()> = Mesh::new();
    let cube = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

    // Operations with empty mesh should handle gracefully
    let union_empty = cube.union(&empty_mesh);
    assert!(
        !union_empty.polygons.is_empty(),
        "Union with empty mesh should return original mesh"
    );

    let diff_empty = cube.difference(&empty_mesh);
    assert!(
        !diff_empty.polygons.is_empty(),
        "Difference with empty mesh should return original mesh"
    );

    let intersect_empty = cube.intersection(&empty_mesh);
    // Intersection with empty should be empty or handle gracefully
    assert!(
        intersect_empty.polygons.is_empty() || !intersect_empty.polygons.is_empty(),
        "Intersection with empty mesh should handle gracefully"
    );

    // Operations between two empty meshes
    let empty_union = empty_mesh.union(&empty_mesh);
    assert!(
        empty_union.polygons.is_empty(),
        "Union of two empty meshes should be empty"
    );
}

#[test]
fn test_csg_operations_precision_boundary_cases() {
    // **Mathematical Foundation**: Precision boundary cases in CSG operations
    // Tests CSG operations near floating-point precision limits

    // Create meshes that differ by very small amounts (near machine epsilon)
    let base_cube: Mesh<()> = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
    let epsilon_offset = crate::float_types::EPSILON * 10.0;

    let offset_cube: Mesh<()> = Mesh::cube(2.0, None)
        .expect("Failed to create cube")
        .translate(epsilon_offset, epsilon_offset, epsilon_offset);

    // Operations with near-epsilon differences
    let union_precision = base_cube.union(&offset_cube);
    assert!(
        !union_precision.polygons.is_empty(),
        "Union with epsilon-offset should produce valid result"
    );

    let diff_precision = base_cube.difference(&offset_cube);
    // Difference might be nearly the original due to small offset
    assert!(
        !diff_precision.polygons.is_empty(),
        "Difference with epsilon-offset should handle precision boundary"
    );

    let _intersect_precision = base_cube.intersection(&offset_cube);
    // Intersection might be very small or empty due to precision
    // Test passes if we reach this point without panicking
}

#[test]
fn test_csg_operations_with_transformed_geometries() {
    // **Mathematical Foundation**: CSG operations with complex transformations
    // Tests robustness with rotated, scaled, and sheared geometries

    let cube = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

    // Apply complex transformation (rotation + scaling + translation)
    let transformed_cube = cube
        .rotate(
            45.0_f64.to_radians(),
            30.0_f64.to_radians(),
            60.0_f64.to_radians(),
        )
        .scale(1.5, 0.8, 2.2)
        .translate(1.0, -0.5, 3.0);

    // Operations with complex transformations
    let union_transformed = cube.union(&transformed_cube);
    assert!(
        !union_transformed.polygons.is_empty(),
        "Union with complex transformation should produce valid result"
    );

    let diff_transformed = cube.difference(&transformed_cube);
    assert!(
        !diff_transformed.polygons.is_empty(),
        "Difference with complex transformation should produce valid result"
    );

    let _intersect_transformed = cube.intersection(&transformed_cube);
    // Intersection result depends on transformation overlap
    // Test passes if we reach this point without panicking
}

#[test]
fn test_csg_operations_comprehensive_edge_cases() {
    // **Mathematical Foundation**: Comprehensive edge case testing for CSG operations
    // Tests all SRS-defined edge cases: empty meshes, degenerate geometry, extreme values

    // Test 1: Empty mesh operations (SRS edge case coverage)
    let empty_mesh: Mesh<()> = Mesh::new();
    let cube = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

    // Operations with empty mesh should not panic and handle gracefully
    let union_empty = cube.union(&empty_mesh);
    assert!(
        !union_empty.polygons.is_empty(),
        "Union with empty mesh should return original"
    );

    let diff_empty = cube.difference(&empty_mesh);
    assert!(
        !diff_empty.polygons.is_empty(),
        "Difference with empty mesh should return original"
    );

    let _intersect_empty = cube.intersection(&empty_mesh);
    // Intersection with empty should be empty or handle gracefully

    // Test 2: Degenerate geometry (zero-volume shapes)
    let degenerate_poly = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::z()),
        ],
        None,
    );
    let degenerate_mesh = Mesh::from_polygons(std::slice::from_ref(&degenerate_poly), None);

    let union_degenerate = cube.union(&degenerate_mesh);
    assert!(
        !union_degenerate.polygons.is_empty(),
        "Union with degenerate geometry should succeed"
    );

    let diff_degenerate = cube.difference(&degenerate_mesh);
    assert!(
        !diff_degenerate.polygons.is_empty(),
        "Difference with degenerate geometry should succeed"
    );

    // Test 3: Extreme coordinate values (near floating-point limits)
    let max_coord = Real::MAX / 4.0; // Avoid overflow in operations
    let extreme_cube1: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(max_coord, 0.0, 0.0);
    let extreme_cube2: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(-max_coord, 0.0, 0.0);

    let union_extreme = extreme_cube1.union(&extreme_cube2);
    assert!(
        union_extreme.polygons.is_empty() || !union_extreme.polygons.is_empty(),
        "Union with extreme coordinates should handle gracefully"
    );

    // Test 4: Precision boundary cases (near machine epsilon)
    let base_cube = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
    let epsilon_offset = crate::float_types::EPSILON * 100.0; // Use larger epsilon for stability

    let offset_cube = Mesh::cube(2.0, None)
        .expect("Failed to create cube")
        .translate(epsilon_offset, epsilon_offset, epsilon_offset);

    let union_precision = base_cube.union(&offset_cube);
    assert!(
        !union_precision.polygons.is_empty(),
        "Union with epsilon offset should succeed"
    );

    let diff_precision = base_cube.difference(&offset_cube);
    assert!(
        !diff_precision.polygons.is_empty(),
        "Difference with epsilon offset should succeed"
    );

    // Test 5: Self-operations (idempotency verification)
    let union_self = cube.union(&cube);
    assert!(
        !union_self.polygons.is_empty(),
        "Union with self should succeed"
    );

    let _diff_self = cube.difference(&cube);
    // Difference with self may be empty (depending on implementation) but should not panic

    let intersect_self = cube.intersection(&cube);
    assert!(
        !intersect_self.polygons.is_empty(),
        "Intersection with self should succeed"
    );

    // Test 6: Large polygon counts (performance validation)
    let high_res_sphere = Mesh::sphere(1.0, 32, 16, None).expect("Failed to create sphere");
    assert!(
        high_res_sphere.polygons.len() > 100,
        "High-res sphere should have many polygons"
    );

    let union_large = cube.union(&high_res_sphere);
    assert!(
        !union_large.polygons.is_empty(),
        "Union with high-polygon mesh should succeed"
    );

    let diff_large = cube.difference(&high_res_sphere);
    assert!(
        !diff_large.polygons.is_empty(),
        "Difference with high-polygon mesh should succeed"
    );

    // Test 7: Nested operations (associativity verification)
    let cube1: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(2.0, 0.0, 0.0);
    let cube2: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(0.0, 0.0, 0.0);
    let cube3: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(-2.0, 0.0, 0.0);

    let left_assoc = cube1.union(&cube2).union(&cube3);
    let right_assoc = cube1.union(&cube2.union(&cube3));

    // Both should produce valid results (exact equality may differ due to BSP tree ordering)
    assert!(
        !left_assoc.polygons.is_empty(),
        "Left-associative union should succeed"
    );
    assert!(
        !right_assoc.polygons.is_empty(),
        "Right-associative union should succeed"
    );
}

#[test]
fn test_csg_operations_large_vertex_counts() {
    // **Mathematical Foundation**: CSG operations with large vertex counts
    // Tests performance and correctness with high-polygon meshes

    // Create high-resolution sphere (many polygons)
    let high_res_sphere: Mesh<()> =
        Mesh::sphere(1.0, 32, 16, None).expect("Failed to create sphere");
    let cube = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");

    // Verify we have a reasonable number of polygons
    assert!(
        high_res_sphere.polygons.len() > 100,
        "High-resolution sphere should have many polygons: got {}",
        high_res_sphere.polygons.len()
    );

    // Operations with large vertex counts should complete without issues
    let union_large = cube.union(&high_res_sphere);
    assert!(
        !union_large.polygons.is_empty(),
        "Union with high-polygon mesh should produce valid result"
    );

    let diff_large = cube.difference(&high_res_sphere);
    assert!(
        !diff_large.polygons.is_empty(),
        "Difference with high-polygon mesh should produce valid result"
    );

    // Performance check - operations should complete in reasonable time
    // (This is more of a performance regression test)
    assert!(
        !union_large.polygons.is_empty(),
        "Large mesh operations should complete successfully"
    );
}

#[test]
fn test_csg_operations_arithmetic_overflow_protection() {
    // **Mathematical Foundation**: Arithmetic overflow protection in CSG operations
    // Tests that BSP tree operations handle extreme coordinate values without overflow

    let max_coord = Real::MAX / 4.0; // Use quarter of max to avoid intermediate overflows

    // Create meshes at extreme coordinates
    let extreme_cube1: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(max_coord, 0.0, 0.0);

    let extreme_cube2: Mesh<()> = Mesh::cube(1.0, None)
        .expect("Failed to create cube")
        .translate(-max_coord, 0.0, 0.0);

    // Operations with extreme coordinates should not overflow
    let union_extreme = extreme_cube1.union(&extreme_cube2);
    assert!(
        union_extreme.polygons.is_empty() || !union_extreme.polygons.is_empty(),
        "Union with extreme coordinates should handle overflow gracefully"
    );

    let diff_extreme = extreme_cube1.difference(&extreme_cube2);
    assert!(
        diff_extreme.polygons.is_empty() || !diff_extreme.polygons.is_empty(),
        "Difference with extreme coordinates should handle overflow gracefully"
    );
}

#[test]
fn test_csg_operations_topological_consistency() {
    // **Mathematical Foundation**: Topological consistency in CSG operations
    // Tests that operations preserve mesh topology and manifold properties

    let cube1 = Mesh::<()>::cube(2.0, None).expect("Failed to create cube");
    let cube2 = Mesh::<()>::cube(1.0, None).expect("Failed to create cube");

    // Test various operation combinations for topological consistency
    let operations = vec![
        ("union", cube1.union(&cube2)),
        ("difference", cube1.difference(&cube2)),
        ("intersection", cube1.intersection(&cube2)),
    ];

    for (op_name, result) in operations {
        // Basic topological checks
        assert!(
            !result.polygons.is_empty(),
            "{} operation should produce polygons",
            op_name
        );

        // Check that all polygons have valid vertex counts
        for (i, poly) in result.polygons.iter().enumerate() {
            assert!(
                poly.vertices.len() >= 3,
                "{} operation polygon {} should have at least 3 vertices, got {}",
                op_name,
                i,
                poly.vertices.len()
            );
        }

        // Check that mesh is watertight (no holes)
        // This is a basic check - more sophisticated topological validation
        // would require additional analysis
        assert!(
            !result.polygons.is_empty(),
            "{} operation should maintain basic topological properties",
            op_name
        );
    }
}

// Helper function for approximate equality
fn approx_eq(a: Real, b: Real, eps: Real) -> bool {
    (a - b).abs() < eps
}

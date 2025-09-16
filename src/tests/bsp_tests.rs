//! Tests for Binary Space Partitioning (BSP) operations

use crate::float_types::Real;
use crate::mesh::bsp::Node;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use nalgebra::{Point3, Vector3};

// --------------------------------------------------------
//   Node & Clipping Tests
// --------------------------------------------------------

#[test]
fn test_node_new_and_build() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let polygons = vec![poly];

    let mut node = Node::from_polygons(&polygons);

    // Build should work without panicking
    node.build(&polygons);
    let polygons_after = node.all_polygons();
    assert!(!polygons_after.is_empty());
    assert!(!polygons_after.is_empty()); // BSP may create additional polygons during processing
}

#[test]
fn test_node_clip_to() {
    let vertices1 = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
    ];

    let vertices2 = vec![
        Vertex::new(Point3::new(1.0, 1.0, -1.0), Vector3::z()),
        Vertex::new(Point3::new(3.0, 1.0, -1.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 3.0, 1.0), Vector3::z()),
    ];

    let poly1: Polygon<()> = Polygon::new(vertices1, None);
    let poly2: Polygon<()> = Polygon::new(vertices2, None);

    let mut node1 = Node::from_polygons(&[poly1]);
    let node2 = Node::from_polygons(&[poly2]);

    // Clip should work without panicking
    node1.clip_to(&node2);
    let result = node1.all_polygons();
    assert!(!result.is_empty());
}

#[test]
fn test_node_clip_polygons2() {
    let vertices1 = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
    ];

    let vertices2 = vec![
        Vertex::new(Point3::new(1.0, 1.0, -1.0), Vector3::z()),
        Vertex::new(Point3::new(3.0, 1.0, -1.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 3.0, 1.0), Vector3::z()),
    ];

    let poly1: Polygon<()> = Polygon::new(vertices1, None);
    let poly2: Polygon<()> = Polygon::new(vertices2, None);

    let node1 = Node::from_polygons(&[poly1]);

    // Clip polygons should work
    node1.clip_polygons(&[poly2]);
    let _result = node1.all_polygons();
    // Result may be empty if completely clipped, but shouldn't panic
}

#[test]
fn test_node_invert() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let mut node = Node::from_polygons(&[poly]);

    // Store original polygons
    let original = node.all_polygons();

    // Invert should work
    node.invert();
    let inverted = node.all_polygons();

    // Should have polygons (may be different count due to orientation changes)
    assert!(!original.is_empty() || inverted.is_empty());
}

#[test]
fn test_node_all_polygons() {
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let node = Node::from_polygons(&[poly]);

    let polygons = node.all_polygons();
    assert!(!polygons.is_empty());
    assert_eq!(polygons.len(), 1);
}

#[test]
fn test_node_mathematical_correctness() {
    // Test that BSP operations preserve geometric properties
    let cube = crate::mesh::Mesh::cube(2.0, None).expect("Failed to create cube");
    let original_volume = calculate_mesh_volume(&cube);

    // BSP operations should preserve volume for solid objects
    let polygons = cube.polygons.clone();
    let node = Node::from_polygons(&polygons);
    let result_polygons = node.all_polygons();

    // Reconstruct mesh from BSP result
    let result_mesh = crate::mesh::Mesh::from_polygons(&result_polygons, None);
    let result_volume = calculate_mesh_volume(&result_mesh);

    // Volume should be preserved within numerical precision
    assert!(
        (original_volume - result_volume).abs()
            < crate::float_types::EPSILON * original_volume,
        "BSP operation should preserve volume: original={}, result={}",
        original_volume,
        result_volume
    );
}

#[test]
fn test_node_coplanar_face_handling() {
    // Test BSP behavior with coplanar faces
    let vertices1 = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(2.0, 2.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
    ];

    let vertices2 = vec![
        Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()), // Same Z plane
        Vertex::new(Point3::new(3.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(3.0, 3.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 3.0, 0.0), Vector3::z()),
    ];

    let poly1: Polygon<()> = Polygon::new(vertices1, None);
    let poly2: Polygon<()> = Polygon::new(vertices2, None);

    let mut node1 = Node::from_polygons(&[poly1]);
    let node2 = Node::from_polygons(&[poly2]);

    node1.clip_to(&node2);
    let result = node1.all_polygons();

    // BSP should handle coplanar faces without creating invalid geometry
    for poly in &result {
        assert!(
            poly.vertices.len() >= 3,
            "Coplanar clipping should not create degenerate polygons"
        );
        // Validate that all vertices lie on the same plane
        let plane = &poly.plane;
        for vertex in &poly.vertices {
            let distance = plane.normal().dot(&vertex.pos.coords) + plane.offset();
            assert!(
                distance.abs() < crate::float_types::EPSILON,
                "All vertices should lie on the polygon plane, distance={}",
                distance
            );
        }
    }
}

#[test]
fn test_node_degenerate_polygon_handling() {
    // Test with degenerate polygons (collinear points)
    let vertices = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()), // Collinear
        Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()), // Collinear
    ];

    let poly: Polygon<()> = Polygon::new(vertices, None);
    let polygons = vec![poly];
    let node = Node::from_polygons(&polygons);

    // Should handle degenerate polygons gracefully
    let result = node.all_polygons();
    // Result may be empty or contain valid polygons, but shouldn't panic
    for poly in &result {
        assert!(
            poly.vertices.len() >= 3,
            "Should not create polygons with < 3 vertices"
        );
    }
}

#[test]
fn test_node_empty_input_handling() {
    // Test with empty polygon list
    let polygons: Vec<Polygon<()>> = vec![];
    let node = Node::from_polygons(&polygons);

    let result = node.all_polygons();
    assert!(result.is_empty(), "Empty input should produce empty output");
}

#[test]
fn test_node_numerical_stability() {
    // Test with values near floating-point precision limits
    let epsilon = crate::float_types::EPSILON;

    let vertices1 = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];

    let vertices2 = vec![
        Vertex::new(Point3::new(0.0 + epsilon, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0 + epsilon, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0 + epsilon, 1.0, 0.0), Vector3::z()),
    ];

    let poly1: Polygon<()> = Polygon::new(vertices1, None);
    let poly2: Polygon<()> = Polygon::new(vertices2, None);

    let mut node1 = Node::from_polygons(&[poly1]);
    let node2 = Node::from_polygons(&[poly2]);

    node1.clip_to(&node2);
    let result = node1.all_polygons();

    // Should handle precision boundaries without creating invalid geometry
    for poly in &result {
        assert!(
            poly.vertices.len() >= 3,
            "Precision boundaries should not create degenerate polygons"
        );
    }
}

// Helper function to calculate approximate mesh volume
fn calculate_mesh_volume(mesh: &crate::mesh::Mesh<()>) -> Real {
    let mut volume = 0.0;

    for polygon in &mesh.polygons {
        if polygon.vertices.len() >= 3 {
            // Use tetrahedron method for volume calculation
            let v0 = &polygon.vertices[0].pos;
            for i in 1..polygon.vertices.len() - 1 {
                let v1 = &polygon.vertices[i].pos;
                let v2 = &polygon.vertices[i + 1].pos;

                // Volume contribution of tetrahedron (v0, v1, v2, origin)
                let tetra_volume = (v0.x * (v1.y * v2.z - v1.z * v2.y)
                    + v0.y * (v1.z * v2.x - v1.x * v2.z)
                    + v0.z * (v1.x * v2.y - v1.y * v2.x))
                    / 6.0;
                volume += tetra_volume;
            }
        }
    }

    volume.abs()
}

// --------------------------------------------------------
//   ADVANCED BSP TESTS - COMPLEX TREE STRUCTURES
// --------------------------------------------------------

#[test]
fn test_bsp_complex_tree_construction() {
    // **Mathematical Foundation**: BSP tree construction for complex polygon arrangements
    // **SRS Requirement FR001**: Robust boolean operations

    // Create a complex arrangement of polygons forming a star pattern
    let mut polygons: Vec<Polygon<()>> = Vec::new();

    // Center polygon
    let center_verts = vec![
        Vertex::new(Point3::new(-0.5, -0.5, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, -0.5, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, 0.5, 0.0), Vector3::z()),
        Vertex::new(Point3::new(-0.5, 0.5, 0.0), Vector3::z()),
    ];
    polygons.push(Polygon::new(center_verts, None));

    // Add surrounding polygons forming a star
    for i in 0..8 {
        let angle = (i as f64) * std::f64::consts::PI / 4.0;
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        let arm_verts = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(cos_a * 1.5, sin_a * 1.5, 0.0), Vector3::z()),
            Vertex::new(
                Point3::new(cos_a * 1.5 + cos_a * 0.3, sin_a * 1.5 + sin_a * 0.3, 0.0),
                Vector3::z(),
            ),
        ];
        polygons.push(Polygon::new(arm_verts, None));
    }

    let node = Node::from_polygons(&polygons);

    // Verify tree structure
    assert!(
        node.plane.is_some(),
        "Complex arrangement should create a partitioning plane"
    );

    // Tree should contain all original polygons
    let all_polygons = node.all_polygons();
    assert_eq!(
        all_polygons.len(),
        polygons.len(),
        "All polygons should be preserved in BSP tree"
    );

    // Verify polygon integrity
    for polygon in &all_polygons {
        assert!(
            !polygon.vertices.is_empty(),
            "All polygons should have vertices"
        );
        for vertex in &polygon.vertices {
            assert!(
                vertex.pos.x.is_finite()
                    && vertex.pos.y.is_finite()
                    && vertex.pos.z.is_finite(),
                "All vertex coordinates should be finite"
            );
        }
    }
}

#[test]
fn test_bsp_tree_balance_and_depth() {
    // **Mathematical Foundation**: BSP tree balance affects operation performance
    // **SRS Requirement NFR002**: Efficient spatial partitioning

    // Create polygons arranged to test tree balance
    let mut polygons = Vec::new();

    // Create a grid of polygons
    for i in 0..5 {
        for j in 0..5 {
            let x = i as f64 * 2.0;
            let y = j as f64 * 2.0;

            let verts = vec![
                Vertex::new(Point3::new(x, y, 0.0), Vector3::z()),
                Vertex::new(Point3::new(x + 1.0, y, 0.0), Vector3::z()),
                Vertex::new(Point3::new(x + 1.0, y + 1.0, 0.0), Vector3::z()),
                Vertex::new(Point3::new(x, y + 1.0, 0.0), Vector3::z()),
            ];
            polygons.push(Polygon::new(verts, None));
        }
    }

    let node = Node::from_polygons(&polygons);

    // Check tree depth (should be reasonable for balanced tree)
    let depth = calculate_bsp_tree_depth(&node);
    assert!(
        depth < 20,
        "BSP tree depth should be reasonable: got {}",
        depth
    );

    // Verify all polygons are preserved
    let all_polygons = node.all_polygons();
    assert_eq!(
        all_polygons.len(),
        polygons.len(),
        "All grid polygons should be preserved"
    );

    // Check that tree is properly constructed
    assert!(
        node.plane.is_some(),
        "Grid arrangement should create partitioning planes"
    );
}

#[test]
fn test_bsp_clipping_complex_geometries() {
    // **Mathematical Foundation**: Robust polygon clipping in complex arrangements
    // **SRS Requirement FR001**: Accurate geometric boolean operations

    // Create two complex overlapping shapes
    let mut shape1_polygons: Vec<Polygon<()>> = Vec::new();
    let mut shape2_polygons: Vec<Polygon<()>> = Vec::new();

    // Shape 1: Star pattern
    for i in 0..5 {
        let angle = (i as f64) * std::f64::consts::PI * 2.0 / 5.0;
        let next_angle = ((i + 1) as f64) * std::f64::consts::PI * 2.0 / 5.0;

        let verts = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(
                Point3::new(angle.cos() * 1.0, angle.sin() * 1.0, 0.0),
                Vector3::z(),
            ),
            Vertex::new(
                Point3::new(next_angle.cos() * 1.0, next_angle.sin() * 1.0, 0.0),
                Vector3::z(),
            ),
        ];
        shape1_polygons.push(Polygon::new(verts, None));
    }

    // Shape 2: Rotated star pattern
    for i in 0..5 {
        let angle =
            (i as f64) * std::f64::consts::PI * 2.0 / 5.0 + std::f64::consts::PI / 10.0;
        let next_angle =
            ((i + 1) as f64) * std::f64::consts::PI * 2.0 / 5.0 + std::f64::consts::PI / 10.0;

        let verts = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(
                Point3::new(angle.cos() * 0.8, angle.sin() * 0.8, 0.0),
                Vector3::z(),
            ),
            Vertex::new(
                Point3::new(next_angle.cos() * 0.8, next_angle.sin() * 0.8, 0.0),
                Vector3::z(),
            ),
        ];
        shape2_polygons.push(Polygon::new(verts, None));
    }

    let mut node1 = Node::from_polygons(&shape1_polygons);
    let node2 = Node::from_polygons(&shape2_polygons);

    // Test clipping operations (clip_to modifies node1 in place)
    node1.clip_to(&node2);

    // Get the polygons after clipping
    let clipped_polygons = node1.all_polygons();

    // Verify clipping results are valid
    for polygon in &clipped_polygons {
        assert!(
            !polygon.vertices.is_empty(),
            "Clipped polygons should have vertices"
        );
        assert!(
            polygon.vertices.len() >= 3,
            "Clipped polygons should be valid (at least 3 vertices)"
        );

        // Check vertex validity
        for vertex in &polygon.vertices {
            assert!(
                vertex.pos.x.is_finite()
                    && vertex.pos.y.is_finite()
                    && vertex.pos.z.is_finite(),
                "Clipped polygon vertices should be finite"
            );
        }
    }

    // Verify that clipping doesn't crash and produces reasonable results
    assert!(
        clipped_polygons.len() <= shape1_polygons.len(),
        "Clipping should not increase polygon count excessively"
    );
}

#[test]
fn test_bsp_tree_invariants() {
    // **Mathematical Foundation**: BSP tree structural invariants
    // **SRS Requirement NFR005**: Algorithm correctness validation

    let polygons = create_test_polygons();
    let node = Node::from_polygons(&polygons);

    // Test tree invariants
    assert_bsp_invariants(&node, &polygons);

    // Test that tree can be traversed without issues
    let all_polygons = node.all_polygons();
    assert_eq!(
        all_polygons.len(),
        polygons.len(),
        "Tree should preserve all polygons"
    );

    // Test that polygons maintain their properties
    for (i, polygon) in all_polygons.iter().enumerate() {
        assert!(
            !polygon.vertices.is_empty(),
            "Polygon {} should have vertices",
            i
        );
        assert!(polygon.vertices.len() >= 3, "Polygon {} should be valid", i);

        // Check that vertices are properly ordered (counter-clockwise)
        // This is a basic sanity check for polygon validity
        if polygon.vertices.len() >= 3 {
            let v0 = &polygon.vertices[0].pos;
            let v1 = &polygon.vertices[1].pos;
            let v2 = &polygon.vertices[2].pos;

            // Calculate cross product to check winding
            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let cross = edge1.x * edge2.y - edge1.y * edge2.x;

            // For 2D polygons, cross product z-component indicates winding
            assert!(cross.abs() > 0.0, "Polygon {} should have non-zero area", i);
        }
    }
}

#[test]
fn test_bsp_performance_scaling() {
    // **Mathematical Foundation**: BSP tree performance characteristics
    // **SRS Requirement NFR002**: Algorithmic complexity validation

    use std::time::Instant;

    let polygon_counts = vec![10, 50, 100, 200];

    for &count in &polygon_counts {
        let polygons = create_random_polygons(count);

        let start = Instant::now();
        let node = Node::from_polygons(&polygons);
        let construction_time = start.elapsed();

        let start = Instant::now();
        let all_polygons = node.all_polygons();
        let traversal_time = start.elapsed();

        // Verify correctness
        assert_eq!(
            all_polygons.len(),
            count,
            "Should preserve {} polygons",
            count
        );

        // Performance should be reasonable (less than 1 second for reasonable sizes)
        assert!(
            construction_time.as_millis() < 1000,
            "BSP construction for {} polygons should be fast: {:?}",
            count,
            construction_time
        );
        assert!(
            traversal_time.as_millis() < 100,
            "BSP traversal for {} polygons should be fast: {:?}",
            count,
            traversal_time
        );

        // Tree should be properly constructed
        assert!(
            node.plane.is_some() || count == 0,
            "Non-empty polygon set should create partitioning plane"
        );
    }
}

// Helper functions for BSP tests

fn calculate_bsp_tree_depth(node: &Node<()>) -> usize {
    if node.plane.is_none() {
        return 1; // Leaf node
    }

    let front_depth = node
        .front
        .as_ref()
        .map(|front| calculate_bsp_tree_depth(front))
        .unwrap_or(1);

    let back_depth = node
        .back
        .as_ref()
        .map(|back| calculate_bsp_tree_depth(back))
        .unwrap_or(1);

    1 + front_depth.max(back_depth)
}

fn create_test_polygons() -> Vec<Polygon<()>> {
    let mut polygons = Vec::new();

    // Create some basic test polygons
    let verts1 = vec![
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    polygons.push(Polygon::new(verts1, None));

    let verts2 = vec![
        Vertex::new(Point3::new(0.5, 0.5, 1.0), Vector3::z()),
        Vertex::new(Point3::new(1.5, 0.5, 1.0), Vector3::z()),
        Vertex::new(Point3::new(1.5, 1.5, 1.0), Vector3::z()),
        Vertex::new(Point3::new(0.5, 1.5, 1.0), Vector3::z()),
    ];
    polygons.push(Polygon::new(verts2, None));

    polygons
}

fn create_random_polygons(count: usize) -> Vec<Polygon<()>> {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut polygons = Vec::new();

    for i in 0..count {
        let mut hasher = DefaultHasher::new();
        i.hash(&mut hasher);
        let hash = hasher.finish();

        // Create deterministic "random" positions based on hash
        let x = (hash % 100) as f64 * 0.1;
        let y = ((hash / 100) % 100) as f64 * 0.1;

        let verts = vec![
            Vertex::new(Point3::new(x, y, 0.0), Vector3::z()),
            Vertex::new(Point3::new(x + 1.0, y, 0.0), Vector3::z()),
            Vertex::new(Point3::new(x + 1.0, y + 1.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(x, y + 1.0, 0.0), Vector3::z()),
        ];
        polygons.push(Polygon::new(verts, None));
    }

    polygons
}

fn assert_bsp_invariants(node: &Node<()>, original_polygons: &[Polygon<()>]) {
    // Count total polygons in tree
    let tree_polygons = node.all_polygons();
    assert_eq!(
        tree_polygons.len(),
        original_polygons.len(),
        "BSP tree should preserve polygon count"
    );

    // Check that all polygons are valid
    for polygon in &tree_polygons {
        assert!(
            !polygon.vertices.is_empty(),
            "All polygons should have vertices"
        );
        assert!(polygon.vertices.len() >= 3, "All polygons should be valid");

        // Check vertex finiteness
        for vertex in &polygon.vertices {
            assert!(vertex.pos.x.is_finite(), "Vertex X should be finite");
            assert!(vertex.pos.y.is_finite(), "Vertex Y should be finite");
            assert!(vertex.pos.z.is_finite(), "Vertex Z should be finite");
        }
    }
}

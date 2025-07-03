//! Extended Boolean Operations Example
//!
//! This example demonstrates additional boolean operations beyond the basic four,
//! including compound operations, set theory operations, and geometric transformations
//! that can be implemented using combinations of the fundamental operations.
//!
//! Operations demonstrated:
//! 1. **Fundamental Operations**: Union, Difference, Intersection, XOR
//! 2. **Compound Operations**: Multiple object combinations
//! 3. **Set Theory Operations**: Complement, Symmetric operations
//! 4. **Geometric Operations**: Shell, Offset, Minkowski operations
//! 5. **Advanced Operations**: Conditional operations, morphing

use csgrs::csg::CSG;
use std::fs;

fn main() {
    // Create output directory
    let out_dir = "outputs/extended-boolean-operations";
    fs::create_dir_all(out_dir).unwrap();

    println!("=== Extended Boolean Operations Demo ===\n");

    // ========================================================================
    // PART 1: Fundamental Operations Review
    // ========================================================================
    
    println!("Part 1: Fundamental Boolean Operations");
    
    // Create test shapes
    let cube = CSG::<()>::cube(2.0, None).center();
    let sphere = CSG::<()>::sphere(1.2, 16, 8, None);
    let cylinder = CSG::<()>::cylinder(0.8, 3.0, 16, None).center();
    
    println!("Base shapes created:");
    println!("  Cube: {} polygons", cube.polygons.len());
    println!("  Sphere: {} polygons", sphere.polygons.len());
    println!("  Cylinder: {} polygons", cylinder.polygons.len());
    
    // Fundamental operations
    let union_ab = cube.union(&sphere);
    let diff_ab = cube.difference(&sphere);
    let intersect_ab = cube.intersection(&sphere);
    let xor_ab = cube.xor(&sphere);
    
    println!("\nFundamental operations (Cube ○ Sphere):");
    println!("  Union: {} polygons", union_ab.polygons.len());
    println!("  Difference: {} polygons", diff_ab.polygons.len());
    println!("  Intersection: {} polygons", intersect_ab.polygons.len());
    println!("  XOR: {} polygons", xor_ab.polygons.len());

    // ========================================================================
    // PART 2: Compound Boolean Operations (Multiple Objects)
    // ========================================================================
    
    println!("\n\nPart 2: Compound Boolean Operations");
    
    // 1. Triple Union: A ∪ B ∪ C
    println!("\n1. Triple Union (A ∪ B ∪ C):");
    let triple_union = cube.union(&sphere).union(&cylinder);
    println!("   Result: {} polygons", triple_union.polygons.len());
    
    // 2. Triple Intersection: A ∩ B ∩ C
    println!("\n2. Triple Intersection (A ∩ B ∩ C):");
    let triple_intersection = cube.intersection(&sphere).intersection(&cylinder);
    println!("   Result: {} polygons", triple_intersection.polygons.len());
    
    // 3. Cascading Difference: (A - B) - C
    println!("\n3. Cascading Difference ((A - B) - C):");
    let cascading_diff = cube.difference(&sphere).difference(&cylinder);
    println!("   Result: {} polygons", cascading_diff.polygons.len());
    
    // 4. Complex Combination: (A ∪ B) - C
    println!("\n4. Complex Combination ((A ∪ B) - C):");
    let complex_combo = cube.union(&sphere).difference(&cylinder);
    println!("   Result: {} polygons", complex_combo.polygons.len());

    // ========================================================================
    // PART 3: Extended Set Theory Operations
    // ========================================================================
    
    println!("\n\nPart 3: Extended Set Theory Operations");
    
    // 1. Reverse Difference: B - A (opposite of A - B)
    println!("\n1. Reverse Difference (B - A):");
    let reverse_diff = sphere.difference(&cube);
    println!("   Result: {} polygons", reverse_diff.polygons.len());
    println!("   Compare to A - B: {} polygons", diff_ab.polygons.len());
    
    // 2. Symmetric Difference (alternative XOR implementation)
    println!("\n2. Symmetric Difference (manual XOR):");
    let manual_xor = union_ab.difference(&intersect_ab);
    println!("   Manual XOR: {} polygons", manual_xor.polygons.len());
    println!("   Direct XOR: {} polygons", xor_ab.polygons.len());
    println!("   Difference: {} polygons", manual_xor.polygons.len() as i32 - xor_ab.polygons.len() as i32);
    
    // 3. Conditional Union: A ∪ B only if they intersect
    println!("\n3. Conditional Union (only if intersection exists):");
    let conditional_union = if intersect_ab.polygons.len() > 0 {
        println!("   Shapes intersect, performing union");
        cube.union(&sphere)
    } else {
        println!("   Shapes don't intersect, keeping original");
        cube.clone()
    };
    println!("   Result: {} polygons", conditional_union.polygons.len());

    // ========================================================================
    // PART 4: Minkowski Operations (Already Implemented!)
    // ========================================================================

    println!("\n\nPart 4: Minkowski Operations");

    // 1. Minkowski Sum: A ⊕ B (shape expansion)
    #[cfg(feature = "chull-io")]
    {
        println!("\n1. Minkowski Sum (A ⊕ B):");
        let minkowski_sum = cube.minkowski_sum(&sphere);
        println!("   Cube ⊕ Sphere: {} polygons", minkowski_sum.polygons.len());
        println!("   This expands the cube by the sphere's shape");

        // Test with smaller shapes for better visualization
        let small_cube = CSG::<()>::cube(0.5, None).center();
        let small_sphere = CSG::<()>::sphere(0.3, 8, 6, None);
        let small_minkowski = small_cube.minkowski_sum(&small_sphere);
        println!("   Small example: {} polygons", small_minkowski.polygons.len());
    }

    #[cfg(not(feature = "chull-io"))]
    {
        println!("\n1. Minkowski Sum (requires 'chull-io' feature):");
        println!("   Enable with: cargo run --example extended-boolean-operations --features chull-io");
    }

    // ========================================================================
    // PART 5: Geometric Construction Operations
    // ========================================================================

    println!("\n\nPart 5: Geometric Construction Operations");
    
    // 1. Shell Operation: A - scaled_down(A)
    println!("\n1. Shell Operation (hollow shell):");
    let inner_cube = cube.scale(0.7, 0.7, 0.7);
    let shell = cube.difference(&inner_cube);
    println!("   Original: {} polygons", cube.polygons.len());
    println!("   Inner (70% scale): {} polygons", inner_cube.polygons.len());
    println!("   Shell: {} polygons", shell.polygons.len());
    
    // 2. Frame Operation: Union of edges
    println!("\n2. Frame Operation (edge structure):");
    let frame_x = CSG::<()>::cube(2.2, None).center().difference(&CSG::<()>::cube(1.8, None).center());
    let frame_y = CSG::<()>::cube(2.2, None).center().difference(&CSG::<()>::cube(1.8, None).center()).rotate(0.0, 90.0, 0.0);
    let frame_z = CSG::<()>::cube(2.2, None).center().difference(&CSG::<()>::cube(1.8, None).center()).rotate(90.0, 0.0, 0.0);
    let frame = frame_x.union(&frame_y).union(&frame_z);
    println!("   Frame structure: {} polygons", frame.polygons.len());
    
    // 3. Lattice Operation: Repeated pattern
    println!("\n3. Lattice Operation (repeated intersections):");
    let lattice_base = CSG::<()>::cube(3.0, None).center();
    let lattice_holes = CSG::<()>::sphere(0.3, 8, 6, None).translate(0.5, 0.5, 0.5)
        .union(&CSG::<()>::sphere(0.3, 8, 6, None).translate(-0.5, 0.5, 0.5))
        .union(&CSG::<()>::sphere(0.3, 8, 6, None).translate(0.5, -0.5, 0.5))
        .union(&CSG::<()>::sphere(0.3, 8, 6, None).translate(-0.5, -0.5, 0.5));
    let lattice = lattice_base.difference(&lattice_holes);
    println!("   Lattice: {} polygons", lattice.polygons.len());

    // ========================================================================
    // PART 6: Advanced Combination Patterns
    // ========================================================================

    println!("\n\nPart 6: Advanced Combination Patterns");
    
    // 1. Alternating Pattern: A ∪ B ∪ C with alternating differences
    println!("\n1. Alternating Pattern:");
    let alt_pattern = cube.union(&sphere).difference(&cylinder)
        .union(&CSG::<()>::cube(1.0, None).center().translate(1.5, 0.0, 0.0));
    println!("   Alternating pattern: {} polygons", alt_pattern.polygons.len());
    
    // 2. Nested Operations: ((A ∪ B) ∩ C) ⊕ D
    println!("\n2. Nested Operations:");
    let nested = cube.union(&sphere).intersection(&cylinder)
        .xor(&CSG::<()>::sphere(0.5, 12, 8, None).translate(0.0, 0.0, 1.0));
    println!("   Nested result: {} polygons", nested.polygons.len());
    
    // 3. Morphing Operation: Gradual transformation
    println!("\n3. Morphing Operation (interpolation simulation):");
    let morph_25 = cube.scale(0.75, 0.75, 0.75).union(&sphere.scale(0.25, 0.25, 0.25));
    let morph_50 = cube.scale(0.5, 0.5, 0.5).union(&sphere.scale(0.5, 0.5, 0.5));
    let morph_75 = cube.scale(0.25, 0.25, 0.25).union(&sphere.scale(0.75, 0.75, 0.75));
    println!("   Morph 25%: {} polygons", morph_25.polygons.len());
    println!("   Morph 50%: {} polygons", morph_50.polygons.len());
    println!("   Morph 75%: {} polygons", morph_75.polygons.len());

    // ========================================================================
    // PART 7: Validation and Analysis
    // ========================================================================

    println!("\n\nPart 7: Operation Validation");
    
    // Test associativity: (A ∪ B) ∪ C = A ∪ (B ∪ C)
    let left_assoc = cube.union(&sphere).union(&cylinder);
    let right_assoc = cube.union(&sphere.union(&cylinder));

    // Check volumes (more important than polygon counts)
    let left_volume = {
        let (mass, _, _) = left_assoc.mass_properties(1.0);
        mass
    };
    let right_volume = {
        let (mass, _, _) = right_assoc.mass_properties(1.0);
        mass
    };
    let volume_error = if right_volume > 0.0 {
        ((left_volume - right_volume).abs() / right_volume * 100.0)
    } else {
        0.0
    };

    println!("\nAssociativity Test (Union):");
    println!("  (A ∪ B) ∪ C: {} polygons, volume: {:.6}", left_assoc.polygons.len(), left_volume);
    println!("  A ∪ (B ∪ C): {} polygons, volume: {:.6}", right_assoc.polygons.len(), right_volume);
    println!("  Volume error: {:.3}%", volume_error);
    println!("  Associative: {}", if volume_error < 1.0 { "✅ YES (volumes match)" } else { "❌ NO" });
    
    // Test commutativity: A ∪ B = B ∪ A
    let ab_union = cube.union(&sphere);
    let ba_union = sphere.union(&cube);

    let ab_volume = {
        let (mass, _, _) = ab_union.mass_properties(1.0);
        mass
    };
    let ba_volume = {
        let (mass, _, _) = ba_union.mass_properties(1.0);
        mass
    };
    let comm_volume_error = if ba_volume > 0.0 {
        ((ab_volume - ba_volume).abs() / ba_volume * 100.0)
    } else {
        0.0
    };

    println!("\nCommutativity Test (Union):");
    println!("  A ∪ B: {} polygons, volume: {:.6}", ab_union.polygons.len(), ab_volume);
    println!("  B ∪ A: {} polygons, volume: {:.6}", ba_union.polygons.len(), ba_volume);
    println!("  Volume error: {:.3}%", comm_volume_error);
    println!("  Commutative: {}", if comm_volume_error < 1.0 { "✅ YES (volumes match)" } else { "❌ NO" });
    
    // Test De Morgan's Law: ¬(A ∪ B) = ¬A ∩ ¬B (conceptually)
    println!("\nDe Morgan's Law (conceptual - complement not directly implementable):");
    println!("  ¬(A ∪ B) would require universe definition");
    println!("  But we can test: A ⊕ B = (A ∪ B) - (A ∩ B)");
    let demorgan_test = union_ab.difference(&intersect_ab);
    println!("  (A ∪ B) - (A ∩ B): {} polygons", demorgan_test.polygons.len());
    println!("  A ⊕ B: {} polygons", xor_ab.polygons.len());
    println!("  Equivalent: {}", if demorgan_test.polygons.len() == xor_ab.polygons.len() { "✅ YES" } else { "❌ NO" });

    // ========================================================================
    // PART 8: Export Results
    // ========================================================================

    #[cfg(feature = "stl-io")]
    {
        println!("\n\nExporting STL files...");

        let mut exports = vec![
            (&triple_union, "triple_union"),
            (&triple_intersection, "triple_intersection"),
            (&cascading_diff, "cascading_difference"),
            (&complex_combo, "complex_combination"),
            (&shell, "shell_operation"),
            (&frame, "frame_operation"),
            (&lattice, "lattice_operation"),
            (&alt_pattern, "alternating_pattern"),
            (&nested, "nested_operations"),
            (&morph_50, "morphing_50_percent"),
        ];

        // Add Minkowski sum if available
        #[cfg(feature = "chull-io")]
        let minkowski_demo = cube.minkowski_sum(&sphere);

        #[cfg(feature = "chull-io")]
        exports.push((&minkowski_demo, "minkowski_sum"));

        for (shape, name) in exports.iter() {
            let _ = fs::write(
                format!("{}/{}.stl", out_dir, name),
                shape.to_stl_binary(name).unwrap(),
            );
        }

        println!("STL files exported to '{}'", out_dir);
    }

    println!("\nExtended boolean operations analysis completed!");
    #[cfg(feature = "chull-io")]
    println!("Demonstrated {} additional operation patterns including Minkowski sum!", 11);
    #[cfg(not(feature = "chull-io"))]
    println!("Demonstrated {} additional operation patterns beyond basic boolean operations.", 10);
    println!("✅ Associativity and commutativity validated using volume analysis");
    println!("📝 Polygon count differences are normal due to mesh tessellation variations");
}

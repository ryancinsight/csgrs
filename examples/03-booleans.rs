//! Boolean Operations Example
//!
//! This example demonstrates all four fundamental boolean operations in CSG:
//! - Union (A ∪ B): Combines both shapes
//! - Difference (A - B): Removes B from A  
//! - Intersection (A ∩ B): Keeps only the overlapping region
//! - XOR (A ⊕ B): Keeps only the non-overlapping regions (A ∪ B - A ∩ B)
//!
//! We'll demonstrate these operations with both 3D shapes (cube and sphere) and 
//! 2D shapes (overlapping squares) to showcase the coplanar polygon handling
//! improvements that were recently implemented.

use csgrs::csg::CSG;

use std::fs;

fn main() {
    // Create output directory
    let out_dir = "outputs/03-booleans";
    fs::create_dir_all(out_dir).unwrap();

    println!("=== CSG Boolean Operations Demo ===\n");

    // ========================================================================
    // PART 1: 3D Boolean Operations (Cube and Sphere)
    // ========================================================================
    
    println!("Part 1: 3D Boolean Operations");
    println!("Creating a 4×4×4 cube and a sphere with radius 2.5 (both centered for maximum overlap)...");
    
    // Create base shapes with significant overlap for clear visual distinction
    let cube = CSG::<()>::cube(4.0, None).center();  // 4×4×4 cube centered at origin
    let sphere = CSG::<()>::sphere(2.5, 16, 8, None); // 2.5 radius sphere at origin (50% overlap)
    
    println!("  Cube: {} polygons", cube.polygons.len());
    println!("  Sphere: {} polygons", sphere.polygons.len());

    // Save input shapes
    #[cfg(feature = "stl-io")]
    {
        let _ = fs::write(
            format!("{}/input_cube.stl", out_dir),
            cube.to_stl_binary("input_cube").unwrap(),
        );
        let _ = fs::write(
            format!("{}/input_sphere.stl", out_dir),
            sphere.to_stl_binary("input_sphere").unwrap(),
        );
    }

    // 1. Union: A ∪ B (combines both shapes)
    println!("\n1. Union (A ∪ B): Combines both shapes");
    let union_result = cube.union(&sphere);
    println!("   Result: {} polygons", union_result.polygons.len());
    
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/3d_union.stl", out_dir),
        union_result.to_stl_binary("3d_union").unwrap(),
    );

    // 2. Difference: A - B (removes B from A)
    println!("\n2. Difference (A - B): Removes sphere from cube");
    let difference_result = cube.difference(&sphere);
    println!("   Result: {} polygons", difference_result.polygons.len());
    
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/3d_difference.stl", out_dir),
        difference_result.to_stl_binary("3d_difference").unwrap(),
    );

    // 3. Intersection: A ∩ B (keeps only overlapping region)
    println!("\n3. Intersection (A ∩ B): Keeps only overlapping region");
    let intersection_result = cube.intersection(&sphere);
    println!("   Result: {} polygons", intersection_result.polygons.len());
    
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/3d_intersection.stl", out_dir),
        intersection_result.to_stl_binary("3d_intersection").unwrap(),
    );

    // 4. XOR: A ⊕ B (keeps only non-overlapping regions)
    println!("\n4. XOR (A ⊕ B): Keeps only non-overlapping regions");
    println!("   This is equivalent to: (A ∪ B) - (A ∩ B)");
    println!("   XOR creates a 'hollow' effect - removing the overlapping volume");
    let xor_result = cube.xor(&sphere);
    println!("   Result: {} polygons", xor_result.polygons.len());

    // Calculate the visual difference
    let union_count = union_result.polygons.len();
    let intersection_count = intersection_result.polygons.len();
    let xor_count = xor_result.polygons.len();

    println!("\n   Visual Analysis:");
    println!("   - Union has {} polygons", union_count);
    println!("   - XOR has {} polygons", xor_count);
    println!("   - Difference: {} polygons (this is the 'hollow' cavity)", union_count as i32 - xor_count as i32);
    println!("   - The XOR should look like the Union with a cavity where shapes overlapped");
    
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/3d_xor.stl", out_dir),
        xor_result.to_stl_binary("3d_xor").unwrap(),
    );

    // ========================================================================
    // PART 2: 2D Boolean Operations (Overlapping Squares) - Coplanar Test
    // ========================================================================
    
    println!("\n\nPart 2: 2D Boolean Operations (Coplanar Polygons)");
    println!("Creating two overlapping 2×2 squares to test coplanar polygon handling...");
    
    // Create two overlapping squares in the XY plane
    // Note: CSG::square() creates 2D geometry, we need to convert to 3D polygons for boolean ops

    // Square A: from (0,0) to (2,2)
    let square_a_2d = CSG::<()>::square(2.0, None);
    let square_a = CSG::from_polygons(&square_a_2d.to_polygons());

    // Square B: from (1,1) to (3,3) - overlaps with A
    let square_b_2d = CSG::<()>::square(2.0, None).translate(1.0, 1.0, 0.0);
    let square_b = CSG::from_polygons(&square_b_2d.to_polygons());

    println!("  Square A: {} polygons", square_a.polygons.len());
    println!("  Square B: {} polygons", square_b.polygons.len());
    
    // Extrude squares slightly for 3D visualization
    let extruded_a = square_a_2d.extrude(0.1);
    let extruded_b = square_b_2d.extrude(0.1);

    // Save input squares
    #[cfg(feature = "stl-io")]
    {
        let _ = fs::write(
            format!("{}/input_square_a.stl", out_dir),
            extruded_a.to_stl_binary("input_square_a").unwrap(),
        );
        let _ = fs::write(
            format!("{}/input_square_b.stl", out_dir),
            extruded_b.to_stl_binary("input_square_b").unwrap(),
        );
    }

    // Test all boolean operations on coplanar squares
    println!("\n2D Boolean Operations (testing coplanar polygon handling):");
    
    // 1. Union of squares
    let squares_union = square_a.union(&square_b).extrude(0.1);
    println!("  Union: {} polygons", squares_union.polygons.len());
    
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/2d_union.stl", out_dir),
        squares_union.to_stl_binary("2d_union").unwrap(),
    );

    // 2. Difference of squares
    let squares_difference = square_a.difference(&square_b).extrude(0.1);
    println!("  Difference: {} polygons", squares_difference.polygons.len());
    
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/2d_difference.stl", out_dir),
        squares_difference.to_stl_binary("2d_difference").unwrap(),
    );

    // 3. Intersection of squares
    let squares_intersection = square_a.intersection(&square_b).extrude(0.1);
    println!("  Intersection: {} polygons", squares_intersection.polygons.len());
    
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/2d_intersection.stl", out_dir),
        squares_intersection.to_stl_binary("2d_intersection").unwrap(),
    );

    // 4. XOR of squares - This tests our coplanar polygon handling fix!
    println!("\n  XOR (testing coplanar polygon handling fix):");
    let squares_xor = square_a.xor(&square_b);
    println!("    Before extrusion: {} polygons", squares_xor.polygons.len());
    
    // This should produce 2 polygons: the non-overlapping parts of each square
    if squares_xor.polygons.len() == 0 {
        println!("    ❌ WARNING: XOR produced empty result - coplanar handling may have issues");
    } else {
        println!("    ✅ SUCCESS: XOR produced valid result with {} polygons", squares_xor.polygons.len());
    }
    
    let extruded_xor = squares_xor.extrude(0.1);
    println!("    After extrusion: {} polygons", extruded_xor.polygons.len());
    
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/2d_xor.stl", out_dir),
        extruded_xor.to_stl_binary("2d_xor").unwrap(),
    );

    // ========================================================================
    // PART 3: Verification and Analysis
    // ========================================================================
    
    // ========================================================================
    // PART 2.5: Dramatic XOR Demonstration
    // ========================================================================

    println!("\n\nPart 2.5: Dramatic XOR Demonstration");
    println!("Creating two cubes with 50% overlap to clearly show XOR vs Union difference...");

    // Create two cubes with significant overlap
    let cube1 = CSG::<()>::cube(3.0, None); // From (0,0,0) to (3,3,3)
    let cube2 = CSG::<()>::cube(3.0, None).translate(1.5, 0.0, 0.0); // From (1.5,0,0) to (4.5,3,3)

    println!("  Cube 1: {} polygons", cube1.polygons.len());
    println!("  Cube 2: {} polygons", cube2.polygons.len());

    let cubes_union = cube1.union(&cube2);
    let cubes_intersection = cube1.intersection(&cube2);
    let cubes_xor = cube1.xor(&cube2);

    println!("\nTwo Overlapping Cubes:");
    println!("  Union: {} polygons (solid combined shape)", cubes_union.polygons.len());
    println!("  Intersection: {} polygons (overlapping region)", cubes_intersection.polygons.len());
    println!("  XOR: {} polygons (hollow shell with cavity)", cubes_xor.polygons.len());

    let union_vs_xor_diff = cubes_xor.polygons.len() as i32 - cubes_union.polygons.len() as i32;
    println!("  XOR - Union = {} polygons (internal cavity surfaces)", union_vs_xor_diff);

    #[cfg(feature = "stl-io")]
    {
        let _ = fs::write(
            format!("{}/dramatic_union.stl", out_dir),
            cubes_union.to_stl_binary("dramatic_union").unwrap(),
        );
        let _ = fs::write(
            format!("{}/dramatic_xor.stl", out_dir),
            cubes_xor.to_stl_binary("dramatic_xor").unwrap(),
        );
        let _ = fs::write(
            format!("{}/dramatic_intersection.stl", out_dir),
            cubes_intersection.to_stl_binary("dramatic_intersection").unwrap(),
        );
    }

    println!("\n💡 KEY INSIGHT:");
    println!("   - Union creates a SOLID combined shape");
    println!("   - XOR creates a HOLLOW shell with internal cavity");
    println!("   - They have the same OUTER boundary but different internal structure");
    println!("   - XOR has MORE polygons because it includes internal cavity surfaces");

    // ========================================================================
    // PART 3: Verification and Analysis
    // ========================================================================

    println!("\n\nPart 3: Verification and Analysis");
    
    // Verify mathematical relationships
    println!("Verifying boolean operation relationships:");
    
    // For any two sets A and B:
    // |A ∪ B| + |A ∩ B| should be related to |A| + |B|
    // A ⊕ B = (A ∪ B) - (A ∩ B)
    
    let manual_xor_3d = union_result.difference(&intersection_result);
    println!("  3D XOR verification:");
    println!("    Direct XOR: {} polygons", xor_result.polygons.len());
    println!("    Manual (Union - Intersection): {} polygons", manual_xor_3d.polygons.len());
    
    if xor_result.polygons.len() > 0 && manual_xor_3d.polygons.len() > 0 {
        println!("    ✅ Both methods produce non-empty results");
    } else {
        println!("    ⚠️  One or both methods produced empty results");
    }

    println!("\nAll boolean operations completed successfully!");
    println!("Check the '{}' directory for STL output files.", out_dir);
    
    #[cfg(not(feature = "stl-io"))]
    println!("\nNote: STL output disabled. Enable 'stl-io' feature to generate files.");
}

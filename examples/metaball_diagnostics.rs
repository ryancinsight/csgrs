//! Metaball Field Diagnostics
//!
//! This example helps diagnose why metaballs aren't producing geometry
//! by examining the actual field values generated.

use csgrs::CSG;
use csgrs::math::metaballs::MetaBall;
use nalgebra::Point3;

fn calculate_metaball_field(metaballs: &[MetaBall], point: &Point3<f64>) -> f64 {
    metaballs.iter().map(|ball| ball.influence(point)).sum()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Metaball Field Diagnostics");
    println!("=========================");
    
    // Test simple configuration first
    println!("\n=== Test 1: Single Metaball ===");
    let single_metaball = vec![
        MetaBall::new(Point3::new(0.0, 0.0, 0.0), 3.0)
    ];
    
    // Test field values at various distances
    for distance in [0.0, 1.0, 2.0, 3.0, 4.0, 5.0] {
        let test_point = Point3::new(distance, 0.0, 0.0);
        let field_value = calculate_metaball_field(&single_metaball, &test_point);
        println!("Distance {:.1}: field = {:.6}", distance, field_value);
    }
    
    // Test what iso_values might work
    println!("\nAt center (0,0,0): field = {:.6}", 
             calculate_metaball_field(&single_metaball, &Point3::new(0.0, 0.0, 0.0)));
    
    // Test single metaball with different iso_values
    for &iso_value in &[10.0, 1.0, 0.1, 0.01, 0.001] {
        let result: CSG<()> = CSG::metaballs(
            &single_metaball,
            (24, 24, 12),
            iso_value,
            2.0,
            None
        );
        println!("Single metaball, iso_value {}: {} polygons", iso_value, result.polygons.len());
    }
    
    println!("\n=== Test 2: Cylinder Junction Configuration ===");
    
    // Recreate our cylinder junction setup
    let cylinder_radius = 3.0;
    let junction_point = Point3::new(0.0, 0.0, 0.0);
    let cylinder1_end = Point3::new(20.0, 0.0, 0.0);
    let angle_radians = 120.0_f64.to_radians();
    let cylinder2_end = Point3::new(
        20.0 * angle_radians.cos(),
        20.0 * angle_radians.sin(),
        0.0
    );
    
    // Our problematic metaball configuration
    let dir1 = (cylinder1_end - junction_point).normalize();
    let dir2 = (cylinder2_end - junction_point).normalize();
    
    let junction_metaballs = vec![
        MetaBall::new(junction_point, cylinder_radius * 2.5),
        MetaBall::new(junction_point + dir1 * (cylinder_radius * 0.8), cylinder_radius * 1.8),
        MetaBall::new(junction_point + dir2 * (cylinder_radius * 0.8), cylinder_radius * 1.8),
    ];
    
    // Test field values at key points
    println!("Junction field values:");
    println!("At junction (0,0,0): {:.6}", 
             calculate_metaball_field(&junction_metaballs, &junction_point));
    
    // Test along cylinder axes
    for t in [0.5, 1.0, 1.5, 2.0] {
        let pos1 = junction_point + dir1 * t;
        let pos2 = junction_point + dir2 * t;
        let field1 = calculate_metaball_field(&junction_metaballs, &pos1);
        let field2 = calculate_metaball_field(&junction_metaballs, &pos2);
        println!("Along cyl1 at {:.1}: {:.6}, along cyl2 at {:.1}: {:.6}", t, field1, t, field2);
    }
    
    // Find maximum field value in the region
    let mut max_field = 0.0;
    let mut max_point = Point3::new(0.0, 0.0, 0.0);
    
    for x in -10..=10 {
        for y in -10..=10 {
            for z in -3..=3 {
                let test_point = Point3::new(x as f64, y as f64, z as f64);
                let field = calculate_metaball_field(&junction_metaballs, &test_point);
                if field > max_field {
                    max_field = field;
                    max_point = test_point;
                }
            }
        }
    }
    
    println!("Maximum field value: {:.6} at ({:.1}, {:.1}, {:.1})", 
             max_field, max_point.x, max_point.y, max_point.z);
    
    // Test with realistic iso_values based on max field
    let realistic_iso_values = [
        max_field * 0.9,
        max_field * 0.5,
        max_field * 0.1,
        max_field * 0.01,
    ];
    
    println!("\nTesting realistic iso_values:");
    for &iso_value in &realistic_iso_values {
        let result: CSG<()> = CSG::metaballs(
            &junction_metaballs,
            (24, 24, 12),
            iso_value,
            cylinder_radius * 2.0,
            None
        );
        println!("iso_value {:.6} ({:.1}% of max): {} polygons", 
                 iso_value, (iso_value / max_field) * 100.0, result.polygons.len());
    }
    
    println!("\n=== Test 3: Working Configuration ===");
    
    // Create a configuration we know should work
    let working_metaballs = vec![
        MetaBall::new(Point3::new(0.0, 0.0, 0.0), 2.0),
        MetaBall::new(Point3::new(1.0, 0.0, 0.0), 2.0),
        MetaBall::new(Point3::new(0.0, 1.0, 0.0), 2.0),
    ];
    
    // Find max field for working config
    let mut working_max = 0.0;
    for x in -5..=5 {
        for y in -5..=5 {
            for z in -2..=2 {
                let test_point = Point3::new(x as f64, y as f64, z as f64);
                let field = calculate_metaball_field(&working_metaballs, &test_point);
                if field > working_max {
                    working_max = field;
                }
            }
        }
    }
    
    println!("Working config max field: {:.6}", working_max);
    
    // Test working config
    let working_iso = working_max * 0.1;
    let working_result: CSG<()> = CSG::metaballs(
        &working_metaballs,
        (24, 24, 12),
        working_iso,
        3.0,
        None
    );
    
    println!("Working config with iso_value {:.6}: {} polygons", 
             working_iso, working_result.polygons.len());
    
    #[cfg(feature = "stl-io")]
    if working_result.polygons.len() > 0 {
        let stl_bytes = working_result.to_stl_binary("working_metaballs")?;
        std::fs::write("working_metaballs.stl", stl_bytes)?;
        println!("Exported: working_metaballs.stl");
    }
    
    println!("\n=== DIAGNOSIS COMPLETE ===");
    println!("Key insights:");
    println!("1. Metaball influence function: I(p) = r²/(|p-c|² + ε)");
    println!("2. Field values decrease rapidly with distance");
    println!("3. iso_value must be < maximum field value to generate geometry");
    println!("4. For cylinder junction, try iso_value ≈ 10% of maximum field");
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_field_calculations() {
        let metaball = MetaBall::new(Point3::new(0.0, 0.0, 0.0), 1.0);
        
        // At center: should be very high (1²/ε)
        let center_field = metaball.influence(&Point3::new(0.0, 0.0, 0.0));
        println!("Center field: {}", center_field);
        assert!(center_field > 1000.0); // Should be very high due to EPSILON
        
        // At radius distance: should be 1²/(1² + ε) ≈ 1
        let radius_field = metaball.influence(&Point3::new(1.0, 0.0, 0.0));
        println!("At radius field: {}", radius_field);
        assert!(radius_field > 0.5 && radius_field < 2.0);
    }
} 
use csgrs::voxels::csg::Voxels;
use csgrs::traits::CSG;

fn main() {
    println!("=== CSG Debug Analysis ===");

    // Create the same shapes as in main.rs
    let sphere: Voxels<()> = Voxels::sphere(1.0, 16, 8, None);
    let cube: Voxels<()> = Voxels::cube(1.5, None);

    println!("Sphere SVO details:");
    println!("  Center: {:?}", sphere.svo().center);
    println!("  Half size: {}", sphere.svo().half);
    println!("  Max depth: {}", sphere.svo().max_depth);
    println!("  Root occupancy: {:?}", sphere.svo().root.occupancy);
    println!("  Polygons: {}", sphere.polygons().len());

    println!("\nCube SVO details:");
    println!("  Center: {:?}", cube.svo().center);
    println!("  Half size: {}", cube.svo().half);
    println!("  Max depth: {}", cube.svo().max_depth);
    println!("  Root occupancy: {:?}", cube.svo().root.occupancy);
    println!("  Polygons: {}", cube.polygons().len());

    // Test union
    let union_result = sphere.union(&cube);
    println!("\nUnion result:");
    println!("  Root occupancy: {:?}", union_result.svo().root.occupancy);
    println!("  Polygons: {}", union_result.polygons().len());

    // Test intersection
    let intersection_result = sphere.intersection(&cube);
    println!("\nIntersection result:");
    println!("  Root occupancy: {:?}", intersection_result.svo().root.occupancy);
    println!("  Polygons: {}", intersection_result.polygons().len());

    println!("\nBounds analysis:");
    let sphere_bounds = sphere.svo();
    let cube_bounds = cube.svo();

    println!("  Sphere center: {:?}, half: {}", sphere_bounds.center, sphere_bounds.half);
    println!("  Cube center: {:?}, half: {}", cube_bounds.center, cube_bounds.half);
}

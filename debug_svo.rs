use csgrs::voxels::csg::Voxels;
use csgrs::traits::CSG;
use csgrs::float_types::Real;

fn main() {
    use nalgebra::Point3;

    // Test the SDF function directly
    let width = 2.0;
    let length = 2.0;
    let height = 2.0;

    let cuboid_sdf = |p: &Point3<Real>| {
        // Box SDF: distance to box with dimensions [width, length, height]
        let q_x = (p.x - width * 0.5).abs() - width * 0.5;
        let q_y = (p.y - length * 0.5).abs() - length * 0.5;
        let q_z = (p.z - height * 0.5).abs() - height * 0.5;

        // Distance to box surface
        let outside_dist = (q_x.max(0.0).powi(2) + q_y.max(0.0).powi(2) + q_z.max(0.0).powi(2)).sqrt();
        let inside_dist = q_x.max(q_y).max(q_z).min(0.0);
        outside_dist + inside_dist
    };

    // Test some points
    let center = Point3::new(1.0, 1.0, 1.0);
    let corner = Point3::new(0.0, 0.0, 0.0);
    let outside = Point3::new(3.0, 3.0, 3.0);

    println!("SDF at center (1,1,1): {}", cuboid_sdf(&center));
    println!("SDF at corner (0,0,0): {}", cuboid_sdf(&corner));
    println!("SDF at outside (3,3,3): {}", cuboid_sdf(&outside));

    // Test SVO-based cube generation
    let cube: Voxels<()> = Voxels::cube_voxelized(2.0, 4, None);

    println!("Cube SVO center: {:?}", cube.svo().center);
    println!("Cube SVO half: {}", cube.svo().half);
    println!("Cube SVO max_depth: {}", cube.svo().max_depth);
    println!("Cube root occupancy: {:?}", cube.svo().root.occupancy);

    let polygons = cube.polygons();
    println!("Extracted polygons: {}", polygons.len());

    // Test bounding box
    let bb = cube.bounding_box();
    println!("Bounding box: {:?} to {:?}", bb.mins, bb.maxs);
}

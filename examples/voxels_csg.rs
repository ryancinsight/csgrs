// examples/voxels_csg.rs
// Demonstrate boolean operations using voxel-native SvoMesh primitives only (no Mesh module)

use std::fs;

use csgrs::traits::CSG; // Use transformation helpers like translate()/rotate()/scale()
use csgrs::voxels::svo_mesh::SvoMesh;
use nalgebra::Point3;

fn main() {
    let _ = fs::create_dir_all("stl_voxels");

    let res = (64, 64, 64);

    // Two overlapping spheres
    let s1 = SvoMesh::<()>::sphere(Point3::origin(), 15.0, res, None);
    let s2 = SvoMesh::<()>::sphere(Point3::origin(), 15.0, res, None).translate(10.0, 0.0, 0.0);

    // Union
    let u = s1.union(&s2);
    let _ = fs::write("stl_voxels/voxels_union_spheres_ascii.stl", u.to_stl_ascii("voxels_union"));

    // Difference
    let d = s1.difference(&s2);
    let _ = fs::write(
        "stl_voxels/voxels_difference_spheres_ascii.stl",
        d.to_stl_ascii("voxels_difference"),
    );

    // Intersection
    let i = s1.intersection(&s2);
    let _ = fs::write(
        "stl_voxels/voxels_intersection_spheres_ascii.stl",
        i.to_stl_ascii("voxels_intersection"),
    );

    // XOR
    let x = s1.xor(&s2);
    let _ = fs::write("stl_voxels/voxels_xor_spheres_ascii.stl", x.to_stl_ascii("voxels_xor"));

    // A mixed-primitive example: cube union cylinder, then subtract a sphere
    let cube = SvoMesh::<()>::cube(40.0, res, None).translate(-10.0, 0.0, 0.0);
    let cyl = SvoMesh::<()>::cylinder(10.0, 40.0, res, None).translate(10.0, 0.0, 0.0);
    let sphere_cut = SvoMesh::<()>::sphere(Point3::origin(), 12.0, res, None);

    let blended = cube.union(&cyl).difference(&sphere_cut);
    let _ = fs::write(
        "stl_voxels/voxels_cubecyl_minus_sphere_ascii.stl",
        blended.to_stl_ascii("voxels_cubecyl_minus_sphere"),
    );

    // If STL binary feature is enabled, also write binary variants of the boolean results
    #[cfg(feature = "stl-io")]
    {
        let _ = fs::write("stl_voxels/voxels_union_spheres_bin.stl", u.to_stl_binary("voxels_union").unwrap());
        let _ = fs::write(
            "stl_voxels/voxels_difference_spheres_bin.stl",
            d.to_stl_binary("voxels_difference").unwrap(),
        );
        let _ = fs::write(
            "stl_voxels/voxels_intersection_spheres_bin.stl",
            i.to_stl_binary("voxels_intersection").unwrap(),
        );
        let _ = fs::write("stl_voxels/voxels_xor_spheres_bin.stl", x.to_stl_binary("voxels_xor").unwrap());
        let _ = fs::write(
            "stl_voxels/voxels_cubecyl_minus_sphere_bin.stl",
            blended.to_stl_binary("voxels_cubecyl_minus_sphere").unwrap(),
        );
    }

    println!("Voxel CSG STL files written to stl_voxels/ (ASCII, and binary if enabled)");
}


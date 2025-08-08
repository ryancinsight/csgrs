// examples/voxels_primitives.rs
// Generate STL outputs of voxel-native primitives directly from SvoMesh (no polygon Mesh involved)

use std::fs;

use csgrs::float_types::Real;
use csgrs::voxels::svo_mesh::SvoMesh;
use nalgebra::Point3;

fn main() {
    // Create output directory for voxel STL files
    let _ = fs::create_dir_all("stl_voxels");

    // Common resolution for SDF extraction
    let res: (usize, usize, usize) = (64, 64, 64);

    // Sphere (centered at origin)
    let sphere = SvoMesh::<()>::sphere(Point3::origin(), 20.0, res, None);
    let _ = fs::write(
        "stl_voxels/sphere_voxel_ascii.stl",
        sphere.to_stl_ascii("sphere_voxel"),
    );

    // Cube (axis-aligned, centered at origin)
    let cube = SvoMesh::<()>::cube(40.0, res, None);
    let _ = fs::write(
        "stl_voxels/cube_voxel_ascii.stl",
        cube.to_stl_ascii("cube_voxel"),
    );

    // Cylinder (z-axis)
    let cylinder = SvoMesh::<()>::cylinder(10.0, 30.0, res, None);
    let _ = fs::write(
        "stl_voxels/cylinder_voxel_ascii.stl",
        cylinder.to_stl_ascii("cylinder_voxel"),
    );

    // Frustum (z-axis)
    let frustum = SvoMesh::<()>::frustum(12.0, 6.0, 30.0, res, None);
    let _ = fs::write(
        "stl_voxels/frustum_voxel_ascii.stl",
        frustum.to_stl_ascii("frustum_voxel"),
    );

    // Cone (z-axis)
    let cone = SvoMesh::<()>::cone(12.0, 30.0, res, None);
    let _ = fs::write(
        "stl_voxels/cone_voxel_ascii.stl",
        cone.to_stl_ascii("cone_voxel"),
    );

    // Torus (major R, minor r)
    let torus = SvoMesh::<()>::torus(20.0, 5.0, res, None);
    let _ = fs::write(
        "stl_voxels/torus_voxel_ascii.stl",
        torus.to_stl_ascii("torus_voxel"),
    );

    // If the binary STL feature is enabled, also write binary variants
    #[cfg(feature = "stl-io")]
    {
        let _ = fs::write(
            "stl_voxels/sphere_voxel_bin.stl",
            sphere.to_stl_binary("sphere_voxel").unwrap(),
        );
        let _ = fs::write(
            "stl_voxels/cube_voxel_bin.stl",
            cube.to_stl_binary("cube_voxel").unwrap(),
        );
        let _ = fs::write(
            "stl_voxels/cylinder_voxel_bin.stl",
            cylinder.to_stl_binary("cylinder_voxel").unwrap(),
        );
        let _ = fs::write(
            "stl_voxels/frustum_voxel_bin.stl",
            frustum.to_stl_binary("frustum_voxel").unwrap(),
        );
        let _ = fs::write(
            "stl_voxels/cone_voxel_bin.stl",
            cone.to_stl_binary("cone_voxel").unwrap(),
        );
        let _ = fs::write(
            "stl_voxels/torus_voxel_bin.stl",
            torus.to_stl_binary("torus_voxel").unwrap(),
        );
    }

    println!("Voxel primitives STL files written to stl_voxels/ (ASCII, and binary if enabled)");
}


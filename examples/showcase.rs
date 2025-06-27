// main.rs
//
// Minimal example of each function of csgrs (which is now generic over the shared-data type S).
// Here, we do not use any shared data, so we'll bind the generic S to ().

use csgrs::Real;
use csgrs::geometry::Plane;
use nalgebra::{Point3, Vector3};
use std::fs;
use std::time::Instant;

#[cfg(feature = "image")]
use image::{GrayImage, ImageBuffer};

#[cfg(feature = "metaballs")]
use csgrs::math::metaballs::MetaBall;

// A type alias for convenience: no shared data, i.e. S = ()
type CSG = csgrs::csg::CSG<()>;

fn main() {
    // Define output directories
    let out_dir = "outputs";
    let categories = [
        "01-basic-shapes", "02-transformations", "03-booleans", "04-advanced-ops",
        "05-2d-shapes", "06-extrusions", "07-mesh-ops", "08-implicit-surfaces",
        "09-gears", "10-curves", "11-platonic-solids", "12-specialized-shapes",
        "13-misc-scenes",
    ];
    for p in &categories {
        fs::create_dir_all(format!("{}/{}", out_dir, p)).unwrap();
    }

    // 1) Basic shapes: cube, sphere, cylinder
    let cube = CSG::cube(2.0, None);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(format!("{}/01-basic-shapes/cube.stl", out_dir), cube.to_stl_binary("cube").unwrap());

    let sphere = CSG::sphere(1.0, 16, 8, None); // center=(0,0,0), radius=1, slices=16, stacks=8, no metadata
    #[cfg(feature = "stl-io")]
    let _ = fs::write(format!("{}/01-basic-shapes/sphere.stl", out_dir), sphere.to_stl_binary("sphere").unwrap());

    let cylinder = CSG::cylinder(1.0, 2.0, 32, None); // start=(0,-1,0), end=(0,1,0), radius=1.0, slices=32
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/01-basic-shapes/cylinder.stl", out_dir),
        cylinder.to_stl_binary("cylinder").unwrap(),
    );

    // 2) Transformations: Translate, Rotate, Scale, Mirror
    let transformed_cube = CSG::cube(2.0, None).center().rotate(45.0, 45.0, 0.0);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/02-transformations/cube_transformed.stl", out_dir),
        transformed_cube
            .to_stl_binary("cube_transformed")
            .unwrap(),
    );

    let plane_x = Plane::from_normal(Vector3::x(), 0.0);
    let mirrored_cube = cube.mirror(plane_x);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/02-transformations/cube_mirrored_x.stl", out_dir),
        mirrored_cube.to_stl_binary("cube_mirrored_x").unwrap(),
    );

    // 3) Boolean operations: Union, Subtract, Intersect
    let base_cube = CSG::cube(3.0, None).center();
    // Reduced resolution for tool_sphere to simplify subsequent operations
    let tool_sphere = CSG::sphere(2.0, 8, 4, None);

    let union_shape = base_cube.union(&tool_sphere);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/03-booleans/union_cube_sphere.stl", out_dir),
        union_shape.to_stl_binary("union_cube_sphere").unwrap(),
    );

    let subtract_shape = base_cube.difference(&tool_sphere);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/03-booleans/subtract_cube_sphere.stl", out_dir),
        subtract_shape
            .to_stl_binary("subtract_cube_sphere")
            .unwrap(),
    );

    let intersect_shape = base_cube.intersection(&tool_sphere);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/03-booleans/intersect_cube_sphere.stl", out_dir),
        intersect_shape
            .to_stl_binary("intersect_cube_sphere")
            .unwrap(),
    );

    // 4) Convex hull
    #[cfg(feature = "chull-io")]
    let hull_of_union = union_shape.convex_hull();
    #[cfg(feature = "stl-io")]
    #[cfg(feature = "chull-io")]
    let _ = fs::write(
        format!("{}/04-advanced-ops/hull_union.stl", out_dir),
        hull_of_union.to_stl_binary("hull_union").unwrap(),
    );

    // 5) Minkowski sum
    #[cfg(feature = "chull-io")]
    let minkowski = cube.minkowski_sum(&sphere);
    #[cfg(feature = "stl-io")]
    #[cfg(feature = "chull-io")]
    let _ = fs::write(
        format!("{}/04-advanced-ops/minkowski_cube_sphere.stl", out_dir),
        minkowski.to_stl_binary("minkowski_cube_sphere").unwrap(),
    );

    // 7) 2D shapes and 2D offsetting
    let square_2d = CSG::square(2.0, None); // 2x2 square, centered
    let _ = fs::write(format!("{}/05-2d-shapes/square_2d.stl", out_dir), square_2d.to_stl_ascii("square_2d"));

    let circle_2d = CSG::circle(1.0, 32, None);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/05-2d-shapes/circle_2d.stl", out_dir),
        circle_2d.to_stl_binary("circle_2d").unwrap(),
    );

    let grown_2d = square_2d.offset(0.5);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/05-2d-shapes/square_2d_grow_0_5.stl", out_dir),
        grown_2d.to_stl_ascii("square_2d_grow_0_5"),
    );

    let shrunk_2d = square_2d.offset(-0.5);
    let _ = fs::write(
        format!("{}/05-2d-shapes/square_2d_shrink_0_5.stl", out_dir),
        shrunk_2d.to_stl_ascii("square_2d_shrink_0_5"),
    );

    // star(num_points, outer_radius, inner_radius)
    let star_2d = CSG::star(5, 2.0, 0.8, None);
    let _ = fs::write(format!("{}/05-2d-shapes/star_2d.stl", out_dir), star_2d.to_stl_ascii("star_2d"));

    // Extrude & Rotate-Extrude
    let extruded_star = star_2d.extrude(1.0);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/06-extrusions/star_extrude.stl", out_dir),
        extruded_star.to_stl_binary("star_extrude").unwrap(),
    );

    let vector_extruded_star = star_2d.extrude_vector(Vector3::new(2.0, 1.0, 1.0));
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/06-extrusions/star_vec_extrude.stl", out_dir),
        vector_extruded_star.to_stl_binary("star_extrude").unwrap(),
    );

    let revolve_circle = circle_2d.translate(10.0, 0.0, 0.0).rotate_extrude(360.0, 32).unwrap();
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/06-extrusions/circle_revolve_360.stl", out_dir),
        revolve_circle.to_stl_binary("circle_revolve_360").unwrap(),
    );

    let partial_revolve = circle_2d.translate(10.0, 0.0, 0.0).rotate_extrude(180.0, 32).unwrap();
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/06-extrusions/circle_revolve_180.stl", out_dir),
        partial_revolve.to_stl_binary("circle_revolve_180").unwrap(),
    );

    // 9) Subdivide triangles (for smoother sphere or shapes):
    let subdiv_sphere = sphere.subdivide_triangles(2.try_into().expect("not 0")); // 2 subdivision levels
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/07-mesh-ops/sphere_subdiv2.stl", out_dir),
        subdiv_sphere.to_stl_binary("sphere_subdiv2").unwrap(),
    );

    // 10) Renormalize polygons (flat shading):
    let mut union_clone = union_shape.clone();
    union_clone.renormalize();
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/07-mesh-ops/union_renormalized.stl", out_dir),
        union_clone.to_stl_binary("union_renormalized").unwrap(),
    );

    // 11) Ray intersection demo (just printing the results)
    {
        let ray_origin = Point3::new(0.0, 0.0, -5.0);
        let ray_dir = Vector3::new(0.0, 0.0, 1.0); // pointing along +Z
        let hits = cube.ray_intersections(&ray_origin, &ray_dir);
        println!("Ray hits on the cube: {:?}", hits);
    }

    // 12) Polyhedron example (simple tetrahedron):
    let points = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.5, 1.0, 0.0],
        [0.5, 0.5, 1.0],
    ];
    let faces = vec![
        vec![0, 2, 1], // base triangle
        vec![0, 1, 3], // side
        vec![1, 2, 3],
        vec![2, 0, 3],
    ];
    let poly = CSG::polyhedron(&points, &faces, None).unwrap();
    #[cfg(feature = "stl-io")]
    let _ = fs::write(format!("{}/11-platonic-solids/tetrahedron.stl", out_dir), poly.to_stl_ascii("tetrahedron"));

    // 13) Text example (2D). Provide a valid TTF font data below:
    // (Replace "asar.ttf" with a real .ttf file in your project.)
    #[cfg(feature = "truetype-text")]
    let font_data = include_bytes!("../asar.ttf");
    #[cfg(feature = "truetype-text")]
    let text_csg = CSG::text("HELLO", font_data, 15.0, None);
    #[cfg(feature = "stl-io")]
    #[cfg(feature = "truetype-text")]
    let _ = fs::write(
        format!("{}/12-specialized-shapes/text_hello_2d.stl", out_dir),
        text_csg.to_stl_binary("text_hello_2d").unwrap(),
    );

    // Optionally extrude the text:
    #[cfg(feature = "truetype-text")]
    let text_extruded = text_csg.extrude(2.0);
    #[cfg(feature = "stl-io")]
    #[cfg(feature = "truetype-text")]
    let _ = fs::write(
        format!("{}/12-specialized-shapes/text_hello_extruded.stl", out_dir),
        text_extruded.to_stl_binary("text_hello_extruded").unwrap(),
    );

    // 14) Mass properties (just printing them)
    let (mass, com, principal_frame) = cube.mass_properties(1.0);
    println!("Cube mass = {}", mass);
    println!("Cube center of mass = {:?}", com);
    println!("Cube principal inertia local frame = {:?}", principal_frame);

    // 1) Create a cube from (-1,-1,-1) to (+1,+1,+1)
    //    (By default, CSG::cube(None) is from -1..+1 if the "radius" is [1,1,1].)
    let cube = CSG::cube(100.0, None);
    // 2) Flatten into the XY plane
    let flattened = cube.flatten();
    let _ = fs::write(
        format!("{}/07-mesh-ops/flattened_cube.stl", out_dir),
        flattened.to_stl_ascii("flattened_cube"),
    );

    // Create a frustum (start=-2, end=+2) with radius1 = 1, radius2 = 2, 32 slices
    let frustum = CSG::frustum_ptp(
        Point3::new(0.0, 0.0, -2.0),
        Point3::new(0.0, 0.0, 2.0),
        1.0,
        2.0,
        32,
        None,
    );
    let _ = fs::write(format!("{}/12-specialized-shapes/frustum.stl", out_dir), frustum.to_stl_ascii("frustum"));

    // 1) Create a cylinder (start=-1, end=+1) with radius=1, 32 slices
    let cyl = CSG::frustum_ptp(
        Point3::new(0.0, 0.0, -1.0),
        Point3::new(0.0, 0.0, 1.0),
        1.0,
        1.0,
        32,
        None,
    );
    // 2) Slice at z=0
    #[cfg(feature = "hashmap")]
    {
        let cross_section = cyl.slice(Plane::from_normal(Vector3::z(), 0.0));
        let _ = fs::write(format!("{}/07-mesh-ops/sliced_cylinder.stl", out_dir), cyl.to_stl_ascii("sliced_cylinder"));
        let _ = fs::write(
            format!("{}/07-mesh-ops/sliced_cylinder_slice.stl", out_dir),
            cross_section.to_stl_ascii("sliced_cylinder_slice"),
        );
    }

    // let poor_geometry_shape = moved_cube.difference(&sphere);
    //#[cfg(feature = "earclip-io")]
    // let retriangulated_shape = poor_geometry_shape.triangulate_earclip();
    //#[cfg(all(feature = "earclip-io", feature = "stl-io"))]
    // let _ = fs::write("stl/retriangulated.stl", retriangulated_shape.to_stl_binary("retriangulated").unwrap());

    let sphere_test = CSG::sphere(1.0, 16, 8, None);
    let cube_test = CSG::cube(1.0, None);
    let res = cube_test.difference(&sphere_test);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        format!("{}/03-booleans/sphere_cube_test.stl", out_dir),
        res.to_stl_binary("sphere_cube_test").unwrap(),
    );
    assert_eq!(res.bounding_box(), cube_test.bounding_box());

    #[cfg(all(feature = "stl-io", feature = "metaballs"))]
    {
        // Suppose we want two overlapping metaballs
        let balls = vec![
            MetaBall::new(Point3::origin(), 1.0),
            MetaBall::new(Point3::new(1.75, 0.0, 0.0), 1.0),
        ];

        let resolution = (60, 60, 60);
        let iso_value = 1.0;
        let padding = 1.0;

        #[cfg(feature = "metaballs")]
        let metaball_csg = CSG::metaballs(&balls, resolution, iso_value, padding, None);

        // For instance, save to STL
        let stl_data = metaball_csg.to_stl_binary("my_metaballs").unwrap();
        std::fs::write(format!("{}/08-implicit-surfaces/metaballs.stl", out_dir), stl_data).expect("Failed to write metaballs.stl");
    }

    #[cfg(feature = "sdf")]
    {
        // Example SDF for a sphere of radius 1.5 centered at (0,0,0)
        let my_sdf = |p: &Point3<Real>| p.coords.norm() - 1.5;

        let resolution = (60, 60, 60);
        let min_pt = Point3::new(-2.0, -2.0, -2.0);
        let max_pt = Point3::new(2.0, 2.0, 2.0);
        let iso_value = 0.0; // Typically zero for SDF-based surfaces

        let csg_shape = CSG::sdf(my_sdf, resolution, min_pt, max_pt, iso_value, None);

        // Now `csg_shape` is your polygon mesh as a CSG you can union, subtract, or export:
        #[cfg(feature = "stl-io")]
        let _ = std::fs::write(
            format!("{}/08-implicit-surfaces/sdf_sphere.stl", out_dir),
            csg_shape.to_stl_binary("sdf_sphere").unwrap(),
        );
    }

    // Create a pie slice of radius 2, from 0 to 90 degrees
    let wedge = CSG::pie_slice(2.0, 0.0, 90.0, 16, None);
    let _ = fs::write(format!("{}/05-2d-shapes/pie_slice.stl", out_dir), wedge.to_stl_ascii("pie_slice"));

    // Create a 2D "metaball" shape from 3 circles
    use nalgebra::Point2;
    let balls_2d = vec![
        (Point2::new(0.0, 0.0), 1.0),
        (Point2::new(1.5, 0.0), 1.0),
        (Point2::new(0.75, 1.0), 0.5),
    ];
    let mb2d = CSG::metaballs2d(&balls_2d, (100, 100), 1.0, 0.25, None);
    let _ = fs::write(format!("{}/08-implicit-surfaces/mb2d.stl", out_dir), mb2d.to_stl_ascii("metaballs2d"));

    // Create a supershape
    let sshape = CSG::supershape(1.0, 1.0, 6.0, 1.0, 1.0, 1.0, 128, None);
    let _ = fs::write(format!("{}/05-2d-shapes/supershape.stl", out_dir), sshape.to_stl_ascii("supershape"));

    // Distribute a square along an arc
    let square = CSG::circle(1.0, 32, None);
    let arc_array = square.distribute_arc(5, 5.0, 0.0, 180.0);
    let _ = fs::write(format!("{}/12-specialized-shapes/arc_array.stl", out_dir), arc_array.to_stl_ascii("arc_array"));

    // Distribute that wedge along a linear axis
    let wedge_line = wedge.distribute_linear(4, nalgebra::Vector3::new(1.0, 0.0, 0.0), 3.0);
    let _ = fs::write(format!("{}/12-specialized-shapes/wedge_line.stl", out_dir), wedge_line.to_stl_ascii("wedge_line"));

    // Make a 4x4 grid of the supershape
    let grid_of_ss = sshape.distribute_grid(4, 4, 3.0, 3.0);
    let _ = fs::write(format!("{}/12-specialized-shapes/grid_of_ss.stl", out_dir), grid_of_ss.to_stl_ascii("grid_of_ss"));

    // 1. Circle with keyway
    let keyway_shape = CSG::circle_with_keyway(10.0, 64, 2.0, 3.0, None);
    let _ = fs::write(
        format!("{}/05-2d-shapes/keyway_shape.stl", out_dir),
        keyway_shape.to_stl_ascii("keyway_shape"),
    );
    // Extrude it 2 units:
    let keyway_3d = keyway_shape.extrude(2.0);
    let _ = fs::write(format!("{}/06-extrusions/keyway_3d.stl", out_dir), keyway_3d.to_stl_ascii("keyway_3d"));

    // 2. D-shape
    let d_shape = CSG::circle_with_flat(5.0, 32, 2.0, None);
    let _ = fs::write(format!("{}/05-2d-shapes/d_shape.stl", out_dir), d_shape.to_stl_ascii("d_shape"));
    let d_3d = d_shape.extrude(1.0);
    let _ = fs::write(format!("{}/06-extrusions/d_3d.stl", out_dir), d_3d.to_stl_ascii("d_3d"));

    // 3. Double-flat circle
    let double_flat = CSG::circle_with_two_flats(8.0, 64, 3.0, None);
    let _ = fs::write(format!("{}/05-2d-shapes/double_flat.stl", out_dir), double_flat.to_stl_ascii("double_flat"));
    let df_3d = double_flat.extrude(0.5);
    let _ = fs::write(format!("{}/06-extrusions/df_3d.stl", out_dir), df_3d.to_stl_ascii("df_3d"));

    // A 3D teardrop shape
    let teardrop_solid = CSG::teardrop(3.0, 5.0, 32, 32, None);
    let _ = fs::write(
        format!("{}/12-specialized-shapes/teardrop_solid.stl", out_dir),
        teardrop_solid.to_stl_ascii("teardrop_solid"),
    );

    // A 3D egg shape
    let egg_solid = CSG::egg(2.0, 4.0, 8, 16, None);
    let _ = fs::write(format!("{}/12-specialized-shapes/egg_solid.stl", out_dir), egg_solid.to_stl_ascii("egg_solid"));

    // An ellipsoid with X radius=2, Y radius=1, Z radius=3
    let ellipsoid = CSG::ellipsoid(2.0, 1.0, 3.0, 16, 8, None);
    let _ = fs::write(format!("{}/12-specialized-shapes/ellipsoid.stl", out_dir), ellipsoid.to_stl_ascii("ellipsoid"));

    // A teardrop 'blank' hole
    let teardrop_cylinder = CSG::teardrop_cylinder(2.0, 4.0, 32.0, 16, None);
    let _ = fs::write(
        format!("{}/06-extrusions/teardrop_cylinder.stl", out_dir),
        teardrop_cylinder.to_stl_ascii("teardrop_cylinder"),
    );

    // 1) polygon()
    let polygon_2d = CSG::polygon(&[[0.0, 0.0], [2.0, 0.0], [1.5, 1.0], [1.0, 2.0]], None);
    let _ = fs::write(format!("{}/05-2d-shapes/polygon_2d.stl", out_dir), polygon_2d.to_stl_ascii("polygon_2d"));

    // 2) rounded_rectangle(width, height, corner_radius, corner_segments)
    let rrect_2d = CSG::rounded_rectangle(4.0, 2.0, 0.3, 8, None);
    let _ = fs::write(
        format!("{}/05-2d-shapes/rounded_rectangle_2d.stl", out_dir),
        rrect_2d.to_stl_ascii("rounded_rectangle_2d"),
    );

    // 3) ellipse(width, height, segments)
    let ellipse = CSG::ellipse(3.0, 1.5, 32, None);
    let _ = fs::write(format!("{}/05-2d-shapes/ellipse.stl", out_dir), ellipse.to_stl_ascii("ellipse"));

    // 4) regular_ngon(sides, radius)
    let ngon_2d = CSG::regular_ngon(6, 1.0, None); // Hexagon
    let _ = fs::write(format!("{}/05-2d-shapes/ngon_2d.stl", out_dir), ngon_2d.to_stl_ascii("ngon_2d"));

    // 6) trapezoid(top_width, bottom_width, height)
    let trap_2d = CSG::trapezoid(1.0, 2.0, 2.0, 0.5, None);
    let _ = fs::write(format!("{}/05-2d-shapes/trapezoid_2d.stl", out_dir), trap_2d.to_stl_ascii("trapezoid_2d"));

    // 8) teardrop(width, height, segments) [2D shape]
    let teardrop_2d = CSG::teardrop_outline(2.0, 3.0, 16, None);
    let _ = fs::write(format!("{}/05-2d-shapes/teardrop_2d.stl", out_dir), teardrop_2d.to_stl_ascii("teardrop_2d"));

    // 9) egg_outline(width, length, segments) [2D shape]
    let egg_2d = CSG::egg_outline(2.0, 4.0, 32, None);
    let _ = fs::write(
        format!("{}/05-2d-shapes/egg_outline_2d.stl", out_dir),
        egg_2d.to_stl_ascii("egg_outline_2d"),
    );

    // 10) squircle(width, height, segments)
    let squircle_2d = CSG::squircle(3.0, 3.0, 32, None);
    let _ = fs::write(format!("{}/05-2d-shapes/squircle_2d.stl", out_dir), squircle_2d.to_stl_ascii("squircle_2d"));

    // 11) keyhole(circle_radius, handle_width, handle_height, segments)
    let keyhole_2d = CSG::keyhole(1.0, 1.0, 2.0, 16, None);
    let _ = fs::write(format!("{}/05-2d-shapes/keyhole_2d.stl", out_dir), keyhole_2d.to_stl_ascii("keyhole_2d"));

    // 12) reuleaux_polygon(sides, side_len, segments)
    let reuleaux3_2d = CSG::reuleaux(3, 2.0, 64, None); // Reuleaux triangle
    let _ = fs::write(
        format!("{}/05-2d-shapes/reuleaux3_2d.stl", out_dir),
        reuleaux3_2d.to_stl_ascii("reuleaux_2d"),
    );

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux4_2d = CSG::reuleaux(4, 2.0, 64, None); // Reuleaux triangle
    let _ = fs::write(
        format!("{}/05-2d-shapes/reuleaux4_2d.stl", out_dir),
        reuleaux4_2d.to_stl_ascii("reuleaux_2d"),
    );

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux5_2d = CSG::reuleaux(5, 2.0, 64, None); // Reuleaux triangle
    let _ = fs::write(
        format!("{}/05-2d-shapes/reuleaux5_2d.stl", out_dir),
        reuleaux5_2d.to_stl_ascii("reuleaux_2d"),
    );

    // 13) ring(inner_diam, thickness, segments)
    let ring_2d = CSG::ring(5.0, 1.0, 32, None);
    let _ = fs::write(format!("{}/05-2d-shapes/ring_2d.stl", out_dir), ring_2d.to_stl_ascii("ring_2d"));

    // 15) from_image(img, threshold, closepaths, metadata) [requires "image" feature]
    #[cfg(feature = "image")]
    {
        // Make a simple 64x64 gray image with a circle in the center
        let mut img: GrayImage = ImageBuffer::new(64, 64);
        // Fill a small circle of "white" pixels in the middle
        let center = (32, 32);
        for y in 0..64 {
            for x in 0..64 {
                let dx = x as i32 - center.0;
                let dy = y as i32 - center.1;
                if dx * dx + dy * dy < 15 * 15 {
                    img.put_pixel(x, y, image::Luma([255u8]));
                }
            }
        }
        let csg_img = CSG::from_image(&img, 128, true, None).center();
        let _ = fs::write(format!("{}/05-2d-shapes/from_image.stl", out_dir), csg_img.to_stl_ascii("from_image"));
    }

    // 16) gyroid(...) – uses the current CSG volume as a bounding region
    // Let's reuse the `cube` from above:
    #[cfg(feature = "stl-io")]
    {
        println!("Starting TPMS generation with potentially simplified hull_of_union...");
        let now = Instant::now();
        // Attempt to use the hull_of_union, which should now be simpler
        let scaled_hull = hull_of_union.scale(20.0, 20.0, 20.0);
        println!("Time to scale hull_of_union: {:?}", now.elapsed());

        let now = Instant::now();
        let _bounding_box = scaled_hull.bounding_box();
        println!("Time to calculate bounding_box for scaled_hull: {:?}", now.elapsed());

        let now = Instant::now();
        let gyroid_inside_cube = scaled_hull.gyroid(64, 2.0, 0.0, None);
        println!("Time to generate gyroid: {:?}", now.elapsed());
        let _ = fs::write(
            format!("{}/08-implicit-surfaces/gyroid_cube.stl", out_dir),
            gyroid_inside_cube.to_stl_binary("gyroid_cube").unwrap(),
        );

        let now = Instant::now();
        let schwarzp_inside_cube = scaled_hull.schwarz_p(64, 2.0, 0.0, None);
        println!("Time to generate schwarz_p: {:?}", now.elapsed());
        let _ = fs::write(
            format!("{}/08-implicit-surfaces/schwarz_p_cube.stl", out_dir),
            schwarzp_inside_cube.to_stl_binary("schwarz_p_cube").unwrap(),
        );

        let now = Instant::now();
        let schwarzd_inside_cube = scaled_hull.schwarz_d(64, 2.0, 0.0, None);
        println!("Time to generate schwarz_d: {:?}", now.elapsed());
        let _ = fs::write(
            format!("{}/08-implicit-surfaces/schwarz_d_cube.stl", out_dir),
            schwarzd_inside_cube.to_stl_binary("schwarz_d_cube").unwrap(),
        );
        println!("Finished TPMS generation.");
    }

    // Define the start point and the arrow direction vector.
    // The arrow's length is the norm of the direction vector.
    let start = Point3::new(1.0, 1.0, 1.0);
    let direction = Vector3::new(10.0, 5.0, 20.0);

    // Define the resolution (number of segments for the cylindrical shaft and head).
    let segments = 16;

    // Create the arrow. We pass `None` for metadata.
    let arrow_csg = CSG::arrow(start, direction, segments, true, None::<()>);
    let _ = fs::write(format!("{}/12-specialized-shapes/arrow.stl", out_dir), arrow_csg.to_stl_ascii("arrow_example"));

    let arrow_reversed_csg = CSG::arrow(start, direction, segments, false, None::<()>);
    let _ = fs::write(
        format!("{}/12-specialized-shapes/arrow_reversed.stl", out_dir),
        arrow_reversed_csg.to_stl_ascii("arrow_example"),
    );

    // 2-D profile for NACA 2412, 1 m chord, 100 pts / surface
    let naca2412 = CSG::airfoil("2412", 1.0, 100, None);
    let _ = fs::write(format!("{}/10-curves/naca2412.stl", out_dir), naca2412.to_stl_ascii("2412"));

    // quick solid wing rib 5 mm thick
    let rib = naca2412.extrude(0.005);
    let _ = fs::write(format!("{}/06-extrusions/naca2412_extruded.stl", out_dir), rib.to_stl_ascii("2412_extruded"));

    // symmetric foil for a centerboard
    let naca0015 = CSG::airfoil("0015", 0.3, 80, None)
        .extrude_vector(nalgebra::Vector3::new(0.0, 0.0, 1.2));
    let _ = fs::write(format!("{}/10-curves/naca0015.stl", out_dir), naca0015.to_stl_ascii("naca0015"));

    let oct = CSG::octahedron(10.0, None);
    let _ = fs::write(format!("{}/11-platonic-solids/octahedron.stl", out_dir), oct.to_stl_ascii("octahedron"));

    // let dodec = CSG::dodecahedron(15.0, None);
    // let _ = fs::write("stl/dodecahedron.stl", dodec.to_stl_ascii(""));

    let ico = CSG::icosahedron(12.0, None);
    let _ = fs::write(format!("{}/11-platonic-solids/icosahedron.stl", out_dir), ico.to_stl_ascii(""));

    let torus = CSG::torus(20.0, 5.0, 48, 24, None);
    let _ = fs::write(format!("{}/12-specialized-shapes/torus.stl", out_dir), torus.to_stl_ascii(""));

    let heart2d = CSG::heart(30.0, 25.0, 128, None);
    let _ = fs::write(format!("{}/05-2d-shapes/heart2d.stl", out_dir), heart2d.to_stl_ascii(""));

    let crescent2d = CSG::crescent(10.0, 7.0, 4.0, 64, None);
    let _ = fs::write(format!("{}/05-2d-shapes/crescent2d.stl", out_dir), crescent2d.to_stl_ascii(""));

    // ---------------------------------------------------------
    // Additional "SCENES" Demonstrating Each Function Minimally
    //
    // In these scenes, we typically:
    //   1) Create the shape
    //   2) Extrude (if 2D) so we can save an STL
    //   3) Optionally union with a small arrow that points to
    //      a location of interest in the shape
    //   4) Save the result as an STL, e.g. "scene_XX_something.stl"
    //
    // Because many shapes are already shown above, these are
    // just short examples to help with explanation.
    // ---------------------------------------------------------

    // Scene A: Demonstrate a right_triangle(width=2, height=1)
    {
        let tri_2d = CSG::right_triangle(2.0, 1.0, None);
        // A tiny arrow pointing from the right-angle corner outward:
        let arrow = CSG::arrow(
            Point3::new(0.0, 0.0, 0.1), // at corner
            Vector3::new(0.5, 0.0, 0.0),
            8,
            true,
            None::<()>,
        )
        .scale(0.05, 0.05, 0.05);
        let scene = tri_2d.extrude(0.1).union(&arrow);
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_right_triangle.stl", out_dir),
            scene.to_stl_ascii("scene_right_triangle"),
        );
    }

    // Scene B: Demonstrate extrude_vector(direction)
    {
        let circle2d = CSG::circle(1.0, 32, None);
        // extrude along an arbitrary vector
        let extruded_along_vec = circle2d.extrude_vector(Vector3::new(0.0, 0.0, 2.0));
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_extrude_vector.stl", out_dir),
            extruded_along_vec.to_stl_ascii("scene_extrude_vector"),
        );
    }

    // Scene E: Demonstrate center() (moves shape so bounding box is centered on the origin)
    {
        let off_center_circle = CSG::circle(1.0, 32, None)
            .translate(5.0, 2.0, 0.0)
            .extrude(0.1);
        let centered_circle = off_center_circle.center();
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_circle_off_center.stl", out_dir),
            off_center_circle.to_stl_ascii("scene_circle_off_center"),
        );
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_circle_centered.stl", out_dir),
            centered_circle.to_stl_ascii("scene_circle_centered"),
        );
    }

    // Scene F: Demonstrate float() (moves shape so bottom is at z=0)
    {
        let sphere_for_float = CSG::sphere(1.0, 16, 8, None).translate(0.0, 0.0, -1.5);
        let floated = sphere_for_float.float();
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_sphere_before_float.stl", out_dir),
            sphere_for_float.to_stl_ascii("scene_sphere_before_float"),
        );
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_sphere_floated.stl", out_dir),
            floated.to_stl_ascii("scene_sphere_floated"),
        );
    }

    // Scene G: Demonstrate inverse() (flips inside/outside)
    {
        // Hard to visualize in STL, but let's do it anyway
        let inv_sphere = sphere.inverse();
        #[cfg(feature = "stl-io")]
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_inverse_sphere.stl", out_dir),
            inv_sphere.to_stl_binary("scene_inverse_sphere").unwrap(),
        );
    }

    // Scene H: Demonstrate tessellate() (forces triangulation)
    {
        let tri_sphere = sphere.tessellate();
        #[cfg(feature = "stl-io")]
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_tessellate_sphere.stl", out_dir),
            tri_sphere.to_stl_binary("scene_tessellate_sphere").unwrap(),
        );
    }

    // Scene I: Demonstrate slice(plane) – slice a cube at z=0
    {
        let plane_z = Plane::from_normal(Vector3::z(), 0.5);
        let sliced_polygons = cube.slice(plane_z);
        let _ = fs::write(format!("{}/13-misc-scenes/scene_sliced_cube.stl", out_dir), cube.to_stl_ascii("sliced_cube"));
        // Save cross-section as well
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_sliced_cube_section.stl", out_dir),
            sliced_polygons.to_stl_ascii("sliced_cube_section"),
        );
    }

    // Scene J: Demonstrate re-computing vertices() or printing them
    {
        let circle_extruded = CSG::circle(1.0, 32, None).extrude(0.5);
        let verts = circle_extruded.vertices();
        println!("Scene J circle_extruded has {} vertices", verts.len());
        // We'll still save an STL so there's a visual
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_j_circle_extruded.stl", out_dir),
            circle_extruded.to_stl_ascii("scene_j_circle_extruded"),
        );
    }

    // Scene K: Demonstrate reuleaux_polygon with a typical triangle shape
    // (already used sides=4 above, so let's do sides=3 here)
    {
        let reuleaux_tri = CSG::reuleaux(3, 2.0, 16, None).extrude(0.1);
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_reuleaux_triangle.stl", out_dir),
            reuleaux_tri.to_stl_ascii("scene_reuleaux_triangle"),
        );
    }

    // Scene L: Demonstrate rotate_extrude (360 deg) on a square
    {
        let small_square = CSG::square(1.0, None).translate(2.0, 0.0, 0.0);
        let revolve = small_square.rotate_extrude(360.0, 24).unwrap();
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_square_revolve_360.stl", out_dir),
            revolve.to_stl_ascii("scene_square_revolve_360"),
        );
    }

    // Scene M: Demonstrate "mirror" across a Y=0 plane
    {
        let plane_y = Plane::from_normal(Vector3::y(), 0.0);
        let shape = CSG::rectangle(2.0, 1.0, None)
            .translate(1.0, 1.0, 0.0)
            .extrude(0.1);
        let mirrored = shape.mirror(plane_y);
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_square_mirrored_y.stl", out_dir),
            mirrored.to_stl_ascii("scene_square_mirrored_y"),
        );
    }

    // Scene N: Demonstrate scale()
    {
        let scaled = sphere.scale(1.0, 2.0, 0.5);
        #[cfg(feature = "stl-io")]
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_scaled_sphere.stl", out_dir),
            scaled.to_stl_binary("scene_scaled_sphere").unwrap(),
        );
    }

    // Scene O: Demonstrate transform() with an arbitrary affine matrix
    {
        use nalgebra::{Matrix4, Translation3};
        let xlate = Translation3::new(2.0, 0.0, 1.0).to_homogeneous();
        // Scale matrix
        let scale_mat = Matrix4::new_scaling(0.5);
        // Combine
        let transform_mat = xlate * scale_mat;
        let shape = CSG::cube(1.0, None).transform(&transform_mat);
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_transform_cube.stl", out_dir),
            shape.to_stl_ascii("scene_transform_cube"),
        );
    }

    // Scene P: Demonstrate offset(distance)
    {
        let poly_2d = CSG::polygon(&[[0.0, 0.0], [2.0, 0.0], [1.0, 1.5]], None);
        let grown = poly_2d.offset(0.2);
        let scene = grown.extrude(0.1);
        let _ = fs::write(
            format!("{}/13-misc-scenes/scene_offset_grown.stl", out_dir),
            scene.to_stl_ascii("scene_offset_grown"),
        );
    }

    let gear_involute_2d = CSG::involute_gear_2d(
        2.0,  // module [mm]
        20,   // z – number of teeth
        20.0, // α – pressure angle [deg]
        0.05, // radial clearance
        0.02, // backlash at pitch line
        14,   // segments per involute flank
        None,
    );
    let _ = fs::write(
        format!("{}/09-gears/gear_involute_2d.stl", out_dir),
        gear_involute_2d.to_stl_ascii("gear_involute_2d"),
    );

    let gear_cycloid_2d = CSG::cycloidal_gear_2d(
        2.0,  // module
        17,   // gear teeth
        18,   // mating pin-wheel teeth (zₚ = z±1)
        0.05, // clearance
        20,   // segments per flank
        None,
    );
    let _ = fs::write(
        format!("{}/09-gears/gear_cycloid_2d.stl", out_dir),
        gear_cycloid_2d.to_stl_ascii("gear_cycloid_2d"),
    );

    let rack_involute = CSG::involute_rack_2d(
        2.0,  // module
        12,   // number of rack teeth to generate
        20.0, // pressure angle
        0.05, // clearance
        0.02, // backlash
        None,
    );
    let _ = fs::write(
        format!("{}/09-gears/rack_involute.stl", out_dir),
        rack_involute.to_stl_ascii("rack_involute"),
    );

    let rack_cycloid = CSG::cycloidal_rack_2d(
        2.0,  // module
        12,   // teeth
        1.0,  // generating-circle radius  (≈ m/2 for a conventional pin-rack)
        0.05, // clearance
        24,   // segments per flank
        None,
    );
    let _ = fs::write(
        format!("{}/09-gears/rack_cycloid.stl", out_dir),
        rack_cycloid.to_stl_ascii("rack_cycloid"),
    );

    let spur_involute = CSG::spur_gear_involute(
        2.0, 20, 20.0, 0.05, 0.02, 14, 12.0, // face-width (extrusion thickness)
        None,
    );
    let _ = fs::write(
        format!("{}/09-gears/spur_involute.stl", out_dir),
        spur_involute.to_stl_ascii("spur_involute"),
    );

    let spur_cycloid = CSG::spur_gear_cycloid(
        2.0, 17, 18, 0.05, 20, 12.0, // thickness
        None,
    );
    let _ = fs::write(
        format!("{}/09-gears/spur_cycloid.stl", out_dir),
        spur_cycloid.to_stl_ascii("spur_cycloid"),
    );

    // let helical = CSG::helical_involute_gear(
    // 2.0,   // module
    // 20,    // z
    // 20.0,  // pressure angle
    // 0.05, 0.02, 14,
    // 25.0,   // face-width
    // 15.0,   // helix angle β [deg]
    // 40,     // axial slices (resolution of the twist)
    // None,
    // );
    // let _ = fs::write("stl/helical.stl", helical.to_stl_ascii("helical"));

    // ---------------------------------------------------------------------
    // Bézier curve demo ----------------------------------------------------
    let bezier_ctrl = &[
        [0.0, 0.0], // P0
        [1.0, 2.0], // P1
        [3.0, 3.0], // P2
        [4.0, 0.0], // P3
    ];
    let bezier_2d = CSG::bezier(bezier_ctrl, 128, None);
    let _ = fs::write(format!("{}/10-curves/bezier_2d.stl", out_dir), bezier_2d.to_stl_ascii("bezier_2d"));

    // give it a little "body" so we can see it in a solid viewer
    let bezier_3d = bezier_2d.extrude(0.25);
    let _ = fs::write(
        format!("{}/06-extrusions/bezier_extruded.stl", out_dir),
        bezier_3d.to_stl_ascii("bezier_extruded"),
    );

    // ---------------------------------------------------------------------
    // B-spline demo --------------------------------------------------------
    let bspline_ctrl = &[[0.0, 0.0], [1.0, 2.5], [3.0, 3.0], [5.0, 0.0], [6.0, -1.5]];
    let bspline_2d = CSG::bspline(
        bspline_ctrl,
        // degree p =
        3,
        // seg/span
        32,
        None,
    );
    let _ = fs::write(format!("{}/10-curves/bspline_2d.stl", out_dir), bspline_2d.to_stl_ascii("bspline_2d"));

    #[cfg(feature = "bevymesh")]
    println!("{:#?}", bezier_3d.to_bevy_mesh());

    // let bspline_3d = bspline_2d.extrude(0.25);
    // let _ = fs::write(
    //    "stl/bspline_extruded.stl",
    //    bspline_3d.to_stl_ascii("bspline_extruded"),
    // );

    // Done!
    println!(
        "All scenes have been created and written to the 'outputs' folder (where applicable)."
    );
}

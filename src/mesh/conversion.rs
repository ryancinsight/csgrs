//! Conversion utilities for external formats and integrations

use super::Mesh;
use std::fmt::Debug;

impl<S: Clone + Send + Sync + Debug> Mesh<S> {
    /// Convert a Mesh into a Bevy `Mesh`.
    #[cfg(feature = "bevymesh")]
    pub fn to_bevy_mesh(&self) -> bevy_mesh::Mesh {
        use bevy_asset::RenderAssetUsages;
        use bevy_mesh::{Indices, Mesh};
        use wgpu_types::PrimitiveTopology;

        let triangulated_mesh = &self.triangulate();
        let polygons = &triangulated_mesh.polygons;

        // Prepare buffers
        let mut positions_32 = Vec::new();
        let mut normals_32 = Vec::new();
        let mut indices = Vec::with_capacity(polygons.len() * 3);

        let mut index_start = 0u32;

        // Each polygon is assumed to have exactly 3 vertices after tessellation.
        for poly in polygons {
            // skip any degenerate polygons
            if poly.vertices.len() != 3 {
                continue;
            }

            // push 3 positions/normals
            for v in &poly.vertices {
                positions_32.push([v.pos.x as f32, v.pos.y as f32, v.pos.z as f32]);
                normals_32.push([v.normal.x as f32, v.normal.y as f32, v.normal.z as f32]);
            }

            // triangle indices
            indices.push(index_start);
            indices.push(index_start + 1);
            indices.push(index_start + 2);
            index_start += 3;
        }

        // Create the mesh with the new 2-argument constructor
        let mut mesh =
            Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());

        // Insert attributes. Note the `<Vec<[f32; 3]>>` usage.
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions_32);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals_32);

        // Insert triangle indices
        mesh.insert_indices(Indices::U32(indices));

        mesh
    }
}

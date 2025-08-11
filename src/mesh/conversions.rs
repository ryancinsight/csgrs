use crate::mesh::plane::Plane as MeshPlane;
use crate::mesh::polygon::Polygon as MeshPolygon;
use crate::mesh::vertex::Vertex as MeshVertex;
use crate::voxels::polygon::Polygon as VoxPolygon;
use crate::voxels::vertex::Vertex as VoxVertex;
use std::sync::OnceLock;

impl<S: Clone + Send + Sync> From<VoxPolygon<S>> for MeshPolygon<S> {
    fn from(p: VoxPolygon<S>) -> Self {
        let verts: Vec<MeshVertex> = p.vertices.into_iter().map(Into::into).collect();
        let plane: MeshPlane = p.plane.into();
        MeshPolygon { vertices: verts, plane, bounding_box: OnceLock::new(), metadata: p.metadata }
    }
}

impl From<VoxVertex> for MeshVertex {
    fn from(v: VoxVertex) -> Self { MeshVertex::new(v.pos, v.normal) }
}


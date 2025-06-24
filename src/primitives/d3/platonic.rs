//! Platonic solid 3D primitive shapes
//!
//! This module contains the five Platonic solids and other regular polyhedra.

use crate::csg::CSG;
use crate::core::float_types::Real;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Regular octahedron scaled by `radius`
    pub fn octahedron(radius: Real, metadata: Option<S>) -> Self {
        let pts = &[
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
        ];
        let faces = vec![
            vec![0, 2, 4],
            vec![2, 1, 4],
            vec![1, 3, 4],
            vec![3, 0, 4],
            vec![5, 2, 0],
            vec![5, 1, 2],
            vec![5, 3, 1],
            vec![5, 0, 3],
        ];
        let scaled: Vec<[Real; 3]> = pts
            .iter()
            .map(|&[x, y, z]| [x * radius, y * radius, z * radius])
            .collect();
        Self::polyhedron(&scaled, &faces, metadata)
    }

    /// Regular icosahedron scaled by `radius`
    pub fn icosahedron(radius: Real, metadata: Option<S>) -> Self {
        // radius scale factor
        let factor = radius * 0.5878;
        // golden ratio
        let phi: Real = (1.0 + 5.0_f64.sqrt() as Real) * 0.5;
        // normalise so the circum-radius is 1
        let inv_len = (1.0 + phi * phi).sqrt().recip();
        let a = inv_len;
        let b = phi * inv_len;

        // 12 vertices ----------------------------------------------------
        let pts: [[Real; 3]; 12] = [
            [-a, b, 0.0],
            [a, b, 0.0],
            [-a, -b, 0.0],
            [a, -b, 0.0],
            [0.0, -a, b],
            [0.0, a, b],
            [0.0, -a, -b],
            [0.0, a, -b],
            [b, 0.0, -a],
            [b, 0.0, a],
            [-b, 0.0, -a],
            [-b, 0.0, a],
        ];

        // 20 faces (counter-clockwise when viewed from outside) ----------
        let faces: [[usize; 3]; 20] = [
            [0, 11, 5],
            [0, 5, 1],
            [0, 1, 7],
            [0, 7, 10],
            [0, 10, 11],
            [1, 5, 9],
            [5, 11, 4],
            [11, 10, 2],
            [10, 7, 6],
            [7, 1, 8],
            [3, 9, 4],
            [3, 4, 2],
            [3, 2, 6],
            [3, 6, 8],
            [3, 8, 9],
            [4, 9, 5],
            [2, 4, 11],
            [6, 2, 10],
            [8, 6, 7],
            [9, 8, 1],
        ];

        let faces_vec: Vec<Vec<usize>> = faces.iter().map(|f| f.to_vec()).collect();

        Self::polyhedron(&pts, &faces_vec, metadata).scale(factor, factor, factor)
    }
} 

use super::CSG;
use crate::core::float_types::Real;
use nalgebra;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Distribute this CSG `count` times around an arc (in XY plane) of radius,
    /// from `start_angle_deg` to `end_angle_deg`.
    /// Returns a new CSG with all copies (their polygons).
    pub fn distribute_arc(
        &self,
        count: usize,
        radius: Real,
        start_angle_deg: Real,
        end_angle_deg: Real,
    ) -> CSG<S> {
        if count < 1 {
            return self.clone();
        }
        let start_rad = start_angle_deg.to_radians();
        let end_rad = end_angle_deg.to_radians();
        let sweep = end_rad - start_rad;

        (0..count)
            .map(|i| {
                let t = if count == 1 {
                    0.5
                } else {
                    i as Real / ((count - 1) as Real)
                };

                let angle = start_rad + t * sweep;
                let rot =
                    nalgebra::Rotation3::from_axis_angle(&nalgebra::Vector3::z_axis(), angle)
                        .to_homogeneous();

                // translate out to radius in x
                let trans = nalgebra::Translation3::new(radius, 0.0, 0.0).to_homogeneous();
                let mat = rot * trans;
                self.transform(&mat)
            })
            .reduce(|acc, csg| acc.union(&csg))
            .unwrap()
    }

    /// Distribute this CSG `count` times along a straight line (vector),
    /// each copy spaced by `spacing`.
    /// E.g. if `dir=(1.0,0.0,0.0)` and `spacing=2.0`, you get copies at
    /// x=0, x=2, x=4, ... etc.
    pub fn distribute_linear(
        &self,
        count: usize,
        dir: nalgebra::Vector3<Real>,
        spacing: Real,
    ) -> CSG<S> {
        if count < 1 {
            return self.clone();
        }
        let step = dir.normalize() * spacing;

        (0..count)
            .map(|i| {
                let offset = step * (i as Real);
                let trans = nalgebra::Translation3::from(offset).to_homogeneous();
                self.transform(&trans)
            })
            .reduce(|acc, csg| acc.union(&csg))
            .unwrap()
    }

    /// Distribute this CSG in a grid of `rows x cols`, with spacing dx, dy in XY plane.
    /// top-left or bottom-left depends on your usage of row/col iteration.
    pub fn distribute_grid(&self, rows: usize, cols: usize, dx: Real, dy: Real) -> CSG<S> {
        if rows < 1 || cols < 1 {
            return self.clone();
        }
        let step_x = nalgebra::Vector3::new(dx, 0.0, 0.0);
        let step_y = nalgebra::Vector3::new(0.0, dy, 0.0);

        (0..rows)
            .flat_map(|r| {
                (0..cols).map(move |c| {
                    let offset = step_x * (c as Real) + step_y * (r as Real);
                    let trans = nalgebra::Translation3::from(offset).to_homogeneous();
                    self.transform(&trans)
                })
            })
            .reduce(|acc, csg| acc.union(&csg))
            .unwrap()
    }
}

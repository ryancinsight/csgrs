//! Provides a `MetaBall` struct and functions for creating a `Sketch` from [MetaBalls](https://en.wikipedia.org/wiki/Metaballs)

use crate::float_types::Real;
use crate::sketch::Sketch;
use crate::traits::CSG;
use geo::{
    CoordsIter, Geometry, GeometryCollection, LineString, Polygon as GeoPolygon, coord,
};
use hashbrown::HashMap;
use std::fmt::Debug;

/// Represents a marching squares cell with its corner values
#[derive(Clone, Copy)]
struct MarchingSquaresCell {
    x0: Real,
    y0: Real,
    x1: Real,
    y1: Real,
    v0: Real,
    v1: Real,
    v2: Real,
    v3: Real,
}

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Create a 2D metaball iso-contour in XY plane from a set of 2D metaballs.
    /// - `balls`: array of (center, radius).
    /// - `resolution`: (nx, ny) grid resolution for marching squares.
    /// - `iso_value`: threshold for the iso-surface.
    /// - `padding`: extra boundary beyond each ball's radius.
    /// - `metadata`: optional user metadata.
    pub fn metaballs(
        balls: &[(nalgebra::Point2<Real>, Real)],
        resolution: (usize, usize),
        iso_value: Real,
        padding: Real,
        metadata: Option<S>,
    ) -> Sketch<S> {
        let (nx, ny) = resolution;
        if balls.is_empty() || nx < 2 || ny < 2 {
            return Sketch::new();
        }

        let bounds = Self::compute_metaballs_bounds(balls, padding);
        let (dx, dy) = Self::compute_grid_spacing(bounds, nx, ny);
        let grid = Self::compute_influence_grid(balls, bounds, nx, ny, dx, dy, iso_value);
        let contours = Self::extract_contours(bounds, nx, ny, dx, dy, &grid);
        Self::contours_to_sketch(contours, metadata)
    }

    /// Compute bounding box around all metaballs with padding
    fn compute_metaballs_bounds(
        balls: &[(nalgebra::Point2<Real>, Real)],
        padding: Real,
    ) -> (Real, Real, Real, Real) {
        let mut min_x = Real::MAX;
        let mut min_y = Real::MAX;
        let mut max_x = -Real::MAX;
        let mut max_y = -Real::MAX;

        for (center, r) in balls {
            let rr = *r + padding;
            min_x = min_x.min(center.x - rr);
            max_x = max_x.max(center.x + rr);
            min_y = min_y.min(center.y - rr);
            max_y = max_y.max(center.y + rr);
        }

        (min_x, min_y, max_x, max_y)
    }

    /// Compute grid spacing for marching squares
    fn compute_grid_spacing(
        bounds: (Real, Real, Real, Real),
        nx: usize,
        ny: usize,
    ) -> (Real, Real) {
        let (min_x, min_y, max_x, max_y) = bounds;
        let dx = (max_x - min_x) / (nx.saturating_sub(1).max(1) as Real);
        let dy = (max_y - min_y) / (ny.saturating_sub(1).max(1) as Real);
        (dx, dy)
    }

    /// Fill grid with metaball influence values minus iso_value
    fn compute_influence_grid(
        balls: &[(nalgebra::Point2<Real>, Real)],
        bounds: (Real, Real, Real, Real),
        nx: usize,
        ny: usize,
        dx: Real,
        dy: Real,
        iso_value: Real,
    ) -> Vec<Real> {
        let (min_x, min_y, _, _) = bounds;
        let mut grid = vec![0.0; nx * ny];
        let index = |ix: usize, iy: usize| -> usize { iy * nx + ix };

        for iy in 0..ny {
            let yv = min_y + (iy as Real) * dy;
            for ix in 0..nx {
                let xv = min_x + (ix as Real) * dx;
                let val = Self::scalar_field(balls, xv, yv) - iso_value;
                grid[index(ix, iy)] = val;
            }
        }

        grid
    }

    /// Compute metaball influence at a point using 2D metaball formula
    /// **Mathematical Foundation**: 2D metaball influence I(p) = r²/(|p-c|² + ε)
    /// **Optimization**: Early termination for distant points to avoid unnecessary computation
    fn scalar_field(balls: &[(nalgebra::Point2<Real>, Real)], x: Real, y: Real) -> Real {
        balls
            .iter()
            .map(|(center, radius)| {
                let dx = x - center.x;
                let dy = y - center.y;
                let distance_sq = dx * dx + dy * dy;

                // Early termination for very distant points
                let threshold_distance_sq = radius * radius * 1000.0;
                if distance_sq > threshold_distance_sq {
                    0.0
                } else {
                    let denominator = distance_sq + crate::float_types::EPSILON;
                    (radius * radius) / denominator
                }
            })
            .sum()
    }

    /// Extract contours using marching squares algorithm
    fn extract_contours(
        bounds: (Real, Real, Real, Real),
        nx: usize,
        ny: usize,
        dx: Real,
        dy: Real,
        grid: &[Real],
    ) -> Vec<LineString<Real>> {
        let (min_x, min_y, _, _) = bounds;
        let mut contours = Vec::<LineString<Real>>::new();
        let index = |ix: usize, iy: usize| -> usize { iy * nx + ix };

        for iy in 0..(ny.saturating_sub(1)) {
            let y0 = min_y + (iy as Real) * dy;
            let y1 = min_y + ((iy + 1) as Real) * dy;

            for ix in 0..(nx.saturating_sub(1)) {
                let x0 = min_x + (ix as Real) * dx;
                let x1 = min_x + ((ix + 1) as Real) * dx;

                let v0 = grid[index(ix, iy)];
                let v1 = grid[index(ix + 1, iy)];
                let v2 = grid[index(ix + 1, iy + 1)];
                let v3 = grid[index(ix, iy + 1)];

                let cell = MarchingSquaresCell {
                    x0,
                    y0,
                    x1,
                    y1,
                    v0,
                    v1,
                    v2,
                    v3,
                };
                let pts = Self::marching_squares_cell(&cell);

                if pts.len() >= 2 {
                    let mut pl = LineString::new(vec![]);
                    for &(px, py) in &pts {
                        pl.0.push(coord! {x: px, y: py});
                    }
                    contours.push(pl);
                }
            }
        }

        contours
    }

    /// Process single marching squares cell and return intersection points
    fn marching_squares_cell(cell: &MarchingSquaresCell) -> Vec<(Real, Real)> {
        // Classify cell corners relative to iso-surface
        let mut c = 0u8;
        if cell.v0 >= 0.0 {
            c |= 1;
        }
        if cell.v1 >= 0.0 {
            c |= 2;
        }
        if cell.v2 >= 0.0 {
            c |= 4;
        }
        if cell.v3 >= 0.0 {
            c |= 8;
        }

        if c == 0 || c == 15 {
            return Vec::new(); // no crossing
        }

        let corners = [
            (cell.x0, cell.y0, cell.v0),
            (cell.x1, cell.y0, cell.v1),
            (cell.x1, cell.y1, cell.v2),
            (cell.x0, cell.y1, cell.v3),
        ];
        let mut pts = Vec::new();

        // Check each edge for crossings
        Self::check_edge(&corners, &mut pts, c, 1, 2, 0, 1);
        Self::check_edge(&corners, &mut pts, c, 2, 4, 1, 2);
        Self::check_edge(&corners, &mut pts, c, 4, 8, 2, 3);
        Self::check_edge(&corners, &mut pts, c, 8, 1, 3, 0);

        pts
    }

    /// Check single edge for iso-surface crossing and add intersection point
    fn check_edge(
        corners: &[(Real, Real, Real); 4],
        pts: &mut Vec<(Real, Real)>,
        c: u8,
        mask_a: u8,
        mask_b: u8,
        a: usize,
        b: usize,
    ) {
        let inside_a = (c & mask_a) != 0;
        let inside_b = (c & mask_b) != 0;
        if inside_a != inside_b {
            let (px, py) = Self::interpolate_edge(corners[a], corners[b]);
            pts.push((px, py));
        }
    }

    /// Interpolate edge intersection point using linear interpolation
    fn interpolate_edge(
        (x1, y1, v1): (Real, Real, Real),
        (x2, y2, v2): (Real, Real, Real),
    ) -> (Real, Real) {
        let denom = (v2 - v1).abs();
        if denom < crate::float_types::EPSILON {
            (x1, y1)
        } else {
            let t = -v1 / (v2 - v1); // crossing at 0
            (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
        }
    }

    /// Convert line contours to Sketch geometry collection
    fn contours_to_sketch(contours: Vec<LineString<Real>>, metadata: Option<S>) -> Sketch<S> {
        let stitched = stitch(&contours);
        let mut gc = GeometryCollection::default();

        for pl in stitched {
            if pl.is_closed() && pl.coords_count() >= 4 {
                let polygon = GeoPolygon::new(pl, vec![]);
                gc.0.push(Geometry::Polygon(polygon));
            }
        }

        Sketch::from_geo(gc, metadata)
    }
}

// helper – quantise to avoid FP noise
#[inline]
fn key(x: Real, y: Real) -> (i64, i64) {
    ((x * 1e8).round() as i64, (y * 1e8).round() as i64)
}

/// stitch all 2-point segments into longer polylines,
/// close them when the ends meet
fn stitch(contours: &[LineString<Real>]) -> Vec<LineString<Real>> {
    // adjacency map  endpoint -> (line index, end-id 0|1)
    let mut adj: HashMap<(i64, i64), Vec<(usize, usize)>> = HashMap::new();
    for (idx, ls) in contours.iter().enumerate() {
        let p0 = ls[0]; // first point
        let p1 = ls[1]; // second point
        adj.entry(key(p0.x, p0.y)).or_default().push((idx, 0));
        adj.entry(key(p1.x, p1.y)).or_default().push((idx, 1));
    }

    let mut used = vec![false; contours.len()];
    let mut chains = Vec::new();

    for start in 0..contours.len() {
        if used[start] {
            continue;
        }
        used[start] = true;

        // current chain of points
        let mut chain = contours[start].0.clone();

        // walk forward
        loop {
            let Some(last) = chain.last().copied() else {
                break; // Empty chain - should not happen with valid input
            };
            let Some(cands) = adj.get(&key(last.x, last.y)) else {
                break;
            };
            let mut found = None;
            for &(idx, end_id) in cands {
                if used[idx] {
                    continue;
                }
                used[idx] = true;
                // choose the *other* endpoint
                let other = contours[idx][1 - end_id];
                chain.push(other);
                found = Some(());
                break;
            }
            if found.is_none() {
                break;
            }
        }

        // close if ends coincide
        if chain.len() >= 3 {
            if let (Some(first), Some(last)) = (chain.first(), chain.last()) {
                if first != last {
                    chain.push(*first);
                }
            }
        }
        chains.push(LineString::new(chain));
    }
    chains
}

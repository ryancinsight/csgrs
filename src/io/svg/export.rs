//! SVG export functionality
//!
//! This module provides functionality for converting Sketch objects
//! back into SVG document format.

use crate::sketch::Sketch;
use geo::Geometry::*;
use svg::node::element;
use svg::node::element::path;

/// Convert Sketch to SVG string representation
pub fn sketch_to_svg<S>(sketch: &Sketch<S>) -> String {
    let mut g = element::Group::new();

    let make_line_string = |line_string: &geo::LineString<f64>| {
        let mut data = path::Data::new();
        let mut points = line_string.coords();

        if let Some(start) = points.next() {
            data = data.move_to(start.x_y());
        }
        for point in points {
            data = data.line_to(point.x_y());
        }

        element::Path::new()
            .set("fill", "none")
            .set("stroke", "black")
            .set("stroke-width", 1)
            .set("vector-effect", "non-scaling-stroke")
            .set("d", data)
    };

    let make_polygon = |polygon: &geo::Polygon<f64>| {
        let mut data = path::Data::new();

        // `svg::Data` accepts a `Vec<f32>` here, so always cast to `f32`.
        let exterior = polygon.exterior();
        data = data.move_to(
            // Skip the last point because it is equal to the first one
            exterior.0[..(exterior.0.len() - 1)]
                .iter()
                .flat_map(|c| [c.x as f32, c.y as f32])
                .collect::<Vec<f32>>(),
        );

        data = data.close();

        for interior in polygon.interiors() {
            data = data.move_to(
                interior.0[..(interior.0.len() - 1)]
                    .iter()
                    .flat_map(|c| [c.x as f32, c.y as f32])
                    .collect::<Vec<f32>>(),
            );
            data = data.close();
        }

        element::Path::new()
            .set("fill", "black")
            .set("fill-rule", "evenodd")
            .set("stroke", "none")
            .set("d", data)
    };

    for geom in &sketch.geometry.0 {
        match geom {
            GeometryCollection(gc) => {
                // Recursively process geometry collections
                for sub_geom in &gc.0 {
                    match sub_geom {
                        Polygon(p) => {
                            g = g.add(make_polygon(p));
                        },
                        LineString(ls) => {
                            g = g.add(make_line_string(ls));
                        },
                        MultiPolygon(mp) => {
                            for p in &mp.0 {
                                g = g.add(make_polygon(p));
                            }
                        },
                        MultiLineString(mls) => {
                            for ls in &mls.0 {
                                g = g.add(make_line_string(ls));
                            }
                        },
                        _ => {
                            // Skip unsupported geometry types for now
                        },
                    }
                }
            },
            Polygon(p) => {
                g = g.add(make_polygon(p));
            },
            LineString(ls) => {
                g = g.add(make_line_string(ls));
            },
            MultiPolygon(mp) => {
                for p in &mp.0 {
                    g = g.add(make_polygon(p));
                }
            },
            MultiLineString(mls) => {
                for ls in &mls.0 {
                    g = g.add(make_line_string(ls));
                }
            },
            _ => {
                // Skip unsupported geometry types for now
            },
        }
    }

    let document = element::SVG::new()
        .set("viewBox", "0 0 100 100")
        .set("xmlns", "http://www.w3.org/2000/svg")
        .add(g);

    document.to_string()
}

#[allow(unused)]
pub trait ToSVG {
    fn to_svg(&self) -> String;
}

impl<S: Clone> ToSVG for Sketch<S> {
    fn to_svg(&self) -> String {
        sketch_to_svg(self)
    }
}

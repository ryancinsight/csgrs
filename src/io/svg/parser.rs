//! SVG parsing implementation
//!
//! This module provides the core functionality for parsing SVG documents
//! into geometric Sketch representations.

use crate::float_types::Real;
use crate::io::IoError;
use crate::sketch::Sketch;
use crate::traits::CSG;
    use crate::io::svg::{parse_svg_transform, svg_path_to_multi_line_string, svg_points_to_line_string};
use nalgebra::Matrix3;
use std::collections::HashMap;

/// Context for SVG transformations and styles
#[derive(Clone, Debug)]
pub struct TransformContext {
    /// Current transformation matrix
    pub transform: Matrix3<Real>,
    /// Style attributes
    pub styles: HashMap<String, String>,
}

/// Parse SVG document into Sketch
pub fn parse_svg_to_sketch(doc: &str) -> Result<Sketch<()>, IoError> {
    use svg::node::element::tag::{self, Type::*};
    use svg::parser::Event;

    macro_rules! expect_attr {
        ($attrs:expr, $attr:literal) => {
            $attrs.get($attr).ok_or_else(|| {
                IoError::MalformedInput(format!("Missing attribute {}", $attr))
            })
        };
    }


    let mut sketch_union = Sketch::<()>::new();

    // Transform stack for nested groups
    let mut transform_stack = vec![TransformContext {
        transform: Matrix3::identity(),
        styles: HashMap::new(),
    }];

    for event in svg::read(doc)? {
        match event {
            Event::Instruction(..)
            | Event::Declaration(..)
            | Event::Text(..)
            | Event::Comment(..)
            | Event::Tag(tag::SVG, ..)
            | Event::Tag(tag::Description, ..)
            | Event::Tag(tag::Text, ..)
            | Event::Tag(tag::Title, ..) => {},

            Event::Error(error) => {
                return Err(error.into());
            },

            Event::Tag(tag::Group, Start, attrs) => {
                // Get current transform context
                let current_context = transform_stack.last().expect("Transform stack should always have at least one element").clone();

                // Parse transform attribute if present
                let mut group_transform = Matrix3::identity();
                if let Some(transform_attr) = attrs.get("transform") {
                    match parse_svg_transform(transform_attr) {
                        Ok(transform_matrix) => {
                            group_transform = transform_matrix;
                        },
                        Err(e) => {
                            eprintln!(
                                "Warning: Failed to parse SVG transform '{}': {}",
                                transform_attr, e
                            );
                        },
                    }
                }

                // Parse style attributes
                let style_metadata = HashMap::new();

                // Push new context
                let new_context = TransformContext {
                    transform: current_context.transform * group_transform,
                    styles: style_metadata,
                };
                transform_stack.push(new_context);
            },

            Event::Tag(tag::Group, End, _) => {
                // Pop transform context
                if transform_stack.len() > 1 {
                    transform_stack.pop();
                }
            },

            Event::Tag(tag::Path, Empty, attrs) => {
                let data = expect_attr!(attrs, "d")?;
                let data = svg::node::element::path::Data::parse(data)?;
                let mls = svg_path_to_multi_line_string(data)?;

                // Convert MultiLineString to Sketch
                let mut sketch = Sketch::from_geo(mls.into(), None);

                // Apply current transform
                let current_context = transform_stack.last().expect("Transform stack should always have at least one element");
                if current_context.transform != Matrix3::identity() {
                    let mut transform_4x4 = nalgebra::Matrix4::identity();
                    transform_4x4
                        .fixed_view_mut::<3, 3>(0, 0)
                        .copy_from(&current_context.transform);
                    sketch = sketch.transform(&transform_4x4);
                }

                sketch_union = sketch_union.union(&sketch);
            },

            Event::Tag(tag::Circle, Empty, attrs) => {
                let cx: Real = expect_attr!(attrs, "cx")?.parse()?;
                let cy: Real = expect_attr!(attrs, "cy")?.parse()?;
                let r: Real = expect_attr!(attrs, "r")?.parse()?;

                let segments = (r.ceil() as usize).max(6);
                let mut sketch = Sketch::circle(r, segments, None).translate(cx, cy, 0.0);

                // Apply current transform
                let current_context = transform_stack.last().expect("Transform stack should always have at least one element");
                if current_context.transform != Matrix3::identity() {
                    let mut transform_4x4 = nalgebra::Matrix4::identity();
                    transform_4x4
                        .fixed_view_mut::<3, 3>(0, 0)
                        .copy_from(&current_context.transform);
                    sketch = sketch.transform(&transform_4x4);
                }

                sketch_union = sketch_union.union(&sketch);
            },

            Event::Tag(tag::Ellipse, Empty, attrs) => {
                let cx: Real = expect_attr!(attrs, "cx")?.parse()?;
                let cy: Real = expect_attr!(attrs, "cy")?.parse()?;
                let rx: Real = expect_attr!(attrs, "rx")?.parse()?;
                let ry: Real = expect_attr!(attrs, "ry")?.parse()?;

                let segments = ((rx + ry) / 2.0).ceil() as usize;
                let mut sketch = Sketch::ellipse(rx * 2.0, ry * 2.0, segments, None)
                    .translate(cx, cy, 0.0);

                // Apply current transform
                let current_context = transform_stack.last().expect("Transform stack should always have at least one element");
                if current_context.transform != Matrix3::identity() {
                    let mut transform_4x4 = nalgebra::Matrix4::identity();
                    transform_4x4
                        .fixed_view_mut::<3, 3>(0, 0)
                        .copy_from(&current_context.transform);
                    sketch = sketch.transform(&transform_4x4);
                }

                sketch_union = sketch_union.union(&sketch);
            },


            Event::Tag(tag::Polygon, Empty, attrs) => {
                let points = expect_attr!(attrs, "points")?;
                let line_string = svg_points_to_line_string(points)?;
                let mut sketch = Sketch::from_geo(line_string.into(), None);

                // Apply current transform
                let current_context = transform_stack.last().expect("Transform stack should always have at least one element");
                if current_context.transform != Matrix3::identity() {
                    let mut transform_4x4 = nalgebra::Matrix4::identity();
                    transform_4x4
                        .fixed_view_mut::<3, 3>(0, 0)
                        .copy_from(&current_context.transform);
                    sketch = sketch.transform(&transform_4x4);
                }

                sketch_union = sketch_union.union(&sketch);
            },

            Event::Tag(tag::Polyline, Empty, attrs) => {
                let points = expect_attr!(attrs, "points")?;
                let line_string = svg_points_to_line_string(points)?;
                let mut sketch = Sketch::from_geo(line_string.into(), None);

                // Apply current transform
                let current_context = transform_stack.last().expect("Transform stack should always have at least one element");
                if current_context.transform != Matrix3::identity() {
                    let mut transform_4x4 = nalgebra::Matrix4::identity();
                    transform_4x4
                        .fixed_view_mut::<3, 3>(0, 0)
                        .copy_from(&current_context.transform);
                    sketch = sketch.transform(&transform_4x4);
                }

                sketch_union = sketch_union.union(&sketch);
            },

            _ => {
                // Non-empty tags (with content) are not currently supported
                // Future enhancement: Parse nested SVG elements like <g>, <defs>, etc.
                // This would require a more sophisticated parser that can handle element hierarchies
            },
        }
    }

    Ok(sketch_union)
}

#[allow(unused)]
pub trait FromSVG: Sized {
    fn from_svg(doc: &str) -> Result<Self, IoError>;
}

impl FromSVG for Sketch<()> {
    fn from_svg(doc: &str) -> Result<Self, IoError> {
        parse_svg_to_sketch(doc)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_svg_io() {
        use crate::io::svg::export::ToSVG;
        let svg_in = r#"
<svg viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg">
<g>
<path d="M0,0 L100,0 L100,100" fill="none" stroke="black" stroke-width="1" vector-effect="non-scaling-stroke"/>
</g>
</svg>
        "#;

        let sketch = Sketch::from_svg(svg_in)
            .expect("Failed to parse basic SVG - this indicates a bug in SVG parsing");
        let svg_out = sketch.to_svg();

        assert_eq!(svg_in.trim(), svg_out.trim());
    }
}

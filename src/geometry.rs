//! Analytic geometry in the native ASM coordinate unit. Unsupported geometry
//! returns an explicit error; all original tokens remain available on Entity.
use crate::spline::{GeometryContext, Spline};
use crate::{
    Error, Result,
    asm::Entity,
    math::{Vec3, add, cross, dot, norm, scale, sub, unit},
};
use serde::Serialize;

/// The geometry operation that could not decode an ASM entity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GeometryKind {
    Curve,
    Surface,
}

fn invalid(entity: &Entity) -> Error {
    Error::MalformedGeometry {
        entity_index: entity.index,
        source_kind: entity.name().to_owned(),
        reason: "incomplete or degenerate geometry".into(),
    }
}
fn unpack(e: &Entity) -> (Vec<Vec3>, Vec<Vec3>, Vec<f64>) {
    (
        e.positions().collect(),
        e.directions().collect(),
        e.reals().collect(),
    )
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Curve {
    Spline(Spline),
    Straight {
        root: Vec3,
        direction: Vec3,
    },
    Ellipse {
        centre: Vec3,
        normal: Vec3,
        major: Vec3,
        ratio: f64,
    },
}
impl Curve {
    pub fn from_context<'a>(e: &'a Entity, context: &GeometryContext<'a>) -> Result<Self> {
        if e.name() == "intcurve" {
            return spline_payload(context.payload(e)?, false).map(Self::Spline);
        }
        Self::from_entity(e)
    }
    pub fn from_entity(e: &Entity) -> Result<Self> {
        if e.name() == "intcurve" {
            let index = e
                .tokens
                .iter()
                .position(|t| t.tag == 15)
                .ok_or_else(|| invalid(e))?;
            return spline_payload(&e.tokens[index + 1..], false).map(Self::Spline);
        }
        let (p, d, r) = unpack(e);
        let kind = e.name();
        let getp = |i: usize| p.get(i).copied().ok_or_else(|| invalid(e));
        let getd = |i: usize| d.get(i).copied().ok_or_else(|| invalid(e));
        let getr = |i: usize| r.get(i).copied().ok_or_else(|| invalid(e));
        let normalize = |v| unit(v).map_err(|_| invalid(e));
        match kind {
            "straight" => Ok(Self::Straight {
                root: getp(0)?,
                direction: normalize(getd(0)?)?,
            }),
            "ellipse" => {
                let major = getd(1)?;
                let ratio = getr(0)?;
                if norm(major) == 0. || !ratio.is_finite() || ratio <= 0. {
                    return Err(invalid(e));
                }
                Ok(Self::Ellipse {
                    centre: getp(0)?,
                    normal: normalize(getd(0)?)?,
                    major,
                    ratio,
                })
            }
            _ => Err(Error::UnsupportedGeometry {
                geometry: GeometryKind::Curve,
                entity_index: e.index,
                source_kind: kind.to_owned(),
            }),
        }
    }
    pub fn point_at(&self, t: f64) -> Vec3 {
        match *self {
            Self::Spline(ref s) => s.point(t, 0.),
            Self::Straight { root, direction } => add(root, scale(direction, t)),
            Self::Ellipse {
                centre,
                normal,
                major,
                ratio,
            } => add(
                centre,
                add(
                    scale(major, t.cos()),
                    scale(cross(normal, major), ratio * t.sin()),
                ),
            ),
        }
    }
    pub fn invert(&self, point: Vec3) -> f64 {
        match *self {
            Self::Spline(ref s) => s.invert(point)[0],
            Self::Straight { root, direction } => dot(sub(point, root), direction),
            Self::Ellipse {
                centre,
                normal,
                major,
                ratio,
            } => {
                let delta = sub(point, centre);
                let minor = scale(cross(normal, major), ratio);
                (dot(delta, minor) / dot(minor, minor)).atan2(dot(delta, major) / dot(major, major))
            }
        }
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Surface {
    Spline(Spline),
    Plane {
        origin: Vec3,
        normal: Vec3,
        u_dir: Vec3,
    },
    Cone {
        base: Vec3,
        axis: Vec3,
        major: Vec3,
        ratio: f64,
        sin_angle: f64,
        cos_angle: f64,
    },
    Sphere {
        centre: Vec3,
        radius: f64,
        pole: Vec3,
        u_ref: Vec3,
    },
    Torus {
        centre: Vec3,
        axis: Vec3,
        major_radius: f64,
        minor_radius: f64,
        u_ref: Vec3,
    },
}
impl Surface {
    pub fn from_context<'a>(e: &'a Entity, context: &GeometryContext<'a>) -> Result<Self> {
        if e.name() == "spline" {
            return spline_payload(context.payload(e)?, true).map(Self::Spline);
        }
        Self::from_entity(e)
    }
    pub fn from_entity(e: &Entity) -> Result<Self> {
        if e.name() == "spline" {
            let index = e
                .tokens
                .iter()
                .position(|t| t.tag == 15)
                .ok_or_else(|| invalid(e))?;
            return spline_payload(&e.tokens[index + 1..], true).map(Self::Spline);
        }
        let (p, d, r) = unpack(e);
        let kind = e.name();
        let getp = |i: usize| p.get(i).copied().ok_or_else(|| invalid(e));
        let getd = |i: usize| d.get(i).copied().ok_or_else(|| invalid(e));
        let getr = |i: usize| r.get(i).copied().ok_or_else(|| invalid(e));
        let normalize = |v| unit(v).map_err(|_| invalid(e));
        match kind {
            "plane" => Ok(Self::Plane {
                origin: getp(0)?,
                normal: normalize(getd(0)?)?,
                u_dir: normalize(getd(1)?)?,
            }),
            "cone" => {
                let major = getd(1)?;
                let cos_angle = getr(2)?;
                if norm(major) == 0. || cos_angle == 0. {
                    return Err(invalid(e));
                }
                Ok(Self::Cone {
                    base: getp(0)?,
                    axis: normalize(getd(0)?)?,
                    major,
                    ratio: getr(0)?,
                    sin_angle: getr(1)?,
                    cos_angle,
                })
            }
            "sphere" => Ok(Self::Sphere {
                centre: getp(0)?,
                radius: getr(0)?,
                pole: normalize(getd(0)?)?,
                u_ref: normalize(getd(1)?)?,
            }),
            "torus" => Ok(Self::Torus {
                centre: getp(0)?,
                axis: normalize(getd(0)?)?,
                major_radius: getr(0)?,
                minor_radius: getr(1)?,
                u_ref: normalize(getd(1)?)?,
            }),
            _ => Err(Error::UnsupportedGeometry {
                geometry: GeometryKind::Surface,
                entity_index: e.index,
                source_kind: kind.to_owned(),
            }),
        }
    }
    pub fn point_at(&self, u: f64, v: f64) -> Vec3 {
        match *self {
            Self::Spline(ref s) => s.point(u, v),
            Self::Plane {
                origin,
                normal,
                u_dir,
            } => add(origin, add(scale(u_dir, u), scale(cross(normal, u_dir), v))),
            Self::Cone {
                base,
                axis,
                major,
                ratio,
                sin_angle,
                cos_angle,
            } => {
                let radial = add(
                    scale(major, u.cos()),
                    scale(cross(axis, major), ratio * u.sin()),
                );
                add(
                    add(base, scale(axis, v)),
                    scale(radial, 1. + v * sin_angle / cos_angle / norm(major)),
                )
            }
            Self::Sphere {
                centre,
                radius,
                pole,
                u_ref,
            } => {
                let radial = add(scale(u_ref, u.cos()), scale(cross(pole, u_ref), u.sin()));
                add(
                    centre,
                    scale(add(scale(radial, v.cos()), scale(pole, v.sin())), radius),
                )
            }
            Self::Torus {
                centre,
                axis,
                major_radius,
                minor_radius,
                u_ref,
            } => {
                let radial = add(scale(u_ref, u.cos()), scale(cross(axis, u_ref), u.sin()));
                add(
                    centre,
                    add(
                        scale(radial, major_radius + minor_radius * v.cos()),
                        scale(axis, minor_radius * v.sin()),
                    ),
                )
            }
        }
    }
}

fn spline_payload(tokens: &[crate::asm::Token], surface: bool) -> Result<Spline> {
    use crate::asm::Value;
    let mut depth = 0;
    for (i, t) in tokens.iter().enumerate() {
        if t.tag == 15 {
            depth += 1;
        } else if t.tag == 16 {
            if depth == 0 {
                break;
            }
            depth -= 1;
        }
        if depth == 0 && matches!(&t.value,Value::Text(s) if s=="nubs" || s=="nurbs") {
            return Spline::parse(&tokens[i..], surface);
        }
    }
    Err(Error::Invalid {
        offset: 0,
        reason: "explicit spline payload unavailable".into(),
    })
}

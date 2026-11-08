//! Analytic geometry in the native ASM coordinate unit. Unsupported geometry
//! returns an explicit error; all original tokens remain available on Entity.
use crate::{
    Error, Result,
    asm::Entity,
    math::{Vec3, add, cross, dot, norm, scale, sub, unit},
};
use serde::Serialize;

fn invalid(kind: &str) -> Error {
    Error::Invalid {
        offset: 0,
        reason: format!("incomplete or degenerate {kind} geometry"),
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
    pub fn from_entity(e: &Entity) -> Result<Self> {
        let (p, d, r) = unpack(e);
        let kind = e.name();
        let getp = |i: usize| p.get(i).copied().ok_or_else(|| invalid(kind));
        let getd = |i: usize| d.get(i).copied().ok_or_else(|| invalid(kind));
        let getr = |i: usize| r.get(i).copied().ok_or_else(|| invalid(kind));
        match kind {
            "straight" => Ok(Self::Straight {
                root: getp(0)?,
                direction: unit(getd(0)?)?,
            }),
            "ellipse" => {
                let major = getd(1)?;
                let ratio = getr(0)?;
                if norm(major) == 0. || !ratio.is_finite() || ratio <= 0. {
                    return Err(invalid(kind));
                }
                Ok(Self::Ellipse {
                    centre: getp(0)?,
                    normal: unit(getd(0)?)?,
                    major,
                    ratio,
                })
            }
            _ => Err(Error::Unsupported(format!(
                "curve {kind:?}; inspect its retained tokens"
            ))),
        }
    }
    pub fn point_at(&self, t: f64) -> Vec3 {
        match *self {
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
    pub fn from_entity(e: &Entity) -> Result<Self> {
        let (p, d, r) = unpack(e);
        let kind = e.name();
        let getp = |i: usize| p.get(i).copied().ok_or_else(|| invalid(kind));
        let getd = |i: usize| d.get(i).copied().ok_or_else(|| invalid(kind));
        let getr = |i: usize| r.get(i).copied().ok_or_else(|| invalid(kind));
        match kind {
            "plane" => Ok(Self::Plane {
                origin: getp(0)?,
                normal: unit(getd(0)?)?,
                u_dir: unit(getd(1)?)?,
            }),
            "cone" => {
                let major = getd(1)?;
                let cos_angle = getr(2)?;
                if norm(major) == 0. || cos_angle == 0. {
                    return Err(invalid(kind));
                }
                Ok(Self::Cone {
                    base: getp(0)?,
                    axis: unit(getd(0)?)?,
                    major,
                    ratio: getr(0)?,
                    sin_angle: getr(1)?,
                    cos_angle,
                })
            }
            "sphere" => Ok(Self::Sphere {
                centre: getp(0)?,
                radius: getr(0)?,
                pole: unit(getd(0)?)?,
                u_ref: unit(getd(1)?)?,
            }),
            "torus" => Ok(Self::Torus {
                centre: getp(0)?,
                axis: unit(getd(0)?)?,
                major_radius: getr(0)?,
                minor_radius: getr(1)?,
                u_ref: unit(getd(1)?)?,
            }),
            _ => Err(Error::Unsupported(format!(
                "surface {kind:?}; inspect its retained tokens"
            ))),
        }
    }
    pub fn point_at(&self, u: f64, v: f64) -> Vec3 {
        match *self {
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

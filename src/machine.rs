//! Fusion machine-definition JSON and forward kinematics. Unknown fields are
//! preserved in `raw`; this is not a controller or collision simulator.
use crate::{
    Error, Result,
    math::{Mat4, Vec3, add, cross, dot, scale, unit},
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RotationAxis {
    pub direction: Vec3,
    pub point: Vec3,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Attachment {
    pub point: Vec3,
    pub x_direction: Vec3,
    pub z_direction: Vec3,
}
impl Attachment {
    pub fn matrix(&self) -> Result<Mat4> {
        let x = unit(self.x_direction)?;
        let z = unit(self.z_direction)?;
        if dot(x, z).abs() > 1e-9 {
            return Err(Error::Invalid {
                offset: 0,
                reason: "attachment axes are not orthogonal".into(),
            });
        }
        let y = cross(z, x);
        let mut matrix = Mat4::IDENTITY;
        for i in 0..3 {
            matrix.0[i] = [x[i], y[i], z[i], self.point[i]];
        }
        if !matrix.is_rigid(1e-9) {
            return Err(Error::Invalid {
                offset: 0,
                reason: "invalid attachment frame".into(),
            });
        }
        Ok(matrix)
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Part {
    pub id: String,
    #[serde(rename = "type")]
    pub kind: String,
    pub name: Option<String>,
    pub display_name: Option<String>,
    pub direction: Option<Vec3>,
    pub axis_of_rotation: Option<RotationAxis>,
    pub offset: Option<Vec3>,
    pub attach_frame: Option<Attachment>,
    pub min: Option<f64>,
    pub max: Option<f64>,
    #[serde(default)]
    pub parts: Vec<Part>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Kinematics {
    pub units: BTreeMap<String, String>,
    pub parts: Vec<Part>,
}
#[derive(Debug, Clone, Serialize)]
pub struct MachineDefinition {
    pub raw: serde_json::Value,
    pub kinematics: Kinematics,
}

impl MachineDefinition {
    pub fn parse(data: &[u8]) -> Result<Self> {
        let raw: serde_json::Value = serde_json::from_slice(data)?;
        let kin = raw
            .pointer("/kinematics/default")
            .ok_or_else(|| Error::Unsupported("no default machine kinematics".into()))?;
        let kinematics: Kinematics = serde_json::from_value(kin.clone())?;
        let model = Self { raw, kinematics };
        let mut ids = BTreeSet::new();
        let mut aliases = BTreeMap::new();
        for part in model.parts() {
            if !ids.insert(part.id.as_str()) {
                return Err(Error::Invalid {
                    offset: 0,
                    reason: format!("duplicate machine part {:?}", part.id),
                });
            }
            if part.is_axis() {
                for alias in [Some(part.id.as_str()), part.name.as_deref()]
                    .into_iter()
                    .flatten()
                {
                    if let Some(previous) = aliases.insert(alias, part.id.as_str())
                        && previous != part.id
                    {
                        return Err(Error::Invalid {
                            offset: 0,
                            reason: format!("ambiguous axis alias {alias:?}"),
                        });
                    }
                }
            }
        }
        Ok(model)
    }
    pub fn name(&self) -> Option<&str> {
        self.raw
            .pointer("/general/model")
            .and_then(serde_json::Value::as_str)
    }
    pub fn parts(&self) -> Vec<&Part> {
        fn visit<'a>(part: &'a Part, out: &mut Vec<&'a Part>) {
            out.push(part);
            for child in &part.parts {
                visit(child, out);
            }
        }
        let mut out = Vec::new();
        for part in &self.kinematics.parts {
            visit(part, &mut out);
        }
        out
    }
    /// Node world transforms, in the machine definition's native length unit.
    /// Inputs accept each axis's ID or command name (e.g. `rotary_0` or `A`).
    /// Missing coordinates are zero; unknown keys, invalid ranges and NaNs fail.
    pub fn pose(&self, coordinates: &BTreeMap<String, f64>) -> Result<BTreeMap<String, Mat4>> {
        let parts = self.parts();
        for key in coordinates.keys() {
            if !parts
                .iter()
                .any(|p| p.is_axis() && (&p.id == key || p.name.as_ref() == Some(key)))
            {
                return Err(Error::Invalid {
                    offset: 0,
                    reason: format!("unknown axis {key:?}"),
                });
            }
        }
        let angle_scale = match self.kinematics.units.get("angle").map(String::as_str) {
            Some("degrees") => std::f64::consts::PI / 180.,
            Some("radians") => 1.,
            other => return Err(Error::Unsupported(format!("machine angle unit {other:?}"))),
        };
        fn visit(
            part: &Part,
            parent: Mat4,
            c: &BTreeMap<String, f64>,
            angle_scale: f64,
            out: &mut BTreeMap<String, Mat4>,
        ) -> Result<()> {
            let by_id = c.get(&part.id);
            let by_name = part.name.as_ref().and_then(|n| c.get(n));
            if let (Some(a), Some(b)) = (by_id, by_name)
                && a != b
            {
                return Err(Error::Invalid {
                    offset: 0,
                    reason: format!("conflicting values for axis {:?}", part.id),
                });
            }
            let q = *by_id.or(by_name).unwrap_or(&0.);
            if part.offset.is_some_and(|v| v != [0.; 3]) {
                return Err(Error::Unsupported(format!(
                    "nonzero machine-part offset on {:?}",
                    part.id
                )));
            }
            if !q.is_finite() || part.min.is_some_and(|v| q < v) || part.max.is_some_and(|v| q > v)
            {
                return Err(Error::Invalid {
                    offset: 0,
                    reason: format!("axis {:?} coordinate {q} is out of range", part.id),
                });
            }
            let local = match part.kind.as_str() {
                "linear" => {
                    let d = part.direction.ok_or_else(|| Error::Invalid {
                        offset: 0,
                        reason: "missing linear direction".into(),
                    })?;
                    Mat4::translation(scale(unit(d)?, q))
                }
                "rotary" => {
                    let a = part
                        .axis_of_rotation
                        .as_ref()
                        .ok_or_else(|| Error::Invalid {
                            offset: 0,
                            reason: "missing rotary axis".into(),
                        })?;
                    Mat4::rotate_about(a.direction, a.point, q * angle_scale)?
                }
                "static" | "head" | "table" => Mat4::IDENTITY,
                kind => return Err(Error::Unsupported(format!("machine part kind {kind:?}"))),
            };
            let world = parent.compose(local);
            out.insert(part.id.clone(), world);
            for child in &part.parts {
                visit(child, world, c, angle_scale, out)?;
            }
            Ok(())
        }
        let mut out = BTreeMap::new();
        for p in &self.kinematics.parts {
            visit(p, Mat4::IDENTITY, coordinates, angle_scale, &mut out)?;
        }
        Ok(out)
    }
    pub fn attachment(&self, id: &str, pose: &BTreeMap<String, Mat4>) -> Result<Mat4> {
        let parts = self.parts();
        let part = parts
            .iter()
            .find(|p| p.id == id)
            .ok_or_else(|| Error::Invalid {
                offset: 0,
                reason: format!("unknown attachment part {id:?}"),
            })?;
        let frame = part
            .attach_frame
            .as_ref()
            .ok_or_else(|| Error::Unsupported(format!("part {id:?} has no attachment frame")))?;
        let world = pose.get(id).ok_or_else(|| Error::Invalid {
            offset: 0,
            reason: format!("pose omits {id:?}"),
        })?;
        Ok(world.compose(frame.matrix()?))
    }
    /// Relative head/spindle attachment, with no cutting-tool length applied.
    pub fn spindle_in_table(&self, coordinates: &BTreeMap<String, f64>) -> Result<Vec3> {
        let head = self
            .raw
            .pointer("/controller/default/head_info_part_id")
            .and_then(|v| v.as_str())
            .ok_or_else(|| Error::Unsupported("head part ID missing".into()))?;
        let table = self
            .raw
            .pointer("/controller/default/table_part_id")
            .and_then(|v| v.as_str())
            .ok_or_else(|| Error::Unsupported("table part ID missing".into()))?;
        let pose = self.pose(coordinates)?;
        let relative = self
            .attachment(table, &pose)?
            .inverse_rigid()?
            .compose(self.attachment(head, &pose)?);
        Ok(add([0.; 3], relative.transform_point([0.; 3])))
    }
}
impl Part {
    pub fn is_axis(&self) -> bool {
        matches!(self.kind.as_str(), "linear" | "rotary")
    }
}

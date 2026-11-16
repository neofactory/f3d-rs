//! Row-major rigid transforms. No implicit unit conversion.
use crate::{Error, Result};
use serde::{Deserialize, Serialize};
pub type Vec3 = [f64; 3];
pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] + b[i])
}
pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] - b[i])
}
pub fn scale(a: Vec3, k: f64) -> Vec3 {
    a.map(|v| v * k)
}
pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}
pub fn unit(a: Vec3) -> Result<Vec3> {
    let n = norm(a);
    if !n.is_finite() || n <= 0. {
        return Err(Error::Invalid {
            offset: 0,
            reason: "zero or nonfinite direction".into(),
        });
    }
    Ok(scale(a, 1. / n))
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Mat4(pub [[f64; 4]; 4]);
impl Default for Mat4 {
    fn default() -> Self {
        Self::IDENTITY
    }
}
impl Mat4 {
    pub const IDENTITY: Self = Self([
        [1., 0., 0., 0.],
        [0., 1., 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ]);
    pub fn translation(v: Vec3) -> Self {
        let mut m = Self::IDENTITY;
        for (i, x) in v.into_iter().enumerate() {
            m.0[i][3] = x;
        }
        m
    }
    pub fn compose(self, other: Self) -> Self {
        Self(std::array::from_fn(|i| {
            std::array::from_fn(|j| (0..4).map(|k| self.0[i][k] * other.0[k][j]).sum())
        }))
    }
    pub fn transform_point(self, p: Vec3) -> Vec3 {
        std::array::from_fn(|i| self.0[i][3] + (0..3).map(|j| self.0[i][j] * p[j]).sum::<f64>())
    }
    pub fn rotate_about(axis: Vec3, pivot: Vec3, angle: f64) -> Result<Self> {
        if !angle.is_finite() || !pivot.iter().all(|v| v.is_finite()) {
            return Err(Error::Invalid {
                offset: 0,
                reason: "nonfinite rotary transform".into(),
            });
        }
        let [x, y, z] = unit(axis)?;
        let (s, c) = angle.sin_cos();
        let d = 1. - c;
        let rotation = Self([
            [c + x * x * d, x * y * d - z * s, x * z * d + y * s, 0.],
            [y * x * d + z * s, c + y * y * d, y * z * d - x * s, 0.],
            [z * x * d - y * s, z * y * d + x * s, c + z * z * d, 0.],
            [0., 0., 0., 1.],
        ]);
        Ok(Self::translation(pivot)
            .compose(rotation)
            .compose(Self::translation(scale(pivot, -1.))))
    }
    pub fn is_rigid(self, tolerance: f64) -> bool {
        if !self.0.iter().flatten().all(|x| x.is_finite()) {
            return false;
        }
        for (value, expected) in self.0[3].iter().zip(Self::IDENTITY.0[3]) {
            if (value - expected).abs() > tolerance {
                return false;
            }
        }
        let cols: [Vec3; 3] = std::array::from_fn(|j| std::array::from_fn(|i| self.0[i][j]));
        for i in 0..3 {
            for j in 0..3 {
                if (dot(cols[i], cols[j]) - if i == j { 1. } else { 0. }).abs() > tolerance {
                    return false;
                }
            }
        }
        (dot(cols[0], cross(cols[1], cols[2])) - 1.).abs() <= tolerance
    }
    pub fn inverse_rigid(self) -> Result<Self> {
        if !self.is_rigid(1e-9) {
            return Err(Error::Invalid {
                offset: 0,
                reason: "matrix is not a proper rigid transform".into(),
            });
        }
        let mut inv = Self::IDENTITY;
        for i in 0..3 {
            for j in 0..3 {
                inv.0[i][j] = self.0[j][i];
            }
            inv.0[i][3] = -(0..3).map(|j| inv.0[i][j] * self.0[j][3]).sum::<f64>();
        }
        Ok(inv)
    }
}

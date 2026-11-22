//! ASM explicit NUBS/NURBS payloads. Knot endpoint multiplicities omit one copy
//! in ASM; controls are Euclidean XYZ followed by weight for rational payloads.
use crate::{
    Error, Result,
    asm::{Entity, Model, Token, Value},
    math::{Vec3, norm, sub},
};
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct Spline {
    pub degrees: [usize; 2],
    pub knots: [Vec<f64>; 2],
    pub counts: [usize; 2],
    pub controls: Vec<[f64; 4]>,
    pub closed: bool,
}
fn bad() -> Error {
    Error::Invalid {
        offset: 0,
        reason: "malformed or unsupported ASM spline payload".into(),
    }
}
struct Reader<'a> {
    tokens: &'a [Token],
    pos: usize,
}
impl Reader<'_> {
    fn take(&mut self) -> Result<&Token> {
        let t = self.tokens.get(self.pos).ok_or_else(bad)?;
        self.pos += 1;
        Ok(t)
    }
    fn number(&mut self) -> Result<f64> {
        match self.take()?.value {
            Value::Real(n) if n.is_finite() => Ok(n),
            _ => Err(bad()),
        }
    }
    fn integer(&mut self) -> Result<usize> {
        match self.take()?.value {
            Value::Integer(n) if (0..=100000).contains(&n) => Ok(n as usize),
            _ => Err(bad()),
        }
    }
}
fn knot_vector(r: &mut Reader, count: usize, degree: usize) -> Result<Vec<f64>> {
    if !(2..=100000).contains(&count) || !(1..=16).contains(&degree) {
        return Err(bad());
    }
    let mut knots = Vec::new();
    for i in 0..count {
        let knot = r.number()?;
        let copies = r.integer()?;
        if copies == 0 || copies > degree + 1 || knots.last().is_some_and(|v| *v >= knot) {
            return Err(bad());
        }
        knots.extend(std::iter::repeat_n(
            knot,
            copies + usize::from(i == 0 || i == count - 1),
        ));
    }
    Ok(knots)
}
impl Spline {
    pub fn parse(tokens: &[Token], surface: bool) -> Result<Self> {
        let rational =
            matches!(&tokens.first().ok_or_else(bad)?.value,Value::Text(s) if s=="nurbs");
        if !matches!(&tokens[0].value,Value::Text(s) if s=="nurbs" || s=="nubs") {
            return Err(bad());
        }
        let mut r = Reader { tokens, pos: 1 };
        let du = r.integer()?;
        let dv = if surface { r.integer()? } else { 0 };
        // Surface form includes an optional rationality direction ("both").
        if surface
            && matches!(r.tokens.get(r.pos).map(|t| &t.value), Some(Value::Text(_)))
            && !matches!(&r.take()?.value,Value::Text(s) if s=="both" || s=="u" || s=="v")
        {
            return Err(bad());
        }
        let mut closed = false;
        for _ in 0..if surface { 4 } else { 1 } {
            let t = r.take()?;
            if t.tag != 21 {
                return Err(bad());
            }
            if !surface {
                closed = t.integer() == Some(2);
            }
        }
        let ku = r.integer()?;
        let kv = if surface { r.integer()? } else { 0 };
        let u = knot_vector(&mut r, ku, du)?;
        let v = if surface {
            knot_vector(&mut r, kv, dv)?
        } else {
            vec![0., 1.]
        };
        let nu = u.len().checked_sub(du + 1).ok_or_else(bad)?;
        let nv = if surface {
            v.len().checked_sub(dv + 1).ok_or_else(bad)?
        } else {
            1
        };
        let count = nu
            .checked_mul(nv)
            .filter(|n| *n <= 100000)
            .ok_or_else(bad)?;
        if nu <= du || nv <= dv {
            return Err(bad());
        }
        let mut controls = Vec::with_capacity(count);
        for _ in 0..count {
            let x = r.number()?;
            let y = r.number()?;
            let z = r.number()?;
            let w = if rational { r.number()? } else { 1. };
            if w <= 0. {
                return Err(bad());
            }
            controls.push([x * w, y * w, z * w, w]);
        }
        Ok(Self {
            degrees: [du, dv],
            knots: [u, v],
            counts: [nu, nv],
            controls,
            closed,
        })
    }
    pub fn domain(&self, axis: usize) -> [f64; 2] {
        let k = &self.knots[axis];
        [k[self.degrees[axis]], k[self.counts[axis]]]
    }
    fn basis(&self, axis: usize, t: f64) -> (usize, Vec<f64>) {
        let p = self.degrees[axis];
        let k = &self.knots[axis];
        let n = self.counts[axis];
        if p == 0 {
            return (0, vec![1.]);
        }
        let t = t.clamp(k[p], k[n]);
        let span = if t >= k[n] {
            n - 1
        } else {
            k.partition_point(|v| *v <= t)
                .saturating_sub(1)
                .clamp(p, n - 1)
        };
        let mut b = vec![0.; p + 1];
        let mut left = vec![0.; p + 1];
        let mut right = vec![0.; p + 1];
        b[0] = 1.;
        for j in 1..=p {
            left[j] = t - k[span + 1 - j];
            right[j] = k[span + j] - t;
            let mut saved = 0.;
            for r in 0..j {
                let denom = right[r + 1] + left[j - r];
                let q = if denom == 0. { 0. } else { b[r] / denom };
                b[r] = saved + right[r + 1] * q;
                saved = left[j - r] * q;
            }
            b[j] = saved;
        }
        (span - p, b)
    }
    pub fn point(&self, u: f64, v: f64) -> Vec3 {
        let [lo, hi] = self.domain(0);
        let u = if self.closed && (u < lo || u > hi) {
            lo + (u - lo).rem_euclid(hi - lo)
        } else {
            u
        };
        let (iu, bu) = self.basis(0, u);
        let (iv, bv) = self.basis(1, v);
        let mut p = [0.; 4];
        for (j, y) in bv.iter().enumerate() {
            for (i, x) in bu.iter().enumerate() {
                let c = self.controls[(iv + j) * self.counts[0] + iu + i];
                for a in 0..4 {
                    p[a] += c[a] * x * y;
                }
            }
        }
        [p[0] / p[3], p[1] / p[3], p[2] / p[3]]
    }
    pub fn invert(&self, point: Vec3) -> [f64; 2] {
        let domains = [self.domain(0), self.domain(1)];
        let samples: Vec<Vec<f64>> = (0..2)
            .map(|a| {
                if self.degrees[a] == 0 {
                    return vec![0.];
                }
                let [lo, hi] = domains[a];
                (0..=24).map(|i| lo + (hi - lo) * i as f64 / 24.).collect()
            })
            .collect();
        let mut best = [domains[0][0], domains[1][0]];
        let mut error = f64::INFINITY;
        for &u in &samples[0] {
            for &v in &samples[1] {
                let d = norm(sub(self.point(u, v), point));
                if d < error {
                    error = d;
                    best = [u, v];
                }
            }
        }
        for _ in 0..40 {
            let p = self.point(best[0], best[1]);
            let residual = sub(p, point);
            let mut derivatives = [[0.; 3]; 2];
            for a in 0..2 {
                if self.degrees[a] == 0 {
                    continue;
                }
                let h = (domains[a][1] - domains[a][0]) * 1e-5;
                let mut l = best;
                let mut r = best;
                l[a] = (l[a] - h).max(domains[a][0]);
                r[a] = (r[a] + h).min(domains[a][1]);
                let diff = sub(self.point(r[0], r[1]), self.point(l[0], l[1]));
                derivatives[a] = diff.map(|v| v / (r[a] - l[a]));
            }
            let dot = crate::math::dot;
            let a = dot(derivatives[0], derivatives[0]);
            let b = dot(derivatives[0], derivatives[1]);
            let c = dot(derivatives[1], derivatives[1]);
            let d = dot(derivatives[0], residual);
            let e = dot(derivatives[1], residual);
            let det = a * c - b * b;
            let delta = if self.degrees[1] == 0 && a > 1e-25 {
                [d / a, 0.]
            } else if det > 1e-25 {
                [(c * d - b * e) / det, (a * e - b * d) / det]
            } else {
                break;
            };
            let mut improved = false;
            for i in 0..12 {
                let scale = 0.5f64.powi(i);
                let candidate = std::array::from_fn(|a| {
                    (best[a] - scale * delta[a]).clamp(domains[a][0], domains[a][1])
                });
                let distance = norm(sub(self.point(candidate[0], candidate[1]), point));
                if distance < error {
                    best = candidate;
                    error = distance;
                    improved = true;
                    break;
                }
            }
            if !improved || error < 1e-10 {
                break;
            }
        }
        best
    }
}
/// Subtype IDs count non-reference blocks in source order, including nested blocks.
/// A `ref` block reuses a definition without reserving a new ID.
pub struct GeometryContext<'a> {
    subtypes: Vec<&'a [Token]>,
}
impl<'a> GeometryContext<'a> {
    pub fn new(model: &'a Model) -> Self {
        let mut subtypes = Vec::new();
        for e in &model.entities {
            for i in 0..e.tokens.len() {
                if e.tokens[i].tag == 15
                    && !matches!(e.tokens.get(i+1).map(|t|&t.value),Some(Value::Text(s)) if s=="ref")
                {
                    subtypes.push(&e.tokens[i + 1..]);
                }
            }
        }
        Self { subtypes }
    }
    pub fn payload(&self, e: &'a Entity) -> Result<&'a [Token]> {
        let i = e.tokens.iter().position(|t| t.tag == 15).ok_or_else(bad)?;
        let p = &e.tokens[i + 1..];
        if matches!(&p.first().ok_or_else(bad)?.value,Value::Text(s) if s=="ref") {
            let id = p
                .get(1)
                .and_then(Token::integer)
                .and_then(|i| usize::try_from(i).ok())
                .ok_or_else(bad)?;
            self.subtypes.get(id).copied().ok_or_else(bad)
        } else {
            Ok(p)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn t(tag: u8, value: Value) -> Token {
        Token { tag, value }
    }
    fn text(s: &str) -> Token {
        t(13, Value::Text(s.into()))
    }
    fn int(n: i64) -> Token {
        t(4, Value::Integer(n))
    }
    fn real(n: f64) -> Token {
        t(6, Value::Real(n))
    }
    fn en(n: i64) -> Token {
        t(21, Value::Integer(n))
    }
    fn bezier(rational: bool, points: &[[f64; 4]]) -> Vec<Token> {
        let p = points.len() - 1;
        let mut v = vec![
            text(if rational { "nurbs" } else { "nubs" }),
            int(p as i64),
            en(0),
            int(2),
            real(0.),
            int(p as i64),
            real(1.),
            int(p as i64),
        ];
        for point in points {
            v.extend(
                point[..if rational { 4 } else { 3 }]
                    .iter()
                    .map(|x| real(*x)),
            );
        }
        v
    }
    #[test]
    fn rational_quarter_circle_and_inverse() {
        let s = Spline::parse(
            &bezier(
                true,
                &[
                    [1., 0., 0., 1.],
                    [1., 1., 0., 0.5f64.sqrt()],
                    [0., 1., 0., 1.],
                ],
            ),
            false,
        )
        .unwrap();
        let p = s.point(0.5, 0.);
        assert!(norm(sub(p, [0.5f64.sqrt(), 0.5f64.sqrt(), 0.])) < 1e-12);
        assert!((s.invert(p)[0] - 0.5).abs() < 1e-9);
        assert_eq!(s.knots[0], vec![0., 0., 0., 1., 1., 1.]);
    }
    #[test]
    fn tensor_surface_control_order_and_inverse() {
        let mut tokens = vec![
            text("nubs"),
            int(1),
            int(1),
            en(0),
            en(0),
            en(0),
            en(0),
            int(2),
            int(2),
        ];
        for _ in 0..2 {
            tokens.extend([real(0.), int(1), real(1.), int(1)]);
        }
        for p in [[0., 0., 0.], [2., 0., 0.], [0., 3., 0.], [2., 3., 1.]] {
            tokens.extend(p.map(real));
        }
        let s = Spline::parse(&tokens, true).unwrap();
        let p = s.point(0.3, 0.7);
        assert!(norm(sub(p, [0.6, 2.1, 0.21])) < 1e-12);
        let uv = s.invert(p);
        assert!((uv[0] - 0.3).abs() < 1e-8 && (uv[1] - 0.7).abs() < 1e-8);
    }
    #[test]
    fn malformed_payload_rejected() {
        let mut tokens = bezier(
            true,
            &[[1., 0., 0., 1.], [1., 1., 0., -1.], [0., 1., 0., 1.]],
        );
        assert!(Spline::parse(&tokens, false).is_err());
        tokens = bezier(false, &[[0., 0., 0., 1.], [1., 1., 1., 1.]]);
        tokens.pop();
        assert!(Spline::parse(&tokens, false).is_err());
    }
    #[test]
    fn closed_curve_wraps_outside_parameter_domain() {
        let mut tokens = bezier(
            false,
            &[[0., 0., 0., 1.], [1., 1., 0., 1.], [0., 0., 0., 1.]],
        );
        tokens[2] = en(2);
        let s = Spline::parse(&tokens, false).unwrap();
        assert_eq!(s.point(0.25, 0.), s.point(1.25, 0.));
        assert_eq!(s.point(0.75, 0.), s.point(-0.25, 0.));
    }
}

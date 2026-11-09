//! Bounded display meshes from live, trimmed analytic ASM faces.
//!
//! Positions retain the input stream's kernel coordinates (normally centimetres
//! for Fusion). Archive occurrence placements and machine motion are not applied.
//! Only absent/identity ASM body transforms are accepted: their nonidentity
//! grammar is not yet verified. Unsupported faces are reported, never filled with
//! substitute solids. Meshes are for display; watertightness is not guaranteed.
//!
//! Edge sampling, surface inversion and periodic-strip handling are adapted in
//! part from ezf3d; see `THIRD-PARTY-NOTICES` and `LICENSE-MIT-ezf3d`.
use crate::{
    Error, Result,
    asm::{Body, Edge, Face, Loop, Model},
    geometry::{Curve, Surface},
    math::{Vec3, add, cross, dot, norm, scale, sub},
};
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    f64::consts::TAU,
};

type Uv = [f64; 2];
type FaceResult<T> = std::result::Result<T, String>;

#[derive(Debug, Clone)]
pub struct MeshOptions {
    /// Chord tolerance in the input kernel unit; 0.01 cm = 0.1 mm in Fusion.
    pub tolerance: f64,
    pub max_edge_segments: usize,
    pub max_face_vertices: usize,
    pub max_face_triangles: usize,
    pub max_vertices: usize,
    pub max_triangles: usize,
    pub max_refinement_depth: usize,
}
impl Default for MeshOptions {
    fn default() -> Self {
        Self {
            tolerance: 0.01,
            max_edge_segments: 1024,
            max_face_vertices: 16_384,
            max_face_triangles: 16_384,
            max_vertices: 1_000_000,
            max_triangles: 1_000_000,
            max_refinement_depth: 8,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct MeshedFace {
    /// ASM body entity index, not the archive component ID.
    pub body_index: usize,
    pub face_index: usize,
    pub triangle_start: usize,
    pub triangle_count: usize,
}
#[derive(Debug, Clone, Serialize)]
pub struct MeshIssue {
    pub body_index: usize,
    pub face_index: usize,
    pub reason: String,
}
#[derive(Debug, Default, Clone, Serialize)]
pub struct MeshReport {
    pub positions: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
    pub faces: Vec<MeshedFace>,
    pub faces_meshed: usize,
    pub faces_skipped: usize,
    pub issues: Vec<MeshIssue>,
}

/// A display mesh for one ASM body. Vertex indices and face triangle ranges
/// refer to this body's arrays, while body/face indices retain source identity.
/// Occurrence placements and machine motion have not been applied.
#[derive(Debug, Default, Clone, Serialize)]
pub struct BodyMesh {
    pub body_index: usize,
    pub positions: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
    pub faces: Vec<MeshedFace>,
    pub issues: Vec<MeshIssue>,
}

impl MeshReport {
    /// Split the report into independently drawable bodies. Shared vertices
    /// are remapped per body, and bodies containing only skipped faces remain
    /// present with their diagnostics. Coverage counters are derived from the
    /// face records rather than copied from the report's summary fields.
    pub fn split_by_body(&self) -> Result<Vec<BodyMesh>> {
        let mut bodies = BTreeMap::<usize, (BodyMesh, BTreeMap<u32, u32>)>::new();
        for face in &self.faces {
            let (body, remap) = bodies.entry(face.body_index).or_insert_with(|| {
                (
                    BodyMesh {
                        body_index: face.body_index,
                        ..Default::default()
                    },
                    BTreeMap::new(),
                )
            });
            let end = face
                .triangle_start
                .checked_add(face.triangle_count)
                .ok_or_else(|| invalid_mesh_range(face.face_index))?;
            let triangles = self
                .triangles
                .get(face.triangle_start..end)
                .ok_or_else(|| invalid_mesh_range(face.face_index))?;
            let start = body.triangles.len();
            for triangle in triangles {
                let mut local = [0; 3];
                for (corner, &index) in triangle.iter().enumerate() {
                    let point = self
                        .positions
                        .get(index as usize)
                        .ok_or_else(|| invalid_mesh_range(face.face_index))?;
                    let next = u32::try_from(body.positions.len())
                        .map_err(|_| Error::Limit("body mesh vertex indices".into()))?;
                    local[corner] = *remap.entry(index).or_insert_with(|| {
                        body.positions.push(*point);
                        next
                    });
                }
                body.triangles.push(local);
            }
            body.faces.push(MeshedFace {
                triangle_start: start,
                ..face.clone()
            });
        }
        for issue in &self.issues {
            let (body, _) = bodies.entry(issue.body_index).or_insert_with(|| {
                (
                    BodyMesh {
                        body_index: issue.body_index,
                        ..Default::default()
                    },
                    BTreeMap::new(),
                )
            });
            body.issues.push(issue.clone());
        }
        Ok(bodies.into_values().map(|(body, _)| body).collect())
    }
}

fn invalid_mesh_range(face_index: usize) -> Error {
    Error::Invalid {
        offset: 0,
        reason: format!("display mesh face {face_index} references missing triangles or vertices"),
    }
}

/// Triangulate body-reachable faces, preserving per-face/body association.
/// Invalid options and global resource exhaustion return errors; unsupported or
/// malformed individual faces are skipped with a diagnostic in the report.
pub fn tessellate(model: &Model, options: &MeshOptions) -> Result<MeshReport> {
    if !options.tolerance.is_finite()
        || options.tolerance <= 0.
        || options.max_edge_segments == 0
        || options.max_face_vertices < 3
        || options.max_vertices > u32::MAX as usize
        || options.max_refinement_depth > 12
    {
        return Err(Error::Invalid {
            offset: 0,
            reason: "invalid mesh tolerance or resource limits".into(),
        });
    }
    let mut report = MeshReport::default();
    let mut seen = BTreeSet::new();
    for body in model.bodies() {
        let body_supported = identity_transform(body);
        for face in body.faces() {
            if !seen.insert(face.index()) {
                continue;
            }
            let built = if body_supported {
                tessellate_face(face, options)
            } else {
                Err("nonidentity or unrecognized ASM body transform".into())
            };
            match built {
                Ok(mesh) => {
                    if report.positions.len().saturating_add(mesh.positions.len())
                        > options.max_vertices
                        || report.triangles.len().saturating_add(mesh.triangles.len())
                            > options.max_triangles
                    {
                        return Err(Error::Limit(format!(
                            "display mesh vertices/triangles at body {} face {}: accumulated {} vertices/{} triangles, face {} vertices/{} triangles",
                            body.index(),
                            face.index(),
                            report.positions.len(),
                            report.triangles.len(),
                            mesh.positions.len(),
                            mesh.triangles.len()
                        )));
                    }
                    let base = report.positions.len() as u32;
                    report.faces.push(MeshedFace {
                        body_index: body.index(),
                        face_index: face.index(),
                        triangle_start: report.triangles.len(),
                        triangle_count: mesh.triangles.len(),
                    });
                    report.positions.extend(mesh.positions);
                    report.triangles.extend(
                        mesh.triangles
                            .into_iter()
                            .map(|triangle| triangle.map(|i| base + i)),
                    );
                    report.faces_meshed += 1;
                }
                Err(reason) => {
                    report.faces_skipped += 1;
                    report.issues.push(MeshIssue {
                        body_index: body.index(),
                        face_index: face.index(),
                        reason,
                    });
                }
            }
        }
    }
    Ok(report)
}

fn identity_transform(body: Body<'_>) -> bool {
    let Some(entity) = body.transform_entity() else {
        return true;
    };
    if entity.name() != "transform" {
        return false;
    }
    let directions: Vec<_> = entity.directions().collect();
    let identity = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [0.; 3]];
    directions.len() >= 4
        && directions[..4]
            .iter()
            .zip(identity)
            .all(|(a, b)| norm(sub(*a, b)) <= 1e-10)
        && entity
            .reals()
            .next()
            .is_none_or(|s| (s - 1.).abs() <= 1e-10)
}

fn edge_polyline(edge: Edge<'_>, options: &MeshOptions) -> FaceResult<Vec<Vec3>> {
    let a = edge
        .start()
        .and_then(|v| v.position())
        .ok_or("missing edge start")?;
    let b = edge
        .end()
        .and_then(|v| v.position())
        .ok_or("missing edge end")?;
    let curve = Curve::from_entity(edge.curve_entity().ok_or("missing boundary curve")?)
        .map_err(|e| e.to_string())?;
    let [mut t0, mut t1] = edge.range().ok_or("missing boundary parameter range")?;
    if !edge.entity().sense() {
        t0 = -t0;
        t1 = -t1;
    }
    let periodic = matches!(curve, Curve::Ellipse { .. });
    if !t0.is_finite()
        || !t1.is_finite()
        || norm(sub(curve.point_at(t0), a)) > 1e-6
        || norm(sub(curve.point_at(t1), b)) > 1e-6
    {
        let direction = if t1 >= t0 { 1. } else { -1. };
        t0 = curve.invert(a);
        t1 = curve.invert(b);
        if periodic {
            if norm(sub(a, b)) < 1e-10 {
                t1 = t0 + direction * TAU;
            } else {
                while (t1 - t0) * direction < 0. {
                    t1 += direction * TAU;
                }
            }
        }
    }
    let segments = match curve {
        Curve::Straight { .. } => 1,
        Curve::Ellipse { major, ratio, .. } => {
            let radius = norm(major) * ratio.max(1.);
            let step = 2. * (1. - (options.tolerance / radius).min(1.)).acos();
            (((t1 - t0).abs() / step).ceil() as usize).max(1)
        }
    };
    if segments > options.max_edge_segments {
        return Err("edge sampling exceeds segment limit".into());
    }
    let mut points: Vec<_> = (0..=segments)
        .map(|i| curve.point_at(t0 + (t1 - t0) * i as f64 / segments as f64))
        .collect();
    points[0] = a;
    points[segments] = b;
    if points.iter().flatten().any(|p| !p.is_finite()) {
        return Err("nonfinite boundary geometry".into());
    }
    Ok(points)
}

fn ring_polyline(ring: Loop<'_>, options: &MeshOptions) -> FaceResult<Vec<Vec3>> {
    let mut points: Vec<Vec3> = Vec::new();
    let mut last = None;
    for coedge in ring.coedges() {
        let edge = coedge.edge().ok_or("missing boundary edge")?;
        if edge.curve_entity().is_none()
            && edge.start().map(|v| v.index()) == edge.end().map(|v| v.index())
        {
            continue;
        }
        let mut line = edge_polyline(edge, options)?;
        if !coedge.entity().sense() {
            line.reverse();
        }
        if let Some(previous) = last
            && norm(sub(previous, line[0])) > 1e-4
        {
            return Err("boundary coedges do not connect".into());
        }
        last = line.last().copied();
        line.pop();
        if points.len().saturating_add(line.len()) > options.max_face_vertices {
            return Err("boundary exceeds face vertex limit".into());
        }
        points.extend(line);
    }
    if points.len() < 3 {
        return Err("no usable closed boundary".into());
    }
    if norm(sub(last.ok_or("empty boundary")?, points[0])) > 1e-4 {
        return Err("boundary is not closed".into());
    }
    Ok(points)
}

fn invert(surface: &Surface, point: Vec3) -> Uv {
    match *surface {
        Surface::Plane {
            origin,
            normal,
            u_dir,
        } => {
            let d = sub(point, origin);
            [dot(d, u_dir), dot(d, cross(normal, u_dir))]
        }
        Surface::Cone {
            base,
            axis,
            major,
            ratio,
            sin_angle,
            cos_angle,
        } => {
            let d = sub(point, base);
            let height = dot(d, axis);
            let radius = norm(major);
            let factor = 1. + height * sin_angle / cos_angle / radius;
            let x = dot(d, major) / (radius * radius * factor);
            let y = dot(d, cross(axis, major)) / (radius * radius * ratio * factor);
            [y.atan2(x), height]
        }
        Surface::Sphere {
            centre,
            pole,
            u_ref,
            ..
        } => {
            let d = sub(point, centre);
            let x = dot(d, u_ref);
            let y = dot(d, cross(pole, u_ref));
            [y.atan2(x), dot(d, pole).atan2(x.hypot(y))]
        }
        Surface::Torus {
            centre,
            axis,
            major_radius,
            minor_radius,
            u_ref,
        } => {
            let d = sub(point, centre);
            let x = dot(d, u_ref);
            let y = dot(d, cross(axis, u_ref));
            [
                y.atan2(x),
                (dot(d, axis) / minor_radius).atan2((x.hypot(y) - major_radius) / minor_radius),
            ]
        }
    }
}
fn periods(surface: &Surface) -> [Option<f64>; 2] {
    match surface {
        Surface::Plane { .. } => [None, None],
        Surface::Torus { .. } => [Some(TAU), Some(TAU)],
        _ => [Some(TAU), None],
    }
}
fn to_uv(surface: &Surface, ring: &[Vec3]) -> Vec<Uv> {
    let mut uv: Vec<_> = ring.iter().map(|p| invert(surface, *p)).collect();
    for (axis, period) in periods(surface).into_iter().enumerate() {
        if let Some(period) = period {
            for i in 1..uv.len() {
                uv[i][axis] -= ((uv[i][axis] - uv[i - 1][axis]) / period).round() * period;
            }
        }
    }
    uv
}
fn signed_area(uv: &[Uv]) -> f64 {
    (0..uv.len())
        .map(|i| {
            let a = uv[i];
            let b = uv[(i + 1) % uv.len()];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f64>()
        * 0.5
}
fn triangle_area(a: Uv, b: Uv, c: Uv) -> f64 {
    ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) * 0.5
}
fn wraps(uv: &[Uv]) -> bool {
    let total: f64 = (0..uv.len())
        .map(|i| {
            let step = uv[(i + 1) % uv.len()][0] - uv[i][0];
            step - (step / TAU).round() * TAU
        })
        .sum();
    (total.abs() - TAU).abs() < 0.2
}

#[derive(Default)]
struct FaceMesh {
    positions: Vec<Vec3>,
    triangles: Vec<[u32; 3]>,
}

fn tessellate_face(face: Face<'_>, options: &MeshOptions) -> FaceResult<FaceMesh> {
    let surface = Surface::from_entity(face.surface_entity().ok_or("missing surface")?)
        .map_err(|e| e.to_string())?;
    let rings: Vec<_> = face
        .loops()
        .into_iter()
        .map(|ring| ring_polyline(ring, options))
        .collect::<FaceResult<_>>()?;
    if rings.is_empty() {
        return Err("no trimming loops (untrimmed analytic face unsupported)".into());
    }
    let fit = options.tolerance.max(1e-4);
    for ring in &rings {
        for p in ring {
            let uv = invert(&surface, *p);
            if !uv.iter().all(|v| v.is_finite())
                || norm(sub(*p, surface.point_at(uv[0], uv[1]))) > fit
            {
                return Err("boundary does not lie on the face surface".into());
            }
        }
    }
    let uvs: Vec<_> = rings.iter().map(|ring| to_uv(&surface, ring)).collect();
    let wrapping = !matches!(surface, Surface::Plane { .. }) && uvs.iter().any(|ring| wraps(ring));
    let (points, parameters, triangles) = if wrapping {
        periodic_strip(&surface, &uvs, options)?
    } else if rings.len() > 1 && !matches!(surface, Surface::Plane { .. }) {
        return Err("nonplanar face with holes needs constrained curved triangulation".into());
    } else if rings.len() == 1 && !matches!(surface, Surface::Plane { .. }) {
        if let Some(triangles) = monotone_triangles(&uvs[0]) {
            (rings[0].clone(), uvs[0].clone(), triangles)
        } else {
            polygon(&surface, &rings, &uvs)?
        }
    } else {
        polygon(&surface, &rings, &uvs)?
    };
    let mut result = FaceMesh::default();
    for triangle in triangles {
        let uv = triangle.map(|i| parameters[i]);
        let xyz = triangle.map(|i| points[i]);
        refine(&surface, uv, xyz, 0, options, &mut result)?;
    }
    if !face.entity().sense() {
        for triangle in &mut result.triangles {
            triangle.swap(1, 2);
        }
    }
    if result.triangles.is_empty() {
        return Err("triangulation produced no nondegenerate triangles".into());
    }
    Ok(result)
}

type Triangulation = (Vec<Vec3>, Vec<Uv>, Vec<[usize; 3]>);
fn monotone_triangles(uv: &[Uv]) -> Option<Vec<[usize; 3]>> {
    for axis in 0..2 {
        let start = (0..uv.len()).min_by(|a, b| uv[*a][axis].total_cmp(&uv[*b][axis]))?;
        let end = (0..uv.len()).max_by(|a, b| uv[*a][axis].total_cmp(&uv[*b][axis]))?;
        if start == end {
            continue;
        }
        let forward: Vec<_> = (0..uv.len())
            .map(|i| (start + i) % uv.len())
            .take_while(|i| *i != end)
            .chain([end])
            .collect();
        let backward: Vec<_> = (0..uv.len())
            .map(|i| (start + uv.len() - i) % uv.len())
            .take_while(|i| *i != end)
            .chain([end])
            .collect();
        if [&forward, &backward].iter().any(|chain| {
            chain
                .windows(2)
                .any(|pair| uv[pair[1]][axis] < uv[pair[0]][axis] - 1e-12)
        }) {
            continue;
        }
        let (mut i, mut j) = (0, 0);
        let mut triangles = Vec::new();
        while i + 1 < forward.len() || j + 1 < backward.len() {
            let advance = j + 1 == backward.len()
                || (i + 1 < forward.len() && uv[forward[i + 1]][axis] <= uv[backward[j + 1]][axis]);
            let triangle = if advance {
                i += 1;
                [forward[i - 1], backward[j], forward[i]]
            } else {
                j += 1;
                [forward[i], backward[j - 1], backward[j]]
            };
            if triangle[0] != triangle[1]
                && triangle[0] != triangle[2]
                && triangle[1] != triangle[2]
            {
                triangles.push(triangle);
            }
        }
        let actual: f64 = triangles
            .iter()
            .map(|t| triangle_area(uv[t[0]], uv[t[1]], uv[t[2]]).abs())
            .sum();
        if (actual - signed_area(uv).abs()).abs() <= 1e-8 * actual.max(1.) {
            return Some(triangles);
        }
    }
    None
}

fn polygon(surface: &Surface, rings: &[Vec<Vec3>], uvs: &[Vec<Uv>]) -> FaceResult<Triangulation> {
    let mut order: Vec<_> = (0..rings.len()).collect();
    order.sort_by(|a, b| {
        signed_area(&uvs[*b])
            .abs()
            .total_cmp(&signed_area(&uvs[*a]).abs())
    });
    let mut points = Vec::new();
    let mut uv = Vec::new();
    let mut holes = Vec::new();
    let outer = order[0];
    let centres: Uv = std::array::from_fn(|axis| {
        uvs[outer].iter().map(|p| p[axis]).sum::<f64>() / uvs[outer].len() as f64
    });
    for (n, &index) in order.iter().enumerate() {
        if n != 0 {
            holes.push(uv.len());
        }
        points.extend_from_slice(&rings[index]);
        let shifts: Uv = std::array::from_fn(|axis| {
            periods(surface)[axis].map_or(0., |period| {
                let mean =
                    uvs[index].iter().map(|p| p[axis]).sum::<f64>() / uvs[index].len() as f64;
                ((centres[axis] - mean) / period).round() * period
            })
        });
        uv.extend(
            uvs[index]
                .iter()
                .map(|p| [p[0] + shifts[0], p[1] + shifts[1]]),
        );
    }
    let flattened: Vec<_> = uv.iter().flatten().copied().collect();
    let indices =
        earcutr::earcut(&flattened, &holes, 2).map_err(|e| format!("trim triangulation: {e}"))?;
    let triangles: Vec<_> = indices
        .chunks_exact(3)
        .map(|t| [t[0], t[1], t[2]])
        .collect();
    let expected = signed_area(&uvs[outer]).abs()
        - order
            .iter()
            .skip(1)
            .map(|i| signed_area(&uvs[*i]).abs())
            .sum::<f64>();
    let actual = triangles
        .iter()
        .map(|t| triangle_area(uv[t[0]], uv[t[1]], uv[t[2]]).abs())
        .sum::<f64>();
    if expected <= 0. || (actual - expected).abs() > 1e-8 * expected.max(1.) {
        return Err("triangulation does not preserve the trimmed region/holes".into());
    }
    Ok((points, uv, triangles))
}

fn periodic_strip(
    surface: &Surface,
    rings: &[Vec<Uv>],
    options: &MeshOptions,
) -> FaceResult<Triangulation> {
    if rings.len() != 2 || !rings.iter().all(|ring| wraps(ring)) {
        return Err("periodic face needs two complete boundary rings".into());
    }
    let mut heights = [0.; 2];
    for (i, ring) in rings.iter().enumerate() {
        heights[i] = ring.iter().map(|p| p[1]).sum::<f64>() / ring.len() as f64;
        if ring.iter().any(|p| (p[1] - heights[i]).abs() > 1e-6) {
            return Err("periodic boundary with varying second parameter unsupported".into());
        }
    }
    heights.sort_by(f64::total_cmp);
    let count = rings.iter().map(Vec::len).max().unwrap_or(3).max(12);
    if count > options.max_edge_segments {
        return Err("periodic strip exceeds segment limit".into());
    }
    let mut points = Vec::with_capacity((count + 1) * 2);
    let mut uv = Vec::with_capacity((count + 1) * 2);
    for v in heights {
        for i in 0..=count {
            let u = i as f64 * TAU / count as f64;
            uv.push([u, v]);
            points.push(surface.point_at(u, v));
        }
    }
    let width = count + 1;
    let triangles = (0..count)
        .flat_map(|i| [[i, i + 1, i + width], [i + 1, i + width + 1, i + width]])
        .collect();
    Ok((points, uv, triangles))
}

fn refine(
    surface: &Surface,
    mut uv: [Uv; 3],
    mut xyz: [Vec3; 3],
    depth: usize,
    options: &MeshOptions,
    result: &mut FaceMesh,
) -> FaceResult<()> {
    let area = triangle_area(uv[0], uv[1], uv[2]);
    let span = (0..2)
        .map(|axis| {
            uv.iter().map(|p| p[axis]).fold(f64::NEG_INFINITY, f64::max)
                - uv.iter().map(|p| p[axis]).fold(f64::INFINITY, f64::min)
        })
        .fold(0., f64::max);
    // Three points on the same curved boundary can have 3D area while enclosing
    // no surface region. Discard these UV slivers before lifting/refinement.
    if area.abs() <= 1e-10 * span * span {
        return Ok(());
    }
    if area < 0. {
        uv.swap(1, 2);
        xyz.swap(1, 2);
    }
    let midpoint_uv: [Uv; 3] =
        std::array::from_fn(|i| std::array::from_fn(|j| (uv[i][j] + uv[(i + 1) % 3][j]) * 0.5));
    let midpoint_xyz = midpoint_uv.map(|p| surface.point_at(p[0], p[1]));
    let edge_error = (0..3)
        .map(|i| {
            norm(sub(
                midpoint_xyz[i],
                scale(add(xyz[i], xyz[(i + 1) % 3]), 0.5),
            ))
        })
        .fold(0., f64::max);
    let centre_uv: Uv = std::array::from_fn(|i| (uv[0][i] + uv[1][i] + uv[2][i]) / 3.);
    let centre_xyz = scale(add(add(xyz[0], xyz[1]), xyz[2]), 1. / 3.);
    let error = edge_error.max(norm(sub(
        centre_xyz,
        surface.point_at(centre_uv[0], centre_uv[1]),
    )));
    if !error.is_finite() {
        return Err("nonfinite surface tessellation".into());
    }
    if error > options.tolerance && !matches!(surface, Surface::Plane { .. }) {
        if depth >= options.max_refinement_depth {
            return Err("curved face exceeds tolerance at refinement limit".into());
        }
        for (u, p) in [
            (
                [uv[0], midpoint_uv[0], midpoint_uv[2]],
                [xyz[0], midpoint_xyz[0], midpoint_xyz[2]],
            ),
            (
                [midpoint_uv[0], uv[1], midpoint_uv[1]],
                [midpoint_xyz[0], xyz[1], midpoint_xyz[1]],
            ),
            (
                [midpoint_uv[2], midpoint_uv[1], uv[2]],
                [midpoint_xyz[2], midpoint_xyz[1], xyz[2]],
            ),
            (midpoint_uv, midpoint_xyz),
        ] {
            refine(surface, u, p, depth + 1, options, result)?;
        }
        return Ok(());
    }
    let normal = cross(sub(xyz[1], xyz[0]), sub(xyz[2], xyz[0]));
    if norm(normal) <= 1e-16 {
        return Ok(());
    }
    let [u, v] = centre_uv;
    let du = sub(surface.point_at(u + 1e-6, v), surface.point_at(u - 1e-6, v));
    let dv = sub(surface.point_at(u, v + 1e-6), surface.point_at(u, v - 1e-6));
    let expected = cross(du, dv);
    // Singular torus patches can leave three boundary samples whose chord
    // triangle has area even though its projected surface region collapses.
    if dot(normal, expected) <= 1e-8 * norm(normal) * norm(expected) {
        return Ok(());
    }
    if result.positions.len().saturating_add(3) > options.max_vertices
        || result.triangles.len().saturating_add(1)
            > options.max_triangles.min(options.max_face_triangles)
    {
        return Err("face exceeds mesh resource limit".into());
    }
    let base = result.positions.len() as u32;
    result.positions.extend(xyz);
    result.triangles.push([base, base + 1, base + 2]);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::asm::{Entity, Header, Token, Value, tag};

    fn token(tag: u8, value: Value) -> Token {
        Token { tag, value }
    }
    fn entity(
        model: &mut Model,
        name: &str,
        base: &str,
        pointers: &[i64],
        fields: Vec<Token>,
    ) -> usize {
        let types: Vec<String> = if name == base {
            vec![name.into()]
        } else {
            vec![name.into(), base.into()]
        };
        let mut tokens: Vec<_> = types
            .iter()
            .map(|s| token(tag::TYPE, Value::Text(s.clone())))
            .collect();
        tokens.extend(
            pointers
                .iter()
                .map(|p| token(tag::POINTER, Value::Integer(*p))),
        );
        tokens.extend(fields);
        let index = model.entities.len();
        model.entities.push(Entity {
            index,
            source_record: index,
            types,
            tokens,
        });
        index
    }
    fn link(model: &mut Model, index: usize, slot: usize, target: usize) {
        model.entities[index]
            .tokens
            .iter_mut()
            .filter(|t| t.tag == tag::POINTER)
            .nth(slot)
            .unwrap()
            .value = Value::Integer(target as i64);
    }
    fn vector(tag: u8, v: Vec3) -> Token {
        token(tag, Value::Vector(v))
    }
    fn plane_model(hole: bool) -> Model {
        let mut model = Model {
            header: Header {
                signature: "synthetic".into(),
                version: 1,
                reserved: [0; 3],
                product: "test".into(),
                kernel_version: "test".into(),
                written: "test".into(),
                sizebox: 10.,
                resabs: 1e-6,
                resnor: 1e-10,
                body_offset: 0,
                word_size: 4,
            },
            entities: Vec::new(),
            history: Vec::new(),
            has_history: false,
            terminated: true,
        };
        entity(&mut model, "body", "body", &[-1, -1, 1, -1, -1], vec![]);
        entity(&mut model, "lump", "lump", &[-1, -1, -1, 2], vec![]);
        entity(&mut model, "shell", "shell", &[-1, -1, -1, -1, 3], vec![]);
        entity(
            &mut model,
            "face",
            "face",
            &[-1, -1, -1, -1, -1, -1, 4],
            vec![],
        );
        entity(
            &mut model,
            "plane",
            "surface",
            &[],
            vec![
                vector(tag::POSITION, [0.; 3]),
                vector(tag::DIRECTION, [0., 0., 1.]),
                vector(tag::DIRECTION, [1., 0., 0.]),
            ],
        );
        let mut previous_loop = None;
        for size in if hole { vec![2., 1.] } else { vec![2.] } {
            let points = [
                [-size, -size, 0.],
                [size, -size, 0.],
                [size, size, 0.],
                [-size, size, 0.],
            ];
            let ring = entity(&mut model, "loop", "loop", &[-1; 5], vec![]);
            if let Some(previous) = previous_loop {
                link(&mut model, previous, 2, ring);
            } else {
                link(&mut model, 3, 3, ring);
            }
            previous_loop = Some(ring);
            let mut vertices = Vec::new();
            for point in points {
                let p = entity(
                    &mut model,
                    "point",
                    "point",
                    &[],
                    vec![vector(tag::POSITION, point)],
                );
                vertices.push(entity(
                    &mut model,
                    "vertex",
                    "vertex",
                    &[-1, -1, -1, p as i64],
                    vec![],
                ));
            }
            let mut coedges = Vec::new();
            for i in 0..4 {
                let delta = sub(points[(i + 1) % 4], points[i]);
                let length = norm(delta);
                let curve = entity(
                    &mut model,
                    "straight",
                    "curve",
                    &[],
                    vec![
                        vector(tag::POSITION, points[i]),
                        vector(tag::DIRECTION, scale(delta, 1. / length)),
                    ],
                );
                let edge = entity(
                    &mut model,
                    "edge",
                    "edge",
                    &[
                        -1,
                        -1,
                        vertices[i] as i64,
                        vertices[(i + 1) % 4] as i64,
                        -1,
                        curve as i64,
                    ],
                    vec![
                        token(tag::DOUBLE, Value::Real(0.)),
                        token(tag::DOUBLE, Value::Real(length)),
                    ],
                );
                coedges.push(entity(
                    &mut model,
                    "coedge",
                    "coedge",
                    &[-1, -1, -1, -1, -1, edge as i64],
                    vec![],
                ));
            }
            for i in 0..4 {
                link(&mut model, coedges[i], 2, coedges[(i + 1) % 4]);
            }
            link(&mut model, ring, 3, coedges[0]);
        }
        model
    }

    #[test]
    fn trimmed_plane_preserves_hole_and_face_sense() {
        let mut model = plane_model(true);
        let report = tessellate(&model, &Default::default()).unwrap();
        assert_eq!((report.faces_meshed, report.faces_skipped), (1, 0));
        assert_eq!(report.faces[0].body_index, 0);
        let mut area = 0.;
        for triangle in &report.triangles {
            let p = triangle.map(|i| report.positions[i as usize]);
            let centre = scale(add(add(p[0], p[1]), p[2]), 1. / 3.);
            assert!(
                centre[0].abs() >= 1. || centre[1].abs() >= 1.,
                "hole filled at {centre:?}"
            );
            let normal = cross(sub(p[1], p[0]), sub(p[2], p[0]));
            assert!(normal[2] > 0.);
            area += norm(normal) * 0.5;
        }
        assert!((area - 12.).abs() < 1e-10);
        model.entities[3]
            .tokens
            .push(token(tag::FALSE, Value::Empty));
        let reversed = tessellate(&model, &Default::default()).unwrap();
        for triangle in reversed.triangles {
            let p = triangle.map(|i| reversed.positions[i as usize]);
            assert!(cross(sub(p[1], p[0]), sub(p[2], p[0]))[2] < 0.);
        }
    }

    #[test]
    fn cylinder_strip_is_on_surface_and_respects_chord_tolerance() {
        let surface = Surface::Cone {
            base: [0.; 3],
            axis: [0., 0., 1.],
            major: [10., 0., 0.],
            ratio: 1.,
            sin_angle: 0.,
            cos_angle: 1.,
        };
        let options = MeshOptions::default();
        let rings: Vec<Vec<Uv>> = [0., 5.]
            .into_iter()
            .map(|v| (0..16).map(|i| [i as f64 * TAU / 16., v]).collect())
            .collect();
        let (points, parameters, triangles) = periodic_strip(&surface, &rings, &options).unwrap();
        let mut mesh = FaceMesh::default();
        for triangle in triangles {
            refine(
                &surface,
                triangle.map(|i| parameters[i]),
                triangle.map(|i| points[i]),
                0,
                &options,
                &mut mesh,
            )
            .unwrap();
        }
        assert!(mesh.triangles.len() > 32);
        for triangle in mesh.triangles {
            let p = triangle.map(|i| mesh.positions[i as usize]);
            for point in p {
                assert!((point[0].hypot(point[1]) - 10.).abs() < 1e-10);
            }
            let centre = scale(add(add(p[0], p[1]), p[2]), 1. / 3.);
            assert!(10. - centre[0].hypot(centre[1]) <= options.tolerance);
            assert!(
                dot(
                    cross(sub(p[1], p[0]), sub(p[2], p[0])),
                    [centre[0], centre[1], 0.]
                ) > 0.
            );
        }
    }

    #[test]
    fn unsupported_faces_are_explicit_and_mesh_limits_are_bounded() {
        let mut model = plane_model(false);
        model.entities[4].types[0] = "spline".into();
        let report = tessellate(&model, &Default::default()).unwrap();
        assert!(report.positions.is_empty());
        assert_eq!(report.faces_skipped, 1);
        assert!(report.issues[0].reason.contains("spline"));
        let report = tessellate(
            &plane_model(false),
            &MeshOptions {
                max_triangles: 1,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(report.triangles.is_empty());
        assert!(report.issues[0].reason.contains("resource limit"));
        assert!(
            tessellate(
                &model,
                &MeshOptions {
                    tolerance: f64::NAN,
                    ..Default::default()
                }
            )
            .is_err()
        );
    }

    #[test]
    fn unverified_body_transforms_are_reported_not_guessed() {
        let mut model = plane_model(false);
        let transform = entity(
            &mut model,
            "transform",
            "transform",
            &[],
            vec![
                vector(tag::DIRECTION, [1., 0., 0.]),
                vector(tag::DIRECTION, [0., 1., 0.]),
                vector(tag::DIRECTION, [0., 0., 1.]),
                vector(tag::DIRECTION, [0.; 3]),
                token(tag::DOUBLE, Value::Real(1.)),
            ],
        );
        link(&mut model, 0, 4, transform);
        assert_eq!(
            tessellate(&model, &Default::default())
                .unwrap()
                .faces_meshed,
            1
        );
        let translation = model.entities[transform]
            .tokens
            .iter_mut()
            .filter(|t| t.tag == tag::DIRECTION)
            .nth(3)
            .unwrap();
        translation.value = Value::Vector([1., 2., 3.]);
        let report = tessellate(&model, &Default::default()).unwrap();
        assert_eq!(report.faces_skipped, 1);
        assert!(report.positions.is_empty());
        assert!(report.issues[0].reason.contains("transform"));
    }
}

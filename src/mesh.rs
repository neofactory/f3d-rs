//! Bounded meshes from live, trimmed analytic ASM faces.
//!
//! Positions retain the input stream's kernel coordinates (normally centimetres
//! for Fusion). Archive occurrence placements and machine motion are not applied.
//! Only absent/identity ASM body transforms are accepted: their nonidentity
//! grammar is not yet verified. Unsupported faces are reported, never filled with
//! substitute solids.
//!
//! Faces that meet at an edge take the same points along it, and a triangle
//! inside a face is split only where its neighbour is split too. So a body comes
//! out as a closed mesh when all its faces mesh and each of its edges joins two
//! faces. [`BodyMesh::edge_counts`] counts the edges where the triangles do not close.
//!
//! Edge sampling, surface inversion and periodic-strip handling are adapted in
//! part from ezf3d; see `THIRD-PARTY-NOTICES` and `LICENSE-MIT-ezf3d`.
use crate::{
    Error, Result,
    asm::{Body, Edge, Face, Loop, Model},
    geometry::{Curve, Surface},
    math::{Vec3, add, cross, dot, norm, scale, sub},
    spline::GeometryContext,
};
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    f64::consts::{PI, TAU},
    rc::Rc,
};

type Uv = [f64; 2];
type FaceResult<T> = std::result::Result<T, FaceFailure>;

#[derive(Debug, Clone)]
struct FaceFailure {
    kind: MeshIssueKind,
    reason: String,
}
impl FaceFailure {
    fn new(kind: MeshIssueKind, reason: impl Into<String>) -> Self {
        Self {
            kind,
            reason: reason.into(),
        }
    }
}
impl From<&str> for FaceFailure {
    fn from(reason: &str) -> Self {
        Self::new(MeshIssueKind::InvalidTopology, reason)
    }
}
impl From<Error> for FaceFailure {
    fn from(error: Error) -> Self {
        let kind = match error {
            Error::UnsupportedGeometry { .. } => MeshIssueKind::UnsupportedGeometry,
            _ => MeshIssueKind::InvalidGeometry,
        };
        Self::new(kind, error.to_string())
    }
}

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
            max_face_triangles: 131_072,
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
    pub kind: MeshIssueKind,
    pub reason: String,
}
/// A stable category for a skipped face, independent of its readable reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MeshIssueKind {
    UnsupportedGeometry,
    InvalidGeometry,
    UnsupportedBodyTransform,
    UnsupportedTopology,
    InvalidTopology,
    ResourceLimit,
    Tessellation,
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

/// How the triangles of a mesh meet. In a closed mesh every side of every
/// triangle has one other triangle on it, running along it the other way.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct EdgeCounts {
    /// Sides with no triangle on the other side: the mesh has a gap there.
    pub open: usize,
    /// Sides that another triangle runs along in the same direction: more than
    /// two triangles meet there, or two that face opposite ways.
    pub repeated: usize,
}
impl EdgeCounts {
    pub fn closed(&self) -> bool {
        self.open == 0 && self.repeated == 0
    }
}
impl BodyMesh {
    /// Count the sides of this body's triangles that do not close, by vertex
    /// index. Faces skipped for an issue leave open sides around them.
    pub fn edge_counts(&self) -> EdgeCounts {
        let mut sides = BTreeMap::<(u32, u32), usize>::new();
        for triangle in &self.triangles {
            for corner in 0..3 {
                *sides
                    .entry((triangle[corner], triangle[(corner + 1) % 3]))
                    .or_default() += 1;
            }
        }
        EdgeCounts {
            open: sides
                .keys()
                .filter(|(a, b)| !sides.contains_key(&(*b, *a)))
                .count(),
            repeated: sides.values().map(|count| count - 1).sum(),
        }
    }
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
    let context = GeometryContext::new(model);
    let mut shared = Shared::new(model, &context);
    let mut report = MeshReport::default();
    let mut seen = BTreeSet::new();
    for body in model.bodies() {
        let body_supported = identity_transform(body);
        for face in body.faces() {
            if !seen.insert(face.index()) {
                continue;
            }
            let before = shared.positions.len();
            let built = if body_supported {
                tessellate_face(face, options, &context, &mut shared)
            } else {
                Err(FaceFailure::new(
                    MeshIssueKind::UnsupportedBodyTransform,
                    "nonidentity or unrecognized ASM body transform",
                ))
            };
            let triangles = built.as_ref().map_or(0, Vec::len);
            if shared.positions.len() > options.max_vertices
                || report.triangles.len().saturating_add(triangles) > options.max_triangles
            {
                return Err(Error::Limit(format!(
                    "display mesh vertices/triangles at body {} face {}: accumulated {} vertices/{} triangles, face {} vertices/{} triangles",
                    body.index(),
                    face.index(),
                    before,
                    report.triangles.len(),
                    shared.positions.len() - before,
                    triangles
                )));
            }
            match built {
                Ok(triangles) => {
                    report.faces.push(MeshedFace {
                        body_index: body.index(),
                        face_index: face.index(),
                        triangle_start: report.triangles.len(),
                        triangle_count: triangles.len(),
                    });
                    report.triangles.extend(triangles);
                    report.faces_meshed += 1;
                }
                Err(failure) => {
                    report.faces_skipped += 1;
                    report.issues.push(MeshIssue {
                        body_index: body.index(),
                        face_index: face.index(),
                        kind: failure.kind,
                        reason: failure.reason,
                    });
                }
            }
        }
    }
    // Leave out the points no triangle uses: samples of edges whose faces were
    // all skipped.
    let mut renumbered = vec![u32::MAX; shared.positions.len()];
    for corner in report.triangles.iter_mut().flatten() {
        let number = &mut renumbered[*corner as usize];
        if *number == u32::MAX {
            *number = report.positions.len() as u32;
            report.positions.push(shared.positions[*corner as usize]);
        }
        *corner = *number;
    }
    Ok(report)
}

/// The points that faces share: one for each vertex of the model and one for
/// each sample along each edge. Two faces that meet at an edge take the same
/// points from here, so their triangles join along it.
struct Shared {
    positions: Vec<Vec3>,
    vertices: BTreeMap<usize, u32>,
    /// The points of vertices by where they are, in cells one `resolution` wide.
    places: BTreeMap<[i64; 3], Vec<u32>>,
    /// The kernel's own resolution. Two vertices nearer than this are one point.
    resolution: f64,
    edges: BTreeMap<usize, FaceResult<Rc<[u32]>>>,
    /// The surfaces of the faces at each edge, by the edge's index.
    surfaces: BTreeMap<usize, Vec<Rc<Surface>>>,
    /// The sides of the triangles made so far that join two shared points,
    /// each as (smaller point, larger point).
    taken: BTreeSet<(u32, u32)>,
}
impl Shared {
    fn new(model: &Model, context: &GeometryContext) -> Self {
        let mut surfaces = BTreeMap::<usize, Vec<Rc<Surface>>>::new();
        for face in model.live_faces() {
            let Some(Ok(surface)) = face
                .surface_entity()
                .map(|entity| Surface::from_context(entity, context))
            else {
                continue;
            };
            let surface = Rc::new(surface);
            for edge in face.edges() {
                let at_edge = surfaces.entry(edge.index()).or_default();
                if !at_edge.iter().any(|other| Rc::ptr_eq(other, &surface)) {
                    at_edge.push(surface.clone());
                }
            }
        }
        let resolution = model.header.resabs;
        Self {
            positions: Vec::new(),
            vertices: BTreeMap::new(),
            places: BTreeMap::new(),
            resolution: if resolution.is_finite() && resolution > 0. {
                resolution
            } else {
                1e-6
            },
            edges: BTreeMap::new(),
            surfaces,
            taken: BTreeSet::new(),
        }
    }
    /// The point of a vertex. A model can have two vertices at one place, where
    /// two boundaries touch. They get one point: a triangle between two points
    /// at one place would have no area.
    fn vertex(&mut self, index: usize, position: Vec3) -> u32 {
        if let Some(point) = self.vertices.get(&index) {
            return *point;
        }
        let cell = position.map(|x| (x / self.resolution).floor() as i64);
        let near = (0..27)
            .filter_map(|n| {
                let cell = [
                    cell[0] + n % 3 - 1,
                    cell[1] + n / 3 % 3 - 1,
                    cell[2] + n / 9 - 1,
                ];
                self.places.get(&cell)
            })
            .flatten()
            .find(|point| norm(sub(self.positions[**point as usize], position)) <= self.resolution);
        let point = near.copied().unwrap_or_else(|| {
            self.positions.push(position);
            let point = (self.positions.len() - 1) as u32;
            self.places.entry(cell).or_default().push(point);
            point
        });
        self.vertices.insert(index, point);
        point
    }
    /// The points along an edge from its start to its end. An edge is sampled
    /// once, whichever face asks first.
    fn edge(
        &mut self,
        edge: Edge<'_>,
        options: &MeshOptions,
        context: &GeometryContext,
    ) -> FaceResult<Rc<[u32]>> {
        if let Some(known) = self.edges.get(&edge.index()) {
            return known.clone();
        }
        let sampled = self.sample(edge, options, context);
        self.edges.insert(edge.index(), sampled.clone());
        sampled
    }
    fn sample(
        &mut self,
        edge: Edge<'_>,
        options: &MeshOptions,
        context: &GeometryContext,
    ) -> FaceResult<Rc<[u32]>> {
        let surfaces = self.surfaces.get(&edge.index());
        let line = edge_polyline(
            edge,
            surfaces.map_or(&[][..], Vec::as_slice),
            self.resolution,
            options,
            context,
        )?;
        let start = edge.start().ok_or("missing edge start")?.index();
        let end = edge.end().ok_or("missing edge end")?.index();
        let last = line.len() - 1;
        let mut points = vec![self.vertex(start, line[0])];
        for position in &line[1..last] {
            self.positions.push(*position);
            points.push((self.positions.len() - 1) as u32);
        }
        points.push(self.vertex(end, line[last]));
        Ok(points.into())
    }
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

fn edge_polyline(
    edge: Edge<'_>,
    surfaces: &[Rc<Surface>],
    resolution: f64,
    options: &MeshOptions,
    context: &GeometryContext,
) -> FaceResult<Vec<Vec3>> {
    let a = edge
        .start()
        .and_then(|v| v.position())
        .ok_or("missing edge start")?;
    let b = edge
        .end()
        .and_then(|v| v.position())
        .ok_or("missing edge end")?;
    let curve = Curve::from_context(
        edge.curve_entity().ok_or("missing boundary curve")?,
        context,
    )?;
    let [mut t0, mut t1] = edge.range().ok_or("missing boundary parameter range")?;
    if !edge.entity().sense() {
        t0 = -t0;
        t1 = -t1;
    }
    let periodic =
        matches!(curve, Curve::Ellipse { .. }) || matches!(&curve, Curve::Spline(s) if s.closed);
    let period = match &curve {
        Curve::Spline(s) => {
            let [a, b] = s.domain(0);
            b - a
        }
        _ => TAU,
    };
    let endpoint_tolerance = if matches!(&curve, Curve::Spline(..)) {
        options.tolerance * 0.25
    } else {
        1e-6
    };
    if (norm(sub(curve.point_at(t0), a)) > endpoint_tolerance
        || norm(sub(curve.point_at(t1), b)) > endpoint_tolerance)
        && norm(sub(curve.point_at(-t0), a)) <= endpoint_tolerance
        && norm(sub(curve.point_at(-t1), b)) <= endpoint_tolerance
    {
        t0 = -t0;
        t1 = -t1;
    }
    if !t0.is_finite()
        || !t1.is_finite()
        || norm(sub(curve.point_at(t0), a)) > endpoint_tolerance
        || norm(sub(curve.point_at(t1), b)) > endpoint_tolerance
    {
        let direction = if t1 >= t0 { 1. } else { -1. };
        t0 = curve.invert(a);
        t1 = curve.invert(b);
        if periodic {
            if norm(sub(a, b)) < 1e-10 {
                t1 = t0 + direction * period;
            } else {
                while (t1 - t0) * direction < 0. {
                    t1 += direction * period;
                }
            }
        }
    }
    // Each sample is a parameter of the curve and the point there.
    let mut samples = if let Curve::Spline(ref spline) = curve {
        let mut parameters = vec![t0];
        let mut knots = Vec::new();
        let start = if spline.closed {
            (t0.min(t1) / period).floor() as i32 - 1
        } else {
            0
        };
        let end = if spline.closed {
            (t0.max(t1) / period).ceil() as i32 + 1
        } else {
            0
        };
        if end - start > 4 {
            return Err(FaceFailure::new(
                MeshIssueKind::ResourceLimit,
                "spline period limit",
            ));
        }
        for n in start..=end {
            knots.extend(
                spline.knots[0]
                    .iter()
                    .map(|t| t + n as f64 * period)
                    .filter(|t| *t > t0.min(t1) && *t < t0.max(t1)),
            );
        }
        knots.sort_by(f64::total_cmp);
        knots.dedup();
        if t1 < t0 {
            knots.reverse();
        }
        parameters.extend(knots);
        parameters.push(t1);
        let mut samples = vec![(t0, a)];
        for pair in parameters.windows(2) {
            sample_spline(&curve, pair[0], pair[1], 0, options, &mut samples)?;
        }
        samples
    } else {
        let segments = match curve {
            Curve::Spline(..) => unreachable!(),
            Curve::Straight { .. } => 1,
            Curve::Ellipse { major, ratio, .. } => {
                let radius = norm(major) * ratio.max(1.);
                let step = 2. * (1. - (options.tolerance / radius).min(1.)).acos();
                (((t1 - t0).abs() / step).ceil() as usize).max(1)
            }
        };
        if segments > options.max_edge_segments {
            return Err(FaceFailure::new(
                MeshIssueKind::ResourceLimit,
                "edge sampling exceeds segment limit",
            ));
        }
        (0..=segments)
            .map(|i| {
                let t = t0 + (t1 - t0) * i as f64 / segments as f64;
                (t, curve.point_at(t))
            })
            .collect()
    };
    samples[0].1 = a;
    samples.last_mut().unwrap().1 = b;
    // Two samples nearer than the kernel's resolution are one point: a knot
    // of a closed curve can lie at the very end of the edge's range. A chord
    // between them would give triangles with no area.
    let last = samples.len() - 1;
    let mut kept = 0;
    for i in 1..=last {
        if norm(sub(samples[i].1, samples[kept].1)) > resolution {
            kept += 1;
        } else if i == last {
            kept = kept.max(1);
        } else {
            continue;
        }
        samples[kept] = samples[i];
    }
    samples.truncate(kept + 1);
    fit_chords(&curve, &mut samples, surfaces, resolution, options);
    if samples
        .iter()
        .any(|(_, p)| p.iter().any(|v| !v.is_finite()))
    {
        return Err(FaceFailure::new(
            MeshIssueKind::InvalidGeometry,
            "nonfinite boundary geometry",
        ));
    }
    Ok(samples.into_iter().map(|(_, point)| point).collect())
}

/// A face does not split the chords of its boundary: the face on the other
/// side would not have the new point. So the chords of an edge are split here,
/// where both faces take them from, until each is within tolerance of both
/// surfaces by the measure a face uses for the sides of its triangles
/// (`chord_error`). A chord is halved at most eight times, and not at all if
/// the new sample would lie off a surface that both its ends lie on: an edge
/// curve can stray from its surface between samples that are on it.
fn fit_chords(
    curve: &Curve,
    samples: &mut Vec<(f64, Vec3)>,
    surfaces: &[Rc<Surface>],
    resolution: f64,
    options: &MeshOptions,
) {
    let fit = options.tolerance.max(1e-4);
    // Where a sample is on each surface, if the surface passes through it.
    let locate = |point: Vec3| -> Vec<_> {
        surfaces
            .iter()
            .map(|surface| {
                let uv = invert(surface, point);
                (norm(sub(point, surface.point_at(uv[0], uv[1]))) <= fit)
                    .then(|| (uv, collapsed(surface, uv, resolution)))
            })
            .collect()
    };
    let mut places: Vec<_> = samples.iter().map(|(_, point)| locate(*point)).collect();
    let mut halvings = vec![0; samples.len()];
    let mut i = 0;
    while i + 1 < samples.len() {
        let ((t0, a), (t1, b)) = (samples[i], samples[i + 1]);
        let ends = |surface: usize| places[i][surface].zip(places[i + 1][surface]);
        let outside = (0..surfaces.len()).any(|surface| {
            ends(surface).is_some_and(|(p, q)| {
                chord_error(&surfaces[surface], p, q, a, b) > options.tolerance
            })
        });
        let middle = (outside && halvings[i] < 8 && samples.len() <= options.max_edge_segments)
            .then(|| {
                let t = (t0 + t1) * 0.5;
                let point = curve.point_at(t);
                (t, point, locate(point))
            })
            .filter(|(_, _, place)| {
                (0..surfaces.len())
                    .all(|surface| ends(surface).is_none() || place[surface].is_some())
            });
        if let Some((t, point, place)) = middle {
            halvings[i] += 1;
            samples.insert(i + 1, (t, point));
            places.insert(i + 1, place);
            halvings.insert(i + 1, halvings[i]);
        } else {
            i += 1;
        }
    }
}

/// How far a chord from `a` to `b` is from a surface: the surface point half
/// way between the two ends' places on the surface, against the middle of the
/// chord. Each end comes with the parameter that means nothing there, if any.
fn chord_error(
    surface: &Surface,
    (mut p, p_collapsed): (Uv, Option<usize>),
    (mut q, q_collapsed): (Uv, Option<usize>),
    a: Vec3,
    b: Vec3,
) -> f64 {
    // Where the surface comes to a point, the parameter that means nothing
    // takes the other end's value.
    if let Some(axis) = p_collapsed {
        p[axis] = q[axis];
    } else if let Some(axis) = q_collapsed {
        q[axis] = p[axis];
    }
    for (axis, period) in periods(surface).into_iter().enumerate() {
        if let Some(period) = period {
            q[axis] -= ((q[axis] - p[axis]) / period).round() * period;
        }
    }
    let middle = surface.point_at((p[0] + q[0]) * 0.5, (p[1] + q[1]) * 0.5);
    norm(sub(middle, scale(add(a, b), 0.5)))
}

/// The points of one boundary of a face, in order round it.
fn ring_points(
    ring: Loop<'_>,
    options: &MeshOptions,
    context: &GeometryContext,
    shared: &mut Shared,
) -> FaceResult<Vec<u32>> {
    let mut points: Vec<u32> = Vec::new();
    let mut last = None;
    for coedge in ring.coedges() {
        let edge = coedge.edge().ok_or("missing boundary edge")?;
        if edge.curve_entity().is_none()
            && edge.start().map(|v| v.index()) == edge.end().map(|v| v.index())
        {
            continue;
        }
        let mut line = shared.edge(edge, options, context)?.to_vec();
        if !coedge.entity().sense() {
            line.reverse();
        }
        let position = |point: u32| shared.positions[point as usize];
        if let Some(previous) = last
            && norm(sub(position(previous), position(line[0]))) > 1e-4
        {
            return Err("boundary coedges do not connect".into());
        }
        last = line.pop();
        if points.len().saturating_add(line.len()) > options.max_face_vertices {
            return Err(FaceFailure::new(
                MeshIssueKind::ResourceLimit,
                "boundary exceeds face vertex limit",
            ));
        }
        points.extend(line);
    }
    // An edge shorter than the kernel's resolution has one point for both
    // its ends.
    points.dedup();
    if points.len() > 1 && points.first() == points.last() {
        points.pop();
    }
    if points.len() < 3 {
        return Err("no usable closed boundary".into());
    }
    let position = |point: u32| shared.positions[point as usize];
    if norm(sub(
        position(last.ok_or("empty boundary")?),
        position(points[0]),
    )) > 1e-4
    {
        return Err("boundary is not closed".into());
    }
    Ok(points)
}

fn invert(surface: &Surface, point: Vec3) -> Uv {
    match *surface {
        Surface::Spline(ref s) => s.invert(point),
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
        Surface::Plane { .. } | Surface::Spline(..) => [None, None],
        Surface::Torus { .. } => [Some(TAU), Some(TAU)],
        _ => [Some(TAU), None],
    }
}
/// Where the points of a boundary are on the surface, in order. `sense` says
/// that the face's outward side is the side of the surface's own normal.
fn to_uv(surface: &Surface, ring: &[Vec3], resolution: f64, sense: bool) -> Vec<Uv> {
    let mut uv: Vec<_> = ring.iter().map(|p| invert(surface, *p)).collect();
    let periods = periods(surface);
    for axis in 0..2 {
        let Some(period) = periods[axis] else {
            continue;
        };
        // The value before, and a place passed since then where the surface
        // comes to a point: the value there means nothing.
        let mut previous: Option<f64> = None;
        let mut passed = None;
        for i in 0..uv.len() {
            if collapsed(surface, uv[i], resolution) == Some(axis) {
                passed = Some(i);
                continue;
            }
            if let Some(previous) = previous {
                // Each value is taken to the turn nearest the one before it.
                let mut step = within(uv[i][axis] - previous, period);
                if let Some(point) = passed
                    && step.abs() > 1e-9
                {
                    // Through the point, the nearest turn can be the wrong
                    // one: the boundary leaves it on a different parameter
                    // line, and the face is the part of the surface on one
                    // side of the two lines, not the other. The face is on
                    // the left of its boundary seen from outside, which says
                    // which way round the point the boundary goes.
                    let mut rise = uv[i][1 - axis] - uv[point][1 - axis];
                    if let Some(other) = periods[1 - axis] {
                        rise = within(rise, other);
                    }
                    let forward = (sense == (rise > 0.)) == (axis == 0);
                    step = step.rem_euclid(period) - if forward { 0. } else { period };
                }
                uv[i][axis] = previous + step;
            }
            previous = Some(uv[i][axis]);
            passed = None;
        }
    }
    uv
}
/// An angle taken to the half turn either side of zero.
fn half_turn(angle: f64) -> f64 {
    within(angle, TAU)
}
/// A difference of two values of a parameter that comes round again, taken to
/// the half period either side of zero.
fn within(difference: f64, period: f64) -> f64 {
    difference - (difference / period).round() * period
}
/// The parameter that means nothing at a place, because the surface comes to
/// a point there: the longitude at the pole of a sphere, at the apex of a cone
/// and where a torus closes on its own axis, and either parameter of a spline
/// surface along a side that is one point. All of that parameter's range moves
/// the point by less than the kernel's resolution.
fn collapsed(surface: &Surface, uv: Uv, resolution: f64) -> Option<usize> {
    let ranges = match surface {
        Surface::Plane { .. } => return None,
        Surface::Spline(spline) => [0, 1].map(|axis| {
            let [low, high] = spline.domain(axis);
            Some(high - low)
        }),
        _ => [Some(TAU), None],
    };
    (0..2).find(|&axis| {
        ranges[axis].is_some_and(|range| {
            let (mut ahead, mut behind) = (uv, uv);
            ahead[axis] += range * 1e-6;
            behind[axis] -= range * 1e-6;
            let moved = sub(
                surface.point_at(ahead[0], ahead[1]),
                surface.point_at(behind[0], behind[1]),
            );
            norm(moved) / 2e-6 <= resolution
        })
    })
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

/// One boundary of a face, in order round it: the point at each place, where
/// that point is in space, and where it is on the face's surface.
struct Ring {
    point: Vec<u32>,
    xyz: Vec<Vec3>,
    uv: Vec<Uv>,
}
impl Ring {
    fn corners(&self) -> impl Iterator<Item = (Uv, Vec3, u32)> + '_ {
        (0..self.uv.len()).map(|i| (self.uv[i], self.xyz[i], self.point[i]))
    }
}

/// A face's triangles in the plane of its surface parameters. A corner is a
/// place in that plane. Two corners can be one point of the solid: the two
/// sides of a seam, or the pole of a sphere.
struct Patch {
    uv: Vec<Uv>,
    xyz: Vec<Vec3>,
    /// The point at each corner. Points from `shared` up are this face's own.
    point: Vec<u32>,
    shared: u32,
    own: Vec<Vec3>,
    /// How many of the corners are on the boundary. They come first.
    boundary: usize,
    /// The chords of the boundary, each as (smaller corner, larger corner).
    fixed: BTreeSet<(usize, usize)>,
    /// One boundary chord, from corner to corner the way the boundary goes.
    along: [usize; 2],
    triangles: Vec<[usize; 3]>,
}
fn chord(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}
impl Patch {
    fn new(shared: u32) -> Self {
        Self {
            uv: Vec::new(),
            xyz: Vec::new(),
            point: Vec::new(),
            shared,
            own: Vec::new(),
            boundary: 0,
            fixed: BTreeSet::new(),
            along: [0, 1],
            triangles: Vec::new(),
        }
    }
    fn corner(&mut self, uv: Uv, xyz: Vec3, point: u32) -> usize {
        self.uv.push(uv);
        self.xyz.push(xyz);
        self.point.push(point);
        self.uv.len() - 1
    }
    /// A point that only this face has.
    fn own_point(&mut self, xyz: Vec3) -> u32 {
        self.own.push(xyz);
        self.shared + (self.own.len() - 1) as u32
    }
    /// One closed boundary, as its corners in order.
    fn boundary(&mut self, corners: impl IntoIterator<Item = (Uv, Vec3, u32)>) {
        let first = self.uv.len();
        for (uv, xyz, point) in corners {
            self.corner(uv, xyz, point);
        }
        let count = self.uv.len() - first;
        for i in 0..count {
            self.fixed.insert(chord(first + i, first + (i + 1) % count));
        }
        self.boundary = self.uv.len();
    }
}

fn tessellate_face(
    face: Face<'_>,
    options: &MeshOptions,
    context: &GeometryContext,
    shared: &mut Shared,
) -> FaceResult<Vec<[u32; 3]>> {
    let surface = Surface::from_context(face.surface_entity().ok_or("missing surface")?, context)?;
    let flat = matches!(surface, Surface::Plane { .. });
    let mut rings = Vec::new();
    for ring in face.loops() {
        let point = ring_points(ring, options, context, shared)?;
        let xyz: Vec<_> = point
            .iter()
            .map(|point| shared.positions[*point as usize])
            .collect();
        let uv = to_uv(&surface, &xyz, shared.resolution, face.entity().sense());
        rings.push(Ring { point, xyz, uv });
    }
    if rings.is_empty() {
        return Err(FaceFailure::new(
            MeshIssueKind::UnsupportedTopology,
            "no trimming loops (untrimmed analytic face unsupported)",
        ));
    }
    let fit = options.tolerance.max(1e-4);
    for (uv, p) in rings.iter().flat_map(|ring| ring.uv.iter().zip(&ring.xyz)) {
        if !uv.iter().all(|v| v.is_finite()) || norm(sub(*p, surface.point_at(uv[0], uv[1]))) > fit
        {
            return Err(FaceFailure::new(
                MeshIssueKind::InvalidGeometry,
                "boundary does not lie on the face surface",
            ));
        }
    }
    // A boundary through a place where the surface comes to a point (the pole
    // of a sphere) gets two corners there. The parameter that means nothing
    // there has, at one, the value the boundary comes in with and, at the
    // other, the value it leaves with.
    for ring in &mut rings {
        for i in (0..ring.uv.len()).rev() {
            if let Some(axis) = collapsed(&surface, ring.uv[i], shared.resolution) {
                let count = ring.uv.len();
                let previous = ring.uv[(i + count - 1) % count][axis];
                let next = ring.uv[(i + 1) % count][axis];
                let mut coming = ring.uv[i];
                coming[axis] = previous;
                ring.uv[i][axis] = next;
                ring.uv.insert(i, coming);
                ring.xyz.insert(i, ring.xyz[i]);
                ring.point.insert(i, ring.point[i]);
            }
        }
    }
    let wrapping = !flat && rings.iter().any(|ring| wraps(&ring.uv));
    let wrapping_v = matches!(surface, Surface::Torus { .. })
        && rings
            .iter()
            .any(|r| wraps(&r.uv.iter().map(|p| [p[1], p[0]]).collect::<Vec<_>>()));
    let mut patch = Patch::new(shared.positions.len() as u32);
    let monotone = (rings.len() == 1 && !flat && !wrapping && !wrapping_v)
        .then(|| monotone_triangles(&rings[0].uv))
        .flatten();
    if wrapping || wrapping_v {
        periodic_strip(
            &surface,
            &rings,
            options,
            wrapping_v && !wrapping,
            &mut patch,
        )?;
    } else if let Some(triangles) = monotone {
        patch.boundary(rings[0].corners());
        patch.triangles = triangles;
    } else {
        polygon(&surface, &rings, &mut patch)?;
    }
    patch.orient();
    patch.restore_boundary();
    let steps = metric(&surface, &patch.uv);
    patch.flip(steps);
    patch.give_way(&surface, &shared.taken);
    if !flat {
        patch.refine(&surface, options)?;
    }
    // A triangle with two corners at one point (a pole) has no area in space.
    let mut triangles: Vec<_> = patch
        .triangles
        .iter()
        .map(|triangle| triangle.map(|corner| patch.point[corner]))
        .filter(|t| t[0] != t[1] && t[1] != t[2] && t[2] != t[0])
        .collect();
    // Seen from outside the solid, the boundary of a face goes round it
    // counter-clockwise, and so must the corners of each triangle. So the
    // triangle on a boundary chord runs along the chord the way the boundary
    // goes. This does not depend on which way the surface's own normal points,
    // which `sense` does: inside the spindle of a torus that normal is turned over.
    let [from, to] = patch.along;
    let side = |a: usize, b: usize| {
        let has = |t: &[usize; 3]| (0..3).any(|k| t[k] == a && t[(k + 1) % 3] == b);
        patch.triangles.iter().any(has)
    };
    let against = if side(to, from) {
        true
    } else if side(from, to) {
        false
    } else {
        !face.entity().sense()
    };
    if against {
        for triangle in &mut triangles {
            triangle.swap(1, 2);
        }
    }
    if triangles.is_empty() {
        return Err(FaceFailure::new(
            MeshIssueKind::Tessellation,
            "triangulation produced no nondegenerate triangles",
        ));
    }
    if triangles.len() > options.max_triangles.min(options.max_face_triangles)
        || patch.own.len() > options.max_vertices
    {
        return Err(FaceFailure::new(
            MeshIssueKind::ResourceLimit,
            "face exceeds mesh resource limit",
        ));
    }
    for triangle in &triangles {
        for k in 0..3 {
            let (a, b) = (triangle[k], triangle[(k + 1) % 3]);
            if a < patch.shared && b < patch.shared {
                shared.taken.insert(if a < b { (a, b) } else { (b, a) });
            }
        }
    }
    shared.positions.extend(patch.own);
    Ok(triangles)
}

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

/// Triangles of a face that does not go round its surface: the largest
/// boundary with the others as holes in it.
fn polygon(surface: &Surface, rings: &[Ring], patch: &mut Patch) -> FaceResult<()> {
    let mut order: Vec<_> = (0..rings.len()).collect();
    order.sort_by(|a, b| {
        signed_area(&rings[*b].uv)
            .abs()
            .total_cmp(&signed_area(&rings[*a].uv).abs())
    });
    let mean = |ring: &Ring, axis: usize| {
        ring.uv.iter().map(|p| p[axis]).sum::<f64>() / ring.uv.len() as f64
    };
    let outer = &rings[order[0]];
    let mut holes = Vec::new();
    for (n, &index) in order.iter().enumerate() {
        if n != 0 {
            holes.push(patch.uv.len());
        }
        let ring = &rings[index];
        let shifts: Uv = std::array::from_fn(|axis| {
            periods(surface)[axis].map_or(0., |period| {
                ((mean(outer, axis) - mean(ring, axis)) / period).round() * period
            })
        });
        patch.boundary(
            ring.corners()
                .map(|(uv, xyz, point)| ([uv[0] + shifts[0], uv[1] + shifts[1]], xyz, point)),
        );
    }
    patch.triangles = earcut(&patch.uv, &holes)?;
    Ok(())
}

/// Triangles of a closed line of corners with holes in it. `holes` gives the
/// place in `uv` where each hole starts; the outer line comes first.
fn earcut(uv: &[Uv], holes: &[usize]) -> FaceResult<Vec<[usize; 3]>> {
    let flattened: Vec<_> = uv.iter().flatten().copied().collect();
    let indices = earcutr::earcut(&flattened, holes, 2).map_err(|e| {
        FaceFailure::new(
            MeshIssueKind::Tessellation,
            format!("trim triangulation: {e}"),
        )
    })?;
    let triangles = indices.as_chunks::<3>().0.to_vec();
    let starts: Vec<_> = [0].into_iter().chain(holes.iter().copied()).collect();
    let areas: Vec<_> = (0..starts.len())
        .map(|i| {
            let end = starts.get(i + 1).copied().unwrap_or(uv.len());
            signed_area(&uv[starts[i]..end]).abs()
        })
        .collect();
    let expected = areas[0] - areas[1..].iter().sum::<f64>();
    let actual = triangles
        .iter()
        .map(|t| triangle_area(uv[t[0]], uv[t[1]], uv[t[2]]).abs())
        .sum::<f64>();
    if expected <= 0. || (actual - expected).abs() > 1e-8 * expected.max(1.) {
        return Err(FaceFailure::new(
            MeshIssueKind::Tessellation,
            format!(
                "triangulation does not preserve the trimmed region/holes: expected {expected}, actual {actual}"
            ),
        ));
    }
    Ok(triangles)
}

/// The band of a face that goes all the way round its surface, between two
/// boundaries that each go round once: a cylinder between two circles. The
/// band is cut open along a seam, so that it lies in the parameter plane as
/// one closed line of corners with the other boundaries as holes in it.
///
/// The seam runs from a corner of one boundary to a corner of the other. A
/// seam that crossed a boundary between two corners would put a point there
/// that the face on the other side of that boundary does not have.
fn periodic_strip(
    surface: &Surface,
    rings: &[Ring],
    options: &MeshOptions,
    swap: bool,
    patch: &mut Patch,
) -> FaceResult<()> {
    // Work in a plane where the parameter that goes round comes first.
    let turned = |p: Uv| if swap { [p[1], p[0]] } else { p };
    let one_turn_on = |p: Uv| [p[0] + TAU, p[1]];
    let plane: Vec<Vec<Uv>> = rings
        .iter()
        .map(|ring| ring.uv.iter().map(|p| turned(*p)).collect())
        .collect();
    let round: Vec<_> = (0..rings.len()).filter(|i| wraps(&plane[*i])).collect();
    if round.len() != 2 {
        return Err(FaceFailure::new(
            MeshIssueKind::UnsupportedTopology,
            "periodic face needs two wrapping boundary rings",
        ));
    }
    // A boundary that goes round, from one of its corners: each corner's place
    // in the ring, and where it is with the angle rising by one turn in all.
    let rising = |ring: usize| {
        let line = &plane[ring];
        (0..line.len())
            .map(|i| half_turn(line[(i + 1) % line.len()][0] - line[i][0]))
            .sum::<f64>()
            > 0.
    };
    let chain = |ring: usize, start: usize| {
        let line = &plane[ring];
        let count = line.len();
        let rising = rising(ring);
        let mut corners = vec![(start, line[start])];
        for k in 1..count {
            let i = (start + if rising { k } else { count - k }) % count;
            let previous = corners[k - 1].1[0];
            corners.push((i, [previous + half_turn(line[i][0] - previous), line[i][1]]));
        }
        corners
    };
    // Seams to try: each corner of one boundary with the two corners of the
    // other that are next to it in angle, one each side. The nearest pairs
    // come first, so a seam straight along the surface is taken when there
    // is one.
    let mut angles: Vec<_> = plane[round[1]]
        .iter()
        .enumerate()
        .map(|(j, p)| (p[0].rem_euclid(TAU), j))
        .collect();
    angles.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut seams: Vec<_> = plane[round[0]]
        .iter()
        .enumerate()
        .flat_map(|(i, p)| {
            let after = angles.partition_point(|a| a.0 < p[0].rem_euclid(TAU));
            [after + angles.len() - 1, after]
                .map(|k| angles[k % angles.len()])
                .map(|(angle, j)| (half_turn(angle - p[0]).abs(), i, j))
        })
        .collect();
    seams.sort_by(|a, b| a.0.total_cmp(&b.0));
    'seam: for (_, i, j) in seams {
        let mut lower = (&rings[round[0]], chain(round[0], i), rising(round[0]));
        let mut upper = (&rings[round[1]], chain(round[1], j), rising(round[1]));
        let apart = upper.1[0].1[0] - lower.1[0].1[0];
        for corner in &mut upper.1 {
            corner.1[0] += half_turn(apart) - apart;
        }
        if lower.1[0].1[1] > upper.1[0].1[1] {
            std::mem::swap(&mut lower, &mut upper);
        }
        // Each chain ends where it began, one turn on.
        for chain in [&mut lower.1, &mut upper.1] {
            let (place, uv) = chain[0];
            chain.push((place, one_turn_on(uv)));
        }
        let (a, b) = (lower.1[0], upper.1[0]);
        if lower.0.point[a.0] == upper.0.point[b.0] {
            continue;
        }
        let mut cut = Vec::new();
        seam_points(
            surface,
            swap,
            (a.1, lower.0.xyz[a.0]),
            (b.1, upper.0.xyz[b.0]),
            options.tolerance,
            0,
            &mut cut,
        )?;
        // The seam, and the seam again one turn on, may meet the two
        // boundaries only at their own ends.
        let seam = [[a.1, b.1], [one_turn_on(a.1), one_turn_on(b.1)]];
        let meets_seam = |p: Uv, q: Uv| seam.iter().any(|s| chords_meet(s[0], s[1], p, q));
        if [&lower.1, &upper.1].iter().any(|chain| {
            chain
                .windows(2)
                .any(|pair| meets_seam(pair[0].1, pair[1].1))
        }) {
            continue;
        }
        let mut outer: Vec<Uv> = lower.1.iter().map(|corner| corner.1).collect();
        outer.extend(cut.iter().map(|(uv, _)| one_turn_on(*uv)));
        outer.extend(upper.1.iter().rev().map(|corner| corner.1));
        outer.extend(cut.iter().rev().map(|(uv, _)| *uv));
        // Every other boundary is a hole, and lies whole between the seam and
        // the seam one turn on.
        let mut holes = Vec::new();
        for (index, (ring, line)) in rings.iter().zip(&plane).enumerate() {
            if round.contains(&index) {
                continue;
            }
            let near = -((line[0][0] - a.1[0] - PI) / TAU).round() * TAU;
            let Some(shift) = [near, near + TAU, near - TAU]
                .into_iter()
                .find(|shift| inside(&outer, [line[0][0] + shift, line[0][1]]))
            else {
                continue 'seam;
            };
            let moved: Vec<Uv> = line.iter().map(|p| [p[0] + shift, p[1]]).collect();
            if moved.iter().any(|p| !inside(&outer, *p))
                || (0..moved.len()).any(|k| meets_seam(moved[k], moved[(k + 1) % moved.len()]))
            {
                continue 'seam;
            }
            holes.push((ring, moved));
        }
        if outer.len() + holes.iter().map(|hole| hole.1.len()).sum::<usize>()
            > options.max_face_vertices
        {
            return Err(FaceFailure::new(
                MeshIssueKind::ResourceLimit,
                "periodic boundary vertex limit",
            ));
        }
        let mut trial = Patch::new(patch.shared);
        let cut: Vec<_> = cut
            .into_iter()
            .map(|(uv, xyz)| (uv, xyz, trial.own_point(xyz)))
            .collect();
        let along = |(ring, chain, _): &(&Ring, Vec<(usize, Uv)>, bool)| -> Vec<_> {
            chain
                .iter()
                .map(|(i, uv)| (turned(*uv), ring.xyz[*i], ring.point[*i]))
                .collect()
        };
        trial.boundary(
            along(&lower)
                .into_iter()
                .chain(cut.iter().map(|c| (turned(one_turn_on(c.0)), c.1, c.2)))
                .chain(along(&upper).into_iter().rev())
                .chain(cut.iter().rev().map(|c| (turned(c.0), c.1, c.2))),
        );
        let mut starts = Vec::new();
        for (ring, moved) in &holes {
            starts.push(trial.uv.len());
            trial.boundary(
                ring.corners()
                    .zip(moved)
                    .map(|((_, xyz, point), uv)| (turned(*uv), xyz, point)),
            );
        }
        let flat: Vec<Uv> = trial.uv.iter().map(|p| turned(*p)).collect();
        let Some(triangles) = holes
            .is_empty()
            .then(|| monotone_triangles(&flat))
            .flatten()
            .or_else(|| earcut(&flat, &starts).ok())
        else {
            continue;
        };
        // The lower chain comes first. It goes the way its boundary goes if
        // that boundary's angle rises.
        trial.along = if lower.2 { [0, 1] } else { [1, 0] };
        trial.triangles = triangles;
        *patch = trial;
        return Ok(());
    }
    Err(FaceFailure::new(
        MeshIssueKind::UnsupportedTopology,
        "no unambiguous periodic seam",
    ))
}

/// The points that cut the straight line from `a` to `b` in the parameter
/// plane into chords within tolerance of the surface. They become boundary
/// chords of the band, one each side of the seam, and are not split again.
fn seam_points(
    surface: &Surface,
    swap: bool,
    a: (Uv, Vec3),
    b: (Uv, Vec3),
    tolerance: f64,
    depth: usize,
    cut: &mut Vec<(Uv, Vec3)>,
) -> FaceResult<()> {
    let uv = [(a.0[0] + b.0[0]) * 0.5, (a.0[1] + b.0[1]) * 0.5];
    let xyz = if swap {
        surface.point_at(uv[1], uv[0])
    } else {
        surface.point_at(uv[0], uv[1])
    };
    let error = norm(sub(xyz, scale(add(a.1, b.1), 0.5)));
    if !error.is_finite() {
        return Err(FaceFailure::new(
            MeshIssueKind::InvalidGeometry,
            "nonfinite surface tessellation",
        ));
    }
    if error > tolerance && depth < 12 {
        seam_points(surface, swap, a, (uv, xyz), tolerance, depth + 1, cut)?;
        cut.push((uv, xyz));
        seam_points(surface, swap, (uv, xyz), b, tolerance, depth + 1, cut)?;
    }
    Ok(())
}

/// Whether two chords in the plane have a point in common, other than an end
/// that is an end of both.
fn chords_meet(p: Uv, q: Uv, r: Uv, s: Uv) -> bool {
    let common = [(p, q, r, s), (p, q, s, r), (q, p, r, s), (q, p, s, r)]
        .into_iter()
        .find(|(end, _, other, _)| end == other);
    if let Some((end, x, _, y)) = common {
        // From their common end they either part or run along each other.
        let (x, y) = (
            [x[0] - end[0], x[1] - end[1]],
            [y[0] - end[0], y[1] - end[1]],
        );
        return x[0] * y[1] - x[1] * y[0] == 0. && x[0] * y[0] + x[1] * y[1] > 0.;
    }
    let apart = |x: f64, y: f64| (x > 0. && y < 0.) || (x < 0. && y > 0.);
    let on = |a: Uv, b: Uv, c: Uv, side: f64| {
        side == 0.
            && (0..2).all(|axis| c[axis] >= a[axis].min(b[axis]) && c[axis] <= a[axis].max(b[axis]))
    };
    let sides = [
        triangle_area(p, q, r),
        triangle_area(p, q, s),
        triangle_area(r, s, p),
        triangle_area(r, s, q),
    ];
    (apart(sides[0], sides[1]) && apart(sides[2], sides[3]))
        || on(p, q, r, sides[0])
        || on(p, q, s, sides[1])
        || on(r, s, p, sides[2])
        || on(r, s, q, sides[3])
}

/// Whether a place in the plane is inside a closed line of corners.
fn inside(line: &[Uv], p: Uv) -> bool {
    let mut inside = false;
    for i in 0..line.len() {
        let (a, b) = (line[i], line[(i + 1) % line.len()]);
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < a[0] + (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1])
        {
            inside = !inside;
        }
    }
    inside
}

/// How long a step of one in each parameter is in space, at the middle of a
/// face's corners. Angles in the parameter plane are measured with it.
fn metric(surface: &Surface, uv: &[Uv]) -> Uv {
    if matches!(surface, Surface::Plane { .. }) || uv.is_empty() {
        return [1.; 2];
    }
    let [u, v]: Uv =
        std::array::from_fn(|axis| uv.iter().map(|p| p[axis]).sum::<f64>() / uv.len() as f64);
    let du = norm(sub(
        surface.point_at(u + 1e-6, v),
        surface.point_at(u - 1e-6, v),
    )) / 2e-6;
    let dv = norm(sub(
        surface.point_at(u, v + 1e-6),
        surface.point_at(u, v - 1e-6),
    )) / 2e-6;
    [du, dv].map(|step| {
        if step.is_finite() && step > 0. {
            step
        } else {
            1.
        }
    })
}

impl Patch {
    /// Put the corners of every triangle in the order that goes counter-clockwise
    /// in the parameter plane. A triangulator gives all its triangles the same way round.
    fn orient(&mut self) {
        let area: f64 = self
            .triangles
            .iter()
            .map(|t| triangle_area(self.uv[t[0]], self.uv[t[1]], self.uv[t[2]]))
            .sum();
        if area < 0. {
            for triangle in &mut self.triangles {
                triangle.swap(1, 2);
            }
        }
    }

    /// The triangulator can run one side of a triangle straight past a corner
    /// that lies exactly on it: a boundary corner on one line with its two
    /// neighbours, which it leaves out, or the corners of two holes that have
    /// sides on one line. The triangles on the other side of the line end at
    /// that corner, so the sides do not match. Cut such a triangle at each
    /// corner on its side.
    fn restore_boundary(&mut self) {
        for _ in 0..8 {
            let sides: BTreeSet<_> = self
                .triangles
                .iter()
                .flat_map(|t| (0..3).map(|k| (t[k], t[(k + 1) % 3])))
                .collect();
            // A side or a boundary chord that has no triangle on its other
            // side where it needs one.
            let lone =
                |a: usize, b: usize| !sides.contains(&(b, a)) && !self.fixed.contains(&chord(a, b));
            let bare =
                |&&(a, b): &&(usize, usize)| !sides.contains(&(a, b)) && !sides.contains(&(b, a));
            let ends: BTreeSet<_> = sides
                .iter()
                .filter(|(a, b)| lone(*a, *b))
                .chain(self.fixed.iter().filter(bare))
                .flat_map(|&(a, b)| [a, b])
                .collect();
            let mut triangles = Vec::with_capacity(self.triangles.len() + ends.len());
            for &t in &self.triangles {
                let past = (0..3).find_map(|k| {
                    let (from, to) = (t[k], t[(k + 1) % 3]);
                    let mut between: Vec<_> = ends
                        .iter()
                        .filter(|_| lone(from, to))
                        .filter_map(|&corner| Some((self.on_side(from, to, corner)?, corner)))
                        .collect();
                    between.sort_by(|a, b| a.0.total_cmp(&b.0));
                    (!between.is_empty()).then_some((from, between, to, t[(k + 2) % 3]))
                });
                let Some((from, between, to, apex)) = past else {
                    triangles.push(t);
                    continue;
                };
                let line: Vec<_> = [from]
                    .into_iter()
                    .chain(between.into_iter().map(|(_, corner)| corner))
                    .chain([to])
                    .collect();
                triangles.extend(line.windows(2).map(|pair| [pair[0], pair[1], apex]));
            }
            if triangles.len() == self.triangles.len() {
                return;
            }
            self.triangles = triangles;
        }
    }
    /// How far along the side from `from` to `to` a corner is, as a fraction
    /// of the side, if it lies on the side between the two.
    fn on_side(&self, from: usize, to: usize, corner: usize) -> Option<f64> {
        let side: Uv = std::array::from_fn(|i| self.uv[to][i] - self.uv[from][i]);
        let reach: Uv = std::array::from_fn(|i| self.uv[corner][i] - self.uv[from][i]);
        let length = side[0] * side[0] + side[1] * side[1];
        let along = (side[0] * reach[0] + side[1] * reach[1]) / length;
        let off = (side[0] * reach[1] - side[1] * reach[0]).abs();
        (corner != from
            && corner != to
            && off <= 1e-12 * length
            && along > 1e-12
            && along < 1. - 1e-12)
            .then_some(along)
    }

    /// Two triangles on a common side make a four-sided shape, and could have
    /// its other diagonal as their common side instead. Take the diagonal that
    /// leaves the larger smallest angle, everywhere. This leaves no flat
    /// triangle (three corners on one line, which a triangulator makes along a
    /// boundary) where one can be avoided, and no fan of long thin triangles
    /// where the corners allow better. Boundary chords stay.
    fn flip(&mut self, scale: Uv) {
        const NONE: usize = usize::MAX;
        // The triangle across each side of each triangle. Side `k` of a
        // triangle goes from its corner `k` to the next.
        let mut owner = BTreeMap::new();
        for (index, t) in self.triangles.iter().enumerate() {
            for k in 0..3 {
                if owner.insert((t[k], t[(k + 1) % 3]), index).is_some() {
                    return;
                }
            }
        }
        let mut across = vec![[NONE; 3]; self.triangles.len()];
        let mut queue = VecDeque::new();
        for (&(a, b), &index) in &owner {
            let other = owner.get(&(b, a)).copied();
            // A side has a triangle across it unless it is a boundary chord.
            if other.is_some() == self.fixed.contains(&chord(a, b)) {
                return;
            }
            let k = (0..3).find(|k| self.triangles[index][*k] == a).unwrap();
            across[index][k] = other.unwrap_or(NONE);
            if a < b && other.is_some() {
                queue.push_back((a, b, index));
            }
        }
        // The sine of a triangle's smallest angle. Zero if the triangle is
        // flat, less if its corners go clockwise.
        let shape = |corners: [usize; 3]| {
            let [a, b, c] = corners.map(|c| [self.uv[c][0] * scale[0], self.uv[c][1] * scale[1]]);
            let mut sides = [[a, b], [b, c], [c, a]].map(|[p, q]| (q[0] - p[0]).hypot(q[1] - p[1]));
            sides.sort_by(f64::total_cmp);
            let longest = sides[1] * sides[2];
            if longest > 0. {
                2. * triangle_area(a, b, c) / longest
            } else {
                0.
            }
        };
        // Undoing a fan along a straight boundary takes a number of flips that
        // grows with the square of its length.
        let count = self.triangles.len();
        let mut budget = count.saturating_mul(count).clamp(64, 4_000_000);
        while let Some((a, b, i)) = queue.pop_front() {
            // The side from a to b, if triangle i still has it.
            let Some(k) =
                (0..3).find(|k| self.triangles[i][*k] == a && self.triangles[i][(k + 1) % 3] == b)
            else {
                continue;
            };
            let j = across[i][k];
            if j == NONE {
                continue;
            }
            let Some(l) =
                (0..3).find(|l| self.triangles[j][*l] == b && self.triangles[j][(l + 1) % 3] == a)
            else {
                continue;
            };
            let (r, s) = (
                self.triangles[i][(k + 2) % 3],
                self.triangles[j][(l + 2) % 3],
            );
            let now = shape([a, b, r]).min(shape([b, a, s]));
            let flipped = shape([a, s, r]).min(shape([s, b, r]));
            if flipped <= now.max(0.) * (1. + 1e-9) + 1e-12 {
                continue;
            }
            // Triangle i becomes (a, s, r) and triangle j becomes (s, b, r).
            let (beyond_br, beyond_ra) = (across[i][(k + 1) % 3], across[i][(k + 2) % 3]);
            let (beyond_as, beyond_sb) = (across[j][(l + 1) % 3], across[j][(l + 2) % 3]);
            self.triangles[i] = [a, s, r];
            self.triangles[j] = [s, b, r];
            across[i] = [beyond_as, j, beyond_ra];
            across[j] = [beyond_sb, beyond_br, i];
            for (beyond, was, is) in [(beyond_as, j, i), (beyond_br, i, j)] {
                if beyond != NONE {
                    for other in &mut across[beyond] {
                        if *other == was {
                            *other = is;
                        }
                    }
                }
            }
            for (from, to, index) in [(a, s, i), (s, b, j), (b, r, j), (r, a, i)] {
                queue.push_back((from, to, index));
            }
            budget -= 1;
            if budget == 0 {
                break;
            }
        }
    }

    /// Two faces can each have a side between the same two points of the
    /// boundary they share: each sees the boundary bend towards itself there.
    /// The solid would then have four triangles on one line. The face that
    /// comes second gives way and splits its side at a point of its own. So
    /// does a face with two sides between the same two points, across a seam.
    fn give_way(&mut self, surface: &Surface, taken: &BTreeSet<(u32, u32)>) {
        loop {
            // Each side between two boundary corners, by its points in the
            // direction its triangle goes round, with its corners.
            let mut sides = BTreeMap::new();
            let mut clashing = BTreeSet::new();
            for t in &self.triangles {
                let points = t.map(|corner| self.point[corner]);
                if points[0] == points[1] || points[1] == points[2] || points[2] == points[0] {
                    continue;
                }
                for k in 0..3 {
                    let (a, b) = (t[k], t[(k + 1) % 3]);
                    if a >= self.boundary || b >= self.boundary {
                        continue;
                    }
                    let (p, q) = (points[k], points[(k + 1) % 3]);
                    let same = sides.insert((p, q), chord(a, b));
                    if same.is_some() || taken.contains(&if p < q { (p, q) } else { (q, p) }) {
                        let free = [Some(chord(a, b)), same]
                            .into_iter()
                            .flatten()
                            .find(|side| !self.fixed.contains(side));
                        clashing.extend(free);
                    }
                }
            }
            if clashing.is_empty() {
                return;
            }
            let split = clashing
                .into_iter()
                .map(|(a, b)| {
                    let uv: Uv = std::array::from_fn(|i| (self.uv[a][i] + self.uv[b][i]) * 0.5);
                    let xyz = surface.point_at(uv[0], uv[1]);
                    let point = self.own_point(xyz);
                    ((a, b), self.corner(uv, xyz, point))
                })
                .collect();
            self.split(&split, &BTreeMap::new(), &mut Vec::new());
        }
    }

    /// Put a corner in the middle of each side in `split`, and in the middle of
    /// each triangle in `centres`, and make the smaller triangles. `quiet`
    /// keeps its entry for a triangle that stays whole.
    fn split(
        &mut self,
        split: &BTreeMap<(usize, usize), usize>,
        centres: &BTreeMap<usize, usize>,
        quiet: &mut Vec<bool>,
    ) {
        let mut triangles = Vec::with_capacity(self.triangles.len() * 2);
        let mut still_quiet = Vec::with_capacity(self.triangles.len() * 2);
        for (index, t) in self.triangles.iter().enumerate() {
            let middle: [_; 3] =
                std::array::from_fn(|k| split.get(&chord(t[k], t[(k + 1) % 3])).copied());
            // Name the corners so that the split sides come first.
            let first = (0..3)
                .find(|k| middle[*k].is_some() && middle[(k + 2) % 3].is_none())
                .unwrap_or(0);
            let [a, b, c]: [_; 3] = std::array::from_fn(|k| t[(first + k) % 3]);
            let far = |p: usize, q: usize| norm(sub(self.xyz[p], self.xyz[q]));
            match std::array::from_fn(|k| middle[(first + k) % 3]) {
                [None, None, None] => match centres.get(&index) {
                    Some(&centre) => {
                        triangles.extend([[a, b, centre], [b, c, centre], [c, a, centre]])
                    }
                    None => {
                        triangles.push(*t);
                        still_quiet.push(quiet.get(index).copied().unwrap_or(false));
                    }
                },
                [Some(ab), None, None] => triangles.extend([[a, ab, c], [ab, b, c]]),
                // The four-sided part takes its shorter diagonal.
                [Some(ab), Some(bc), None] if far(a, bc) < far(ab, c) => {
                    triangles.extend([[ab, b, bc], [a, ab, bc], [a, bc, c]])
                }
                [Some(ab), Some(bc), None] => {
                    triangles.extend([[ab, b, bc], [ab, bc, c], [a, ab, c]])
                }
                [Some(ab), Some(bc), Some(ca)] => {
                    triangles.extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]])
                }
                _ => unreachable!("a split side is named first"),
            }
            still_quiet.resize(triangles.len(), false);
        }
        self.triangles = triangles;
        *quiet = still_quiet;
    }

    /// Split the sides of triangles until every triangle is within tolerance
    /// of the surface.
    ///
    /// A side is judged from its own two ends, and the two triangles on it are
    /// split at the same point in the same pass. So no triangle is left with
    /// a neighbour's point in the middle of one of its sides.
    ///
    /// Boundary chords are not split: the face on the other side would not
    /// have the point. Edge sampling has already put them within tolerance.
    fn refine(&mut self, surface: &Surface, options: &MeshOptions) -> FaceResult<()> {
        let tolerance = options.tolerance;
        let nonfinite = || {
            FaceFailure::new(
                MeshIssueKind::InvalidGeometry,
                "nonfinite surface tessellation",
            )
        };
        // For each side looked at: how far the surface point at the middle of
        // its parameters is from the middle of the side, and that point.
        let mut middles = BTreeMap::<(usize, usize), (f64, Uv, Vec3)>::new();
        // A triangle is quiet when it asks for no split itself. A split that
        // a neighbour asks for on a common side still splits it.
        let mut quiet = vec![false; self.triangles.len()];
        let mut pass = 0;
        loop {
            let mut split = BTreeMap::<(usize, usize), usize>::new();
            let mut centres = BTreeMap::<usize, usize>::new();
            for (index, quiet) in quiet.iter_mut().enumerate() {
                if *quiet {
                    continue;
                }
                let t = self.triangles[index];
                let sides: [_; 3] = std::array::from_fn(|k| chord(t[k], t[(k + 1) % 3]));
                let free = sides.map(|side| !self.fixed.contains(&side));
                let mut errors = [0.; 3];
                for k in 0..3 {
                    let (a, b) = sides[k];
                    errors[k] = middles
                        .entry(sides[k])
                        .or_insert_with(|| {
                            let uv: Uv =
                                std::array::from_fn(|i| (self.uv[a][i] + self.uv[b][i]) * 0.5);
                            let xyz = surface.point_at(uv[0], uv[1]);
                            let chord_middle = scale(add(self.xyz[a], self.xyz[b]), 0.5);
                            (norm(sub(xyz, chord_middle)), uv, xyz)
                        })
                        .0;
                    if !errors[k].is_finite() {
                        return Err(nonfinite());
                    }
                }
                // A side too far from the surface is split.
                for k in 0..3 {
                    if free[k] && errors[k] > tolerance && !split.contains_key(&sides[k]) {
                        let (_, uv, xyz) = middles[&sides[k]];
                        let point = self.own_point(xyz);
                        split.insert(sides[k], self.corner(uv, xyz, point));
                    }
                }
                if sides.iter().any(|side| split.contains_key(side)) {
                    continue;
                }
                *quiet = true;
                let points = t.map(|corner| self.point[corner]);
                if points[0] == points[1] || points[1] == points[2] || points[2] == points[0] {
                    continue;
                }
                // The sides are near enough. The middle of the triangle can
                // still be too far: then its worst side is split.
                let uv: Uv =
                    std::array::from_fn(|i| t.iter().map(|c| self.uv[*c][i]).sum::<f64>() / 3.);
                let xyz = surface.point_at(uv[0], uv[1]);
                let flat = scale(
                    t.iter().fold([0.; 3], |sum, c| add(sum, self.xyz[*c])),
                    1. / 3.,
                );
                let error = norm(sub(flat, xyz));
                if !error.is_finite() {
                    return Err(nonfinite());
                }
                if error <= tolerance {
                    continue;
                }
                let length = |k: usize| norm(sub(self.xyz[sides[k].0], self.xyz[sides[k].1]));
                let worst = (0..3).filter(|k| free[*k]).max_by(|a, b| {
                    errors[*a]
                        .total_cmp(&errors[*b])
                        .then(length(*a).total_cmp(&length(*b)))
                });
                match worst {
                    // Three boundary chords: only a point in the middle helps.
                    None => {
                        let point = self.own_point(xyz);
                        centres.insert(index, self.corner(uv, xyz, point));
                    }
                    // A boundary chord that could not be made finer is to
                    // blame, and the other sides are already short.
                    Some(k)
                        if errors[k] * 8. <= tolerance
                            && (0..3).any(|k| !free[k] && errors[k] > tolerance) => {}
                    Some(k) => {
                        let (_, uv, xyz) = middles[&sides[k]];
                        let point = self.own_point(xyz);
                        split.insert(sides[k], self.corner(uv, xyz, point));
                    }
                }
            }
            if split.is_empty() && centres.is_empty() {
                return Ok(());
            }
            if pass >= options.max_refinement_depth * 3 {
                return Err(FaceFailure::new(
                    MeshIssueKind::ResourceLimit,
                    "curved face exceeds tolerance at refinement limit",
                ));
            }
            self.split(&split, &centres, &mut quiet);
            if self.triangles.len() > options.max_triangles.min(options.max_face_triangles)
                || self.own.len() > options.max_vertices
            {
                return Err(FaceFailure::new(
                    MeshIssueKind::ResourceLimit,
                    "face exceeds mesh resource limit",
                ));
            }
            pass += 1;
        }
    }
}

fn sample_spline(
    curve: &Curve,
    a: f64,
    b: f64,
    depth: usize,
    options: &MeshOptions,
    samples: &mut Vec<(f64, Vec3)>,
) -> FaceResult<()> {
    let p = curve.point_at(a);
    let q = curve.point_at(b);
    let error = [0.25, 0.5, 0.75]
        .into_iter()
        .map(|f| {
            norm(sub(
                curve.point_at(a + (b - a) * f),
                add(scale(p, 1. - f), scale(q, f)),
            ))
        })
        .fold(0., f64::max);
    if error > options.tolerance * 0.25 {
        if depth >= 20 || samples.len() >= options.max_edge_segments {
            return Err(FaceFailure::new(
                MeshIssueKind::ResourceLimit,
                "spline edge sampling limit",
            ));
        }
        let m = (a + b) * 0.5;
        sample_spline(curve, a, m, depth + 1, options, samples)?;
        sample_spline(curve, m, b, depth + 1, options, samples)?;
    } else {
        if samples.len() > options.max_edge_segments {
            return Err(FaceFailure::new(
                MeshIssueKind::ResourceLimit,
                "spline edge sampling limit",
            ));
        }
        samples.push((b, q));
    }
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
    fn real(value: f64) -> Token {
        token(tag::DOUBLE, Value::Real(value))
    }
    fn empty_model() -> Model {
        Model {
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
        }
    }
    fn plane_model(hole: bool) -> Model {
        plane_model_facing(hole, false)
    }
    /// A square in the plane z = 0, with a square hole if asked. Its boundary
    /// goes counter-clockwise seen from +z, or clockwise if `down`.
    fn plane_model_facing(hole: bool, down: bool) -> Model {
        let mut model = empty_model();
        entity(&mut model, "body", "body", &[-1, -1, 1, -1, -1], vec![]);
        entity(&mut model, "lump", "lump", &[-1, -1, -1, 2], vec![]);
        entity(&mut model, "shell", "shell", &[-1, -1, -1, -1, 3], vec![]);
        entity(
            &mut model,
            "face",
            "face",
            &[-1, -1, -1, -1, -1, -1, 4],
            if down {
                vec![token(tag::FALSE, Value::Empty)]
            } else {
                vec![]
            },
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
            let mut points = [
                [-size, -size, 0.],
                [size, -size, 0.],
                [size, size, 0.],
                [-size, size, 0.],
            ];
            if down {
                points.reverse();
            }
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
                    vec![real(0.), real(length)],
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

    /// A solid made by hand: vertices, the edges between them, and faces that
    /// each give their boundaries as edges with the direction they are gone
    /// along. Seen from outside, a boundary goes counter-clockwise round its face.
    struct Solid {
        model: Model,
        places: BTreeMap<usize, Vec3>,
        lines: BTreeMap<(usize, usize), usize>,
        last_face: Option<usize>,
    }
    impl Solid {
        fn new() -> Self {
            let mut model = empty_model();
            entity(&mut model, "body", "body", &[-1, -1, 1, -1, -1], vec![]);
            entity(&mut model, "lump", "lump", &[-1, -1, -1, 2], vec![]);
            entity(&mut model, "shell", "shell", &[-1; 5], vec![]);
            Self {
                model,
                places: BTreeMap::new(),
                lines: BTreeMap::new(),
                last_face: None,
            }
        }
        fn vertex(&mut self, place: Vec3) -> usize {
            let point = entity(
                &mut self.model,
                "point",
                "point",
                &[],
                vec![vector(tag::POSITION, place)],
            );
            let vertex = entity(
                &mut self.model,
                "vertex",
                "vertex",
                &[-1, -1, -1, point as i64],
                vec![],
            );
            self.places.insert(vertex, place);
            vertex
        }
        fn edge(&mut self, from: usize, to: usize, curve: usize, range: [f64; 2]) -> usize {
            entity(
                &mut self.model,
                "edge",
                "edge",
                &[-1, -1, from as i64, to as i64, -1, curve as i64],
                range.map(real).into(),
            )
        }
        /// The straight edge between two vertices, and whether it goes from
        /// the first to the second. It is made the first time it is asked for.
        fn line(&mut self, from: usize, to: usize) -> (usize, bool) {
            let (a, b) = (from.min(to), from.max(to));
            if !self.lines.contains_key(&(a, b)) {
                let delta = sub(self.places[&b], self.places[&a]);
                let curve = entity(
                    &mut self.model,
                    "straight",
                    "curve",
                    &[],
                    vec![
                        vector(tag::POSITION, self.places[&a]),
                        vector(tag::DIRECTION, scale(delta, 1. / norm(delta))),
                    ],
                );
                let edge = self.edge(a, b, curve, [0., norm(delta)]);
                self.lines.insert((a, b), edge);
            }
            (self.lines[&(a, b)], from == a)
        }
        /// An arc of a circle about `centre`. It starts at `centre + major`
        /// when its parameter is 0 and turns counter-clockwise about `normal`.
        fn arc(
            &mut self,
            from: usize,
            to: usize,
            centre: Vec3,
            normal: Vec3,
            major: Vec3,
            range: [f64; 2],
        ) -> usize {
            let curve = entity(
                &mut self.model,
                "ellipse",
                "curve",
                &[],
                vec![
                    vector(tag::POSITION, centre),
                    vector(tag::DIRECTION, normal),
                    vector(tag::DIRECTION, major),
                    real(1.),
                ],
            );
            self.edge(from, to, curve, range)
        }
        fn surface(&mut self, name: &str, fields: Vec<Token>) -> usize {
            entity(&mut self.model, name, "surface", &[], fields)
        }
        fn plane(&mut self, origin: Vec3, normal: Vec3, u_dir: Vec3) -> usize {
            self.surface(
                "plane",
                vec![
                    vector(tag::POSITION, origin),
                    vector(tag::DIRECTION, normal),
                    vector(tag::DIRECTION, u_dir),
                ],
            )
        }
        /// A face on a surface. Each boundary is a list of edges, each with
        /// whether the boundary goes along it from its start to its end.
        fn face(&mut self, surface: usize, boundaries: &[&[(usize, bool)]]) {
            let face = entity(
                &mut self.model,
                "face",
                "face",
                &[-1, -1, -1, -1, -1, -1, surface as i64],
                vec![],
            );
            match self.last_face.replace(face) {
                Some(previous) => link(&mut self.model, previous, 2, face),
                None => link(&mut self.model, 2, 4, face),
            }
            let mut previous_loop = None;
            for boundary in boundaries {
                let ring = entity(&mut self.model, "loop", "loop", &[-1; 5], vec![]);
                match previous_loop.replace(ring) {
                    Some(previous) => link(&mut self.model, previous, 2, ring),
                    None => link(&mut self.model, face, 3, ring),
                }
                let coedges: Vec<_> = boundary
                    .iter()
                    .map(|&(edge, forward)| {
                        entity(
                            &mut self.model,
                            "coedge",
                            "coedge",
                            &[-1, -1, -1, -1, -1, edge as i64],
                            if forward {
                                vec![]
                            } else {
                                vec![token(tag::FALSE, Value::Empty)]
                            },
                        )
                    })
                    .collect();
                for i in 0..coedges.len() {
                    link(
                        &mut self.model,
                        coedges[i],
                        2,
                        coedges[(i + 1) % coedges.len()],
                    );
                }
                link(&mut self.model, ring, 3, coedges[0]);
            }
        }
        /// A flat face with straight sides, by its corners in order.
        fn flat(&mut self, normal: Vec3, u_dir: Vec3, corners: &[usize]) {
            let plane = self.plane(self.places[&corners[0]], normal, u_dir);
            let boundary: Vec<_> = (0..corners.len())
                .map(|i| self.line(corners[i], corners[(i + 1) % corners.len()]))
                .collect();
            self.face(plane, &[&boundary]);
        }
    }

    /// Mesh a solid of one body. No face may be skipped and the mesh must be
    /// closed. The volume inside it, positive when the triangles face outwards.
    fn closed_volume(model: &Model) -> f64 {
        let report = tessellate(model, &Default::default()).unwrap();
        assert_eq!(report.faces_skipped, 0, "{:?}", report.issues);
        let bodies = report.split_by_body().unwrap();
        assert_eq!(bodies.len(), 1);
        assert_eq!(bodies[0].edge_counts(), EdgeCounts::default());
        bodies[0]
            .triangles
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| bodies[0].positions[i as usize]);
                dot(a, cross(b, c)) / 6.
            })
            .sum()
    }
    /// The volume of a mesh at a chord tolerance is a little under the true
    /// volume of a solid that bulges outwards.
    fn assert_just_under(volume: f64, exact: f64) {
        assert!(
            volume > exact * 0.97 && volume <= exact,
            "volume {volume}, exact {exact}"
        );
    }

    #[test]
    fn box_is_closed() {
        let mut solid = Solid::new();
        let corners: Vec<_> = (0..8)
            .map(|i| {
                solid.vertex([
                    (i & 1) as f64 * 2.,
                    (i >> 1 & 1) as f64 * 3.,
                    (i >> 2) as f64 * 4.,
                ])
            })
            .collect();
        for (normal, u_dir, quad) in [
            ([0., 0., -1.], [1., 0., 0.], [0, 2, 3, 1]),
            ([0., 0., 1.], [1., 0., 0.], [4, 5, 7, 6]),
            ([0., -1., 0.], [1., 0., 0.], [0, 1, 5, 4]),
            ([0., 1., 0.], [1., 0., 0.], [2, 6, 7, 3]),
            ([-1., 0., 0.], [0., 1., 0.], [0, 4, 6, 2]),
            ([1., 0., 0.], [0., 1., 0.], [1, 3, 7, 5]),
        ] {
            solid.flat(normal, u_dir, &quad.map(|i| corners[i]));
        }
        assert!((closed_volume(&solid.model) - 24.).abs() < 1e-12);
    }

    /// A round bar along z from 0 to `height`. The one vertex of its top
    /// circle is `turned` round the axis from the one vertex of its bottom
    /// circle, so the seam of the side cannot follow the surface straight up.
    fn bar(radius: f64, height: f64, turned: f64) -> Model {
        let mut solid = Solid::new();
        let top_major = [radius * turned.cos(), radius * turned.sin(), 0.];
        let low = solid.vertex([radius, 0., 0.]);
        let high = solid.vertex(add(top_major, [0., 0., height]));
        let up = [0., 0., 1.];
        let bottom = solid.arc(low, low, [0.; 3], up, [radius, 0., 0.], [0., TAU]);
        let top = solid.arc(high, high, [0., 0., height], up, top_major, [0., TAU]);
        let side = solid.surface(
            "cone",
            vec![
                vector(tag::POSITION, [0.; 3]),
                vector(tag::DIRECTION, up),
                vector(tag::DIRECTION, [radius, 0., 0.]),
                real(1.),
                real(0.),
                real(1.),
            ],
        );
        solid.face(side, &[&[(bottom, true)], &[(top, false)]]);
        let floor = solid.plane([0.; 3], [0., 0., -1.], [1., 0., 0.]);
        solid.face(floor, &[&[(bottom, false)]]);
        let roof = solid.plane([0., 0., height], up, [1., 0., 0.]);
        solid.face(roof, &[&[(top, true)]]);
        solid.model
    }

    #[test]
    fn round_bar_is_closed_with_a_straight_seam_and_a_slanted_one() {
        for turned in [0., 1.] {
            assert_just_under(closed_volume(&bar(1., 2., turned)), PI * 2.);
        }
    }

    #[test]
    fn curved_face_between_flat_faces_is_closed() {
        // Half a round bar: the half with y > 0.
        let (radius, height) = (1., 2.);
        let mut solid = Solid::new();
        let [a, b] = [radius, -radius].map(|x| solid.vertex([x, 0., 0.]));
        let [a_top, b_top] = [radius, -radius].map(|x| solid.vertex([x, 0., height]));
        let up = [0., 0., 1.];
        let low = solid.arc(a, b, [0.; 3], up, [radius, 0., 0.], [0., PI]);
        let high = solid.arc(
            a_top,
            b_top,
            [0., 0., height],
            up,
            [radius, 0., 0.],
            [0., PI],
        );
        let round = solid.surface(
            "cone",
            vec![
                vector(tag::POSITION, [0.; 3]),
                vector(tag::DIRECTION, up),
                vector(tag::DIRECTION, [radius, 0., 0.]),
                real(1.),
                real(0.),
                real(1.),
            ],
        );
        let boundary = [
            (low, true),
            solid.line(b, b_top),
            (high, false),
            solid.line(a_top, a),
        ];
        solid.face(round, &[&boundary]);
        solid.flat([0., -1., 0.], [1., 0., 0.], &[b, a, a_top, b_top]);
        let floor = solid.plane([0.; 3], [0., 0., -1.], [1., 0., 0.]);
        let boundary = [solid.line(a, b), (low, false)];
        solid.face(floor, &[&boundary]);
        let roof = solid.plane([0., 0., height], up, [1., 0., 0.]);
        let boundary = [(high, true), solid.line(b_top, a_top)];
        solid.face(roof, &[&boundary]);
        assert_just_under(closed_volume(&solid.model), PI);
    }

    #[test]
    fn sphere_corner_with_a_vertex_at_the_pole_is_closed() {
        // The eighth of a ball with x, y and z all positive. The sphere's
        // pole is the vertex on z, where its longitude means nothing.
        let mut solid = Solid::new();
        let o = solid.vertex([0.; 3]);
        let [x, y, z] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]].map(|p| solid.vertex(p));
        let quarter = [0., PI / 2.];
        let xy = solid.arc(x, y, [0.; 3], [0., 0., 1.], [1., 0., 0.], quarter);
        let yz = solid.arc(y, z, [0.; 3], [1., 0., 0.], [0., 1., 0.], quarter);
        let zx = solid.arc(z, x, [0.; 3], [0., 1., 0.], [0., 0., 1.], quarter);
        let ball = solid.surface(
            "sphere",
            vec![
                vector(tag::POSITION, [0.; 3]),
                real(1.),
                vector(tag::DIRECTION, [0., 0., 1.]),
                vector(tag::DIRECTION, [1., 0., 0.]),
            ],
        );
        solid.face(ball, &[&[(xy, true), (yz, true), (zx, true)]]);
        for (normal, u_dir, from, arc, to) in [
            ([0., 0., -1.], [1., 0., 0.], y, xy, x),
            ([-1., 0., 0.], [0., 1., 0.], z, yz, y),
            ([0., -1., 0.], [0., 0., 1.], x, zx, z),
        ] {
            let plane = solid.plane([0.; 3], normal, u_dir);
            let boundary = [solid.line(o, from), (arc, false), solid.line(to, o)];
            solid.face(plane, &[&boundary]);
        }
        assert_just_under(closed_volume(&solid.model), PI / 6.);
        // The round face is one eighth of the sphere, not the other seven.
        let report = tessellate(&solid.model, &Default::default()).unwrap();
        let round = &report.faces[0];
        let area: f64 = report.triangles[..round.triangle_count]
            .iter()
            .map(|t| {
                let [a, b, c] = t.map(|i| report.positions[i as usize]);
                norm(cross(sub(b, a), sub(c, a))) * 0.5
            })
            .sum();
        assert!(area > PI / 2. * 0.97 && area <= PI / 2., "area {area}");
    }

    #[test]
    fn barrel_cut_from_a_sphere_is_closed() {
        // The part of a ball between two parallel planes. The seam of the
        // round face is a quarter of a great circle, cut into chords.
        let lift = 0.7f64;
        let (rim, half) = (lift.cos(), lift.sin());
        let mut solid = Solid::new();
        let up = [0., 0., 1.];
        let low = solid.vertex([rim, 0., -half]);
        let high = solid.vertex([0., rim, half]);
        let bottom = solid.arc(low, low, [0., 0., -half], up, [rim, 0., 0.], [0., TAU]);
        let top = solid.arc(high, high, [0., 0., half], up, [0., rim, 0.], [0., TAU]);
        let ball = solid.surface(
            "sphere",
            vec![
                vector(tag::POSITION, [0.; 3]),
                real(1.),
                vector(tag::DIRECTION, up),
                vector(tag::DIRECTION, [1., 0., 0.]),
            ],
        );
        solid.face(ball, &[&[(bottom, true)], &[(top, false)]]);
        let floor = solid.plane([0., 0., -half], [0., 0., -1.], [1., 0., 0.]);
        solid.face(floor, &[&[(bottom, false)]]);
        let roof = solid.plane([0., 0., half], up, [1., 0., 0.]);
        solid.face(roof, &[&[(top, true)]]);
        // A slice of a ball of radius 1 from -h to h has volume 2 pi (h - h^3 / 3).
        let exact = TAU * (half - half.powi(3) / 3.);
        assert_just_under(closed_volume(&solid.model), exact);
    }

    #[test]
    fn pipe_elbow_cut_from_a_torus_is_closed() {
        // A quarter of a ring: the tube goes round its own circle, the
        // second parameter of the torus.
        let (major, minor) = (3., 1.);
        let mut solid = Solid::new();
        let start = solid.vertex([major + minor, 0., 0.]);
        let end = solid.vertex([0., major + minor, 0.]);
        let first = solid.arc(
            start,
            start,
            [major, 0., 0.],
            [0., -1., 0.],
            [minor, 0., 0.],
            [0., TAU],
        );
        let second = solid.arc(
            end,
            end,
            [0., major, 0.],
            [1., 0., 0.],
            [0., minor, 0.],
            [0., TAU],
        );
        let ring = solid.surface(
            "torus",
            vec![
                vector(tag::POSITION, [0.; 3]),
                vector(tag::DIRECTION, [0., 0., 1.]),
                real(major),
                real(minor),
                vector(tag::DIRECTION, [1., 0., 0.]),
            ],
        );
        solid.face(ring, &[&[(first, false)], &[(second, true)]]);
        let near = solid.plane([major, 0., 0.], [0., -1., 0.], [1., 0., 0.]);
        solid.face(near, &[&[(first, true)]]);
        let far = solid.plane([0., major, 0.], [-1., 0., 0.], [0., 1., 0.]);
        solid.face(far, &[&[(second, false)]]);
        assert_just_under(
            closed_volume(&solid.model),
            PI * PI * major * minor * minor / 2.,
        );
    }

    #[test]
    fn edge_counts_find_gaps_and_triangles_on_one_side_twice() {
        let mut solid = BodyMesh {
            positions: vec![[0.; 3], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            triangles: vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]],
            ..Default::default()
        };
        assert!(solid.edge_counts().closed());
        solid.triangles.push([0, 2, 1]);
        assert_eq!(
            solid.edge_counts(),
            EdgeCounts {
                open: 0,
                repeated: 3
            }
        );
        solid.triangles.truncate(3);
        assert_eq!(
            solid.edge_counts(),
            EdgeCounts {
                open: 3,
                repeated: 0
            }
        );
    }

    #[test]
    fn trimmed_plane_preserves_hole_and_boundary_direction() {
        let report = tessellate(&plane_model(true), &Default::default()).unwrap();
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
        // The same face seen from the other side: its boundary goes the other way.
        let reversed = tessellate(&plane_model_facing(true, true), &Default::default()).unwrap();
        assert_eq!(reversed.triangles.len(), report.triangles.len());
        for triangle in reversed.triangles {
            let p = triangle.map(|i| reversed.positions[i as usize]);
            assert!(cross(sub(p[1], p[0]), sub(p[2], p[0]))[2] < 0.);
        }
    }

    /// Boundaries given on a surface by their parameters, with a point for each place.
    fn rings_on(surface: &Surface, boundaries: &[Vec<Uv>]) -> Vec<Ring> {
        let mut next = 0;
        boundaries
            .iter()
            .map(|uv| Ring {
                point: uv.iter().map(|_| (next, next += 1).0).collect(),
                xyz: uv.iter().map(|p| surface.point_at(p[0], p[1])).collect(),
                uv: uv.clone(),
            })
            .collect()
    }
    /// The band between boundaries that go round a surface, as its triangles
    /// before any is split.
    fn band(surface: &Surface, boundaries: &[Vec<Uv>], swap: bool) -> Patch {
        let rings = rings_on(surface, boundaries);
        let points = rings.iter().map(|ring| ring.point.len()).sum::<usize>();
        let mut patch = Patch::new(points as u32);
        periodic_strip(surface, &rings, &Default::default(), swap, &mut patch).unwrap();
        patch.orient();
        patch.restore_boundary();
        patch.flip(metric(surface, &patch.uv));
        patch
    }
    /// Every boundary chord has one triangle and every other side has two,
    /// one each way.
    fn assert_joined(patch: &Patch) {
        let mut sides = BTreeMap::new();
        for t in &patch.triangles {
            for k in 0..3 {
                assert!(sides.insert((t[k], t[(k + 1) % 3]), ()).is_none());
            }
        }
        for &(a, b) in sides.keys() {
            assert_eq!(
                sides.contains_key(&(b, a)),
                !patch.fixed.contains(&chord(a, b)),
                "side {a} {b}"
            );
        }
        assert_eq!(
            sides
                .keys()
                .filter(|(a, b)| patch.fixed.contains(&chord(*a, *b)))
                .count(),
            patch.fixed.len()
        );
    }
    fn parameter_area(patch: &Patch) -> f64 {
        patch
            .triangles
            .iter()
            .map(|t| triangle_area(patch.uv[t[0]], patch.uv[t[1]], patch.uv[t[2]]))
            .sum()
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
        // The upper boundary is half a step round from the lower one.
        let boundaries: Vec<Vec<Uv>> = [(0., 0.), (0.5, 5.)]
            .into_iter()
            .map(|(turned, v)| {
                (0..96)
                    .map(|i| [(i as f64 + turned) * TAU / 96., v])
                    .collect()
            })
            .collect();
        let mut patch = band(&surface, &boundaries, false);
        patch.refine(&surface, &options).unwrap();
        assert_joined(&patch);
        assert!(patch.triangles.len() >= 192);
        for triangle in &patch.triangles {
            let p = triangle.map(|i| patch.xyz[i]);
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
    fn varying_periodic_boundary_and_hole_preserve_area() {
        let surface = Surface::Cone {
            base: [0.; 3],
            axis: [0., 0., 1.],
            major: [10., 0., 0.],
            ratio: 1.,
            sin_angle: 0.,
            cos_angle: 1.,
        };
        let lower: Vec<_> = (0..64)
            .map(|i| {
                let u = i as f64 * TAU / 64.;
                [u, 0.2 * u.sin()]
            })
            .collect();
        let upper: Vec<_> = lower
            .iter()
            .rev()
            .map(|p| [p[0], 2. + 0.2 * p[0].sin()])
            .collect();
        let hole = vec![[1., 0.8], [1.2, 0.8], [1.2, 1.2], [1., 1.2]];
        let patch = band(&surface, &[lower, upper, hole], false);
        assert_joined(&patch);
        assert!((parameter_area(&patch) - (TAU * 2. - 0.08)).abs() < 1e-8);
        // The seam joins corners of the two boundaries: the cut adds no point
        // of its own to either.
        assert_eq!(patch.uv.len(), 64 + 64 + 2 + 4);
    }

    #[test]
    fn torus_minor_period_is_preserved() {
        let surface = Surface::Torus {
            centre: [0.; 3],
            axis: [0., 0., 1.],
            u_ref: [1., 0., 0.],
            major_radius: 3.,
            minor_radius: 1.,
        };
        let boundaries: Vec<Vec<Uv>> = [0.2, 1.1]
            .into_iter()
            .map(|u| (0..32).map(|i| [u, i as f64 * TAU / 32.]).collect())
            .collect();
        let patch = band(&surface, &boundaries, true);
        assert_joined(&patch);
        assert!(
            patch
                .xyz
                .iter()
                .zip(&patch.uv)
                .all(|(p, uv)| norm(sub(*p, surface.point_at(uv[0], uv[1]))) < 1e-10)
        );
        assert!((parameter_area(&patch).abs() - 0.9 * TAU).abs() < 1e-8);
    }

    #[test]
    fn a_face_gives_way_on_a_side_another_face_has() {
        // Two triangles on a square. Another face already has a side between
        // the points of the square's diagonal, so the diagonal is split.
        let surface = Surface::Plane {
            origin: [0.; 3],
            normal: [0., 0., 1.],
            u_dir: [1., 0., 0.],
        };
        let square = vec![[0., 0.], [1., 0.], [1., 1.], [0., 1.]];
        let rings = rings_on(&surface, &[square]);
        let mut patch = Patch::new(4);
        patch.boundary(rings[0].corners());
        patch.triangles = vec![[0, 1, 2], [0, 2, 3]];
        patch.give_way(&surface, &BTreeSet::from([(0, 2)]));
        assert_joined(&patch);
        assert_eq!(patch.triangles.len(), 4);
        assert_eq!(patch.own, [[0.5, 0.5, 0.]]);
        assert!((parameter_area(&patch) - 1.).abs() < 1e-12);
    }

    #[test]
    fn boundary_corner_left_out_by_the_triangulator_is_put_back() {
        // A square with a corner in the middle of its lower side, and
        // triangles that join the ends of that side directly.
        let surface = Surface::Plane {
            origin: [0.; 3],
            normal: [0., 0., 1.],
            u_dir: [1., 0., 0.],
        };
        let outline = vec![[0., 0.], [0.5, 0.], [1., 0.], [1., 1.], [0., 1.]];
        let rings = rings_on(&surface, &[outline]);
        let mut patch = Patch::new(5);
        patch.boundary(rings[0].corners());
        patch.triangles = vec![[0, 2, 3], [0, 3, 4]];
        patch.restore_boundary();
        assert_joined(&patch);
        assert_eq!(patch.triangles.len(), 3);
        assert!((parameter_area(&patch) - 1.).abs() < 1e-12);
    }

    #[test]
    fn triangle_side_that_runs_past_corners_is_cut_at_them() {
        // A kite with two corners on its long diagonal. The triangle above
        // has the whole diagonal as one side; the three below end at the
        // corners on it.
        let surface = Surface::Plane {
            origin: [0.; 3],
            normal: [0., 0., 1.],
            u_dir: [1., 0., 0.],
        };
        let outline = vec![[0., 0.], [1.5, -1.], [3., 0.], [1.5, 1.]];
        let rings = rings_on(&surface, &[outline]);
        let mut patch = Patch::new(6);
        patch.boundary(rings[0].corners());
        let on_diagonal = [1., 2.].map(|u| patch.corner([u, 0.], [u, 0., 0.], 4 + u as u32 - 1));
        let [first, second] = on_diagonal;
        patch.triangles = vec![[0, 2, 3], [0, 1, first], [first, 1, second], [second, 1, 2]];
        patch.restore_boundary();
        assert_joined(&patch);
        assert_eq!(patch.triangles.len(), 6);
        assert!((parameter_area(&patch) - 3.).abs() < 1e-12);
    }

    #[test]
    fn flat_triangle_on_a_straight_boundary_is_traded_away() {
        // The same outline, with a triangle of no area on the lower side.
        let surface = Surface::Plane {
            origin: [0.; 3],
            normal: [0., 0., 1.],
            u_dir: [1., 0., 0.],
        };
        let outline = vec![[0., 0.], [0.5, 0.], [1., 0.], [1., 1.], [0., 1.]];
        let rings = rings_on(&surface, &[outline]);
        let mut patch = Patch::new(5);
        patch.boundary(rings[0].corners());
        patch.triangles = vec![[0, 1, 2], [0, 2, 3], [0, 3, 4]];
        patch.flip([1.; 2]);
        assert_joined(&patch);
        for t in &patch.triangles {
            assert!(triangle_area(patch.uv[t[0]], patch.uv[t[1]], patch.uv[t[2]]) > 0.1);
        }
    }

    #[test]
    fn unsupported_faces_are_explicit_and_mesh_limits_are_bounded() {
        let mut model = plane_model(false);
        model.entities[4].types[0] = "unknown-surface".into();
        let report = tessellate(&model, &Default::default()).unwrap();
        assert!(report.positions.is_empty());
        assert_eq!(report.faces_skipped, 1);
        assert_eq!(report.issues[0].kind, MeshIssueKind::UnsupportedGeometry);
        assert!(report.issues[0].reason.contains("unknown-surface"));
        model.entities[4].types[0] = "plane".into();
        model.entities[4].tokens.truncate(2);
        let malformed = tessellate(&model, &Default::default()).unwrap();
        assert_eq!(malformed.issues[0].kind, MeshIssueKind::InvalidGeometry);
        let report = tessellate(
            &plane_model(false),
            &MeshOptions {
                max_triangles: 1,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(report.triangles.is_empty());
        assert!(report.positions.is_empty());
        assert_eq!(report.issues[0].kind, MeshIssueKind::ResourceLimit);
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
                real(1.),
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
        assert_eq!(
            report.issues[0].kind,
            MeshIssueKind::UnsupportedBodyTransform
        );
        assert!(report.issues[0].reason.contains("transform"));
    }
}

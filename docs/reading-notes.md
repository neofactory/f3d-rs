# Reading geometry and assembly data

This guide describes the supported binary records and the interpretation of
reader and mesh output. Start with the [README](../README.md) for usage.

## Format coverage

| Layer | Support |
|---|---|
| Archive | Stored, Deflate, Zstandard ZIP entries; CRC checks; concatenated Zstandard frames; raw entry access |
| Packages | Recursively read nested F3D/F3Z documents with depth and decompression limits |
| Manifest | Document header, identities, schema table; version-dependent tail retained in raw archive bytes and its unread length reported |
| Meta/Bulk streams | Type registrations, root IDs, object offsets, object payloads, optional schema footer; older streams without it are supported |
| ASM | `ASM BinaryFile4`, `ASM BinaryFile8`, `ACIS BinaryFile`; all SAB tags used by ezf3d; main/history section separation |
| Topology | Body/lump/shell/face/loop/coedge/edge/vertex traversal, tolerant subclasses, cycle guards, live face/edge sets |
| Analytic curves | Straight lines and ellipses, evaluation and parameter inversion |
| Analytic surfaces | Planes, cones/cylinders, spheres, signed-radius tori; point evaluation |
| Display meshes | Trimmed analytic and explicit spline faces, holes, periodic bands, per-face/body triangle ranges, explicit skipped-face diagnostics, closed meshes for solid bodies whose faces all mesh |
| Splines | Entire token records retained; typed NURBS extraction/evaluation not implemented |
| Components | Names and snapshot body-map/container/component links with reciprocal owner checks |
| Placements | Unique rigid-matrix candidates from registered occurrence records; **not a full occurrence-schema decoder** |
| Other assets | Exact raw previews, materials, CAM XML, and unknown binary payloads available through the archive API |

No standard mesh-file exporter, graphics-cache decoder, parametric timeline
execution, sketch solver, CAD-kernel operations, or F3D writer is included.
Reading a complete ASM token stream does not certify
surface validity or a closed manifold.

Snapshot ownership decoding currently covers a limited, validated envelope
of Fusion design records. Unsupported body-map envelopes produce explicit
document issues and retain their raw payloads. Component-name discovery scans native strings;
it is not a general typed browser-tree decoder.

Always inspect `Document::issues`. Binary geometry failures are errors with
the entry path attached. Unsupported metadata is retained with a diagnostic,
rather than made to look like successfully decoded data.

## Units and coordinates

ASM geometry remains in its native coordinate unit, matching ezf3d.
Fusion normally uses centimetres; verify the units of each source. No uniform
multiplication is applied to token values: angles, ratios, knot values, normals, and lengths have different
semantics. Design placement candidates carry an explicit `cm` unit label.

Diagnostic JSON encodes nonfinite ASM reals as `"f64:0x<IEEE-754 bits>"` so
NaN payloads and infinite sentinels are retained. Finite values stay numbers.

Geometry is not automatically placed by assembly occurrences. Apply a verified
component placement explicitly when building a viewer. The mesh API accepts
absent/identity ASM body transforms; it skips faces under unverified nonidentity
transforms rather than displaying geometry in a guessed location.
`Document::brep_owners` returns all supported ownership bindings for a B-Rep
stream. Each binding's `placement_candidates` lookup stays within its owning
design segment: numeric component IDs from different segments are not joined.
Multiple owners or occurrence candidates remain explicit ambiguity.

## Mesh coverage and diagnostics

The default chord tolerance is 0.01 cm (0.1 mm for Fusion). `MeshOptions`
bounds edge sampling, boundary sizes, refinement depth, per-face triangles,
and total output vertices/triangles. Explicit ASM NUBS/NURBS curves and tensor
surfaces are evaluated from their control points and knots, including shared
subtype references. Adaptive sampling preserves the requested chord tolerance.
Periodic cylindrical/conical/torus strips retain varying boundaries and holes;
spherical pole trims retain their neighboring longitudes. Unknown spline
representations, unsupported trims, and faces exceeding limits are skipped.
There are no substitute boxes or filled-in holes. Output may be incomplete.

## Closed meshes

Vertex indices are shared. Each topological vertex has one mesh vertex, and each
edge is sampled once; the two faces at an edge use the same vertices along it.
Inside a face, a triangle side is split only together with the triangle on its
other side, and the chords of a face's boundary are not split by the face at
all: an edge is sampled finely enough for both of its faces beforehand. The
seam that opens a periodic band runs between existing boundary vertices.

The result for a solid body, where every edge joins two faces, is a closed
mesh: every triangle side has exactly one other triangle on it, running along
it in the opposite direction. This holds when every face of the body is meshed;
a skipped face leaves the sides around it open. `BodyMesh::edge_counts` returns the
number of open sides and of sides that more than one triangle runs along in
the same direction. Both are zero for a closed, consistently oriented mesh.
A body made of sheets rather than a solid has open sides in its source.

Triangles are wound so that they run along the face's boundary in the
boundary's own direction, which is counter-clockwise seen from outside the
solid. The face `sense` flag is used only where the boundary passes through a
point where the surface comes to a point (a sphere's pole, a cone's apex, the
place where a torus closes on its axis), to tell which side of the boundary
the face is on. Winding does not rely on the direction of the surface's
parameter normal, which is reversed inside the spindle of a self-intersecting
torus.

Two vertices closer together than the kernel's `resabs` are one mesh vertex,
and edge samples that close are merged, so no triangle has a side of zero
length. A triangle with two corners at one vertex is left out. Where two faces
would each put a triangle side between the same two vertices of their common
boundary, the second face splits its side at a vertex of its own.

A closed mesh is not a certified solid. Triangles follow the surface within
the chord tolerance but are not checked against each other for intersection,
and a few thin triangles along a curved boundary can stand steeply to the
surface.

Geometry decoding distinguishes `Error::UnsupportedGeometry` from
`Error::MalformedGeometry`. Both identify the source entity and its original
kind; unsupported geometry also identifies whether a curve or surface was
requested. Original ASM tokens remain available. Binary file errors retain
their existing `Invalid`, `Truncated`, ZIP, and entry-path variants.

Each skipped face has a `MeshIssueKind` alongside its readable `reason`, so
callers can distinguish unsupported geometry or topology, malformed geometry
or topology, unsupported body transforms, resource limits, and tessellation
failures without interpreting message text. The kind is serialized in mesh JSON.

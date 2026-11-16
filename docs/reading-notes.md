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
| Display meshes | Trimmed analytic faces, planar holes, periodic bands, per-face/body triangle ranges, explicit skipped-face diagnostics |
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
and total output vertices/triangles. Spline surfaces and spline boundary
curves remain unsupported. Nonplanar faces with holes, complicated periodic
trims, and faces exceeding the refinement limits are explicitly skipped.
There are no substitute boxes or filled-in holes. Output is for display and
may be incomplete or nonwatertight.

Geometry decoding distinguishes `Error::UnsupportedGeometry` from
`Error::MalformedGeometry`. Both identify the source entity and its original
kind; unsupported geometry also identifies whether a curve or surface was
requested. Original ASM tokens remain available. Binary file errors retain
their existing `Invalid`, `Truncated`, ZIP, and entry-path variants.

Each skipped face has a `MeshIssueKind` alongside its readable `reason`, so
callers can distinguish unsupported geometry or topology, malformed geometry
or topology, unsupported body transforms, resource limits, and tessellation
failures without interpreting message text. The kind is serialized in mesh JSON.

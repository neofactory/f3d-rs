# f3d-rs

A native Rust library for reading geometry assets from Autodesk Fusion `.f3d`
archives and nested `.f3z` packages, including ASM B-Rep records, components,
and placement information.
It does not require Python, Fusion, or an Autodesk service at runtime.

This is a read-only implementation based on portions of
[ezf3d](https://github.com/AlexSabaka/ezf3d), with its MIT notice retained.
It is not a complete port of ezf3d's feature set.

Machine definitions and axis motion live in the separate
[machine-rs](https://github.com/neofactory/machine-rs) project. This crate owns
geometry assets and package reading; it has no dependency on the machine project.

## Use from Rust

```toml
[dependencies]
f3d = { package = "f3d-rs", git = "https://github.com/neofactory/f3d-rs" }
```

```rust,no_run
use f3d::FusionFile;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let file = FusionFile::open("model.f3d")?;

    for document in file.documents() {
        println!("{:#?}", document.summary());

        for brep in &document.breps {
            println!("{}: {:?}", brep.path, brep.model.census());
            for face in brep.model.live_faces() {
                if let Some(record) = face.surface_entity() {
                    // Unsupported spline surfaces return an error here;
                    // their complete records remain in the model.
                    if let Ok(surface) = f3d::geometry::Surface::from_entity(record) {
                        println!("{:?}", surface.point_at(0.37, 0.23));
                    }
                }
            }
        }

        for assembly in &document.assemblies {
            for binding in &assembly.body_bindings {
                println!("{} -> {}", binding.blob, binding.component_name);
            }
        }

        for issue in &document.issues {
            eprintln!("{} [{}]: {}", issue.path, issue.stage, issue.message);
        }

        // Exact decompressed bytes, including previews and opaque metadata.
        if let Ok(png) = document.archive.bytes(
            "FusionAssetName[Active]/Previews/small.png"
        ) {
            std::fs::write("preview.png", png)?;
        }
    }
    Ok(())
}
```

`FusionFile::read` accepts any `Read + Seek` source and explicit `Limits`.
Package documents remain separate. XRef relationships are not resolved into
one flattened assembly.

## Command-line examples

```sh
cargo run -q --example inspect -- model.f3d
cargo run -q --example inspect -- model.f3d --summary-json
cargo run -q --example inspect -- model.f3d --json > decoded.json
cargo run --release --example mesh -- model.f3d --json > mesh.json
```

## Implemented reading scope

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

Snapshot ownership decoding currently covers the envelope exercised by the
Brother file. Unsupported body-map envelopes produce explicit document issues
and retain their raw payloads. Component-name discovery scans native strings;
it is not a general typed browser-tree decoder.

Always inspect `Document::issues`. Binary geometry failures are errors with
the entry path attached. Unsupported metadata is retained with a diagnostic,
rather than made to look like successfully decoded data.

## Units and coordinates

ASM geometry remains in its native coordinate unit, matching ezf3d. The
Brother archive uses centimetres. No uniform multiplication is applied to
token values: angles, ratios, knot values, normals, and lengths have different
semantics. Design placement candidates carry an explicit `cm` unit label.

Diagnostic JSON encodes nonfinite ASM reals as `"f64:0x<IEEE-754 bits>"` so
NaN payloads and infinite sentinels are retained. Finite values stay numbers.

Geometry is not automatically placed by assembly occurrences. Apply a verified
component placement explicitly when building a viewer. The mesh API accepts
absent/identity ASM body transforms; it skips faces under unverified nonidentity
transforms rather than displaying geometry in a guessed location.
The scene's animation ownership is separate from source placement recovery.
`Document::brep_owners` returns all supported ownership bindings for a B-Rep
stream. Each binding's `placement_candidates` lookup stays within its owning
design segment: numeric component IDs from different segments are not joined.
Multiple owners or occurrence candidates remain explicit ambiguity.

## Build display meshes

```rust,no_run
# fn main() -> f3d::Result<()> {
# let file = f3d::FusionFile::open("model.f3d")?;
for document in file.documents() {
    for brep in &document.breps {
        let mesh = f3d::mesh::tessellate(&brep.model, &Default::default())?;
        // Native kernel units; Fusion normally uses centimetres.
        println!("{} triangles; {} faces skipped", mesh.triangles.len(), mesh.faces_skipped);
        // positions and triangles can become a Three.js BufferGeometry.
        // Independently drawable bodies keep source face IDs and skipped faces.
        for body in mesh.split_by_body()? {
            println!("body {}: {} triangles", body.body_index, body.triangles.len());
        }
        // Always surface mesh.issues to users when coverage is partial.
    }
}
# Ok(())
# }
```

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

## Validation

Pull requests are checked by [GitHub Actions](.github/workflows/ci.yml).
See [CI and repository rules](.github/README.md) for required checks and release-tag permissions.

```sh
cargo test
cargo clippy --all-targets -- -D warnings
cargo fmt --check
```

Tests build synthetic inputs; no vendor machine geometry is bundled. They
exercise 32/64-bit ASM, truncation, invalid strings and tags, nested subtypes,
history pointer addressing, cyclic topology, all three ZIP compression types,
multi-frame Zstandard, CRC failures, resource limits, F3Z nesting, native
object indexing, and preservation of embedded assets.

The optional Python differential checker requires an environment containing
ezf3d. This dependency is for development only:

```sh
python tools/compare_ezf3d.py /path/to/model.f3d
python tools/check_mesh.py /path/to/model.f3d
```

For the Brother U500Xd2, the comparison checks every token in 10 B-Rep streams,
57,837 entities, and 729,612 tokens, plus entity counts, vertex bounds, live
topology, 7,510 analytic curve evaluations, and 2,749 surface evaluations.
The expected live face count is 3,060. When the earlier viewer audit is beside
the source file, all 10 component links and neutral placement matrices are
also checked. `validation-output/comparison.json` records the local result.

The native display mesh for that same input contains 47,788 triangles from
2,646 of 3,060 faces (86.5% coverage) at the default tolerance. The optional
mesh checker independently checks vertices against ezf3d's surfaces, triangle
centroid deviation and winding, and planar face areas including holes.

This is one real-model validation case, not a claim of compatibility with
every Fusion release. New files should be admitted with explicit diagnostics
and additional independent fixtures.

## License

MIT. Keep `LICENSE`, `LICENSE-MIT-ezf3d`, and `THIRD-PARTY-NOTICES` with
distributed adaptations. The upstream reference revision is recorded there.
Input CAD models have their own licenses; those are not changed by this crate.

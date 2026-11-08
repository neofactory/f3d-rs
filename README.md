# f3d-rs

A native Rust library for reading Autodesk Fusion `.f3d` archives, nested
`.f3z` documents, ASM B-Rep records, and Fusion `.mch` machine definitions.
It does not require Python, Fusion, or an Autodesk service at runtime.

This is a read-only implementation based on portions of
[ezf3d](https://github.com/AlexSabaka/ezf3d), with its MIT notice retained.
It is not a complete port of ezf3d's feature set.

## Use from Rust

```toml
[dependencies]
f3d = { package = "f3d-rs", git = "https://github.com/neofactory/f3d-rs" }
```

```rust,no_run
use f3d::FusionFile;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let file = FusionFile::open("machine.f3d")?;

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

## Read a machine and evaluate its pose

```rust,no_run
use std::collections::BTreeMap;
use f3d::machine::MachineDefinition;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let machine = MachineDefinition::parse(&std::fs::read("machine.mch")?)?;
    let coordinates = BTreeMap::from([
        ("X".to_owned(), 100.0),
        ("Y".to_owned(), 50.0),
        ("Z".to_owned(), -100.0),
        ("A".to_owned(), 90.0),
        ("C".to_owned(), 1080.0),
    ]);
    let transforms = machine.pose(&coordinates)?;
    println!("A: {:?}", transforms.get("rotary_0"));
    println!("spindle in table: {:?}", machine.spindle_in_table(&coordinates)?);
    Ok(())
}
```

Axis IDs and command names are accepted. Missing coordinates are zero.
Continuous axes have no invented range. Unknown names, conflicting aliases,
nonfinite coordinates, and coordinates outside declared limits are errors.
Parent transforms carry their descendants. Rotary motion uses
`T(pivot) * R(axis, angle) * T(-pivot)`.

The result is a geometric pose in the definition's length unit. It does not
apply controller offsets, tool length, work offsets, post-processor logic,
or collision rules. Nonzero part offsets and unsupported node kinds are
explicitly refused rather than assigned guessed motion semantics.

## Command-line examples

```sh
cargo run -q --example inspect -- machine.f3d
cargo run -q --example inspect -- machine.f3d --summary-json
cargo run -q --example inspect -- machine.f3d --json > decoded.json
cargo run -q --example machine_pose -- machine.mch X=100 Y=50 Z=-100 A=90 C=1080
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
| Splines | Entire token records retained; typed NURBS extraction/evaluation not implemented |
| Components | Names and snapshot body-map/container/component links with reciprocal owner checks |
| Placements | Unique rigid-matrix candidates from registered occurrence records; **not a full occurrence-schema decoder** |
| Machine JSON | Complete original JSON values plus typed kinematic tree, attachments, hierarchy-aware poses |
| Other assets | Exact raw previews, materials, CAM XML, and unknown binary payloads available through the archive API |

No tessellation, mesh export, graphics-cache decoder, parametric timeline
execution, sketch solver, face triangulation, CAD-kernel operations, or F3D
writer is included. Reading a complete ASM token stream does not certify
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
Machine JSON transforms use `kinematics.default.units` independently.

Diagnostic JSON encodes nonfinite ASM reals as `"f64:0x<IEEE-754 bits>"` so
NaN payloads and infinite sentinels are retained. Finite values stay numbers.

Geometry is not automatically placed by assembly occurrences. Apply a verified
component placement and any body transform explicitly when building a viewer.
The scene's animation ownership is separate from source placement recovery.

## Validation

```sh
cargo test
cargo clippy --all-targets -- -D warnings
cargo fmt --check
```

Tests build synthetic inputs; no vendor machine geometry is bundled. They
exercise 32/64-bit ASM, truncation, invalid strings and tags, nested subtypes,
history pointer addressing, cyclic topology, all three ZIP compression types,
multi-frame Zstandard, CRC failures, resource limits, F3Z nesting, native
object indexing, and machine kinematics.

The optional Python differential checker requires an environment containing
ezf3d. This dependency is for development only:

```sh
python tools/compare_ezf3d.py /path/to/model.f3d
```

For the Brother U500Xd2, the comparison checks every token in 10 B-Rep streams,
57,837 entities, and 729,612 tokens, plus entity counts, vertex bounds, live
topology, 7,510 analytic curve evaluations, and 2,749 surface evaluations.
The expected live face count is 3,060. Machine JSON is compared without
rounding its floating-point values. When the earlier viewer audit is beside
the source file, all 10 component links and neutral placement matrices are
also checked. `validation-output/comparison.json` records the local result.

This is one real-model validation case, not a claim of compatibility with
every Fusion release. New files should be admitted with explicit diagnostics
and additional independent fixtures.

## License

MIT. Keep `LICENSE`, `LICENSE-MIT-ezf3d`, and `THIRD-PARTY-NOTICES` with
distributed adaptations. The upstream reference revision is recorded there.
Input CAD models have their own licenses; those are not changed by this crate.

# Development and validation

Pull requests are checked by [GitHub Actions](../.github/workflows/ci.yml).
See [CI and repository rules](../.github/CONTRIBUTING-CI.md) for required checks and release-tag permissions.

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

## Optional reference checks

The Python differential checkers require **Python 3.13**, Git, and the Rust
toolchain. The pinned ezf3d revision requires Python `>=3.13,<3.14`.
Neither Python nor ezf3d is needed to build or use the Rust library or run its
synthetic tests. From the repository root, create an isolated environment:

```sh
python3.13 -m venv .venv-validation
.venv-validation/bin/python -m pip install -r tools/requirements-validation.txt
.venv-validation/bin/python -m pip check
.venv-validation/bin/python tools/compare_ezf3d.py /path/to/model.f3d
.venv-validation/bin/python tools/check_mesh.py /path/to/model.f3d
```

On Windows, use `py -3.13 -m venv .venv-validation` and replace
`.venv-validation/bin/python` with `.venv-validation\Scripts\python.exe`.
The requirements pin the ezf3d source revision recorded in
`THIRD-PARTY-NOTICES` and NumPy; ezf3d's other dependencies are resolved by pip.

Supply your own F3D input. The tools write decoded data and reports under the
ignored `validation-output/` directory; the environment is also ignored.
Keep private source models outside the repository. These optional checks are
not part of CI and do not require the private validation model described below.

## Recorded validation on a private model

The following results were recorded locally using a **private Brother U500Xd2
model**. The source CAD, recovered viewer audit, and generated reports are not
distributed with this repository, so these figures are not a bundled,
independently reproducible benchmark.

That comparison checked every token in 10 B-Rep streams,
57,837 entities, and 729,612 tokens, plus entity counts, vertex bounds, live
topology, 7,510 analytic curve evaluations, and 2,749 surface evaluations.
The live face count was 3,060. All 10 component links and neutral placement
matrices were also checked against the private recovered viewer audit.
The comparison tool performs this additional check only when
`recovered/component-mapping.json` exists beside the input model.
`validation-output/comparison.json` records each local run's result.

The recorded native display mesh for that same input contained 47,788 triangles
from 2,646 of 3,060 faces (86.5% coverage) at the default tolerance. The optional
mesh checker independently checks vertices against ezf3d's surfaces, triangle
centroid deviation and winding, and planar face areas including holes.

This is one real-model validation case, not a claim of compatibility with
every Fusion release. New files should be admitted with explicit diagnostics
and additional independent fixtures.

## Autodesk Brother source regression

The explicit spline and trim implementation was also checked against Autodesk's
Brother U500Xd2 F3D (SHA-256
`6031fdce96f223ca1d0b838d73e560e1744ca31ec81516393b30107257169397`).
At the default 0.01 cm chord tolerance, all 3,060 source faces mesh without
skipped-face diagnostics, and each of the 58 bodies has a closed mesh with
positive volume (`BodyMesh::edge_counts` reports no open or repeated sides). This
supersedes the historical mesh coverage above. It checks face coverage and mesh
closure; it is not a manufacturing certification, and triangles are not checked
for intersection. The source is downloaded separately and is not included in Git.

```sh
F3D_BROTHER_MODEL=/path/to/brother.f3d cargo test --release --test autodesk -- --ignored
```

Synthetic tests cover rational curve evaluation/inversion, tensor control order,
malformed spline payloads, periodic curve wrapping, variable-height periodic
trims with holes, and torus minor-period trims. Hand-built solids (a box, a
round bar with a straight and with a slanted seam, half a round bar, a corner
of a ball with a vertex at the sphere's pole, a barrel cut from a sphere, and a
pipe elbow cut from a torus) must each mesh closed with a volume just under the
exact one.

`tools/check_mesh.py` compares triangle winding with the face `sense` flag and
ezf3d's surface normal. It has not been run against meshes wound by boundary
direction; on the Autodesk Brother source it is expected to object to the one
face that lies inside the spindle of a self-intersecting torus.

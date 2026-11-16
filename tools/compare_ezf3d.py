"""Optional development check: Python ezf3d versus the independent Rust reader.

Set up the Python 3.13 reference environment as described in README.md, then:
  .venv-validation/bin/python tools/compare_ezf3d.py /path/to/model.f3d
No source models are copied into this crate.
"""
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
from ezf3d.asm import Shape, census, parse
from ezf3d.asm.geometry import read_curve, read_surface
from ezf3d.container.archive import F3DArchive

root = Path(__file__).resolve().parents[1]
model = Path(sys.argv[1]).resolve()
out = root / "validation-output"
out.mkdir(exist_ok=True)
rust_json = out / "reference-dump.json"
with rust_json.open("w") as stream:
    subprocess.run(["cargo", "run", "-q", "--example", "reference_dump", "--", str(model)], cwd=root, stdout=stream, check=True)
rust = json.loads(rust_json.read_text())
assert rust["file"]["kind"] == "document"
breps = {b["path"]: b["model"] for b in rust["file"]["breps"]}
checks = {c["path"]: c for c in rust["checks"]}
totals = {"streams": 0, "entities": 0, "tokens": 0, "analytic_curves": 0, "analytic_surfaces": 0, "live_faces": 0}


def equal(a, b):
    if isinstance(a, (list, tuple)):
        return isinstance(b, list) and len(a) == len(b) and all(equal(x, y) for x, y in zip(a, b))
    if isinstance(a, float):
        return isinstance(b, (int, float)) and math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12)
    return a == b


with F3DArchive(model) as archive:
    for path, rmodel in breps.items():
        py = parse(archive.read(path))
        assert py.terminated == rmodel["terminated"]
        assert py.has_history == rmodel["has_history"]
        assert py.header.word_size == rmodel["header"]["word_size"]
        assert len(py.entities) == len(rmodel["entities"])
        assert len(py.history) == len(rmodel["history"])
        for entity, rent in zip(py.entities, rmodel["entities"]):
            assert list(entity.types) == rent["types"]
            assert len(entity.tokens) == len(rent["tokens"])
            for (tag, value), token in zip(entity.tokens, rent["tokens"]):
                assert int(tag) == token["tag"] and equal(value, token["value"]), (path, entity.index, tag, value, token)
                totals["tokens"] += 1
        shape = Shape(py)
        check = checks[path]
        assert dict(py.counts()) == check["counts"]
        assert len(list(shape.faces())) == check["live_faces"]
        assert len(list(shape.edges())) == check["live_edges"]
        bounds = census(py).vertex_bounds
        assert equal([bounds.min, bounds.max], check["bounds"])
        for row in check["curves"]:
            point = read_curve(py.entities[row["entity"]], py).point_at(0.37)
            assert np.allclose(point, row["point"], rtol=1e-11, atol=1e-10)
            totals["analytic_curves"] += 1
        for row in check["surfaces"]:
            point = read_surface(py.entities[row["entity"]], py).point_at(0.37, 0.23)
            assert np.allclose(point, row["point"], rtol=1e-11, atol=1e-10)
            totals["analytic_surfaces"] += 1
        totals["streams"] += 1
        totals["entities"] += len(py.entities)
        totals["live_faces"] += check["live_faces"]

# Compare ownership/placement with the independently recovered viewer audit
# when this validation is run on the original Brother input.
audit_path = model.parent / "recovered/component-mapping.json"
if audit_path.exists():
    audit = json.loads(audit_path.read_text())
    bindings = {b["blob"]: b for a in rust["file"]["assemblies"] for b in a["body_bindings"]}
    placements = {p["component_id"]: p for a in rust["file"]["assemblies"] for p in a["placements"]}
    for part in audit:
        binding = bindings[part["filename"]]
        assert binding["component_id"] == part["component_id"]
        assert binding["component_name"] == part["name"]
        matrix = np.array(placements[part["component_id"]]["matrix"])
        matrix[:3, 3] *= 10
        assert np.allclose(matrix, part["placement"]["matrix_mm"], rtol=0, atol=1e-9)
    totals["component_bindings"] = len(audit)
    totals["placement_matrices"] = len(audit)

report = {"status": "passed", "model": str(model), "reference": "ezf3d 0.1.1", "compared": totals}
(out / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))

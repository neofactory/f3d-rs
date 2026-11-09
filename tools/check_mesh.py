"""Check native display meshes against independent ezf3d analytic geometry.

Development only: requires Python ezf3d and numpy. Does not copy source CAD.
  python tools/check_mesh.py /path/model.f3d
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from ezf3d.asm import Shape, parse
from ezf3d.asm.geometry import Plane
from ezf3d.container.archive import F3DArchive
from ezf3d.mesh.tessellate import tessellate_face

root = Path(__file__).resolve().parents[1]
source = Path(sys.argv[1]).resolve()
out = root / "validation-output"
out.mkdir(exist_ok=True)
with (out / "mesh.json").open("w") as stream:
    subprocess.run(["cargo", "run", "--release", "-q", "--example", "mesh", "--", str(source), "--json"], cwd=root, stdout=stream, check=True)

totals = {"streams": 0, "faces_meshed": 0, "faces_skipped": 0, "triangles": 0, "planar_areas_compared": 0}
max_deviation = 0.0
with F3DArchive(source) as archive:
    for row in json.loads((out / "mesh.json").read_text()):
        model = parse(archive.read(row["path"]))
        faces = {f.index: f for f in Shape(model).faces()}
        mesh = row["mesh"]
        points = np.array(mesh["positions"], dtype=float)
        triangles = np.array(mesh["triangles"], dtype=int)
        assert np.isfinite(points).all()
        if len(triangles):
            assert triangles.min() >= 0 and triangles.max() < len(points)
        assert mesh["faces_meshed"] + mesh["faces_skipped"] == len(faces)
        for entry in mesh["faces"]:
            face = faces[entry["face_index"]]
            surface = face.surface
            indices = triangles[entry["triangle_start"]:entry["triangle_start"] + entry["triangle_count"]]
            corners = points[indices]
            assert len(corners)
            for point in corners.reshape(-1, 3):
                assert surface.distance_to(point) <= 1e-4, (row["path"], face.index, point)
            centroids = corners.mean(axis=1)
            deviations = [surface.distance_to(p) for p in centroids]
            max_deviation = max(max_deviation, max(deviations))
            assert max(deviations) <= 0.010001, (row["path"], face.index, max(deviations))
            normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
            for normal, centroid in zip(normals, centroids):
                u, v = surface.invert(centroid)
                wanted = surface.normal_at(u, v) * (1 if face.sense else -1)
                # Kernel trim endpoints and singular torus patches can leave
                # nearly tangent slivers; allow numerical angular roundoff.
                assert np.dot(normal, wanted) >= -1e-6 * np.linalg.norm(normal), (row["path"], face.index, normal, wanted)
            if isinstance(surface, Plane):
                reference, reason, _ = tessellate_face(face, 0.01)
                if reason is None:
                    area = np.linalg.norm(normals, axis=1).sum() * 0.5
                    rc = reference.corners()
                    expected = np.linalg.norm(np.cross(rc[:, 1] - rc[:, 0], rc[:, 2] - rc[:, 0]), axis=1).sum() * 0.5
                    assert abs(area - expected) < max(1.0, expected) * 1e-8, (row["path"], face.index, area, expected)
                    totals["planar_areas_compared"] += 1
        totals["streams"] += 1
        totals["faces_meshed"] += mesh["faces_meshed"]
        totals["faces_skipped"] += mesh["faces_skipped"]
        totals["triangles"] += len(triangles)

result = {"status": "passed", "model": str(source), **totals, "max_centroid_deviation_cm": max_deviation}
(out / "mesh-comparison.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))

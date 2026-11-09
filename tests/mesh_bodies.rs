use f3d::mesh::{MeshIssue, MeshIssueKind, MeshReport, MeshedFace};

fn report() -> MeshReport {
    MeshReport {
        positions: vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
        triangles: vec![[0, 1, 2], [2, 1, 3], [0, 2, 3]],
        faces: vec![
            MeshedFace {
                body_index: 3,
                face_index: 11,
                triangle_start: 0,
                triangle_count: 1,
            },
            MeshedFace {
                body_index: 8,
                face_index: 12,
                triangle_start: 1,
                triangle_count: 1,
            },
            MeshedFace {
                body_index: 3,
                face_index: 13,
                triangle_start: 2,
                triangle_count: 1,
            },
        ],
        faces_meshed: 3,
        faces_skipped: 2,
        issues: vec![
            MeshIssue {
                body_index: 8,
                face_index: 14,
                kind: MeshIssueKind::UnsupportedTopology,
                reason: "unsupported trim".into(),
            },
            MeshIssue {
                body_index: 20,
                face_index: 15,
                kind: MeshIssueKind::UnsupportedGeometry,
                reason: "unsupported surface".into(),
            },
        ],
    }
}

#[test]
fn splitting_bodies_preserves_geometry_face_identity_and_missing_faces() {
    let source = report();
    let bodies = source.split_by_body().unwrap();
    assert_eq!(
        bodies
            .iter()
            .map(|body| body.body_index)
            .collect::<Vec<_>>(),
        [3, 8, 20]
    );
    assert_eq!(bodies[0].positions, source.positions);
    assert_eq!(bodies[0].triangles, [[0, 1, 2], [0, 2, 3]]);
    assert_eq!(
        bodies[0]
            .faces
            .iter()
            .map(|f| (f.face_index, f.triangle_start))
            .collect::<Vec<_>>(),
        [(11, 0), (13, 1)]
    );
    assert_eq!(
        bodies[1].positions,
        [[0., 1., 0.], [1., 0., 0.], [0., 0., 1.]]
    );
    assert_eq!(bodies[1].triangles, [[0, 1, 2]]);
    assert_eq!(bodies[1].faces[0].triangle_start, 0);
    assert_eq!(bodies[1].issues[0].face_index, 14);
    assert!(bodies[2].positions.is_empty());
    assert!(bodies[2].triangles.is_empty());
    assert!(bodies[2].faces.is_empty());
    assert_eq!(bodies[2].issues[0].face_index, 15);
    assert_eq!(
        bodies.iter().map(|b| b.faces.len()).sum::<usize>(),
        source.faces_meshed
    );
    assert_eq!(
        bodies.iter().map(|b| b.issues.len()).sum::<usize>(),
        source.faces_skipped
    );
}

#[test]
fn malformed_mesh_ranges_are_errors_instead_of_panics() {
    let mut mesh = report();
    mesh.faces[0].triangle_start = usize::MAX;
    assert!(mesh.split_by_body().is_err());
    let mut mesh = report();
    mesh.faces[0].triangle_count = 100;
    assert!(mesh.split_by_body().is_err());
    let mut mesh = report();
    mesh.triangles[0][0] = 100;
    assert!(mesh.split_by_body().is_err());
}

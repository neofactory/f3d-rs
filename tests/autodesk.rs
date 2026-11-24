//! Opt-in source regression; downloaded manufacturer data stays outside Git.
#[test]
#[ignore = "set F3D_BROTHER_MODEL to the checksum-verified Autodesk F3D"]
fn brother_has_complete_face_coverage_and_closed_bodies() {
    let path = std::env::var("F3D_BROTHER_MODEL").expect("F3D_BROTHER_MODEL");
    let file = f3d::FusionFile::open(path).unwrap();
    let mut faces = 0;
    let mut bodies = 0;
    for document in file.documents() {
        for brep in &document.breps {
            let result = f3d::mesh::tessellate(&brep.model, &Default::default()).unwrap();
            assert!(
                result.issues.is_empty(),
                "{}: {:?}",
                brep.path,
                result.issues
            );
            assert_eq!(result.faces_meshed, brep.model.live_faces().len());
            assert!(result.positions.iter().flatten().all(|v| v.is_finite()));
            faces += result.faces_meshed;
            // Every body is a solid in the source: each of its edges joins two
            // faces. So every mesh is closed, with its triangles facing outwards.
            for body in result.split_by_body().unwrap() {
                let at = format!("{} body {}", brep.path, body.body_index);
                assert!(
                    body.edge_counts().closed(),
                    "{at}: {:?}",
                    body.edge_counts()
                );
                let volume: f64 = body
                    .triangles
                    .iter()
                    .map(|triangle| {
                        let [a, b, c] = triangle.map(|i| body.positions[i as usize]);
                        let across = [
                            b[1] * c[2] - b[2] * c[1],
                            b[2] * c[0] - b[0] * c[2],
                            b[0] * c[1] - b[1] * c[0],
                        ];
                        (a[0] * across[0] + a[1] * across[1] + a[2] * across[2]) / 6.
                    })
                    .sum();
                assert!(volume > 0., "{at}: volume {volume}");
                bodies += 1;
            }
        }
    }
    assert_eq!((faces, bodies), (3060, 58));
}

//! Opt-in source regression; downloaded manufacturer data stays outside Git.
#[test]
#[ignore = "set F3D_BROTHER_MODEL to the checksum-verified Autodesk F3D"]
fn brother_has_complete_face_coverage() {
    let path = std::env::var("F3D_BROTHER_MODEL").expect("F3D_BROTHER_MODEL");
    let file = f3d::FusionFile::open(path).unwrap();
    let mut total = 0;
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
            total += result.faces_meshed;
        }
    }
    assert_eq!(total, 3060);
}

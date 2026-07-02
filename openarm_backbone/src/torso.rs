//! OpenArm torso facts the governor needs but the URDF does not carry: the torso body
//! name and a tight convex proxy for its concave mesh. The bimanual stand is unchanged
//! between v1.0 and v2.0 (`body_link0_symp.stl` is byte-identical, same mount transforms),
//! so this proxy is shared across both generations. Hand-fitted to that torso mesh; a
//! different robot needs its own proxy here. The collision model's `build()` rejects a
//! URDF without `TORSO_BODY`, so a mismatch aborts at bringup rather than running against
//! the wrong geometry.

use bimanual_collision_model::{ConvexPiece, Point3};

/// The torso body whose concave mesh is replaced by the tight convex proxy below.
/// The model's auto-fit single hull bridges the torso's features into one bulging
/// solid that reads a false near-contact against the grippers at rest; these boxes
/// are the tight proxy the collision model was validated against.
pub const TORSO_BODY: &str = "openarm_body_link0";

/// Base plate, shoulder flare, rear brace, central column, and head block bounding
/// the OpenArm torso mesh in the root frame. See the collision model's torso fixture
/// for the per-feature derivation.
pub fn torso_hulls() -> Vec<ConvexPiece> {
    let ring = |y: f64, z: f64| {
        [
            Point3::new(-0.033, -y, z),
            Point3::new(0.033, -y, z),
            Point3::new(0.033, y, z),
            Point3::new(-0.033, y, z),
        ]
    };
    let flare = ConvexPiece::from_points(
        ring(0.085, 0.016)
            .into_iter()
            .chain(ring(0.034, 0.080))
            .collect(),
    );
    vec![
        ConvexPiece::aabb(
            Point3::new(-0.157, -0.097, -0.002),
            Point3::new(0.097, 0.097, 0.026),
        ),
        flare,
        ConvexPiece::aabb(
            Point3::new(-0.156, -0.034, 0.006),
            Point3::new(-0.029, 0.034, 0.226),
        ),
        ConvexPiece::aabb(
            Point3::new(-0.032, -0.032, 0.058),
            Point3::new(0.032, 0.032, 0.604),
        ),
        ConvexPiece::aabb(
            Point3::new(-0.087, -0.082, 0.598),
            Point3::new(0.067, 0.082, 0.775),
        ),
    ]
}

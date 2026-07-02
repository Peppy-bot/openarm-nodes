//! OpenArm torso facts the governor needs but the URDF does not carry: the torso
//! body name and the clip regions that decompose its concave mesh. The bimanual
//! stand is unchanged between v1.0 and v2.0 (`body_link0_symp.stl` is
//! byte-identical, same mount transforms), so the decomposition is shared across
//! both generations; a different robot needs its own regions here. The collision
//! model's `build()` rejects a URDF without `TORSO_BODY` and verifies the fitted
//! pieces contain the torso mesh, so a mismatch aborts at bringup rather than
//! running against the wrong geometry.

use bimanual_collision_model::{ClipRegion, Point3};

const INF: f64 = f64::INFINITY;

/// The torso body whose concave mesh is decomposed by the regions below. The
/// model's auto-fit single hull bridges the torso's features into one bulging
/// solid that reads a false near-contact against the grippers at rest; clipped
/// per feature, each piece gets the same rounded mesh fit a link gets.
pub const TORSO_BODY: &str = "openarm_body_link0";

/// Plate, flare, gusset, column, and four head bands decomposing the OpenArm
/// torso mesh (metres, root frame). Numbers mirror the collision model's torso
/// fixture, which carries the measured feature extents and the region placement
/// rules (overlap the cuts by >= 3 mm, keep bounds ~1 mm off flat mesh faces).
pub fn torso_regions() -> Vec<ClipRegion> {
    [
        // plate: the full-footprint base, everything below the flare skirt.
        ([-INF, -INF, -INF], [INF, INF, 0.012]),
        // flare: skirt + column bottom, fenced off the gusset at x -0.033.
        ([-0.033, -0.086, 0.006], [0.033, 0.086, 0.083]),
        // gusset: the diagonal web behind the column; hulls to a tight wedge.
        ([-INF, -0.031, 0.009], [-0.0305, 0.031, 0.226]),
        // column: the square shaft, fenced off the gusset at x -0.0335.
        ([-0.0335, -INF, 0.078], [INF, INF, 0.6025]),
        // head bands: collar and skirt, waist and mid, wide band, top taper.
        ([-INF, -INF, 0.5995], [INF, INF, 0.638]),
        ([-INF, -INF, 0.633], [INF, INF, 0.688]),
        ([-INF, -INF, 0.682], [INF, INF, 0.719]),
        ([-INF, -INF, 0.713], [INF, INF, INF]),
    ]
    .into_iter()
    .map(|(min, max)| {
        ClipRegion::new(
            Point3::new(min[0], min[1], min[2]),
            Point3::new(max[0], max[1], max[2]),
        )
        .expect("torso region bounds are static and valid")
    })
    .collect()
}

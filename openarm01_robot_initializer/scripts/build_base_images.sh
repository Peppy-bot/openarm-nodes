#!/usr/bin/env bash
# Build and push base Docker images with baked-in robot assets.
# Run whenever assets or base image versions change, then rebuild the SIF.
#
# To bump a sim version: update the version variable below, rebuild, and push.
# apptainer.def From: tags must be updated to match after pushing.
#
# Usage:
#   RCLONE_S3_ACCESS_KEY_ID=<key> RCLONE_S3_SECRET_ACCESS_KEY=<secret> bash scripts/build_base_images.sh
set -euo pipefail

export RCLONE_S3_ACCESS_KEY_ID="${RCLONE_S3_ACCESS_KEY_ID:?RCLONE_S3_ACCESS_KEY_ID must be set}"
export RCLONE_S3_SECRET_ACCESS_KEY="${RCLONE_S3_SECRET_ACCESS_KEY:?RCLONE_S3_SECRET_ACCESS_KEY must be set}"

# ── Version manifest ──────────────────────────────────────────────────────────
ISAAC_VERSION="5.1.0"   # mirrors nvcr.io/nvidia/isaac-sim upstream version
MUJOCO_VERSION="3.8.1"  # mirrors mujoco PyPI version (requirements.mujoco.txt)
IMAGE_REV="8"           # bump when image content changes without an upstream version bump
# ─────────────────────────────────────────────────────────────────────────────

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ISAAC_IMAGE="aaqibmahamood/openarm01-isaac-sim:${ISAAC_VERSION}-${IMAGE_REV}"
MUJOCO_IMAGE="aaqibmahamood/openarm01-mujoco-sim:${MUJOCO_VERSION}-${IMAGE_REV}"

echo "==> Building Isaac base image (${ISAAC_IMAGE})..."
DOCKER_BUILDKIT=1 docker build \
    --network=host \
    --secret id=rclone_key,env=RCLONE_S3_ACCESS_KEY_ID \
    --secret id=rclone_secret,env=RCLONE_S3_SECRET_ACCESS_KEY \
    --build-arg ISAAC_VERSION="${ISAAC_VERSION}" \
    -t "${ISAAC_IMAGE}" \
    -f "${REPO_ROOT}/scripts/Dockerfile.isaac" \
    "${REPO_ROOT}"

echo "==> Building MuJoCo base image (${MUJOCO_IMAGE})..."
DOCKER_BUILDKIT=1 docker build \
    --network=host \
    --secret id=rclone_key,env=RCLONE_S3_ACCESS_KEY_ID \
    --secret id=rclone_secret,env=RCLONE_S3_SECRET_ACCESS_KEY \
    -t "${MUJOCO_IMAGE}" \
    -f "${REPO_ROOT}/scripts/Dockerfile.mujoco" \
    "${REPO_ROOT}"

echo "==> Pushing images..."
docker push "${ISAAC_IMAGE}"
docker push "${MUJOCO_IMAGE}"

echo "==> Done."
echo "    Update apptainer.def From: tags to match, then run:"
echo "    peppy node build openarm01_robot_initializer:v1"

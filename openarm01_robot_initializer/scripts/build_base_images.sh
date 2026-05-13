#!/usr/bin/env bash
# Build and push base Docker images with baked-in robot assets.
# Run whenever assets or base image versions change, then rebuild the SIF.
#
# Usage:
#   RCLONE_S3_ACCESS_KEY_ID=<key> RCLONE_S3_SECRET_ACCESS_KEY=<secret> bash scripts/build_base_images.sh
set -euo pipefail

export RCLONE_S3_ACCESS_KEY_ID="${RCLONE_S3_ACCESS_KEY_ID:?RCLONE_S3_ACCESS_KEY_ID must be set}"
export RCLONE_S3_SECRET_ACCESS_KEY="${RCLONE_S3_SECRET_ACCESS_KEY:?RCLONE_S3_SECRET_ACCESS_KEY must be set}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "==> Building Isaac base image..."
DOCKER_BUILDKIT=1 docker build \
    --secret id=rclone_key,env=RCLONE_S3_ACCESS_KEY_ID \
    --secret id=rclone_secret,env=RCLONE_S3_SECRET_ACCESS_KEY \
    -t aaqibmahamood/openarm01-isaac-sim:5.1.0 \
    -f "${REPO_ROOT}/scripts/Dockerfile.isaac" \
    "${REPO_ROOT}"

echo "==> Building MuJoCo base image..."
DOCKER_BUILDKIT=1 docker build \
    --secret id=rclone_key,env=RCLONE_S3_ACCESS_KEY_ID \
    --secret id=rclone_secret,env=RCLONE_S3_SECRET_ACCESS_KEY \
    -t aaqibmahamood/openarm01-mujoco-sim:0.1.0 \
    -f "${REPO_ROOT}/scripts/Dockerfile.mujoco" \
    "${REPO_ROOT}"

echo "==> Pushing images..."
docker push aaqibmahamood/openarm01-isaac-sim:5.1.0
docker push aaqibmahamood/openarm01-mujoco-sim:0.1.0

echo "==> Done. Rebuild SIFs with: peppy node build openarm01_robot_initializer:0.1.0"

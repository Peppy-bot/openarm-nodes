#!/usr/bin/env bash
# Build and push base Docker images with baked-in robot assets.
# Run whenever assets or base image versions change, then rebuild the SIF.
#
# The MuJoCo image is published as a multi-arch manifest (linux/amd64 +
# linux/arm64) so the node runs on both x86_64 and aarch64 hosts. Isaac Sim is
# x86_64 only upstream (nvcr.io/nvidia/isaac-sim ships no arm64 image), so it
# stays amd64.
#
# To bump a sim version: update the version variable below, rebuild, and push.
# apptainer.def From: tags must be updated to match after pushing.
#
# Requires a Docker Hub login (docker login) and a Docker daemon with buildx.
# The script provisions QEMU binfmt handlers and a docker-container buildx
# builder so the amd64 host can cross-build the arm64 layers.
#
# Usage:
#   RCLONE_S3_ACCESS_KEY_ID=<key> RCLONE_S3_SECRET_ACCESS_KEY=<secret> bash scripts/build_base_images.sh
set -euo pipefail

export RCLONE_S3_ACCESS_KEY_ID="${RCLONE_S3_ACCESS_KEY_ID:?RCLONE_S3_ACCESS_KEY_ID must be set}"
export RCLONE_S3_SECRET_ACCESS_KEY="${RCLONE_S3_SECRET_ACCESS_KEY:?RCLONE_S3_SECRET_ACCESS_KEY must be set}"

# ── Version manifest ──────────────────────────────────────────────────────────
ISAAC_VERSION="5.1.0"    # mirrors nvcr.io/nvidia/isaac-sim upstream version
MUJOCO_VERSION="3.10.0"  # mirrors mujoco PyPI version (requirements.mujoco.txt)
IMAGE_REV="18"           # bump when image content changes without an upstream version bump
IMAGE_NAMESPACE="peppybot"  # Docker Hub namespace these base images are pushed to

# Target platforms. MuJoCo ships wheels for both arches; Isaac Sim is amd64 only.
MUJOCO_PLATFORMS="linux/amd64,linux/arm64"
ISAAC_PLATFORMS="linux/amd64"
BUILDX_BUILDER="peppy-multiarch"
# ─────────────────────────────────────────────────────────────────────────────

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ISAAC_IMAGE="${IMAGE_NAMESPACE}/openarm-isaac-sim:${ISAAC_VERSION}-${IMAGE_REV}"
MUJOCO_IMAGE="${IMAGE_NAMESPACE}/openarm-mujoco-sim:${MUJOCO_VERSION}-${IMAGE_REV}"

# ── buildx + cross-arch emulation setup ───────────────────────────────────────
# Multi-arch builds need a docker-container driver builder; the default "docker"
# driver can neither build multiple platforms nor push a manifest list. QEMU
# binfmt handlers let the host emulate arm64 while building the arm64 layers.
echo "==> Ensuring QEMU binfmt handlers are registered (arm64 emulation)..."
if [[ ! -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]]; then
    docker run --privileged --rm tonistiigi/binfmt --install arm64
fi

echo "==> Ensuring buildx builder '${BUILDX_BUILDER}' exists..."
if ! docker buildx inspect "${BUILDX_BUILDER}" >/dev/null 2>&1; then
    # network=host preserves the host networking the apt/pip steps rely on.
    docker buildx create --name "${BUILDX_BUILDER}" \
        --driver docker-container \
        --driver-opt network=host \
        --bootstrap
fi
# ─────────────────────────────────────────────────────────────────────────────

echo "==> Building + pushing Isaac base image (${ISAAC_IMAGE}) [${ISAAC_PLATFORMS}]..."
docker buildx build \
    --builder "${BUILDX_BUILDER}" \
    --platform "${ISAAC_PLATFORMS}" \
    --allow network.host \
    --network=host \
    --secret id=rclone_key,env=RCLONE_S3_ACCESS_KEY_ID \
    --secret id=rclone_secret,env=RCLONE_S3_SECRET_ACCESS_KEY \
    --build-arg ISAAC_VERSION="${ISAAC_VERSION}" \
    -t "${ISAAC_IMAGE}" \
    -f "${REPO_ROOT}/scripts/Dockerfile.isaac" \
    --push \
    "${REPO_ROOT}"

echo "==> Building + pushing MuJoCo base image (${MUJOCO_IMAGE}) [${MUJOCO_PLATFORMS}]..."
docker buildx build \
    --builder "${BUILDX_BUILDER}" \
    --platform "${MUJOCO_PLATFORMS}" \
    --allow network.host \
    --network=host \
    --secret id=rclone_key,env=RCLONE_S3_ACCESS_KEY_ID \
    --secret id=rclone_secret,env=RCLONE_S3_SECRET_ACCESS_KEY \
    -t "${MUJOCO_IMAGE}" \
    -f "${REPO_ROOT}/scripts/Dockerfile.mujoco" \
    --push \
    "${REPO_ROOT}"

echo "==> Done."
echo "    Pushed:"
echo "      ${ISAAC_IMAGE}   (${ISAAC_PLATFORMS})"
echo "      ${MUJOCO_IMAGE}   (${MUJOCO_PLATFORMS})"
echo "    Update apptainer.def From: tags to match, then run:"
echo "    peppy node build openarm_robot_initializer_mujoco:v1 (and _isaac)"

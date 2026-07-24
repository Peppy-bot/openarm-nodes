#!/usr/bin/env bash
# Build and push base Docker images with baked-in robot assets.
# Run whenever assets or base image versions change, then rebuild the SIF.
#
# The MuJoCo image is published as a multi-arch manifest (linux/amd64 +
# linux/arm64) so the node runs on both x86_64 and aarch64 hosts. Isaac Sim is
# x86_64 only upstream (nvcr.io/nvidia/isaac-sim ships no arm64 image), so it
# stays amd64.
#
# To bump a sim version: update the version variable below and rerun. This
# manifest is the single source of truth for the base image tags; the script
# rebuilds, pushes, and rewrites the sibling sim nodes' apptainer.def From:
# tags to match, so the tag is never edited by hand in two places.
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
#
# Bootstrap images are pinned by digest (multi-arch OCI indexes) so the
# emulation and BuildKit versions are reproducible and tamper-evident. Refresh
# with `docker buildx imagetools inspect <ref>` when intentionally upgrading.
BINFMT_IMAGE="tonistiigi/binfmt@sha256:400a4873b838d1b89194d982c45e5fb3cda4593fbfd7e08a02e76b03b21166f0"
BUILDKIT_IMAGE="moby/buildkit@sha256:2f5adac4ecd194d9f8c10b7b5d7bceb5186853db1b26e5abd3a657af0b7e26ec"
# BuildKit denies host-network RUN steps unless its daemon starts with the
# network.host entitlement (a buildkitd flag, settable only at creation). The
# per-build --allow network.host requests it; this grants it.
BUILDKITD_FLAGS="--allow-insecure-entitlement network.host"

echo "==> Registering QEMU binfmt handlers (arm64 emulation)..."
docker run --privileged --rm "${BINFMT_IMAGE}" --install arm64

echo "==> Ensuring buildx builder '${BUILDX_BUILDER}' has the host-network entitlement..."
# Reuse the builder only if its buildkit daemon already carries the entitlement;
# otherwise replace it, since the flag cannot be added to a running builder. The
# underlying buildkit container's config is the reliable source of truth (a
# substring test on captured output avoids a grep -q/pipefail SIGPIPE race).
if docker buildx inspect "${BUILDX_BUILDER}" >/dev/null 2>&1 \
    && [[ "$(docker inspect "buildx_buildkit_${BUILDX_BUILDER}0" 2>/dev/null || true)" == *network.host* ]]; then
    echo "    reusing existing builder"
else
    docker buildx rm "${BUILDX_BUILDER}" >/dev/null 2>&1 || true
    # network=host puts the buildkit daemon on the host network; the buildkitd
    # flag additionally permits host-network RUN steps in the build.
    docker buildx create --name "${BUILDX_BUILDER}" \
        --driver docker-container \
        --driver-opt network=host \
        --driver-opt "image=${BUILDKIT_IMAGE}" \
        --buildkitd-flags "${BUILDKITD_FLAGS}" \
        --bootstrap
fi

# Fail fast if the active builder cannot actually emulate arm64, rather than
# silently publishing an amd64-only "multi-arch" manifest.
echo "==> Verifying builder '${BUILDX_BUILDER}' offers linux/arm64..."
BUILDER_PLATFORMS="$(docker buildx inspect --bootstrap "${BUILDX_BUILDER}" || true)"
if [[ "${BUILDER_PLATFORMS}" != *linux/arm64* ]]; then
    echo "ERROR: builder '${BUILDX_BUILDER}' does not report linux/arm64 support;" >&2
    echo "       arm64 emulation is unavailable. Aborting before publish." >&2
    exit 1
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

# ── Sync sim node apptainer.def From: tags ────────────────────────────────────
# This manifest owns the base image tags; the sibling sim nodes track it here
# rather than being hand-edited. peppy's `node build` invokes a bare
# `apptainer build <sif> <def>` with no --build-arg support, so the tag cannot
# be templated at build time and is stamped into the def files instead.
NODES_DIR="$(cd "${REPO_ROOT}/.." && pwd)"
sync_def_from() {
    local def_file="$1" image="$2"
    sed -i -E "s|^(From:[[:space:]]+).*|\1${image}|" "${def_file}"
    echo "    $(basename "$(dirname "${def_file}")")/apptainer.def -> ${image}"
}
echo "==> Syncing sim node apptainer.def From: tags..."
sync_def_from "${NODES_DIR}/openarm_sim_isaac/apptainer.def" "${ISAAC_IMAGE}"
sync_def_from "${NODES_DIR}/openarm_sim_mujoco/apptainer.def" "${MUJOCO_IMAGE}"

echo "==> Done."
echo "    Pushed and synced:"
echo "      ${ISAAC_IMAGE}   (${ISAAC_PLATFORMS})"
echo "      ${MUJOCO_IMAGE}   (${MUJOCO_PLATFORMS})"
echo "    Commit the updated apptainer.def files, then run:"
echo "    peppy node build openarm_robot_initializer_mujoco:v1 (and _isaac)"

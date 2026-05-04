#!/usr/bin/env bash
# Download robot assets from R2 into staging dirs that apptainer.def %setup copies into the SIF.
# Run this once before `peppy node build` whenever assets change.
#
# Usage:
#   RCLONE_S3_ACCESS_KEY_ID=<key> RCLONE_S3_SECRET_ACCESS_KEY=<secret> bash scripts/download_assets.sh
set -euo pipefail

KEY_ID="${RCLONE_S3_ACCESS_KEY_ID:?RCLONE_S3_ACCESS_KEY_ID must be set}"
SECRET="${RCLONE_S3_SECRET_ACCESS_KEY:?RCLONE_S3_SECRET_ACCESS_KEY must be set}"
ENDPOINT="https://b9abcee11c090aef5279f874ff078826.r2.cloudflarestorage.com"
BUCKET="peppy-data01"

if ! command -v rclone &>/dev/null; then
    echo "ERROR: rclone not found. Install with: sudo apt-get install rclone" >&2
    exit 1
fi

_rclone() {
    RCLONE_CONFIG_R2_TYPE=s3 \
    RCLONE_CONFIG_R2_PROVIDER=Cloudflare \
    RCLONE_CONFIG_R2_ACCESS_KEY_ID="${KEY_ID}" \
    RCLONE_CONFIG_R2_SECRET_ACCESS_KEY="${SECRET}" \
    RCLONE_CONFIG_R2_ENDPOINT="${ENDPOINT}" \
    rclone "$@"
}

echo "==> Downloading MuJoCo assets..."
rm -rf /tmp/.peppy_backbone_mujoco
mkdir -p /tmp/.peppy_backbone_mujoco
_rclone copy "r2:${BUCKET}/openarm01/mujoco/assets/" /tmp/.peppy_backbone_mujoco/ --progress

echo "==> Downloading Isaac assets..."
rm -rf /tmp/.peppy_backbone_isaac
mkdir -p /tmp/.peppy_backbone_isaac
_rclone copy "r2:${BUCKET}/openarm01/isaac/assets/" /tmp/.peppy_backbone_isaac/ --progress

echo "==> Done. Now run: peppy node build openarm01_backbone:0.1.0"

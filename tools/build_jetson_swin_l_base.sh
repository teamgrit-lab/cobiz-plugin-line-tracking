#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE_IMAGE="${SWIN_L_JETSON_BASE_SOURCE_IMAGE:-cobiz:jetson}"
TORCH_IMAGE="${SWIN_L_JETSON_TORCH_IMAGE:-cobiz-plugin-line-tracking-swin-l-jetson:torch}"
TARGET_IMAGE="${SWIN_L_JETSON_BASE_IMAGE:-cobiz-plugin-line-tracking-swin-l-jetson:r36.5}"
CUDA_VERSION="${SWIN_L_CUDA_VERSION:-12.6}"
JETSON_CONTAINERS_BIN="${JETSON_CONTAINERS_BIN:-}"

if [[ -z "${JETSON_CONTAINERS_BIN}" ]]; then
    JETSON_CONTAINERS_BIN="$(command -v jetson-containers || true)"
fi

if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "[jetson-build] this build must run on the Jetson aarch64 host" >&2
    exit 2
fi

if [[ -z "${JETSON_CONTAINERS_BIN}" ]]; then
    cat >&2 <<'EOF'
[jetson-build] jetson-containers is not installed or is not on PATH.
Install it on the Jetson host with:
  git clone --recursive https://github.com/dusty-nv/jetson-containers.git
  cd jetson-containers
  sudo bash install.sh
EOF
    exit 2
fi

if ! docker image inspect "${SOURCE_IMAGE}" >/dev/null 2>&1; then
    echo "[jetson-build] source image is not available locally: ${SOURCE_IMAGE}" >&2
    exit 2
fi

echo "[jetson-build] source=${SOURCE_IMAGE} cuda=${CUDA_VERSION}" >&2
echo "[jetson-build] torch layer=${TORCH_IMAGE}" >&2
echo "[jetson-build] final image=${TARGET_IMAGE}" >&2

# jetson-containers selects the Jetson-compatible PyTorch/torchvision wheels
# and falls back to a matching source build when a wheel is unavailable.  The
# explicit CUDA version keeps this aligned with L4T R36.5.0 on the target.
BUILD_LOG="$(mktemp)"
trap 'rm -f "${BUILD_LOG}"' EXIT

CUDA_VERSION="${CUDA_VERSION}" "${JETSON_CONTAINERS_BIN}" build \
    --base="${SOURCE_IMAGE}" \
    --name="${TORCH_IMAGE}" \
    pytorch torchvision transformers 2>&1 | tee "${BUILD_LOG}"

# jetson-containers may append its package/L4T tag to --name.  Prefer the
# requested name when it was created as-is; otherwise recover the final name
# from the tool's success line and create the stable alias used by this repo.
BUILT_IMAGE=""
if docker image inspect "${TORCH_IMAGE}" >/dev/null 2>&1; then
    BUILT_IMAGE="${TORCH_IMAGE}"
else
    BUILT_IMAGE="$(awk '/✅.*jetson-containers build/ { line=$0 } END {
        sub(/^.*\\) \\(/, "", line)
        sub(/\\).*/, "", line)
        print line
    }' "${BUILD_LOG}")"
fi

if [[ -z "${BUILT_IMAGE}" ]] || ! docker image inspect "${BUILT_IMAGE}" >/dev/null 2>&1; then
    echo "[jetson-build] could not resolve the image produced by jetson-containers" >&2
    echo "[jetson-build] inspect ${BUILD_LOG} for the build output" >&2
    exit 1
fi

if [[ "${BUILT_IMAGE}" != "${TORCH_IMAGE}" ]]; then
    docker tag "${BUILT_IMAGE}" "${TORCH_IMAGE}"
fi

docker build \
    --build-arg "BASE_IMAGE=${TORCH_IMAGE}" \
    --tag "${TARGET_IMAGE}" \
    --file "${PROJECT_ROOT}/Dockerfile.swin-l-jetson-base" \
    "${PROJECT_ROOT}"

echo "[jetson-build] completed: ${TARGET_IMAGE}" >&2

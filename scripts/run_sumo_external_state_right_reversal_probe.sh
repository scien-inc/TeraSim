#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
builder_image=${SUMO_RIGHT_REVERSAL_PROBE_BUILDER_IMAGE:-terasim-sumo-right-reversal-probe-builder:20260814}
build_jobs=${SUMO_RIGHT_REVERSAL_PROBE_BUILD_JOBS:-2}
artifact_dir=${SUMO_RIGHT_REVERSAL_PROBE_ARTIFACT_DIR:-"${repo_root}/outputs/carla_diagnostics/sumo_external_state_right_reversal_probe"}
artifact_uid=$(id -u)
artifact_gid=$(id -g)

mkdir -p "${artifact_dir}"

docker build \
    --progress=plain \
    --target sumo-builder \
    --build-arg "SUMO_BUILD_JOBS=${build_jobs}" \
    -t "${builder_image}" \
    -f "${repo_root}/Dockerfile.sumo-external-state" \
    "${repo_root}"

docker run --rm \
    -v "${repo_root}:/workspace:ro" \
    -v "${artifact_dir}:/probe-output" \
    -w /workspace \
    -e "SUMO_RIGHT_REVERSAL_PROBE_BUILD_JOBS=${build_jobs}" \
    -e "SUMO_RIGHT_REVERSAL_PROBE_ARTIFACT_UID=${artifact_uid}" \
    -e "SUMO_RIGHT_REVERSAL_PROBE_ARTIFACT_GID=${artifact_gid}" \
    --entrypoint /bin/bash \
    "${builder_image}" \
    -c '
set -euo pipefail
restore_artifact_owner() {
    chown -R "${SUMO_RIGHT_REVERSAL_PROBE_ARTIFACT_UID}:${SUMO_RIGHT_REVERSAL_PROBE_ARTIFACT_GID}" /probe-output
}
trap restore_artifact_owner EXIT

cmake --build /opt/sumo-source/build-external-state \
    --target testlibsumo \
    -j"${SUMO_RIGHT_REVERSAL_PROBE_BUILD_JOBS}"
g++ \
    -std=gnu++14 \
    -O2 \
    -pthread \
    -DFLOAT_MATH_FUNCTIONS \
    -DHAVE_LIBSUMOGUI \
    -I/opt/sumo-source/build-external-state/src \
    -I/opt/sumo-source/src \
    -isystem /usr/include/freetype2 \
    -isystem /usr/include/eigen3 \
    -I/usr/include/fox-1.6 \
    /workspace/tests/test_integration/support/sumo_external_state_right_reversal_probe.cpp \
    -Wl,-rpath,/opt/sumo-source/bin \
    /opt/sumo-source/bin/libsumocpp.so \
    /usr/lib/x86_64-linux-gnu/libxerces-c.so \
    /usr/lib/x86_64-linux-gnu/libz.so \
    /usr/lib/x86_64-linux-gnu/libproj.so \
    -o /tmp/sumo_external_state_right_reversal_probe
python3 -m pip install --disable-pip-version-check -q pytest==8.3.5
SUMO_EXTERNAL_STATE_RIGHT_REVERSAL_PROBE=/tmp/sumo_external_state_right_reversal_probe \
SUMO_EXTERNAL_STATE_RIGHT_REVERSAL_ARTIFACT_DIR=/probe-output \
PYTHONPATH=/workspace/packages/terasim-service:/workspace/packages/terasim:/workspace/packages/terasim-nde-nade \
PYTHONDONTWRITEBYTECODE=1 \
python3 -m pytest \
    -o addopts= \
    -p no:cacheprovider \
    -q \
    tests/test_integration/test_sumo_external_state_right_reversal_probe.py
'

printf 'Right-reversal probe artifacts: %s\n' "${artifact_dir}"

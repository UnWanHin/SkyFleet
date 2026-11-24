#!/usr/bin/env bash
set -euo pipefail
VERSION=ns-3.46
PREFIX="${SKYFLEET_NS3_PREFIX:-${HOME}/.local/opt/${VERSION}}"
BUILD="${SKYFLEET_NS3_BUILD_DIR:-${PWD}/.build/ns3-bridge}"
if [[ ! -f "${PREFIX}/lib64/cmake/ns3/ns3Config.cmake" && ! -f "${PREFIX}/lib/cmake/ns3/ns3Config.cmake" ]]; then
  printf 'ns-3.46 not found at %s\n' "${PREFIX}" >&2
  printf 'Build/install ns-3.46 first; see ns3_bridge/README.md\n' >&2
  exit 2
fi
cmake -S ns3_bridge -B "${BUILD}" -G Ninja -DCMAKE_PREFIX_PATH="${PREFIX}"
cmake --build "${BUILD}" --parallel "${SKYFLEET_BUILD_JOBS:-4}"
printf 'export SKYFLEET_NS3_BRIDGE=%q\n' "${BUILD}/skyfleet-ns3-bridge"

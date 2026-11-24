#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

PYTHON="$ROOT/.venv/bin/python"
BRIDGE="${SKYFLEET_NS3_BRIDGE:-$ROOT/.build/ns3-bridge/skyfleet-ns3-bridge}"
if [[ ! -x "$PYTHON" ]]; then
  printf "Python environment missing: %s (see README.md)\n" "$PYTHON" >&2
  exit 2
fi
if [[ ! -x "$BRIDGE" ]]; then
  printf "ns-3 bridge missing: %s (run bash ns3_bridge/build.sh)\n" "$BRIDGE" >&2
  exit 2
fi

export SKYFLEET_NS3_BRIDGE="$BRIDGE"
exec "$PYTHON" -m skyfleet.cli live \
  --config skyfleet/scenarios/ns3_mujoco_urban_multihop.json \
  --city skyfleet/scenarios/d5_urban_city.geojson \
  --duration 120 --tick-seconds 0.1 --port 8765 --viewer mujoco \
  "$@"

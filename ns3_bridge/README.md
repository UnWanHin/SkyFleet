# SkyFleet × ns-3.46 × MuJoCo

`network_backend=ns3` uses ns-3.46 ad-hoc IEEE 802.11a MAC/PHY and a 3D
LogDistance path-loss channel. SkyFleet remains responsible for mission,
relay next-hop choice, ACK/retry, flight telemetry, station correction and
safety logic. MuJoCo advances physical aircraft; every motion step exports
3D positions to ns-3; every actual Wi-Fi reception returns to SkyFleet at its
ns-3 virtual timestamp. **The classic slotted channel is not used in ns3 mode.**

## Build once (Fedora / Linux, user-local install)

```bash
# From the SkyFleet repository root:
mkdir -p .build
git clone --depth 1 --branch ns-3.46 https://gitlab.com/nsnam/ns-3-dev.git .build/ns-3.46
cmake -S .build/ns-3.46 -B .build/ns-3.46/build -G Ninja \
  -DNS3_EXAMPLES=OFF -DNS3_TESTS=OFF \
  -DNS3_ENABLED_MODULES='core;network;mobility;wifi;internet'
cmake --build .build/ns-3.46/build --parallel 4
cmake --install .build/ns-3.46/build --prefix "$HOME/.local/opt/ns-3.46"
./ns3_bridge/build.sh
export SKYFLEET_NS3_BRIDGE="$PWD/.build/ns3-bridge/skyfleet-ns3-bridge"
```

Dependencies: CMake, Ninja, C++23 compiler and Python 3. For a fresh
checkout, create the Python environment from the repository root before
running the commands below:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install mujoco numpy
.venv/bin/python -c 'import mujoco, numpy; print(mujoco.__version__, numpy.__version__)'
```

The ns-3 bridge runs as a separate process; MuJoCo and NumPy are required by
`flightlab` in `.venv`, not by the classic SkyFleet backend.
`SKYFLEET_NS3_PREFIX`/`SKYFLEET_NS3_BUILD_DIR` can override install
and bridge build locations. The bridge build is local and ignored by Git.

## Run

```bash
export SKYFLEET_NS3_BRIDGE="$PWD/.build/ns3-bridge/skyfleet-ns3-bridge"
.venv/bin/python -m skyfleet.cli run --config skyfleet/scenarios/ns3_mujoco.json \
  --duration 15 --log out/logs/ns3_mujoco.jsonl
.venv/bin/python -m skyfleet.cli metrics --log out/logs/ns3_mujoco.jsonl
.venv/bin/python -m skyfleet.cli replay --log out/logs/ns3_mujoco.jsonl \
  --out out/replay/ns3_mujoco.html
SKYFLEET_NS3_BRIDGE="$SKYFLEET_NS3_BRIDGE" \
  .venv/bin/python -m unittest tests.integration.test_ns3_bridge -v
```

To check the closed-loop traffic in the documented seed-1, 15-second run,
count received flight telemetry and station corrections in the event log:

```bash
.venv/bin/python - <<'PY'
import json
from collections import Counter

with open('out/logs/ns3_mujoco.jsonl', encoding='utf-8') as events:
    received = Counter(
        row['msg_type']
        for line in events
        if (row := json.loads(line)).get('kind') == 'MSG_RX'
    )
print('received telemetry:', received['TELEMETRY'])
print('received corrections:', received['FLIGHT_CORRECTION'])
assert received['TELEMETRY'] > 0 and received['FLIGHT_CORRECTION'] > 0
PY
```

This checks radio delivery; `tests.integration.test_ns3_bridge` additionally
checks that a delivered correction changes the MuJoCo flight controller.
`replay` is a top-down 2D visualization. `replay3d` produces an offline,
self-contained WebGL2 browser replay with extruded buildings, interpolated
3D drone positions/tracks and observed ns-3 TX/receive link segments:

```bash
.venv/bin/python -m skyfleet.cli replay3d --log out/logs/ns3_mujoco.jsonl \
  --out out/replay/ns3_mujoco_3d.html
```

Open the resulting HTML in a WebGL2-capable browser; drag to orbit, scroll
to zoom, and use the timeline to scrub or play. This visualizes logged
samples, not a live MuJoCo window; buildings do not have MuJoCo collision
meshes. `flightlab view --swarm 11` remains an independent visual demo.

## Fixed D5 building-shadow comparison

```bash
export SKYFLEET_NS3_BRIDGE="$PWD/.build/ns3-bridge/skyfleet-ns3-bridge"
.venv/bin/python -m skyfleet.study.ns3_d5 --out out/ns3_d5
.venv/bin/python -m skyfleet.cli replay3d --log out/ns3_d5/relay.jsonl \
  --out out/ns3_d5/relay3d.html
.venv/bin/python -m skyfleet.cli replay3d --log out/ns3_d5/control.jsonl \
  --out out/ns3_d5/control3d.html
.venv/bin/python -m unittest tests.integration.test_ns3_bridge -q
```

The experiment uses seed 1, `physics_movers=true`, ns-3 Wi-Fi, and a
20 × 20 m, 60 m tall building spanning x=40..60, y=-10..10. D5 (`S5` in
code) follows a fixed MuJoCo route from (0,45,30) via (100,0,30) back to
(0,45,30). D3 (`R3` in code) is at (50,60,80); L is at (0,0,30). Other
agents are out of radio range; there are no assigned deliveries. Both runs
use the same flight, seed, building geometry and 120 dB attenuation. Only
`relay_direct_threshold` changes: 0.6 enables D3 and 0.0 is direct-only.

The D5 study logs flight positions every 0.2 s for the 3D replay;
other scenarios retain the original 5 s sampling unless configured with
`replay_sample_period`. The HTML frames nodes near the building in this fixture so distant
parked agents do not dominate the camera framing.
Orange/red radio segments indicate whether that particular transmitted hop
has a matching reception/forward event; they do not predict future signal.
The generated `out/ns3_d5/report.json` records transition times, sent
telemetry, **actual** received telemetry matched by message ID, and
forwarded messages matched to reception. The two JSONL logs retain the
source events. The margin after each transition excludes packets queued
while the radio ray is moving through the building boundary. In the
measured seed-1 run: occlusion enters at 6.771 s and exits at 15.583 s;
inside the steady blocked window the direct-only arm sends 13 packets to L
and L receives 0, while the D3 arm sends 13 through R3 and L receives all
13 with matching `MSG_FORWARD` events. In the recovery window 27 direct
telemetry packets reach L in each arm; D5 changes to R3 at the first
observed version-2 TX (8.769 s) and returns to L at version-3 TX
(17.629 s). These are **first TX observations of the route version**, not
exact switch timestamps. OCCLUSION_ENTER/EXIT describe SkyFleet's geometric
assessment; the matched MSG_RX rows establish ns-3 delivery separately.

## Synthetic urban district: buildings, roads, park and hill

The original one-building D5 fixture stays as the fixed numeric baseline.
For a fuller visual scene, `d5_urban_city.geojson` adds 14 varied-height
buildings to the **same** ns-3 radio and SkyFleet routing geometry. It also
carries an explicitly `visual-only` terrain layer: a low hill, street strips
and a park. Those terrain features appear in the WebGL2 replay but do **not**
change MuJoCo collisions, flight physics or radio propagation. The synthetic
scene is not a surveyed real city or a terrain-aware physical simulation.

```bash
export SKYFLEET_NS3_BRIDGE="$PWD/.build/ns3-bridge/skyfleet-ns3-bridge"
.venv/bin/python -m skyfleet.study.ns3_d5 \
  --city skyfleet/scenarios/d5_urban_city.geojson --out out/ns3_urban
.venv/bin/python -m skyfleet.cli replay3d --log out/ns3_urban/relay.jsonl \
  --out out/ns3_urban/relay3d.html
.venv/bin/python -m skyfleet.cli replay3d --log out/ns3_urban/control.jsonl \
  --out out/ns3_urban/control3d.html
```

The urban fixture now adds a genuine open-air **archway**: two pillars
`ARCH_WEST` and `ARCH_EAST` flank an elevated `ARCH_BEAM` at x=39..61,
y=64..86, base=48 m and roof=65 m. After the original D5 obstruction
window, the MuJoCo-driven D5 takes four extra waypoints and passes under
the beam at about 30 m altitude (sampled passage at 32.9–34.5 s). The
WebGL model and ns-3 radio shadow both use this base-height geometry; a
unit test checks SkyFleet's clear low-altitude link and an integration test
checks ns-3 low-path reception, high-path loss and sampled clearance.
This is a **verified trajectory/clearance**, not a MuJoCo building-collision
mesh; the controller does not dynamically avoid other structures.
The extra route creates later city shadow events. The original first D5
window remains the report's comparison window: relay arm 13 forwarded-and-
received telemetry packets versus direct-only 0 receptions; 27 direct
packets reach L after first recovery. The report and HTML are under
`out/ns3_urban/`.

## Four accepted votes for D1 takeover (opt-in)

```bash
SKYFLEET_NS3_BRIDGE="$PWD/.build/ns3-bridge/skyfleet-ns3-bridge" \
  .venv/bin/python -m skyfleet.cli run --config skyfleet/scenarios/ns3_quorum4.json
SKYFLEET_NS3_BRIDGE="$PWD/.build/ns3-bridge/skyfleet-ns3-bridge" \
  .venv/bin/python -m skyfleet.cli run --config skyfleet/scenarios/ns3_quorum3.json
```

Set `election_votes_required=4` to turn on quorum; default `0` preserves
legacy election behavior and the classic regression baseline. On leader
failure the lowest live non-relay candidate (S1, corresponding to D1)
counts its own vote and receives one ballot from each other candidate over
the selected transport. Only accepted ballots from distinct, live,
non-grounded nodes in the current epoch count; ballots are staggered to
reduce simultaneous collisions, and retries are handled by SkyFleet.
Fewer than four accepted votes writes `ELECTION_DEGRADED` once per failed
leader and does not hand out the leader role. If still degraded after a
three-heartbeat grace period, live non-grounded drones enter HOVER with a
`SAFETY_ACTION` event carrying `cause=ELECTION_QUORUM`; earlier threshold
recovery cancels this scheduled action. In seed-1 ns-3 scenarios, the 4-voter
case receives four ballots then announces S1 at 2.676 s; the 3-voter
case records three ballots without a leader announcement. A transient
`ELECTION_DEGRADED` appears before quorum fills even in the 4-voter case.
This is a compact threshold demonstration, not a formal partition-tolerant
consensus implementation; candidates with incompatible views and dynamic
partition healing require further protocol design.

## City geometry inputs and reusable data

The D5 fixture is [`d5_city.geojson`](../skyfleet/scenarios/d5_city.geojson):
**synthetic**, one axis-aligned 20 × 20 m building with a 60 m roof. It is
loaded through `SimConfig.city_geojson` by both SkyFleet's route model and
ns-3's building-shadow model, replacing the seeded grid; the original grid
remains the default. Input must explicitly declare `skyfleet_units: "metres"`,
provide projected local XY metres, positive `properties.height` in metres,
and consist of closed, hole-free **axis-aligned rectangles**. Optional
`properties.base_height` (metres, below `height`) makes an elevated beam
with a clear volume underneath; legacy blocks start at zero. Geographic
lon/lat and arbitrary polygon bounding boxes are rejected: a bounding box
can incorrectly block a radio line that misses the original footprint.
Convert a chosen city's building data to projected metric coordinates and
rectangular footprints before using it. This currently models radio
occlusion only: MuJoCo has no building collision meshes or rendered 3D city.

There **are** open-source city model sources: [OpenStreetMap](https://www.openstreetmap.org/copyright)
provides attributed street/building footprints; [CityJSON public datasets](https://www.cityjson.org/datasets/)
provide city-model geometry; [Cesium 3D Tiles samples](https://github.com/CesiumGS/3d-tiles-samples)
demonstrate ready-to-render 3D tiles. The [Microsoft Global ML Building
Footprints](https://github.com/microsoft/GlobalMLBuildingFootprints) README
states CDLA Permissive 2.0; measured building heights are not present for
every footprint. Rendering assets in Cesium/CityJSON format do not directly
become exact ns-3 blockers or MuJoCo collision meshes: projection, geometry
simplification, open-air clearance and material/radio assumptions must be
mapped explicitly. The synthetic arch is the small deterministic proof of
this shared geometry interface, not a downloaded photorealistic mesh.

Useful external sources for preparing a separate, attributed dataset:
[OpenStreetMap building footprints](https://wiki.openstreetmap.org/wiki/Key:building)
(heights are optional; check ODbL attribution),
[Microsoft Global ML Building Footprints](https://github.com/microsoft/GlobalMLBuildingFootprints)
(coverage, geometry and licence vary by region), and
[CityJSON example datasets](https://www.cityjson.org/datasets/)
(3D geometry, normally requires simplification). These are **input sources**,
not assets already downloaded into SkyFleet; verify projected CRS, origin,
footprint geometry, heights and dataset rights per source.

## Model/interpretation limits

- ns-3 provides 802.11a MAC/PHY, 6 Mbit/s fixed data rate, 25 dBm radio,
  LogDistance exponent 2.0, and 3D radio positions; these are **initial
  assumptions**, not calibrated measurements. SkyFleet's seeded static building
  footprints/heights are passed to a second ns-3 propagation-loss model.
  Set `ns3_building_loss_db` (0 by default) to add a **single** fixed dB
  attenuation if a 3D radio segment intersects a building below its roof.
  This is a controlled obstruction experiment, not calibrated diffraction,
  material penetration, or a validated ns-3 Buildings module. It evaluates
  attenuation at packet transmission time with piecewise-constant positions;
  it does not apply a separate SkyFleet packet-loss lottery. The classic
  `link_model` still guides next-hop selection, and is not a radio-loss model
  in ns-3 mode. Example geometry tests: `test_building_blocks_wifi_and_moving_out_restores_delivery`
  and `test_building_relay_bypasses_shadow_and_overflight_clears_it`.
- The bridge carries the message ID in an ns-3 packet padded to the existing
  estimated `Msg.size_bits`; payload/state remains in SkyFleet memory. Wi-Fi
  receives physical frames, but this is not an IP/UDP application model.
- A packet with no reception causes the existing SkyFleet ACK/timeout/safety
  machinery to act; Wi-Fi retransmissions and SkyFleet retries are separate.
  The bridge limits Wi-Fi MAC queue residence to 1 s and releases SkyFleet
  packet bookkeeping after another 0.1 s. This bounds retention for dropped
  and broadcast frames; `DONE` is bookkeeping expiry, not a delivery or PHY
  completion report. `MSG_TX` means handed to ns-3 MAC, not proof of PHY emission.
- Motion is sampled every `motion_period` (default 0.1 s). ns-3 events in
  between are returned at their real virtual timestamp. Positions are held
  piecewise constant between motion updates. The current station command
  adjusts `v_max`, not individual motor RPM.
- The optional backend preserves the classic default for existing reports.
  Results from the ns-3 backend must not be compared as though the classic
  `COLLISION`/`OCCLUDED` drop-reason counts were identical; ns-3 receive loss
  is observed as a missing receive followed by protocol timeout.

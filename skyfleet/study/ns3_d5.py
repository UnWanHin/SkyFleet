"""Fixed D5/D3 building-shadow comparison on MuJoCo flight + ns-3 Wi-Fi.

Run: python -m skyfleet.study.ns3_d5 --out out/ns3_d5
The log is the evidence; the summary pairs SkyFleet TX, relay forwarding, and
actual ns-3 RX by message ID. The route-version timeline is observed at TX.
"""
from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

from skyfleet.kernel.config import load_config
from skyfleet.observe.log import read_log
from skyfleet.sim.engine import Engine

SCENARIO = Path(__file__).resolve().parents[1] / "scenarios" / "ns3_mujoco.json"
CITY = Path(__file__).resolve().parents[1] / "scenarios" / "d5_city.geojson"
URBAN_CITY = Path(__file__).resolve().parents[1] / "scenarios" / "d5_urban_city.geojson"
# The remaining agents are parked out of radio range so that D5's route and
# flight can be compared with identical traffic, seeds, and initial positions.
POSITIONS = {"L": (0.0, 0.0, 30.0), "S5": (0.0, 45.0, 30.0),
             "R3": (50.0, 60.0, 80.0)}
ROUTE = [(100.0, 0.0, 30.0), (0.0, 45.0, 30.0)]
# Optional extension after the baseline D5 shadow/recovery window: weave north
# through the elevated arch between two towers, then return to the start.
ARCH_ROUTE = [(50.0, 25.0, 30.0), (50.0, 45.0, 30.0),
              (50.0, 100.0, 30.0), (0.0, 45.0, 30.0)]


def _run_arm(folder: Path, arm: str, city: Path) -> tuple[list[dict], Path]:
    path = folder / f"{arm}.jsonl"
    extended = city.resolve() == URBAN_CITY.resolve()
    config = replace(load_config(SCENARIO), slaves=5, relays=3, tasks=0,
                     duration=55.0 if extended else 31.0, log_path=str(path),
                     replay_sample_period=0.2,
                     ns3_building_loss_db=120.0, city_geojson=str(city),
                     relay_direct_threshold=0.0 if arm == "control" else 0.6)
    engine = Engine(config).build()
    for index, name in enumerate(engine.nodes):
        engine.nodes[name].pos = POSITIONS.get(
            name, (4000.0 + index * 150, 4000.0 + index * 150, 100.0))
    engine.movers["S5"].set_route(ROUTE + ARCH_ROUTE if extended else ROUTE)
    engine.run()
    return list(read_log(path)), path


def _summary(events: list[dict], enter: float, exit_: float) -> dict:
    # Exclude transition margins: MAC queues and one-second hysteresis can
    # cross the geometry boundary. Match packets to their own actual RX.
    middle = (enter + 1.5, exit_ - 0.5)
    late = (exit_ + 2.0, 31.0)
    tx = [e for e in events if e["kind"] == "MSG_TX" and
          e.get("node_id") == "S5" and e.get("msg_type") == "TELEMETRY"]
    rx = {e["msg_id"] for e in events if e["kind"] == "MSG_RX" and
          e.get("node_id") == "L" and e.get("src") == "S5" and
          e.get("msg_type") == "TELEMETRY"}
    forwards = {e["msg_id"] for e in events if e["kind"] == "MSG_FORWARD"
                and e.get("node_id") == "R3"}
    blocked = [e for e in tx if middle[0] <= e["t"] < middle[1]]
    recovered = [e for e in tx if late[0] <= e["t"] < late[1]]
    upstream = [(e["next_hop"], e["route_version"]) for e in tx]
    switches = [
        {"t": e["t"], "next_hop": e["next_hop"],
         "version": e["route_version"]}
        for previous, e in zip(tx, tx[1:])
        if (previous["next_hop"], previous["route_version"]) !=
           (e["next_hop"], e["route_version"])
    ]
    return {
        "occluded_window": list(middle), "recovered_window": list(late),
        "occluded_direct_tx": sum(e["next_hop"] == "L" for e in blocked),
        "occluded_direct_rx": sum(e["next_hop"] == "L" and e["msg_id"] in rx for e in blocked),
        "occluded_relay_tx": sum(e["next_hop"] == "R3" for e in blocked),
        "occluded_relay_rx": sum(e["next_hop"] == "R3" and e["msg_id"] in rx for e in blocked),
        "occluded_forwarded_rx": sum(e["next_hop"] == "R3" and e["msg_id"] in forwards and e["msg_id"] in rx for e in blocked),
        "recovered_direct_rx": sum(e["next_hop"] == "L" and e["msg_id"] in rx for e in recovered),
        "switches_to_d3": sum(a != "R3" and b == "R3" for (a, _), (b, _) in zip(upstream, upstream[1:])),
        "switches_back_direct": sum(a == "R3" and b == "L" for (a, _), (b, _) in zip(upstream, upstream[1:])),
        "route_switches_observed_at_tx": switches,
    }


def run_comparison(output: Path, city: Path = CITY) -> dict:
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    relay, relay_path = _run_arm(output, "relay", city)
    control, control_path = _run_arm(output, "control", city)

    def boundaries(events: list[dict]) -> tuple[float, float]:
        changes = [(e["kind"], e["t"]) for e in events if
                   e.get("node_id") == "S5" and e["kind"] in
                   ("OCCLUSION_ENTER", "OCCLUSION_EXIT")]
        if len(changes) < 2 or [kind for kind, _ in changes[:2]] != [
                "OCCLUSION_ENTER", "OCCLUSION_EXIT"]:
            raise AssertionError(f"unexpected D5 occlusion transitions: {changes}")
        if city.resolve() != URBAN_CITY.resolve() and len(changes) != 2:
            raise AssertionError(f"unexpected baseline transitions: {changes}")
        return changes[0][1], changes[1][1]

    enter, exit_ = boundaries(relay)
    if boundaries(control) != (enter, exit_):
        raise AssertionError("relay and control trajectories differ")
    from skyfleet.world.city_import import load_buildings_geojson
    buildings = load_buildings_geojson(city)
    building = buildings[0]
    report = {"building": [*building.rect, building.height],
              "building_count": len(buildings), "city_geojson": str(city),
              "ns3_building_loss_db": 120.0, "seed": 1,
              "initial_positions": POSITIONS,
              "d5_route": ROUTE + ARCH_ROUTE if city.resolve() == URBAN_CITY.resolve() else ROUTE,
              "occlusion_enter": enter, "occlusion_exit": exit_,
              "relay_log": str(relay_path), "control_log": str(control_path),
              "relay": _summary(relay, enter, exit_),
              "control": _summary(control, enter, exit_)}
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("out/ns3_d5"))
    parser.add_argument("--city", type=Path, default=CITY,
                        help="projected-metre city GeoJSON (default: original D5 block)")
    args = parser.parse_args()
    report = run_comparison(args.out, city=args.city)
    print(json.dumps({k: v for k, v in report.items() if k not in
                      ("initial_positions", "d5_route")}, indent=2))

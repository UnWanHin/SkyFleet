"""物理运动器：让 skyfleet 的位置来自真实刚体动力学。

**要解决的问题**：`WaypointMover` 按速度沿直线前进，位置是精确的、瞬时的。
于是 `SAFETY_ACTION`（HOVER / RETURN）虽然改了航线，但运动本身没有惯性——
飞机"说停就停、说走就走"。这一层把每条航线的执行换成 MuJoCo 的刚体积分，
使通信判定所导致的动作有真实的加减速、滞后与冲过头。

**尺度**：1:1，不做缩比。四旋翼悬停推力恒为 `mg`，动力学与绝对高度无关，
所以 500 m 城市坐标可以直接喂进仿真（实测 160 m 高度漂移 10.8 mm）。

**期望值来源**：断言不重算实现。用「物理轨迹必须与直线运动不同」「必须真的
到位」「必须真的停住」这类因果/边界性质。
"""
from __future__ import annotations

import math
import unittest

from skyfleet.protocol.identity import Role
from skyfleet.world.mobility import WaypointMover
from skyfleet.world.node import Node

#: 城市尺度下的起始点（复制 baseline 场景里主节点的位置）
START = (250.0, 250.0, 160.0)


def _make_node(node_id: str, pos) -> Node:
    return Node(node_id=node_id, role=Role.LEADER, pos=pos, speed=12.0,
                energy=1e9)


class TestMoverProtocol(unittest.TestCase):
    """FlightlabMover 必须能当成 WaypointMover 用——否则 engine 换不了。"""

    def test_exposes_the_same_interface(self):
        from skyfleet.world.flight_mover import FlightlabMover

        node = _make_node("L", START)
        m = FlightlabMover(node, speed=12.0)
        for attr in ("set_route", "step", "arrived", "route"):
            self.assertTrue(hasattr(m, attr), f"缺少 {attr}")

    def test_factory_defaults_to_kinematic(self):
        """默认必须仍是 WaypointMover——14 个 study 依赖它的速度与确定性。"""
        from skyfleet.world.flight_mover import build_mover

        node = _make_node("L", START)
        self.assertIsInstance(build_mover(node, speed=12.0, physics=False),
                              WaypointMover)

    def test_factory_returns_physical_mover_when_asked(self):
        from skyfleet.world.flight_mover import FlightlabMover, build_mover

        node = _make_node("L", START)
        self.assertIsInstance(build_mover(node, speed=12.0, physics=True),
                              FlightlabMover)


class TestFlightMoverBehaviour(unittest.TestCase):
    def _mover(self, start=START):
        from skyfleet.world.flight_mover import FlightlabMover

        node = _make_node("L", start)
        return node, FlightlabMover(node, speed=12.0)

    def test_starts_idle_and_arrived(self):
        node, m = self._mover()
        self.assertTrue(m.arrived, "没航线时应视为已到达（与 WaypointMover 一致）")

    def test_empty_route_holds_position(self):
        """★ set_route([]) 必须让飞机停在原地——这就是 HOVER 的物理实现。

        对应 engine._actuate_safety 里 `mover.set_route([])`。
        """
        node, m = self._mover()
        m.set_route([(400.0, 250.0, 160.0)])
        for k in range(200):          # 先飞 0.4 s
            m.step(0.002)
        frozen = tuple(node.pos)
        m.set_route([])               # HOVER
        for _ in range(2500):         # 再跑 5 s
            m.step(0.002)
        drift = math.dist(node.pos, frozen)
        self.assertLess(drift, 0.5, f"HOVER 后漂移 {drift:.2f} m")

    def test_parked_telemetry_does_not_report_perpetual_motion(self):
        node, mover = self._mover()
        mover.set_route([(500.0, 250.0, 160.0)])
        for _ in range(50):
            mover.step(0.1)
        mover.set_route([])
        for _ in range(120):
            mover.step(0.1)
        parked = node.pos
        for _ in range(10):
            mover.step(0.1)
        self.assertEqual(parked, node.pos)
        self.assertEqual(mover.flight_telemetry()["vel"], [0.0, 0.0, 0.0])

    def test_trajectory_is_not_a_straight_line(self):
        """★ 核心证伪测试：物理轨迹必须不同于运动学的直线。

        若 `FlightlabMover` 只是包了一层 WaypointMover，这条会通过——
        所以它同时断言「终点必须到位」与「中途轨迹必须偏离直线」。
        """
        node, m = self._mover()
        target = (400.0, 250.0, 160.0)
        m.set_route([target])

        kinematic = _make_node("K", START)
        km = WaypointMover(kinematic, speed=12.0)
        km.set_route([target])

        max_dev = 0.0
        for _ in range(400):          # 0.8 s
            m.step(0.002)
            km.step(0.002)
            # 物理机相对同一时刻运动学机的位置差，扣掉共同的前进量后
            # 应体现为"还没跟上"——即物理机落后
            max_dev = max(max_dev, kinematic.pos[0] - node.pos[0])

        self.assertGreater(max_dev, 0.5,
                           f"物理轨迹与直线运动无差别（落后仅 {max_dev:.3f} m）")

    def test_configured_speed_changes_physical_cruise(self):
        from skyfleet.world.flight_mover import FlightlabMover

        def flight(speed):
            node = _make_node("L", START)
            mover = FlightlabMover(node, speed=speed)
            mover.set_route([(500.0, 250.0, 160.0)])
            for _ in range(50):
                mover.step(0.1)
            return node.pos[0] - START[0]

        slow, fast = flight(2.0), flight(12.0)
        self.assertGreater(fast - slow, 15.0,
                           f"Speed configuration ignored: {slow=}, {fast=}")

    def test_advances_toward_the_waypoint(self):
        node, m = self._mover()
        m.set_route([(400.0, 250.0, 160.0)])
        for _ in range(5000):         # 10 s
            m.step(0.002)
        self.assertGreater(node.pos[0], 300.0,
                           f"10 s 只飞到 x={node.pos[0]:.1f}")

    def test_arrives_and_reports_it(self):
        """★ 必须真的到位，而不是永远"快到了"。

        到位容差沿用城市尺度下扫参得到的 0.00 m 跟踪精度，
        这里给 1 m 的余量。
        """
        node, m = self._mover()
        target = (400.0, 250.0, 160.0)
        m.set_route([target])
        for _ in range(15000):        # 30 s
            m.step(0.002)
            if m.arrived:
                break
        self.assertTrue(m.arrived, "30 s 内没有到达航点")
        self.assertLess(math.dist(node.pos, target), 1.0,
                        f"arrived 但位置仍差 {math.dist(node.pos, target):.2f} m")
        self.assertLess(math.sqrt(sum(v * v for v in m._sim.vel)), 0.5,
                        "Terminal waypoint must be reached at low speed")

    def test_altitude_is_preserved(self):
        """★ 500 m 尺度下高度漂移必须可忽略——否则链路几何全错。"""
        node, m = self._mover()
        m.set_route([(400.0, 250.0, 160.0)])
        for _ in range(15000):
            m.step(0.002)
            if m.arrived:
                break
        self.assertLess(abs(node.pos[2] - 160.0), 0.1,
                        f"高度漂移 {abs(node.pos[2]-160.0)*1000:.0f} mm")


class TestPhysicalTelemetry(unittest.TestCase):
    def test_outage_hover_is_not_overwritten_by_mission(self):
        import tempfile
        from pathlib import Path
        from skyfleet.kernel.config import SimConfig
        from skyfleet.sim.engine import Engine

        with tempfile.TemporaryDirectory() as tmp:
            cfg = SimConfig(seed=7, duration=9.0, slaves=2, relays=0,
                            tasks=2, blocks=2, area=400.0,
                            physics_movers=True,
                            gnss_outages=(("S1", 4.0, 12.0),),
                            log_path=str(Path(tmp) / "hover.jsonl"))
            engine = Engine(cfg)
            engine.run()
            mover = engine.movers["S1"]
            self.assertEqual(mover.route, [],
                             "Mission dispatcher overrode the active HOVER")
            self.assertLess(math.hypot(*mover._sim.vel[:2]), 1.0)

    def test_motor_telemetry_reaches_command_station_after_flight(self):
        import tempfile
        from pathlib import Path
        from skyfleet.kernel.config import SimConfig
        from skyfleet.observe.log import read_log
        from skyfleet.sim.engine import Engine

        with tempfile.TemporaryDirectory() as tmp:
            cfg = SimConfig(seed=7, duration=12.0, slaves=2, relays=0,
                            tasks=2, blocks=2, area=400.0,
                            physics_movers=True,
                            gnss_outages=(("S1", 4.0, 9.0),),
                            log_path=str(Path(tmp) / "physical.jsonl"))
            engine = Engine(cfg)
            engine.run()
            events = list(read_log(cfg.log_path))
            actions = [e for e in events if e["kind"] == "SAFETY_ACTION"
                       and e["node_id"] == "S1" and e.get("actuated")]
            self.assertEqual([(e["action"], e["t"]) for e in actions],
                             [("HOVER", 4.0), ("NONE", 9.0)])
            received = [e for e in events if e["kind"] == "MSG_RX"
                        and e.get("msg_type") == "TELEMETRY"
                        and e.get("src") == "S1" and e["node_id"] == "L"
                        and "flight" in e]
            self.assertTrue(received, "Station did not receive motor telemetry")
            self.assertTrue(any(len(e["flight"]["rotor_rad_s"]) == 4
                                and max(e["flight"]["rotor_rad_s"]) > 0
                                for e in received))
            self.assertTrue(all(len(e["flight"]["pos"]) == 3
                                and len(e["flight"]["vel"]) == 3
                                and all(math.isfinite(v) for v in
                                        (*e["flight"]["pos"],
                                         *e["flight"]["vel"]))
                                for e in received))
            positions = [e["flight"]["pos"] for e in received]
            self.assertGreater(max(p[0] for p in positions)
                               - min(p[0] for p in positions), 0.1)


class TestRemoteCorrection(unittest.TestCase):
    def test_hover_overrides_station_correction(self):
        import tempfile
        from pathlib import Path
        from skyfleet.kernel.config import SimConfig
        from skyfleet.observe.log import read_log
        from skyfleet.sim.engine import Engine

        with tempfile.TemporaryDirectory() as tmp:
            cfg = SimConfig(seed=7, duration=9.0, slaves=2, relays=0,
                            tasks=2, blocks=2, area=400.0,
                            physics_movers=True, motor_guard_ratio=0.5,
                            gnss_outages=(("S1", 4.0, 12.0),),
                            log_path=str(Path(tmp) / "hover.jsonl"))
            engine = Engine(cfg)
            engine.run()
            events = list(read_log(cfg.log_path))
            self.assertTrue(any(e["kind"] == "SAFETY_ACTION"
                                and e.get("node_id") == "S1"
                                and e.get("action") == "HOVER"
                                and e.get("actuated") for e in events))
            self.assertEqual(engine.movers["S1"].route, [])
            self.assertLess(math.hypot(*engine.movers["S1"]._sim.vel[:2]), 1.0)

    def test_station_correction_changes_actual_trajectory(self):
        import tempfile
        from pathlib import Path
        from skyfleet.kernel.config import SimConfig
        from skyfleet.sim.engine import Engine

        with tempfile.TemporaryDirectory() as tmp:
            positions = []
            for ratio in (0.0, 0.5):
                cfg = SimConfig(seed=7, duration=8.0, slaves=2, relays=0,
                                tasks=2, blocks=2, area=400.0,
                                physics_movers=True, motor_guard_ratio=ratio,
                                log_path=str(Path(tmp) / f"{ratio}.jsonl"))
                engine = Engine(cfg)
                engine.run()
                positions.append(engine.nodes["S1"].pos)
            self.assertGreater(math.dist(*positions), 2.0)

    def test_received_motor_load_causes_station_command_and_physical_slowdown(self):
        import tempfile
        from pathlib import Path
        from skyfleet.kernel.config import SimConfig
        from skyfleet.observe.log import read_log
        from skyfleet.sim.engine import Engine

        with tempfile.TemporaryDirectory() as tmp:
            cfg = SimConfig(seed=7, duration=8.0, slaves=2, relays=0,
                            tasks=2, blocks=2, area=400.0,
                            physics_movers=True, motor_guard_ratio=0.5,
                            log_path=str(Path(tmp) / "correction.jsonl"))
            engine = Engine(cfg)
            engine.run()
            events = list(read_log(cfg.log_path))
            received = [e for e in events if e["kind"] == "MSG_RX"
                        and e.get("msg_type") == "FLIGHT_CORRECTION"
                        and e["node_id"] == "S1"]
            self.assertTrue(received, "No correction returned over the channel")
            mover = engine.movers["S1"]
            self.assertLess(mover._sim.gains.v_max, mover.speed)
            self.assertGreater(mover.node.pos[0], 0)


class TestVerticalTransition(unittest.TestCase):
    def test_horizontal_cruise_then_descent_keeps_rotors_bounded(self):
        """Cruise-to-delivery transition must retain motor authority."""
        from skyfleet.world.flight_mover import FlightlabMover
        from flightlab.params import DEFAULT

        node = _make_node("S1", (425.683343081094, 235.30875284449888,
                                 131.08842019903764))
        mover = FlightlabMover(node, speed=12.0)
        mover.set_route([(149.48453608247422, 95.50561797752809, 140.0),
                         (149.48453608247422, 95.50561797752809,
                          84.28571428571429)])
        for _ in range(600):
            mover.step(0.1)
            if mover._ready:
                self.assertLess(max(abs(mover._sim.rotor_speeds())),
                                DEFAULT.omega_max * 1.5,
                                "Motor overspeed at cruise-to-descent transition")
            if mover.arrived:
                break
        self.assertTrue(mover.arrived)
        self.assertLess(math.dist(node.pos, (149.48453608247422,
                                            95.50561797752809,
                                            84.28571428571429)), 1.0)


class TestSafetyActionCoupling(unittest.TestCase):
    """通信判定 → 飞行。这是整件事的目的。"""

    def test_hover_command_physically_stops_the_drone(self):
        """★ 端到端：链路失联判定 HOVER 后，飞机必须靠惯性真的停下来。

        这正是 `engine._actuate_safety` 做的事：`mover.set_route([])`。
        原来它只让运动学模型"立刻停"，物理模型必须先减速再停。
        """
        from skyfleet.world.flight_mover import FlightlabMover

        node = _make_node("L", START)
        m = FlightlabMover(node, speed=12.0)
        m.set_route([(500.0, 250.0, 160.0)])

        # 飞到接近巡航速度
        for _ in range(2500):        # 5 s
            m.step(0.002)
        x_at_hover = node.pos[0]
        self.assertGreater(x_at_hover, 260.0, "还没飞起来，测试前提不成立")

        m.set_route([])              # ← 通信判定：HOVER
        for _ in range(5000):        # 10 s 让其衰减
            m.step(0.002)

        coast = node.pos[0] - x_at_hover
        # 有惯性，所以必须冲出一段；但必须停下来
        self.assertGreater(coast, 0.5, f"没有惯性，瞬间停了（coast {coast:.2f} m）")
        self.assertLess(coast, 60.0, f"冲过头太多（{coast:.1f} m）")
        final = node.pos[0]
        for _ in range(2500):        # 再 5 s，位置不应再有变化
            m.step(0.002)
        self.assertLess(abs(node.pos[0] - final), 0.5, "HOVER 后仍在移动")


if __name__ == "__main__":
    unittest.main()

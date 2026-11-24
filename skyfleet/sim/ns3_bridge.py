"""One ns-3 Wi-Fi process per SkyFleet run; nanosecond, monotone virtual clock."""
from __future__ import annotations

import math
import os
import subprocess
from pathlib import Path
from typing import Mapping, Sequence

from skyfleet.world.building import Building


class Ns3Bridge:
    def __init__(self, positions: Mapping[str, tuple[float, float, float]], seed: int,
                 binary: str | None = None, buildings: Sequence[Building] = (),
                 building_loss_db: float = 0.0) -> None:
        exe = binary or os.environ.get("SKYFLEET_NS3_BRIDGE")
        if not exe or not Path(exe).is_file():
            raise RuntimeError("network_backend=ns3 needs SKYFLEET_NS3_BRIDGE pointing to the built bridge executable")
        self.proc = subprocess.Popen([exe], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, bufsize=1)
        self.now = 0.0
        self._names = frozenset(positions)
        try:
            self._command(f"INIT {len(positions)} {seed}")
            if not math.isfinite(building_loss_db) or not 0 <= building_loss_db <= 200:
                raise ValueError("building_loss_db must be finite and between 0 and 200")
            self._command(f"BUILDING_LOSS {building_loss_db:.12g}")
            for building in buildings:
                x, y, w, d = building.rect
                if not all(math.isfinite(v) for v in (x, y, w, d, building.height, building.base_height)) or w <= 0 or d <= 0 or building.height <= 0 or not 0 <= building.base_height < building.height:
                    raise ValueError(f"invalid ns-3 building: {building.id!r}")
                self._command(f"BUILDING {x:.12g} {y:.12g} {w:.12g} {d:.12g} {building.height:.12g} {building.base_height:.12g}")
            for index, (name, pos) in enumerate(positions.items()):
                if not name.isalnum():
                    raise ValueError(f"Invalid node id for ns-3 bridge: {name!r}")
                self._command(f"NAME {index} {name}")
                self.position(name, pos)
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _ns(t: float) -> int:
        if not math.isfinite(t) or t < 0:
            raise ValueError("ns-3 virtual time must be finite and nonnegative")
        return round(t * 1_000_000_000)

    def _line(self) -> str:
        assert self.proc.stdout is not None
        line = self.proc.stdout.readline()
        if not line:
            diagnostic = self.proc.stderr.read() if self.proc.stderr else ""
            raise RuntimeError(f"ns-3 bridge exited unexpectedly (code {self.proc.wait()}): {diagnostic[-3000:]}")
        return line.strip()

    def _command(self, text: str) -> None:
        if self.proc.poll() is not None:
            raise RuntimeError(f"ns-3 bridge exited with code {self.proc.returncode}")
        assert self.proc.stdin is not None
        self.proc.stdin.write(text + "\n")
        self.proc.stdin.flush()
        response = self._line()
        if response != "OK":
            raise RuntimeError(f"ns-3 bridge rejected {text.split()[0]}: {response}")

    def position(self, node: str, pos: tuple[float, float, float]) -> None:
        if node not in self._names or len(pos) != 3 or not all(math.isfinite(x) for x in pos):
            raise ValueError("invalid ns-3 node position")
        self._command(f"POS {node} {pos[0]:.12g} {pos[1]:.12g} {pos[2]:.12g}")

    def send(self, msg_id: str, src: str, dst: str, size_bits: int, t: float) -> None:
        if not msg_id.isalnum() or src not in self._names or (dst != "*" and dst not in self._names):
            raise ValueError("invalid ns-3 send identity")
        if self._ns(t) < self._ns(self.now):
            raise ValueError("ns-3 send in the past")
        # The MAC payload identifies the application message. Packet size is kept >= msg id.
        size = max((size_bits + 7) // 8, len(msg_id) + 1)
        self._command(f"SEND {msg_id} {src} {dst} {size} {self._ns(t)}")

    def advance(self, t: float) -> list[dict]:
        if self._ns(t) < self._ns(self.now):
            raise ValueError("ns-3 clock must not move backwards")
        assert self.proc.stdin is not None
        self.proc.stdin.write(f"RUN {self._ns(t)}\n")
        self.proc.stdin.flush()
        deliveries = []
        while True:
            line = self._line().split()
            if line[0] == "END":
                reached = int(line[1]) / 1e9
                if reached > t + 1e-9 or reached < self.now - 1e-9:
                    raise RuntimeError("ns-3 clock desynchronized")
                self.now = reached
                break
            if len(line) == 3 and line[0] == "DONE":
                deliveries.append({"id": line[1], "done": True, "t": int(line[2]) / 1e9})
                continue
            if len(line) not in (4, 5) or line[0] != "RX":
                raise RuntimeError(f"Malformed ns-3 bridge reply: {line}")
            event = {"id": line[1], "dst": line[2], "t": int(line[3]) / 1e9}
            if len(line) == 5:
                power = float(line[4])
                if not math.isfinite(power):
                    raise RuntimeError("Invalid ns-3 modeled receive power")
                event["modeled_rx_dbm"] = power
            deliveries.append(event)
        return deliveries

    def close(self) -> None:
        if self.proc.poll() is None:
            try:
                self._command("QUIT")
            except (BrokenPipeError, RuntimeError):
                pass
        if self.proc.stdin:
            self.proc.stdin.close()
        if self.proc.stdout:
            self.proc.stdout.close()
        if self.proc.stderr:
            self.proc.stderr.close()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()

    def __enter__(self) -> "Ns3Bridge":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

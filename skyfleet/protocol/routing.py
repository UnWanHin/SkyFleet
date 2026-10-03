"""路由迟滞与震荡计数。

报告 §3.2：城市峡谷会造成链路质量反复变化，若因一次探测变化就切换路径，
会产生路由震荡、ACK 与重传增加、控制消息排队与尾部时延升高。
因此切换必须"持续满足条件超过 T_hold"才生效，并单独统计短窗口内切回的次数。
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RoutingParams:
    h_up: float = 0.75      # 优于该质量才考虑采纳
    h_down: float = 0.35    # 低于该质量才考虑离开
    t_hold: float = 1.0     # 条件需持续的时间


class _Keep:
    """哨兵：表示保持现状，不启动任何候选。"""

    def __repr__(self) -> str:  # pragma: no cover
        return "<KEEP>"


_KEEP = _Keep()


class RoutingManager:
    """单节点的上游选择。

    parent=None 表示直连主节点。
    """

    def __init__(self, params: RoutingParams, parent: str | None = None,
                 flap_window: float = 5.0) -> None:
        self.params = params
        self.parent = parent
        self.flap_window = flap_window
        self.route_version = 0
        self.switch_count = 0
        self.flap_count = 0
        self._candidate: str | None = None
        self._candidate_since = 0.0
        self._pending = False
        self._last_switch_at = float("-inf")

    def consider(self, node_id: str, quality: float, t: float) -> bool:
        """提交一次链路质量观测。返回是否发生了路由切换。"""
        desired = self._desired(node_id, quality)

        if desired is _KEEP:
            self._pending = False
            return False

        if not self._pending or self._candidate != desired:
            self._pending = True
            self._candidate = desired
            self._candidate_since = t
            return False

        if t - self._candidate_since >= self.params.t_hold:
            self._apply_switch(desired, t)
            self._pending = False
            return True
        return False

    def _desired(self, node_id: str, quality: float):
        if quality >= self.params.h_up:
            return _KEEP if node_id == self.parent else node_id
        if quality < self.params.h_down:
            # 当前上游已不可用 -> 考虑脱离；其他节点劣化则不采纳
            return None if node_id == self.parent else _KEEP
        # 迟滞带内：保持现状
        return _KEEP

    def _apply_switch(self, new_parent: str | None, t: float) -> None:
        self.parent = new_parent
        self.route_version += 1
        self.switch_count += 1
        if t - self._last_switch_at < self.flap_window:
            self.flap_count += 1
        self._last_switch_at = t

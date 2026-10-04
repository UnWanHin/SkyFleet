"""P1 非视距链路模型：绕射 + 快衰落 + 高度相关。

P0 的几何遮挡模型是二值的：被楼挡住就是 0.05，看不见就是按距离线性衰减。
真实低空链路的遮挡不是"断了"，而是**绕射衰减**——信号越过楼顶绕过来，
衰减多少取决于遮挡物高出视线多少、离两端各多远。这决定了同一遮挡条件下
"飞高一点"到底能挽回多少，是报告里"飞行高度影响链路质量"这句话的落点。

本模块只实现 `LinkModel` 协议（`evaluate(a, b, t) -> LinkQuality`），
协议层不感知它的存在——这正是 §9 所说的"P1 的保险栓"。

物理依据：
- 绕射损失用 ITU-R P.526 单刃峰近似（不是自造公式）。
- 路径损耗用对数距离模型，视距/非视距取不同衰减指数。
- 快衰落用分块 Rician 抽样，相干时间可配。
"""
from __future__ import annotations

import hashlib
import math
import random
from collections import deque
from dataclasses import dataclass

from skyfleet.world.building import Building
from skyfleet.world.geometry import distance_3d, segment_intersects_rect
from skyfleet.world.link import LinkQuality
from skyfleet.world.node import Node

#: 光速，用于计算波长
_C = 299_792_458.0


@dataclass(frozen=True)
class NlosParams:
    """链路预算与传播参数。

    默认值取 2.4 GHz 低空链路的常见量级。**这些是假设值**：
    报告必须声明它们来自工程常用取值而非本文实测，P1 的标定实验
    可以据此做敏感性分析（见 study/specs/link_budget.json）。
    """

    frequency_hz: float = 2.4e9
    tx_power_dbm: float = 20.0            # 250 mW
    #: 2 MHz 带宽的热噪声底：-174 dBm/Hz + 10log10(2e6) + 7 dB 噪声系数
    noise_floor_dbm: float = -104.0
    ref_distance_m: float = 1.0
    path_loss_exp_los: float = 2.0
    #: 遮挡链路的额外衰减由**绕射项**承担，这里只比自由空间略高一点。
    #: 若同时用 3.5 这样的强非视距指数**又**加绕射损失，等于把遮挡算两遍，
    #: 结果是所有被挡链路一律判死，绕射模型退化成二值模型。
    path_loss_exp_nlos: float = 2.2
    #: quality=0 的锚点比"视距 max_range"再低这么多 dB，
    #: 给绕射损失留出可表达的动态范围（否则擦顶绕射一律归零）。
    diffraction_margin_db: float = 20.0
    #: quality 的归一端点由**链路的自身动态范围**决定，而不是固定信噪比。
    #: 固定窗口（如 5~25 dB）在 250 mW / 2.4 GHz 下会让数百米内全部饱和到
    #: quality=1：距离和衰落都失去辨别度，"链路质量随距离变化"当场失效。
    #: 现取"视距下 max_range 处 = 0，near_fraction*max_range 处 = 1"，
    #: 随场景自动缩放，且遮挡/绕射损失在归一化**之前**扣掉。
    near_fraction: float = 0.1
    near_floor_m: float = 20.0
    #: 分块快衰落的相干时间；每个相干区间独立抽样
    fade_step_s: float = 0.05
    #: Rician K 因子（dB）。0 表示 Rayleigh（无直射分量）
    rician_k_db: float = 6.0
    trend_window: int = 8
    min_quality: float = 0.05


class NlosDiffractionLinkModel:
    """绕射 + 快衰落链路模型，实现 `LinkModel` 协议。"""

    def __init__(self, buildings: list[Building], max_range: float,
                 seed: int, params: NlosParams | None = None) -> None:
        self.buildings = list(buildings)
        self.max_range = max_range
        self.params = params or NlosParams()
        self.seed = seed
        self._history: dict[tuple[str, str], deque[float]] = {}
        # 衰落按 (链路, 相干区间) 缓存。缓存不是性能优化，是**正确性要求**：
        # 同一次评估可能被调用多次（发送判定、日志、路由），若无条件推进
        # 随机过程，结果会依赖调用次数与顺序，确定性当场失效。
        self._fade_cache: dict[tuple[str, str, int], float] = {}
        self._wavelength = _C / self.params.frequency_hz
        near = max(self.params.near_floor_m,
                   self.params.near_fraction * max_range)
        # 视距、无衰落时的信噪比，作为 quality=1 / quality=0 的两个锚点。
        # 下锚点再下移一个 diffraction_margin_db，使绕射损失落在量程之内。
        self._snr_hi = self._snr_db(near, self.params.path_loss_exp_los, 0.0)
        self._snr_lo = (self._snr_db(max_range, self.params.path_loss_exp_los,
                                     0.0)
                        - self.params.diffraction_margin_db)
        if self._snr_hi - self._snr_lo < 1e-6:
            self._snr_lo = self._snr_hi - 1.0

    # ---------- LinkModel 协议 ----------

    def evaluate(self, a: Node, b: Node, t: float) -> LinkQuality:
        distance = distance_3d(a.pos, b.pos)
        blocker, nu = self._worst_edge(a, b)
        obstructed = blocker is not None and nu > -1.0

        exponent = (self.params.path_loss_exp_nlos if obstructed
                    else self.params.path_loss_exp_los)
        extra = self._diffraction_loss(nu) if obstructed else 0.0
        fade_db = self._fade(a, b, t)
        snr_db = self._snr_db(distance, exponent, extra) + fade_db

        if distance > self.max_range:
            quality = 0.0
        else:
            quality = self._snr_to_quality(snr_db)

        history = self._history.setdefault(
            (a.node_id, b.node_id), deque(maxlen=self.params.trend_window))
        history.append(quality)
        trend = (history[-1] - history[0]) if len(history) > 1 else 0.0

        return LinkQuality(quality=quality, obstructed=obstructed,
                           blocked_by=blocker, trend=trend)

    # ---------- 传播 ----------

    def _snr_db(self, distance: float, exponent: float,
                extra_loss_db: float) -> float:
        loss = self._path_loss(distance, exponent) + extra_loss_db
        return self.params.tx_power_dbm - loss - self.params.noise_floor_dbm

    def _path_loss(self, distance: float, exponent: float) -> float:
        d = max(distance, self.params.ref_distance_m)
        return 20.0 * math.log10(4.0 * math.pi * self.params.ref_distance_m
                                 / self._wavelength) \
            + 10.0 * exponent * math.log10(d / self.params.ref_distance_m)

    def _diffraction_loss(self, nu: float) -> float:
        """ITU-R P.526 单刃峰绕射损失（dB）。

        ν 是 Fresnel-Kirchhoff 绕射参数：ν <= -0.78 时视为无阻挡。
        遮挡越深（ν 越大）损失越大，这正是"飞高一点能挽回多少"的量化来源。
        """
        if nu <= -1.0:
            return 0.0
        term = math.sqrt((nu - 0.1) ** 2 + 1.0) + nu - 0.1
        if term <= 0.0:
            return 0.0
        # 有意偏离 ITU-R P.526 的 -0.78 硬截断：该截断会在余隙刚好
        # 掠过楼顶时产生约 4.8 dB 的阶跃，节点在边界附近会反复触发
        # 路由切换。改让公式自然衰减到 0 再截断，损失随余隙连续变化。
        return max(0.0, 6.9 + 20.0 * math.log10(term))

    def _worst_edge(self, a: Node, b: Node) -> tuple[str | None, float]:
        """找出绕射参数最不利的那栋楼。"""
        start = (a.pos[0], a.pos[1])
        end = (b.pos[0], b.pos[1])
        span = math.dist(start, end)
        worst_id: str | None = None
        worst_nu = -math.inf

        for building in self.buildings:
            if not segment_intersects_rect(start, end, building.rect):
                continue
            d1, d2 = self._edge_distances(start, end, building)
            if d1 <= 0.0 or d2 <= 0.0:
                continue
            # 楼顶到视线的垂直余隙：正表示楼顶高过视线（构成阻挡）
            ray_z = a.pos[2] + (b.pos[2] - a.pos[2]) * (d1 / span) \
                if span > 0 else a.pos[2]
            clearance = building.height - ray_z
            nu = clearance * math.sqrt(
                2.0 * (d1 + d2) / (self._wavelength * d1 * d2))
            if nu > worst_nu:
                worst_nu, worst_id = nu, building.id

        return worst_id, worst_nu

    @staticmethod
    def _edge_distances(start: tuple[float, float], end: tuple[float, float],
                        building: Building) -> tuple[float, float]:
        """楼块沿水平视线把链路分成的两段长度。"""
        x0, y0, w, h = building.rect
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length == 0.0:
            return 0.0, 0.0
        ux, uy = dx / length, dy / length

        # 视线与矩形的交参数范围（slab method）
        t_near, t_far = 0.0, length
        for origin, direction, low, high in ((start[0], ux, x0, x0 + w),
                                             (start[1], uy, y0, y0 + h)):
            if abs(direction) < 1e-12:
                if origin < low or origin > high:
                    return 0.0, 0.0
                continue
            t1 = (low - origin) / direction
            t2 = (high - origin) / direction
            if t1 > t2:
                t1, t2 = t2, t1
            t_near = max(t_near, t1)
            t_far = min(t_far, t2)
            if t_near > t_far:
                return 0.0, 0.0
        return t_near, length - t_far

    # ---------- 快衰落 ----------

    def _fade(self, a: Node, b: Node, t: float) -> float:
        """分块 Rician 衰落的 dB 增益。

        按 (链路, 相干区间) 确定性派生，而不是"每次调用推进一次随机过程"。
        后者会让同一时刻的评估结果依赖**被调用的次数**——发送判定调用一次、
        写日志再调用一次，两次都不一样，仿真立刻不可复现。
        """
        step = self.params.fade_step_s
        index = int(math.floor(t / step)) if step > 0 else 0
        key = (a.node_id, b.node_id, index)
        cached = self._fade_cache.get(key)
        if cached is not None:
            return cached

        rng = random.Random(self._stream(a.node_id, b.node_id, index))
        k = 10.0 ** (self.params.rician_k_db / 10.0)
        # Rician 包络：直射分量 + 散射分量
        los = math.sqrt(k / (k + 1.0))
        scatter = math.sqrt(1.0 / (k + 1.0))
        i = rng.gauss(0.0, 1.0)
        q = rng.gauss(0.0, 1.0)
        envelope = math.hypot(los + scatter * i, scatter * q)
        gain_db = 20.0 * math.log10(max(envelope, 1e-6))

        # 只保留最近若干区间的缓存，避免长仿真里无界增长
        if len(self._fade_cache) > 20000:
            self._fade_cache.clear()
        self._fade_cache[key] = gain_db
        return gain_db

    def _stream(self, a: str, b: str, index: int) -> int:
        """按无序节点对派生：a->b 与 b->a 必须是同一条链路的同一个衰落。"""
        lo, hi = sorted((a, b))
        raw = f"fade:{self.seed}:{lo}:{hi}:{index}".encode("utf-8")
        return int.from_bytes(hashlib.blake2b(raw, digest_size=8).digest(),
                              "big")

    def _snr_to_quality(self, snr_db: float) -> float:
        """把信噪比线性映射到 [0, 1]。

        锚点随 max_range 缩放（见 NlosParams.near_fraction）：quality=1 表示
        "视距近距离"，quality=0 表示"视距恰好到 max_range"。因此 quality
        的读法是**相对该链路量程的位置**，而不是绝对信噪比。
        """
        span = self._snr_hi - self._snr_lo
        q = (snr_db - self._snr_lo) / span
        return max(0.0, min(1.0, q))

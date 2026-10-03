"""独立随机流。

关键：绝不用内建 hash() 派生种子——字符串 hash() 受 PYTHONHASHSEED 影响，
逐进程变化，会直接破坏仿真的可复现性。
"""
from __future__ import annotations

import hashlib
import random


def derive_seed(master_seed: int, stream_name: str) -> int:
    """由主种子与流名称稳定派生一个 64 位种子。"""
    digest = hashlib.blake2b(f"{master_seed}:{stream_name}".encode("utf-8"),
                             digest_size=8).digest()
    return int.from_bytes(digest, "big")


class RngRegistry:
    """每个随机来源一条独立流。

    若所有随机数共用一条流，增删节点会使后续所有抽样整体位移，
    导致任何 A/B 对比在统计上失去意义。
    """

    def __init__(self, master_seed: int) -> None:
        self.master_seed = master_seed
        self._streams: dict[str, random.Random] = {}

    def get(self, name: str) -> random.Random:
        """取得（必要时创建）命名随机流。同名返回同一实例。"""
        if name not in self._streams:
            self._streams[name] = random.Random(
                derive_seed(self.master_seed, name))
        return self._streams[name]

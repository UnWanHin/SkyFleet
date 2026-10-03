"""JSONL 事件日志。

日志是**唯一事实来源**：指标与回放都从它重算，仿真过程不直接累计指标。
好处有三：
1. 指标定义演进时无需重跑仿真；
2. 回放与指标共用一份数据，不会出现两套口径；
3. 可人工抽查某条消息的完整链路。

normalised_digest 是确定性验收的核心工具：相同种子两次运行，
该摘要必须一致。
"""
from __future__ import annotations

import enum
import hashlib
import json
from pathlib import Path
from typing import Any, Iterator


class EventLog:
    """追加写入的 JSONL 日志。"""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("a", encoding="utf-8")

    def write(self, kind: Any, t: float, **fields: Any) -> None:
        record: dict[str, Any] = {
            "t": round(float(t), 9),
            "kind": kind.value if isinstance(kind, enum.Enum) else str(kind),
        }
        for key, value in fields.items():
            record[key] = value.value if isinstance(value, enum.Enum) else value
        self._handle.write(
            json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
        self._handle.flush()

    def close(self) -> None:
        if not self._handle.closed:
            self._handle.close()

    def __enter__(self) -> "EventLog":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def read_log(path: str | Path) -> Iterator[dict[str, Any]]:
    """逐行读取日志。空行跳过。"""
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(str(path))
    with p.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                yield json.loads(line)


def normalised_digest(path: str | Path) -> str:
    """对日志做规范化摘要：字段排序、紧凑分隔，保证与书写顺序无关。"""
    digest = hashlib.sha256()
    for record in read_log(path):
        payload = json.dumps(record, sort_keys=True, separators=(",", ":"))
        digest.update(payload.encode("utf-8"))
        digest.update(b"\n")
    return digest.hexdigest()

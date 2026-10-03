"""让 `python -m skyfleet` 等价于 `python -m skyfleet.cli`。"""
from __future__ import annotations

from skyfleet.cli import main

if __name__ == "__main__":
    raise SystemExit(main())

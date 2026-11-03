"""让 `python -m flightlab` 可用。"""
import sys

from flightlab.cli import main

sys.exit(main())

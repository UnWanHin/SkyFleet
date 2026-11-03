"""可视化字体的**噪声**过滤：只静音无害的可变字体字重警告。

为什么值得一条测试：过滤器的默认写法（整个 logger 静音）会连「找不到
字族」一起吞掉，症状是**图上全是方框却没有任何提示**——比报警更难查。
所以这里两个方向都要钉住：该静音的静音、该报的必须报。
"""
from __future__ import annotations

import logging
import unittest

from skyfleet.viz.plots import _DropVariableFontWeightWarning  # noqa: F401


class TestFontWarningFilter(unittest.TestCase):
    def setUp(self):
        # plots.py 在 import 时就会设置字体，这里直接取用那个过滤器类
        from skyfleet.viz import plots
        self.filter = plots._DropVariableFontWeightWarning()
        self.candidates = plots._CJK_CANDIDATES

    def _record(self, message: str) -> logging.LogRecord:
        return logging.LogRecord(name="matplotlib.font_manager",
                                 level=logging.WARNING, pathname="", lineno=0,
                                 msg=message, args=(), exc_info=None)

    def test_the_variable_font_weight_warning_is_dropped(self):
        record = self._record(
            "Failed to find font weight normal for Noto Sans CJK SC, "
            "now using 100")
        self.assertFalse(self.filter.filter(record),
                         "可变字体的字重警告没有被静音——首次运行的用户会"
                         "看到四行 findfont: Failed to...，以为字体坏了")

    def test_a_genuinely_missing_font_still_warns(self):
        """★ 真正的缺字体必须照常报出来。

        若这条失败，说明过滤器把 logger 整个静音了——那会让"图上全是方框"
        变成无声故障，比原警告危险得多。
        """
        record = self._record(
            "findfont: Font family 'Absolutely Not Installed' not found.")
        self.assertTrue(self.filter.filter(record),
                        "缺字体警告被一起静音了——无声故障比报警更难查")

    def test_other_matplotlib_warnings_pass_through(self):
        record = self._record("Glyph 12345 missing from current font.")
        self.assertTrue(self.filter.filter(record),
                        "与本问题无关的 matplotlib 警告不应被拦截")


if __name__ == "__main__":
    unittest.main()

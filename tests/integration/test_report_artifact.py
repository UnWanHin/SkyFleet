"""报告交付物的防漂移守卫。

**为什么需要它**：报告章节的 HTML 是**生成**出来的（数字来自
`out/study/*/runs.csv`），而 `.docx` 又是从 HTML 转出来的。这两层
转换都不会在数据变化时自己报警——扫描重跑后，正文里的数字会与
数据静默脱节，而这正是本项目已经犯过一次的错误。

本模块把「交付物与数据一致」变成一条会变红的断言，代价是几秒钟。

覆盖：
1. HTML 与生成器一致（生成器的 `--check` 模式）。
2. 必须出现的关键小节与表号。
3. `.docx` 能构建，且通过 OOXML 分部件校验与表格结构校验。
"""
from __future__ import annotations

import io
import re
import subprocess
import sys
import tempfile
import unittest
import zipfile
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REPORT_DIR = ROOT / "docs/report"
GENERATOR = REPORT_DIR / "build_energy_section.py"
HTML = REPORT_DIR / "2026-10-04-results-chapter.html"
DOCX_BUILDER = REPORT_DIR / "build_docx.py"

#: 课程要求报告的九项指标里，「能耗趋势」一项此前在正文中完全缺席。
#: 这些标记是用来防止它再次被删掉的——删掉任何一项都会变红。
REQUIRED_MARKERS = (
    "表 11b",                    # 布局方差分解
    "表 13", "表 14", "表 15",   # 推力余裕 / 能量模型 / 机型
    "表 16", "表 17", "表 18",   # 耗尽处置 / 布局×策略 / 双峰
    "方法学更正",                # 统计节更正
    "推力/重量比",
    "三种策略都是双峰",
)


class TestReportHTMLMatchesGenerator(unittest.TestCase):
    """HTML 必须与生成器逐字节一致，否则正文数字可能已与数据脱节。"""

    def test_generator_reports_no_drift(self):
        proc = subprocess.run(
            [sys.executable, str(GENERATOR), "--check"],
            capture_output=True, text=True, cwd=ROOT)
        self.assertEqual(
            proc.returncode, 0,
            "报告 HTML 与生成器不一致。数据变了之后必须重跑：\n"
            f"  python3 {GENERATOR.relative_to(ROOT)}\n"
            f"stdout: {proc.stdout.strip()}\nstderr: {proc.stderr.strip()}")

    def test_required_sections_are_present(self):
        html = HTML.read_text(encoding="utf-8")
        missing = [m for m in REQUIRED_MARKERS if m not in html]
        self.assertEqual(missing, [],
                         f"报告 HTML 缺少必需内容：{missing}")


class TestReportDocxIsBuildable(unittest.TestCase):
    """`.docx` 必须能构建且结构合法——坏掉的交付物不该被提交。"""

    def test_docx_builds_and_passes_validation(self):
        sys.path.insert(0, str(REPORT_DIR))
        try:
            from build_docx import (parse_html, validate_part,
                                    check_table_structure)
        except ImportError as exc:            # pragma: no cover
            self.fail(f"无法导入转换器：{exc}")

        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "chapter.docx"
            proc = subprocess.run(
                [sys.executable, str(DOCX_BUILDER), str(HTML), str(out)],
                capture_output=True, text=True, cwd=ROOT)
            self.assertEqual(proc.returncode, 0,
                             f"构建 docx 失败：{proc.stderr[-800:]}")
            self.assertTrue(out.exists(), "构建脚本退出码为 0 却没有产出文件")

            with zipfile.ZipFile(out) as z:
                self.assertIsNone(z.testzip(), "docx 压缩包损坏")
                names = z.namelist()
                for name in names:
                    if name.endswith((".xml", ".rels")):
                        validate_part(name, z.read(name).decode("utf-8"))
                document = z.read("word/document.xml").decode("utf-8")
                media = [n for n in names if n.startswith("word/media/")]

        check_table_structure(document)

        # 图片必须真的嵌进包里，而不是留成外链——早前用 LibreOffice
        # 转换就是这个下场：docx 里一张图都没有，却不报任何错。
        self.assertGreaterEqual(len(media), 4,
                                f"内嵌图片只有 {len(media)} 张，预期至少 4 张")

        # 新增小节必须真的进入正文，而不只是留在 HTML 里。
        text = re.sub(r"\s+", " ", re.sub(r"<[^>]+>", " ", document))
        for marker in ("表 13", "表 17", "方法学更正", "三种策略都是双峰"):
            self.assertIn(marker, text, f"docx 正文缺少 {marker}")

    def test_converter_rejects_unknown_tags_loudly(self):
        """转换器对不认识的标签必须报错，而不是静默丢内容。

        「静默丢弃」是文档交付物最危险的失效模式：少一段话没有任何
        格式检查能发现。这里固定住这个行为。
        """
        sys.path.insert(0, str(REPORT_DIR))
        from build_docx import parse_html
        with tempfile.TemporaryDirectory() as tmp:
            bad = Path(tmp) / "bad.html"
            bad.write_text("<h1>t</h1><blink>nope</blink>", encoding="utf-8")
            # <blink> 不是块级元素，会被当行内文本吞掉；真正该报错的是
            # 未知的*块级*元素，用 <section> 验证。
            bad.write_text("<h1>t</h1><section>x</section>", encoding="utf-8")
            with self.assertRaises(ValueError):
                parse_html(bad)


if __name__ == "__main__":
    unittest.main()

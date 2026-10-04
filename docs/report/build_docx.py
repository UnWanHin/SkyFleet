#!/usr/bin/env python3
"""把结果章节的 HTML 源转换成可编辑的 Word 文档（.docx）。

**为什么不用 LibreOffice 转换**：无头模式下 LibreOffice 的 HTML 导入
只把 <img> 记为**链接**而不嵌入，导出的 docx 里一张图都没有
（实测三种路径写法皆是如此）。与其在交付物里留一个隐形的坏图，
不如直接生成 OOXML——零依赖、结果可复现，而且图片确实在包里。

用法：
    python3 docs/report/build_docx.py            # 用默认路径
    python3 docs/report/build_docx.py 源.html 目标.docx

只支持本目录 HTML 用到的那一小撮标签（h1/h2/h3、p、table、img、
code、b、i、sup、caption）。它**不是**通用转换器：遇到不认识的标签
直接报错，避免"静默丢内容"——文档交付物里少一段话是查不出来的。
"""
from __future__ import annotations

import re
import struct
import sys
import zipfile
from html.parser import HTMLParser
from pathlib import Path
from xml.sax.saxutils import escape

# 1 cm = 360000 EMU；A4 宽 21 cm，左右各留 3.17 cm 页边距
EMU_PER_CM = 360000
CONTENT_WIDTH_CM = 21.0 - 2 * 3.17

#: 正文与标题字体。Fedora 上有 Noto CJK；Word 上会回退到雅黑/宋体。
FONT_CJK = "Noto Serif CJK SC"
FONT_SANS = "Noto Sans CJK SC"
FONT_LATIN = "Times New Roman"

W = 'xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"'
R = 'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"'
#: 图片用的命名空间必须在根元素上声明。只在 wp:inline 上写 <wp:...> 而
#: 不绑定前缀，生成的是**非法 XML**：Word/LibreOffice 只会报
#: "source file could not be loaded"，不会指出是哪个前缀没声明。
WP = ('xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/'
      'wordprocessingDrawing" xmlns:a="http://schemas.openxmlformats.org/'
      'drawingml/2006/main" xmlns:pic="http://schemas.openxmlformats.org/'
      'drawingml/2006/picture"')


# --------------------------------------------------------------------------
# HTML 解析：把本目录的 HTML 子集转成中间表示
# --------------------------------------------------------------------------

class Block:
    """一个块级元素。kind ∈ {h1,h2,h3,p,table,img,caption}。"""

    def __init__(self, kind: str, **kw):
        self.kind = kind
        self.__dict__.update(kw)


class Extract(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.blocks: list[Block] = []
        self.stack: list[str] = []
        self.text: list[str] = []          # 当前段落的行内文本（含标记）
        self.rows: list[list[tuple[str, list[str]]]] = []
        self.row: list[tuple[str, list[str]]] | None = None
        self.cell: list[str] | None = None
        self.cell_tag = "td"
        #: 标题文本缓冲。**不能**复用 table_caption 本身：</caption> 之后
        #: 到 <tr> 之间还有空白文本，复用会把字符串追加进已转成 run 列表的
        #: 同一个列表里，等到渲染时才以 AttributeError 暴露。
        self.caption_buf: list[str] | None = None
        self.table_caption: list[dict] | None = None
        self.img = None

    # ---- 行内标记 ----
    def _inline(self, tag, attrs):
        if tag == "b":
            self.text.append("<b>")
        elif tag == "i":
            self.text.append("<i>")
        elif tag == "sup":
            self.text.append("<sup>")
        elif tag == "code":
            self.text.append("<code>")

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if tag in ("h1", "h2", "h3"):
            self.stack.append(tag)
            self.text = []
        elif tag == "p":
            self.stack.append(tag)
            self.text = []
        elif tag == "table":
            self.stack.append(tag)
            self.rows = []
        elif tag == "tr":
            self.row = []
        elif tag in ("td", "th"):
            self.cell_tag = tag
            self.cell = []
        elif tag == "caption":
            self.caption_buf = []
        elif tag == "img":
            src = a.get("src", "")
            if not src:
                raise ValueError("<img> 没有 src")
            # 立即入列：HTML 里 <img> 可以不闭合，HTMLParser 不会为它调用
            # handle_endtag，挂在 self.img 上等结束标签等于是等到永远，
            # 结果是 docx 里一张图都没有却不报任何错。
            self.img = Block("img", src=src)
            self.blocks.append(self.img)
            self.img = None
        elif tag in ("b", "i", "sup", "code"):
            self._inline(tag, attrs)
        elif tag == "br":
            self.text.append("\n")

    def handle_endtag(self, tag):
        if tag in ("h1", "h2", "h3", "p"):
            self.stack.pop()
            self.blocks.append(Block(tag, runs=self._runs("".join(self.text))))
            self.text = []
        elif tag in ("b", "i", "sup", "code"):
            self.text.append(f"</{tag}>")
        elif tag in ("td", "th"):
            assert self.row is not None and self.cell is not None
            self.row.append((self.cell_tag, self._runs("".join(self.cell))))
            self.cell = None
        elif tag == "tr":
            assert self.row is not None
            self.rows.append(self.row)
            self.row = None
        elif tag == "caption":
            self.table_caption = self._runs("".join(self.caption_buf or []))
            self.caption_buf = None
        elif tag == "table":
            self.stack.pop()
            self.blocks.append(Block("table", rows=self.rows,
                                     caption=self.table_caption))
            self.table_caption = None
        elif tag == "img":
            pass                          # 已在 starttag 处入列

    def handle_data(self, data):
        if self.caption_buf is not None and self.cell is None:
            self.caption_buf.append(data)
        elif self.cell is not None:
            self.cell.append(data)
        else:
            self.text.append(data)

    # ---- 行内标记 → run 列表 ----
    @staticmethod
    def _runs(raw: str) -> list[dict]:
        """把含 <b>/<i>/<sup>/<code> 的文本拆成带格式的 run。"""
        raw = re.sub(r"\s+", " ", raw)
        runs: list[dict] = []
        pos = 0
        pattern = re.compile(r"</?(b|i|sup|code)>")
        bold = italic = sup = code = False
        for m in pattern.finditer(raw):
            if m.start() > pos:
                runs.append({"text": raw[pos:m.start()], "bold": bold,
                             "italic": italic, "sup": sup, "code": code})
            closing = m.group(0).startswith("</")
            which = m.group(1)
            value = not closing
            if which == "b":
                bold = value
            elif which == "i":
                italic = value
            elif which == "sup":
                sup = value
            elif which == "code":
                code = value
            pos = m.end()
        if pos < len(raw):
            runs.append({"text": raw[pos:], "bold": bold, "italic": italic,
                         "sup": sup, "code": code})
        return [r for r in runs if r["text"]]


def parse_html(path: Path) -> list[Block]:
    parser = Extract()
    parser.feed(path.read_text(encoding="utf-8"))
    unknown = [b.kind for b in parser.blocks
               if b.kind not in ("h1", "h2", "h3", "p", "table", "img")]
    if unknown:
        raise ValueError(f"出现未处理的块级元素：{set(unknown)}")
    return parser.blocks


# --------------------------------------------------------------------------
# OOXML 生成
# --------------------------------------------------------------------------

def runs_xml(runs: list[dict]) -> str:
    out = []
    for r in runs:
        props = []
        if r.get("bold"):
            props.append("<w:b/>")
        if r.get("italic"):
            props.append("<w:i/>")
        if r.get("sup"):
            props.append('<w:vertAlign w:val="superscript"/>')
        if r.get("code"):
            props.append(f'<w:rFonts w:ascii="DejaVu Sans Mono" '
                         f'w:hAnsi="DejaVu Sans Mono"/>')
        rpr = f"<w:rPr>{''.join(props)}</w:rPr>" if props else ""
        text = escape(r["text"])
        out.append(f'<w:r>{rpr}<w:t xml:space="preserve">{text}</w:t></w:r>')
    return "".join(out)


def para_xml(runs: list[dict], style: str, jc: str = "both") -> str:
    return (f'<w:p><w:pPr><w:pStyle w:val="{style}"/>'
            f'<w:jc w:val="{jc}"/></w:pPr>{runs_xml(runs)}</w:p>')


def table_xml(block: Block, rel_ids: dict) -> str:
    ncols = max((len(r) for r in block.rows), default=1)
    width = int(CONTENT_WIDTH_CM * EMU_PER_CM / ncols)
    # 表题必须是 <w:tbl> 的**兄弟**节点。放在 <w:tblPr> 与 <w:tblGrid> 之间
    # 虽然 XML 合法，却不在 CT_Tbl 允许的序列里，渲染器会**静默丢弃**它——
    # 交付物里少一行表题，任何格式检查都不会报错。
    parts = []
    if block.caption:
        parts.append(para_xml(block.caption, "TableCaption", jc="left"))
    parts.append('<w:tbl><w:tblPr><w:tblStyle w:val="TableGrid"/>'
                 '<w:tblW w:w="5000" w:type="pct"/><w:tblBorders>'
                 + "".join(f'<w:{s} w:val="single" w:sz="4" w:color="444444"/>'
                           for s in ("top", "left", "bottom", "right",
                                     "insideH", "insideV"))
                 + "</w:tblBorders></w:tblPr>")
    parts.append("<w:tblGrid>"
                 + f'<w:gridCol w:w="{width}"/>' * ncols + "</w:tblGrid>")
    for row in block.rows:
        cells = []
        for tag, runs in row:
            style = "TableHeading" if tag == "th" else "TableContents"
            cells.append(
                f'<w:tc><w:tcPr><w:tcW w:w="{width}" w:type="dxa"/></w:tcPr>'
                + para_xml(runs, style, jc="left") + "</w:tc>")
        header = "<w:trPr><w:tblHeader/></w:trPr>" if row and row[0][0] == "th" else ""
        parts.append(f"<w:tr>{header}{''.join(cells)}</w:tr>")
    parts.append("</w:tbl>")
    # 表格后补一个空段，避免连续两个表格被 Word 合并
    parts.append('<w:p><w:pPr><w:pStyle w:val="BodyText"/></w:pPr></w:p>')
    return "".join(parts)


def png_size(path: Path) -> tuple[int, int]:
    data = path.read_bytes()[:32]
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"不是 PNG：{path}")
    w, h = struct.unpack(">II", data[16:24])
    return w, h


def image_xml(block: Block, rel_ids: dict, base: Path) -> str:
    path = (base / block.src).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"图档不存在：{path}")
    px_w, px_h = png_size(path)
    width_emu = int(CONTENT_WIDTH_CM * 0.95 * EMU_PER_CM)
    height_emu = int(width_emu * px_h / px_w)
    rel_id = rel_ids[path]
    return (
        '<w:p><w:pPr><w:pStyle w:val="BodyText"/>'
        '<w:jc w:val="center"/></w:pPr><w:r><w:drawing>'
        '<wp:inline distT="0" distB="0" distL="0" distR="0">'
        f'<wp:extent cx="{width_emu}" cy="{height_emu}"/>'
        '<wp:docPr id="1" name="figure"/>'
        '<a:graphic>'
        '<a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture">'
        '<pic:pic>'
        '<pic:nvPicPr><pic:cNvPr id="0" name="figure"/><pic:cNvPicPr/></pic:nvPicPr>'
        f'<pic:blipFill><a:blip r:embed="{rel_id}"/><a:stretch><a:fillRect/>'
        '</a:stretch></pic:blipFill>'
        f'<pic:spPr><a:xfrm><a:off x="0" y="0"/>'
        f'<a:ext cx="{width_emu}" cy="{height_emu}"/></a:xfrm>'
        '<a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr>'
        '</pic:pic></a:graphicData></a:graphic></wp:inline></w:drawing></w:r></w:p>')


STYLES = f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles {W}>
<w:docDefaults><w:rPrDefault><w:rPr>
<w:rFonts w:ascii="{FONT_LATIN}" w:hAnsi="{FONT_LATIN}" w:eastAsia="{FONT_CJK}"/>
<w:sz w:val="21"/></w:rPr></w:rPrDefault>
<w:pPrDefault><w:pPr><w:spacing w:line="360" w:lineRule="auto"/></w:pPr></w:pPrDefault>
</w:docDefaults>
<w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/></w:style>
<w:style w:type="paragraph" w:styleId="BodyText"><w:name w:val="Body Text"/>
<w:pPr><w:spacing w:before="60" w:after="60"/><w:jc w:val="both"/></w:pPr>
<w:rPr><w:sz w:val="21"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="Heading1"><w:name w:val="heading 1"/>
<w:pPr><w:keepNext/><w:spacing w:before="360" w:after="200"/><w:jc w:val="left"/></w:pPr>
<w:rPr><w:rFonts w:eastAsia="{FONT_SANS}"/><w:b/><w:sz w:val="32"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="Heading2"><w:name w:val="heading 2"/>
<w:pPr><w:keepNext/><w:spacing w:before="280" w:after="160"/><w:jc w:val="left"/></w:pPr>
<w:rPr><w:rFonts w:eastAsia="{FONT_SANS}"/><w:b/><w:sz w:val="26"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="Heading3"><w:name w:val="heading 3"/>
<w:pPr><w:keepNext/><w:spacing w:before="220" w:after="120"/><w:jc w:val="left"/></w:pPr>
<w:rPr><w:rFonts w:eastAsia="{FONT_SANS}"/><w:b/><w:sz w:val="23"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="TableCaption"><w:name w:val="Table Caption"/>
<w:pPr><w:keepNext/><w:spacing w:before="160" w:after="60"/><w:jc w:val="left"/></w:pPr>
<w:rPr><w:b/><w:sz w:val="19"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="TableHeading"><w:name w:val="Table Heading"/>
<w:pPr><w:jc w:val="left"/></w:pPr><w:rPr><w:b/><w:sz w:val="19"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="TableContents"><w:name w:val="Table Contents"/>
<w:pPr><w:spacing w:line="240" w:lineRule="auto"/></w:pPr>
<w:rPr><w:sz w:val="19"/></w:rPr></w:style>
<w:style w:type="table" w:styleId="TableGrid"><w:name w:val="Table Grid"/>
<w:tblPr><w:tblBorders>
<w:top w:val="single" w:sz="4" w:color="444444"/>
<w:left w:val="single" w:sz="4" w:color="444444"/>
<w:bottom w:val="single" w:sz="4" w:color="444444"/>
<w:right w:val="single" w:sz="4" w:color="444444"/>
<w:insideH w:val="single" w:sz="4" w:color="444444"/>
<w:insideV w:val="single" w:sz="4" w:color="444444"/>
</w:tblBorders></w:tblPr></w:style>
</w:styles>"""


def validate_part(name: str, data: str) -> None:
    """解析每个 XML 部件，并检查用到的命名空间前缀都已声明。

    只依赖标准库：手写 OOXML 最容易犯的错就是漏声明前缀，而 Word 对此
    的报错只有一句 "could not be loaded"，无法定位。
    """
    import xml.etree.ElementTree as ET
    try:
        ET.fromstring(data)
    except ET.ParseError as exc:
        raise ValueError(f"{name} 不是合法 XML：{exc}") from exc
    declared = set(re.findall(r'xmlns:([A-Za-z0-9_]+)=', data))
    used = set(re.findall(r'<([A-Za-z0-9_]+):', data))
    missing = sorted(used - declared)
    if missing:
        raise ValueError(f"{name} 使用了未声明的命名空间前缀：{missing}")


#: CT_Tbl 允许的子元素及顺序。表题写成 <w:tbl> 的子元素时 XML 依然合法，
#: 但不在这个序列里，渲染器会**静默丢弃**——这类错误只能用结构检查发现。
TBL_SEQUENCE = ("tblPr", "tblGrid", "tr")


def check_table_structure(document: str) -> None:
    """按 CT_Tbl 的允许序列检查每个表格。"""
    import xml.etree.ElementTree as ET
    w = "{http://schemas.openxmlformats.org/wordprocessingml/2006/main}"
    root = ET.fromstring(document)
    for table in root.iter(f"{w}tbl"):
        tags = [child.tag[len(w):] for child in table]
        allowed = set(TBL_SEQUENCE)
        extra = [t for t in tags if t not in allowed]
        if extra:
            raise ValueError(
                f"<w:tbl> 内出现不允许的子元素 {extra}（允许：{list(TBL_SEQUENCE)}）。"
                f"这类内容会被渲染器静默丢弃，必须移到表格之外。")
        body = [t for t in tags if t != "tblPr"]
        if body != sorted(body, key=TBL_SEQUENCE.index):
            raise ValueError(f"<w:tbl> 子元素顺序不符合 CT_Tbl：{tags}")


def build(html_path: Path, docx_path: Path) -> tuple[int, int, int]:
    blocks = parse_html(html_path)
    base = html_path.parent

    images = [b for b in blocks if b.kind == "img"]
    rel_ids = {p: f"rIdImg{i + 1}" for i, p in enumerate(
        dict.fromkeys((base / b.src).resolve() for b in images))}

    body = []
    for b in blocks:
        if b.kind in ("h1", "h2", "h3"):
            body.append(para_xml(b.runs, f"Heading{b.kind[1]}", jc="left"))
        elif b.kind == "p":
            body.append(para_xml(b.runs, "BodyText"))
        elif b.kind == "table":
            body.append(table_xml(b, rel_ids))
        elif b.kind == "img":
            body.append(image_xml(b, rel_ids, base))
    # Word 要求 sectPr 在 body 末尾
    sect = ('<w:sectPr><w:pgSz w:w="11906" w:h="16838"/>'
            '<w:pgMar w:top="1440" w:right="1797" w:bottom="1440" w:left="1797" '
            'w:header="851" w:footer="992" w:gutter="0"/></w:sectPr>')
    document = (f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                f'<w:document {W} {R} {WP}><w:body>{"".join(body)}{sect}</w:body>'
                f'</w:document>')

    image_rels = "".join(
        f'<Relationship Id="{rid}" Type="http://schemas.openxmlformats.org/'
        f'officeDocument/2006/relationships/image" Target="media/img{i + 1}.png"/>'
        for i, (path, rid) in enumerate(rel_ids.items()))
    doc_rels = (f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                f'<Relationships xmlns="http://schemas.openxmlformats.org/'
                f'package/2006/relationships">'
                f'<Relationship Id="rIdStyles" Type="http://schemas.openxmlformats.org/'
                f'officeDocument/2006/relationships/styles" Target="styles.xml"/>'
                f'{image_rels}</Relationships>')
    root_rels = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                 '<Relationships xmlns="http://schemas.openxmlformats.org/'
                 'package/2006/relationships">'
                 '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/'
                 'officeDocument/2006/relationships/officeDocument" '
                 'Target="word/document.xml"/></Relationships>')
    pngs = "".join('<Default Extension="png" ContentType="image/png"/>'
                   for _ in [1])
    content_types = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-'
        'package.relationships+xml"/>'
        '<Default Extension="xml" ContentType="application/xml"/>' + pngs +
        '<Override PartName="/word/document.xml" ContentType="application/vnd.'
        'openxmlformats-officedocument.wordprocessingml.document.main+xml"/>'
        '<Override PartName="/word/styles.xml" ContentType="application/vnd.'
        'openxmlformats-officedocument.wordprocessingml.styles+xml"/>'
        '</Types>')

    for caption in (b.caption for b in blocks
                    if b.kind == "table" and b.caption):
        text = "".join(r["text"] for r in caption)
        if text not in document:
            raise ValueError(f"表题没有进入正文（会被渲染器丢弃）：{text}")
    check_table_structure(document)

    for _name, _data in (("document.xml", document), ("styles.xml", STYLES),
                         ("document.xml.rels", doc_rels), ("_rels/.rels", root_rels),
                         ("[Content_Types].xml", content_types)):
        validate_part(_name, _data)

    docx_path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(docx_path, "w", zipfile.ZIP_DEFLATED) as z:
        # 固定时间戳，使同一份源产出逐字节相同的 docx
        for name, data in (
            ("[Content_Types].xml", content_types),
            ("_rels/.rels", root_rels),
            ("word/document.xml", document),
            ("word/styles.xml", STYLES),
            ("word/_rels/document.xml.rels", doc_rels),
        ):
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            z.writestr(info, data)
        for i, path in enumerate(rel_ids):
            info = zipfile.ZipInfo(f"word/media/img{i + 1}.png",
                                   date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_STORED   # PNG 已压缩
            z.writestr(info, path.read_bytes())

    n_tables = sum(1 for b in blocks if b.kind == "table")
    n_paras = sum(1 for b in blocks if b.kind in ("h1", "h2", "h3", "p"))
    return len(blocks), n_tables, len(rel_ids)


def main() -> int:
    here = Path(__file__).parent
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else here / "2026-10-04-results-chapter.html"
    dst = Path(sys.argv[2]) if len(sys.argv) > 2 else here / "2026-10-04-results-chapter.docx"
    total, tables, images = build(src, dst)
    print(f"已生成 {dst}")
    print(f"  块级元素 {total} 个（表格 {tables} 个，嵌入图片 {images} 张）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

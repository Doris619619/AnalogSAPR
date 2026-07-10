#!/usr/bin/env python3
"""文件职责：将 sa_trace.json 导出为可直接用 Excel 打开的 sa_trace.xlsx。"""

from __future__ import annotations

import argparse
import json
import zipfile
from pathlib import Path
from xml.sax.saxutils import escape


# 将 0-based 列号转为 Excel 列名，例如 0 -> A，26 -> AA。
def excel_column_name(index: int) -> str:
    if index < 0:
        raise ValueError(f"invalid column index: {index}")
    name = ""
    current = index
    while True:
        current, rem = divmod(current, 26)
        name = chr(ord("A") + rem) + name
        if current == 0:
            break
        current -= 1
    return name


# 生成单元格引用，例如 (0, 0) -> A1。
def cell_ref(row: int, col: int) -> str:
    return f"{excel_column_name(col)}{row + 1}"


# 写出字符串单元格（inlineStr，避免 sharedStrings 复杂度）。
def xml_string_cell(row: int, col: int, value: str) -> str:
    return (
        f'<c r="{cell_ref(row, col)}" t="inlineStr">'
        f"<is><t>{escape(value)}</t></is></c>"
    )


# 写出数值单元格。
def xml_number_cell(row: int, col: int, value: float | int) -> str:
    return f'<c r="{cell_ref(row, col)}"><v>{value}</v></c>'


# 写出布尔单元格（Excel 用 0/1）。
def xml_bool_cell(row: int, col: int, value: bool) -> str:
    return f'<c r="{cell_ref(row, col)}" t="b"><v>{1 if value else 0}</v></c>'


# 根据 sa_trace.json 内容拼出 sheet1.xml。
def build_sheet_xml(trace: dict) -> str:
    headers = [
        "index",
        "total",
        "move",
        "accept",
        "next_cost",
        "current_cost",
        "best_cost",
        "temperature",
    ]
    rows = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>']
    rows.append(
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        "<sheetData>"
    )

    header_cells = "".join(xml_string_cell(0, col, name) for col, name in enumerate(headers))
    rows.append(f'<row r="1">{header_cells}</row>')

    for row_index, entry in enumerate(trace.get("iterations", []), start=1):
        cells = [
            xml_number_cell(row_index, 0, int(entry.get("index", 0))),
            xml_number_cell(row_index, 1, int(entry.get("total", 0))),
            xml_string_cell(row_index, 2, str(entry.get("move", ""))),
            xml_bool_cell(row_index, 3, bool(entry.get("accept", False))),
            xml_number_cell(row_index, 4, float(entry.get("next_cost", 0.0))),
            xml_number_cell(row_index, 5, float(entry.get("current_cost", 0.0))),
            xml_number_cell(row_index, 6, float(entry.get("best_cost", 0.0))),
            xml_number_cell(row_index, 7, float(entry.get("temperature", 0.0))),
        ]
        rows.append(f'<row r="{row_index + 1}">{"".join(cells)}</row>')

    rows.append("</sheetData></worksheet>")
    return "".join(rows)


# 将 sa_trace.json 写成最小合法 xlsx（单 sheet，无额外依赖）。
def write_sa_trace_xlsx(trace_path: Path, output_path: Path) -> None:
    # 兼容 PowerShell/部分编辑器写出的 UTF-8 BOM。
    trace = json.loads(trace_path.read_text(encoding="utf-8-sig"))
    content_types = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>
  <Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
</Types>
"""
    root_rels = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>
</Relationships>
"""
    workbook = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"
          xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <sheets>
    <sheet name="sa_trace" sheetId="1" r:id="rId1"/>
  </sheets>
</workbook>
"""
    workbook_rels = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>
</Relationships>
"""
    sheet_xml = build_sheet_xml(trace)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types)
        archive.writestr("_rels/.rels", root_rels)
        archive.writestr("xl/workbook.xml", workbook)
        archive.writestr("xl/_rels/workbook.xml.rels", workbook_rels)
        archive.writestr("xl/worksheets/sheet1.xml", sheet_xml)


# 命令行入口：读取 JSON 并写出同目录或指定路径的 xlsx。
def main() -> int:
    parser = argparse.ArgumentParser(description="Export sa_trace.json to Excel xlsx")
    parser.add_argument("--trace", required=True, type=Path, help="Path to sa_trace.json")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output xlsx path; default is sa_trace.xlsx next to the JSON",
    )
    args = parser.parse_args()
    output = args.output if args.output is not None else args.trace.with_name("sa_trace.xlsx")
    write_sa_trace_xlsx(args.trace, output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

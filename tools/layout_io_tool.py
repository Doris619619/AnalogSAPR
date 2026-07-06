#!/usr/bin/env python3
"""Unified layout IO conversion and visualization tool for AnalogSAPR.

Subcommands:
  render-io: render an existing SAPR input/output directory.
  gds-to-io: convert GDSII plus optional LVS SPICE netlist into SAPR IO.
  oa-to-io: convert OA-exported src.net plus optional LVS SPICE into SAPR IO.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from collections import Counter
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parent
if os.fspath(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, os.fspath(TOOLS_DIR))


def infer_sp_path(gds_path: Path, circuit_name: str) -> Path | None:
    """Find the conventional LVS SPICE path used by the Lambda dataset."""
    sp_dir = (gds_path.parent / ".." / ".." / ".." / "0A_SRC.NET").resolve()
    candidates = [
        circuit_name,
        re.sub(r"_(ILP|LP|MILP|MILP2|P|7947|3887)$", "", circuit_name),
    ]
    for sp_name in candidates:
        path = sp_dir / f"{sp_name}.sp"
        if path.exists():
            return path
    return None


def run_render_io(args: argparse.Namespace) -> int:
    from render_layout import render_layout

    render_name = args.name or args.output.name or "layout"
    out_path = render_layout(
        input_dir=args.input,
        output_dir=args.output,
        render_name=render_name,
        dpi=args.dpi,
        show_labels=not args.no_labels,
        show_pins=not args.no_pins,
    )
    print(os.fspath(out_path))
    return 0


def run_gds_to_io(args: argparse.Namespace) -> int:
    try:
        import gds2io
    except ImportError as exc:
        print("ERROR: gds-to-io requires gdstk, matplotlib, and numpy.", file=sys.stderr)
        print(f"Import failed: {exc}", file=sys.stderr)
        return 2

    gds_path = args.gds.resolve()
    case_dir = args.case_dir.resolve()
    name = args.name

    print(f"Loading GDS: {gds_path}")
    lib = gds2io.gdstk.read_gds(os.fspath(gds_path))
    top_cells = lib.top_level()
    if not top_cells:
        print("ERROR: no top-level cells found in GDS.", file=sys.stderr)
        return 2

    top = top_cells[0]
    bb = top.bounding_box()
    if bb is not None:
        width = bb[1][0] - bb[0][0]
        height = bb[1][1] - bb[0][1]
        print(f"Top cell: {top.name}, {width:.0f}x{height:.0f} um, {len(top.references)} refs")
    else:
        print(f"Top cell: {top.name}, empty bbox, {len(top.references)} refs")

    devices = gds2io.extract_all(top)
    print(f"Devices: {len(devices)} ({dict(Counter(d['type'] for d in devices))})")
    if not devices:
        print("ERROR: no devices extracted from GDS.", file=sys.stderr)
        return 2

    ok_count, verify_errors = gds2io.verify(devices)
    if verify_errors:
        print(f"VERIFY: {ok_count}/{len(devices)} OK, {len(verify_errors)} mismatches")
        for error in verify_errors[:10]:
            print(f"  {error}")
    else:
        print(f"VERIFY: {ok_count}/{len(devices)} OK")

    sp_path = args.sp.resolve() if args.sp else infer_sp_path(gds_path, name)
    sp_instances = {}
    if sp_path and sp_path.exists():
        sp_instances = gds2io.parse_sp_netlist(os.fspath(sp_path))
        print(f"SPICE: {sp_path} ({len(sp_instances)} instances)")
    else:
        print("SPICE: not found; nets.txt will be empty")

    match_map = gds2io.match_sp_to_gds(sp_instances, devices)
    nets = gds2io.build_nets(sp_instances, match_map)
    routing = gds2io.extract_routing(top, devices)
    gds2io.write_io(devices, os.fspath(case_dir), name, routing=routing, nets=nets)

    assigned = sum(1 for segment in routing if segment["net"] != "UNASSIGNED")
    print(f"Wrote: {case_dir / 'input'}")
    print(f"Wrote: {case_dir / 'output'} ({len(routing)} route segments, {assigned} net-assigned)")

    if args.vis:
        png_path = case_dir / "output" / f"{name}_layout.png"
        gds2io.render(
            devices,
            os.fspath(png_path),
            name,
            show_labels=not args.no_labels,
            show_pins=not args.no_pins,
            dpi=args.dpi,
            routing=routing,
        )
        print(f"Saved: {png_path}")
    return 0


def run_oa_to_io(args: argparse.Namespace) -> int:
    import parse_oa

    case_dir = args.case_dir.resolve()
    src_net = args.src_net.resolve()
    sp_path = args.sp.resolve() if args.sp else None
    if sp_path and not sp_path.exists():
        print(f"ERROR: SPICE file does not exist: {sp_path}", file=sys.stderr)
        return 2

    parse_oa.convert(
        os.fspath(src_net),
        os.fspath(sp_path) if sp_path else None,
        os.fspath(case_dir),
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert GDS/OA/SPICE data to SAPR IO and render layout PNGs."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    render_io = subparsers.add_parser("render-io", help="Render existing SAPR input/output text files.")
    render_io.add_argument("--input", required=True, type=Path, help="Directory containing modules.txt and pins.txt.")
    render_io.add_argument("--output", required=True, type=Path, help="Directory containing placement.txt and routing.txt.")
    render_io.add_argument("--name", default=None, help="PNG basename prefix.")
    render_io.add_argument("--dpi", type=int, default=200, help="Output PNG DPI.")
    render_io.add_argument("--no-labels", action="store_true", help="Hide pin labels.")
    render_io.add_argument("--no-pins", action="store_true", help="Hide pin markers.")
    render_io.set_defaults(func=run_render_io)

    gds_to_io = subparsers.add_parser("gds-to-io", help="Convert GDSII plus optional SPICE netlist to SAPR IO.")
    gds_to_io.add_argument("--gds", required=True, type=Path, help="Input .gds or .calibre.db path.")
    gds_to_io.add_argument("--name", required=True, help="Circuit/case name.")
    gds_to_io.add_argument("--case-dir", required=True, type=Path, help="Output case directory containing input/ and output/.")
    gds_to_io.add_argument("--sp", type=Path, default=None, help="Optional LVS SPICE .sp path.")
    gds_to_io.add_argument("--vis", action="store_true", help="Render a layout PNG after conversion.")
    gds_to_io.add_argument("--dpi", type=int, default=200, help="Output PNG DPI.")
    gds_to_io.add_argument("--no-labels", action="store_true", help="Hide labels in visualization.")
    gds_to_io.add_argument("--no-pins", action="store_true", help="Hide pin markers.")
    gds_to_io.set_defaults(func=run_gds_to_io)

    oa_to_io = subparsers.add_parser("oa-to-io", help="Convert OA-exported src.net plus optional SPICE to SAPR IO.")
    oa_to_io.add_argument("--name", required=True, help="Circuit/case name, kept for command readability.")
    oa_to_io.add_argument("--src-net", required=True, type=Path, help="OA-exported auCdl .src.net path.")
    oa_to_io.add_argument("--sp", type=Path, default=None, help="Optional LVS SPICE .sp path.")
    oa_to_io.add_argument("--case-dir", required=True, type=Path, help="Output case directory containing input/ and output/.")
    oa_to_io.set_defaults(func=run_oa_to_io)

    return parser


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")

    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())

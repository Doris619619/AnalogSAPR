#!/usr/bin/env python3
"""
gds2io.py — GDS → IO format, one file, zero abstractions.

Reads a GDSII file via gdstk, extracts every device reference from the top cell,
writes IO/format files (modules.txt, pins.txt, placement.txt), and optionally
renders a PNG layout.

Usage:
    python3 IO/gds2io.py <gds_path> --name <circuit_name>
    python3 IO/gds2io.py <gds_path> --name <circuit_name> --vis --no-labels
"""

import gdstk, math, os, sys, re, argparse
from collections import Counter, defaultdict

# ── Netlist parsing (.sp LVS SPICE) ──────────────────────────────────
def parse_sp_netlist(sp_path):
    """Parse Calibre LVS .sp file → {instance: {model, type, nets: {pin: net}}}.

    The .sp file format:
      .SUBCKT <circuit> <port1> <port2> ...
      X<name> <node1> <node2> ... <model> $T=... $X=<x> $Y=<y>
      .ENDS

    Returns dict keyed by instance name (e.g. 'X4').
    """
    if not sp_path or not os.path.exists(sp_path):
        return {}

    with open(sp_path, encoding="utf-8") as f:
        text = f.read()

    # Find the .SUBCKT matching the circuit name (from filename)
    circuit = os.path.splitext(os.path.basename(sp_path))[0]
    top_subckt = None
    for m in re.finditer(r'^\.SUBCKT\s+(\S+)\s+(.*?)$', text, re.MULTILINE):
        name = m.group(1)
        ports = m.group(2).split()
        if name == circuit:
            start = m.end()
            depth = 1
            end = start
            for m2 in re.finditer(r'^\.(SUBCKT|ENDS)\b', text[start:], re.MULTILINE):
                if m2.group(1) == 'SUBCKT':
                    depth += 1
                else:
                    depth -= 1
                if depth == 0:
                    end = start + m2.start()
                    break
            top_subckt = {'name': name, 'ports': ports, 'body': text[start:end]}
            break

    if top_subckt is None:
        return {}

    instances = {}
    port_set = set(top_subckt['ports'])
    # Net name lookup: numbers → port names if they match
    net_names = {}
    for i, p in enumerate(top_subckt['ports']):
        net_names[str(i + 1)] = p

    for line in top_subckt['body'].split('\n'):
        line = line.strip()
        if not line.startswith('X'):
            continue
        parts = line.split()
        if len(parts) < 3:
            continue
        inst_name = parts[0]
        # Find model: the part containing '_CDNS_' or matching known PDK patterns.
        # $T=x y mirror rot introduces 4 extra space-separated values after $T=
        # that must be skipped.
        model_idx = None
        for i in range(1, len(parts)):
            p = parts[i]
            if p.startswith('$'):
                # Skip $T=... and its 4 numeric arguments
                if p.startswith('$T=') and i + 4 < len(parts):
                    i += 4  # skip x y mirror rot
                continue
            # Check if this looks like a model name (contains _CDNS_ or device keyword)
            pl = p.lower()
            if '_cdns_' in pl or any(kw in pl for kw in
                ['nch_', 'pch_', 'nmos', 'pmos', 'cfmom', 'rupoly', 'rppoly',
                 'ressppoly', 'rpod', 'rnod', 'mimcap']):
                model_idx = i
                break
        if model_idx is None:
            # Fallback: take the last non-$ part
            for i in range(len(parts) - 1, 0, -1):
                if not parts[i].startswith('$'):
                    model_idx = i
                    break
        if model_idx is None:
            continue
        model = parts[model_idx]
        nodes = parts[1:model_idx]

        # Map numeric nodes to port names
        resolved_nodes = []
        for n in nodes:
            resolved_nodes.append(net_names.get(n, n))

        # Determine device type and pin mapping from model name
        model_lower = model.lower()
        base = model.split('_CDNS_')[0] if '_CDNS_' in model else model
        base_lower = base.lower()

        if base_lower.startswith('nch') or 'nmos' in base_lower:
            dtype = 'nmos'
            # Nodes: D G S B
            pin_order = ['D', 'G', 'S', 'B']
        elif base_lower.startswith('pch') or 'pmos' in base_lower:
            dtype = 'pmos'
            pin_order = ['D', 'G', 'S', 'B']
        elif 'cfmom' in base_lower or 'mimcap' in base_lower:
            dtype = 'capacitor'
            pin_order = ['PLUS', 'MINUS']
        elif 'rupoly' in base_lower or 'rppoly' in base_lower or 'ressppoly' in base_lower:
            dtype = 'resistor'
            pin_order = ['PLUS', 'MINUS']  # resistors have 2 terminals
        else:
            dtype = 'unknown'
            pin_order = [f'N{i}' for i in range(len(resolved_nodes))]

        nets = {}
        for pi, pn in enumerate(pin_order):
            if pi < len(resolved_nodes):
                nets[pn] = resolved_nodes[pi]

        # Extract $X $Y position for matching
        mx = re.search(r'\$X=(\S+)', line)
        my = re.search(r'\$Y=(\S+)', line)
        pos_x = float(mx.group(1)) if mx else 0.0
        pos_y = float(my.group(1)) if my else 0.0

        instances[inst_name] = {
            'model': model, 'base': base, 'type': dtype,
            'nets': nets, 'nodes': resolved_nodes,
            'x': pos_x, 'y': pos_y,
        }

    return instances


def match_sp_to_gds(sp_instances, devices):
    """Match .sp instances to GDS devices by type + spatial order.

    Returns {sp_name: device_dict} mapping.
    """
    if not sp_instances:
        return {}

    # Group by type
    sp_by_type = defaultdict(list)
    for name, inst in sp_instances.items():
        sp_by_type[inst['type']].append((name, inst))

    gds_by_type = defaultdict(list)
    for d in devices:
        gds_by_type[d['type']].append(d)

    match = {}
    for dtype in sp_by_type:
        sp_list = sp_by_type[dtype]
        gds_list = gds_by_type.get(dtype, [])

        if not gds_list:
            continue

        # Sort both by position (x then y)
        sp_sorted = sorted(sp_list, key=lambda x: (x[1]['x'], x[1]['y']))
        gds_sorted = sorted(gds_list, key=lambda d: (d['x'], d['y']))

        # Match 1-to-1 by position order (works when counts match)
        for i in range(min(len(sp_sorted), len(gds_sorted))):
            sp_name, sp_inst = sp_sorted[i]
            gds_dev = gds_sorted[i]
            match[sp_name] = gds_dev

    return match


def build_nets(sp_instances, match_map):
    """Build netlist from .sp instances matched to GDS devices.

    Returns dict: {net_name: [(dev_id, pin_name), ...]}
    """
    nets = defaultdict(list)
    for sp_name, sp_inst in sp_instances.items():
        gds_dev = match_map.get(sp_name)
        if gds_dev is None:
            continue
        dev_id = gds_dev['id']
        for pin, net_name in sp_inst['nets'].items():
            nets[net_name].append(f'{dev_id}.{pin}')
    return dict(nets)

# ── Layer mapping (TSMC N28) ────────────────────────────────────────
# GDS layer number → metal name string
GDS_LAYER_TO_METAL = {
    31: 'M1', 32: 'M2', 33: 'M3', 34: 'M4', 35: 'M5', 36: 'M6', 37: 'M7',
    131: 'M1', 132: 'M2', 133: 'M3', 134: 'M4', 135: 'M5', 136: 'M6', 137: 'M7',
}
# GDS layer → metal int 1-7
def gds_to_metal_int(layer):
    s = GDS_LAYER_TO_METAL.get(layer, 'M1')
    return int(s[1:])

# ── Device classification ───────────────────────────────────────────
def classify_cell(cell_name):
    """cell name → 'nmos'|'pmos'|'capacitor'|'resistor'|None"""
    n = cell_name.lower()
    # NMOS
    for p in ['nch_', 'nmos_', 'nmos1v_']:
        if n.startswith(p): return 'nmos'
    # PMOS
    for p in ['pch_', 'pmos_', 'pmos1v_']:
        if n.startswith(p): return 'pmos'
    # Capacitor
    for p in ['cfmom', 'mimcap', 'mim_']:
        if p in n: return 'capacitor'
    # Resistor
    for p in ['rupoly', 'rppoly', 'ressppoly', 'rpod', 'rnod']:
        if p in n: return 'resistor'
    return None

# ── Orient ──────────────────────────────────────────────────────────
def ref_to_orient(ref):
    """gdstk Reference → (angle_degrees, orient_string)"""
    rot = round(math.degrees(ref.rotation) % 360)
    if ref.x_reflection:
        code = {0: 109, 90: 437, 180: 203, 270: 887}.get(rot, 109)
    else:
        code = rot
    orient_map = {0: 'R0', 90: 'R90', 180: 'R180', 270: 'R270',
                  109: 'MX', 203: 'MY', 437: 'MXR90', 887: 'MYR90'}
    angle_map = {0: 0, 109: 0, 203: 180, 90: 90, 437: 90, 180: 180, 270: 270, 887: 270}
    return angle_map[code], orient_map[code]

# ── Device extraction ───────────────────────────────────────────────
def extract_device(ref, idx, top_cell):
    """Extract one device from a gdstk Reference. Returns dict or None."""
    cell = ref.cell
    if cell is None:
        return None

    dtype = classify_cell(cell.name)
    if dtype is None:
        return None

    bb = cell.bounding_box()
    if bb is None:
        return None

    ox, oy = bb[0][0], bb[0][1]
    w = bb[1][0] - ox
    h = bb[1][1] - oy
    angle, orient = ref_to_orient(ref)

    # Device ID: prefix by type + index
    prefix = {'nmos': 'MM', 'pmos': 'MP', 'capacitor': 'XC', 'resistor': 'XR'}[dtype]
    dev_id = f'{prefix}{idx}'

    # Active area from OD layer (6)
    active = (0.0, 0.0, w, h)  # default: whole cell minus margin
    od_polys = [p for p in cell.polygons if p.layer == 6]
    if od_polys:
        all_x, all_y = [], []
        for p in od_polys:
            for pt in p.points:
                all_x.append(pt[0] - ox)
                all_y.append(pt[1] - oy)
        if all_x:
            active = (round(min(all_x), 3), round(min(all_y), 3),
                      round(max(all_x), 3), round(max(all_y), 3))

    # Pins from labels
    pins = {}
    pin_labels = {'S', 'D', 'G', 'B', 'PLUS', 'MINUS'}
    for lbl in cell.labels:
        text = lbl.text.strip()
        if text not in pin_labels:
            continue
        metal_int = gds_to_metal_int(lbl.layer)
        # Pin coords relative to cell origin
        px = round(lbl.origin[0] - ox, 3)
        py = round(lbl.origin[1] - oy, 3)
        # For MOS: keep first occurrence of each name (S/D may have
        # multiple labels in multi-finger cells)
        if text not in pins:
            pins[text] = (px, py, metal_int)

    # Infer MOS gate pin from PO layer (17) if missing
    if dtype in ('nmos', 'pmos') and 'G' not in pins:
        po_polys = [p for p in cell.polygons if p.layer == 17]
        if po_polys:
            best_area = 0
            best_center = None
            for p in po_polys:
                pts = [(pt[0], pt[1]) for pt in p.points]
                if len(pts) < 3: continue
                xs = [t[0] for t in pts]; ys = [t[1] for t in pts]
                area = (max(xs)-min(xs)) * (max(ys)-min(ys))
                cx = sum(xs)/len(xs)
                if area > best_area and ox < cx < ox + w:
                    best_area = area
                    best_center = (sum(xs)/len(xs), sum(ys)/len(ys))
            if best_center:
                pins['G'] = (round(best_center[0] - ox, 3),
                             round(best_center[1] - oy, 3), 1)

    # Infer bulk pin
    if dtype in ('nmos', 'pmos') and 'B' not in pins:
        pins['B'] = (round(w/2, 3), round(h/2, 3), 1)

    # Capacitor pins from M1 polygons if labels are absent
    if dtype == 'capacitor' and len(pins) < 2:
        m1_polys = [p for p in cell.polygons if p.layer == 31]
        if len(m1_polys) >= 2:
            centers = []
            for p in m1_polys:
                pts = [(pt[0], pt[1]) for pt in p.points]
                centers.append((sum(x for x,y in pts)/len(pts) - ox,
                                sum(y for x,y in pts)/len(pts) - oy))
            centers.sort(key=lambda c: c[1])
            pins['MINUS'] = (round(centers[0][0], 3), round(centers[0][1], 3), 1)
            pins['PLUS'] = (round(centers[-1][0], 3), round(centers[-1][1], 3), 1)

    return {
        'id': dev_id, 'type': dtype,
        'x': round(ref.origin[0], 3), 'y': round(ref.origin[1], 3),
        'w': round(w, 3), 'h': round(h, 3),
        'ox': round(ox, 3), 'oy': round(oy, 3),
        'angle': angle, 'orient': orient,
        'active': active, 'pins': pins,
        'cell': cell.name,
        '_ref': ref,  # for verification
    }

# ── Extract all devices (with recursive module support) ─────────────
def _has_device_children(cell):
    """Check if a cell contains device-type references."""
    for ref in cell.references:
        if classify_cell(ref.cell.name) is not None:
            return True
    return False


def _globalize_origin(sub_ref, parent_ref):
    """Compute where sub-device's cell origin (0,0) maps to globally.

    Uses gdstk nested Reference → flatten of a tiny marker at (0,0).
    Returns (global_x, global_y) or None.
    """
    # Create a tiny marker at (0,0) in the sub-device cell's coordinate system
    marker = gdstk.Cell('_mo')
    marker.add(gdstk.rectangle((-0.005, -0.005), (0.005, 0.005), layer=1))
    tmp = gdstk.Cell('_mo1')
    tmp.add(gdstk.Reference(
        marker,
        (sub_ref.origin[0], sub_ref.origin[1]),
        sub_ref.rotation, 1.0, sub_ref.x_reflection))
    tmp2 = gdstk.Cell('_mo2')
    tmp2.add(gdstk.Reference(
        tmp,
        (parent_ref.origin[0], parent_ref.origin[1]),
        parent_ref.rotation, 1.0, parent_ref.x_reflection))
    flat = tmp2.flatten()
    pts = [(pt[0], pt[1]) for p in flat.polygons for pt in p.points]
    if not pts:
        return None
    # The marker is centered at (0,0) in device cell coords —
    # after transforms, its center is at the global origin position
    return (round(sum(x for x,y in pts) / len(pts), 3),
            round(sum(y for x,y in pts) / len(pts), 3))


def extract_all(top_cell, lib_cells=None):
    """Return list of device dicts — recursively into hierarchical modules."""
    devices = []
    dev_idx = 0

    for ref in top_cell.references:
        dev = extract_device(ref, dev_idx, top_cell)
        if dev is not None:
            devices.append(dev)
            dev_idx += 1
            continue

        # Not a leaf device — check if it's a module with device children
        cell_name = ref.cell.name if ref.cell else ''
        sub_cell = ref.cell
        if sub_cell is None:
            continue
        if not _has_device_children(sub_cell):
            continue

        # Recursively extract devices from the sub-module
        for sub_ref in sub_cell.references:
            sub_dev = extract_device(sub_ref, dev_idx, top_cell)
            if sub_dev is None:
                continue

            # Compute global origin position using gdstk nesting
            global_orig = _globalize_origin(sub_ref, ref)
            if global_orig is None:
                continue
            gx, gy = global_orig

            # Compute COMBINED orient for nested device
            p_angle, _ = ref_to_orient(ref)
            d_angle = sub_dev['angle']
            p_xref = ref.x_reflection
            d_xref = (sub_dev['orient'] in ('MX', 'MY', 'MXR90', 'MYR90'))

            combined_xref = p_xref ^ d_xref
            if d_xref:
                combined_angle = (d_angle - p_angle) % 360
            else:
                combined_angle = (p_angle + d_angle) % 360

            orient_code = ({0: 109, 90: 437, 180: 203, 270: 887}.get(combined_angle, combined_angle)
                          if combined_xref else combined_angle)
            orient_map = {0: 'R0', 90: 'R90', 180: 'R180', 270: 'R270',
                          109: 'MX', 203: 'MY', 437: 'MXR90', 887: 'MYR90'}

            # Keep original cell properties (ox, oy, w, h, pins)
            # Update only the global position and combined orient
            sub_dev['x'] = gx
            sub_dev['y'] = gy
            sub_dev['angle'] = combined_angle
            sub_dev['orient'] = orient_map.get(orient_code, 'R0')
            sub_dev['_ref'] = sub_ref
            devices.append(sub_dev)
            dev_idx += 1

    return devices

# ── Self-verification ───────────────────────────────────────────────
def verify(devices):
    """Check every device matches gdstk raw data. Returns (ok, errors)."""
    errors = []
    for d in devices:
        ref = d['_ref']
        bb = ref.cell.bounding_box()
        gw = bb[1][0] - bb[0][0]
        gh = bb[1][1] - bb[0][1]
        gox, goy = bb[0][0], bb[0][1]
        ga, go = ref_to_orient(ref)

        # For nested devices, position fields are globalized — only check w/h
        is_nested = (abs(d['x'] - ref.origin[0]) > 1.0 or
                     abs(d['y'] - ref.origin[1]) > 1.0)

        checks = []
        if not is_nested:
            if abs(d['x'] - ref.origin[0]) > 0.001: checks.append(f'x:{d["x"]} vs {ref.origin[0]:.3f}')
            if abs(d['y'] - ref.origin[1]) > 0.001: checks.append(f'y:{d["y"]} vs {ref.origin[1]:.3f}')
            if abs(d['ox'] - gox) > 0.001: checks.append(f'ox:{d["ox"]} vs {gox:.3f}')
            if abs(d['oy'] - goy) > 0.001: checks.append(f'oy:{d["oy"]} vs {goy:.3f}')
            if d['orient'] != go: checks.append(f'orient:{d["orient"]} vs {go}')
            if d['angle'] != ga: checks.append(f'angle:{d["angle"]} vs {ga}')
        if abs(d['w'] - gw) > 0.001: checks.append(f'w:{d["w"]} vs {gw:.3f}')
        if abs(d['h'] - gh) > 0.001: checks.append(f'h:{d["h"]} vs {gh:.3f}')

        if checks:
            errors.append(f'{d["id"]} ({d["cell"][:40]}): {", ".join(checks)}')

    return len(devices) - len(errors), errors

# ── Routing extraction ──────────────────────────────────────────────
def extract_routing(top_cell, devices):
    """Extract routing wire segments from top-level metal polygons.

    Flattens the top cell, filters out metal inside device BBs,
    converts remaining polygons to centerline wire segments.
    Returns list of {net, layer, x1, y1, x2, y2, width}.
    """
    METAL_LAYERS = {31, 32, 33, 34, 35, 36, 37}

    # 1. Compute device global BBs AND fill/substrate BBs (to exclude)
    exclude_bbs = []

    # Device BBs
    for d in devices:
        corners = d.get('_corners')
        if corners is None:
            continue
        gx = [c[0] for c in corners]; gy = [c[1] for c in corners]
        exclude_bbs.append((min(gx), min(gy), max(gx), max(gy)))

    # Fill / substrate / N-well reference BBs (these connect everything)
    fill_patterns = ['m1_sub', 'm1_nw', 'm1_psub']
    for ref in top_cell.references:
        cn = ref.cell.name.lower()
        if not any(p in cn for p in fill_patterns):
            continue
        # Compute this fill ref's global BB using gdstk
        tmp = gdstk.Cell('_fx')
        tmp.add(gdstk.Reference(ref.cell, ref.origin, ref.rotation,
                                ref.magnification, ref.x_reflection))
        flat = tmp.flatten()
        pts = [(pt[0], pt[1]) for p in flat.polygons for pt in p.points]
        if pts:
            gx = [p[0] for p in pts]; gy = [p[1] for p in pts]
            exclude_bbs.append((min(gx), min(gy), max(gx), max(gy)))

    # 2. Collect metal polygons from each non-device, non-fill reference
    #    INDIVIDUALLY — avoids gdstk flatten() merging adjacent rectangles
    #    into complex polygons that lose individual segment identity.
    routing_polys = []
    device_patterns = ['nch_', 'pch_', 'nmos', 'pmos', 'cfmom', 'rupoly', 'rppoly', 'ressppoly']

    for ref in top_cell.references:
        cn = ref.cell.name.lower()

        # Skip device cells (their metal is device-internal)
        if any(p in cn for p in device_patterns):
            continue
        # Skip fill/substrate
        if any(p in cn for p in fill_patterns):
            continue

        # Flatten this single reference — its polygons stay separate
        tmp = gdstk.Cell('_rp')
        tmp.add(gdstk.Reference(ref.cell, ref.origin, ref.rotation,
                                ref.magnification, ref.x_reflection))
        flat_one = tmp.flatten()

        for poly in flat_one.polygons:
            if poly.layer not in METAL_LAYERS:
                continue
            pts = [(pt[0], pt[1]) for pt in poly.points]
            if len(pts) < 3:
                continue
            xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
            pw = max(xs) - min(xs); ph = max(ys) - min(ys)

            # Skip tiny via-sized polygons
            if pw < 0.1 and ph < 0.1:
                continue

            # Skip if entirely inside excluded BB
            pminx, pmaxx = min(xs), max(xs)
            pminy, pmaxy = min(ys), max(ys)
            inside = False
            for (dx1, dy1, dx2, dy2) in exclude_bbs:
                if pminx >= dx1 - 0.01 and pmaxx <= dx2 + 0.01 and \
                   pminy >= dy1 - 0.01 and pmaxy <= dy2 + 0.01:
                    inside = True
                    break
            if inside:
                continue

            routing_polys.append((poly.layer, pts))

        for path in flat_one.paths:
            layer = path.layers[0] if hasattr(path, 'layers') and path.layers else 0
            if layer not in METAL_LAYERS:
                continue
            pw_path = getattr(path, 'width', 0.5)
            spine = [(pt[0], pt[1]) for pt in path.spine()]
            if len(spine) < 2:
                continue
            for i in range(len(spine) - 1):
                routing_polys.append((layer, [spine[i], spine[i + 1]], pw_path))

    # 3. Also process direct polygons in the top cell (not in any reference)
    for poly in top_cell.polygons:
        if poly.layer not in METAL_LAYERS:
            continue
        pts = [(pt[0], pt[1]) for pt in poly.points]
        if len(pts) < 3:
            continue
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        pw = max(xs) - min(xs); ph = max(ys) - min(ys)
        if pw < 0.1 and ph < 0.1:
            continue
        routing_polys.append((poly.layer, pts))

    for path in top_cell.paths:
        layer = path.layers[0] if hasattr(path, 'layers') and path.layers else 0
        if layer not in METAL_LAYERS:
            continue
        pw_path = getattr(path, 'width', 0.5)
        spine = [(pt[0], pt[1]) for pt in path.spine()]
        if len(spine) < 2:
            continue
        for i in range(len(spine) - 1):
            routing_polys.append((layer, [spine[i], spine[i + 1]], pw_path))

    # 4. Convert polygons to centerline wire segments
    segments = []
    for item in routing_polys:
        if len(item) == 3:  # path segment with explicit width
            layer, pts, width = item
            x1, y1 = pts[0]
            x2, y2 = pts[-1]
        else:  # polygon — extract centerline
            layer, pts = item
            xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
            w = max(xs) - min(xs); h = max(ys) - min(ys)
            cx = sum(xs) / len(xs); cy = sum(ys) / len(ys)

            if w >= h:
                # Horizontal segment: center Y through centroid
                x1, x2 = min(xs), max(xs)
                y1 = y2 = cy
                width = h
            else:
                # Vertical segment: center X through centroid
                y1, y2 = min(ys), max(ys)
                x1 = x2 = cx
                width = w

            # Skip segments with negligible length
            if abs(x2 - x1) + abs(y2 - y1) < 0.05:
                continue

        metal_name = f'M{layer - 30}'
        segments.append({
            'net': 'UNASSIGNED',
            'layer': metal_name,
            'x1': round(x1, 3), 'y1': round(y1, 3),
            'x2': round(x2, 3), 'y2': round(y2, 3),
            'width': round(width, 3),
        })

    # 5. Net assignment from port labels
    _assign_nets(segments, top_cell, devices)

    # 6. Merge colinear same-net same-layer segments
    segments = _merge_segments(segments)

    # 7. Remove duplicate segments
    seen = set()
    unique = []
    for s in segments:
        key = (s['net'], s['layer'], s['x1'], s['y1'], s['x2'], s['y2'], s['width'])
        if key not in seen:
            seen.add(key)
            unique.append(s)

    return unique


def _assign_nets(segments, top_cell, devices):
    """Assign net names to segments via connectivity from port labels and device pins."""
    if not segments:
        return

    # Collect seeds: port labels in the top cell
    seeds = []  # (x, y, layer_int, net_name)
    for lbl in top_cell.labels:
        metal_int = {131: 1, 132: 2, 133: 3, 134: 4, 135: 5, 136: 6, 137: 7,
                     31: 1, 32: 2, 33: 3, 34: 4, 35: 5, 36: 6, 37: 7}.get(lbl.layer)
        if metal_int:
            seeds.append((lbl.origin[0], lbl.origin[1], metal_int, lbl.text.strip()))

    if not seeds:
        return

    # Build segment lookup: for each metal layer, list of (idx, bbox)
    by_layer = {1: [], 2: [], 3: [], 4: [], 5: [], 6: [], 7: []}
    for i, s in enumerate(segments):
        li = int(s['layer'][1:])
        x1, x2 = min(s['x1'], s['x2']), max(s['x1'], s['x2'])
        y1, y2 = min(s['y1'], s['y2']), max(s['y1'], s['y2'])
        # Expand bbox by half width for overlap detection
        hw = s['width'] / 2
        by_layer[li].append((i, x1 - hw, y1 - hw, x2 + hw, y2 + hw))

    # BFS from each seed — process all seeds, assign to first-come
    all_assigned = {}  # idx -> net
    for sx, sy, sl, net in seeds:
        # Find starting segment (must be unassigned or same-net)
        start_idx = None
        for idx, x1, y1, x2, y2 in by_layer[sl]:
            if x1 <= sx <= x2 and y1 <= sy <= y2:
                if idx not in all_assigned or all_assigned[idx] == net:
                    start_idx = idx
                break
        if start_idx is None:
            continue

        # BFS: expand through UNASSIGNED segments only
        queue = [start_idx]
        visited = {start_idx}

        while queue:
            cur = queue.pop(0)
            cur_s = segments[cur]
            cur_l = int(cur_s['layer'][1:])
            cx1 = min(cur_s['x1'], cur_s['x2']); cx2 = max(cur_s['x1'], cur_s['x2'])
            cy1 = min(cur_s['y1'], cur_s['y2']); cy2 = max(cur_s['y1'], cur_s['y2'])
            chw = cur_s['width'] / 2

            # Only flood within the SAME metal layer (don't traverse vias)
            for nl in (cur_l,):
                if nl not in by_layer:
                    continue
                for idx, bx1, by1, bx2, by2 in by_layer[nl]:
                    if idx in visited:
                        continue
                    if idx in all_assigned and all_assigned[idx] != net:
                        continue
                    if cx2 + chw >= bx1 and cx1 - chw <= bx2 and \
                       cy2 + chw >= by1 and cy1 - chw <= by2:
                        visited.add(idx)
                        queue.append(idx)

        for idx in visited:
            if idx not in all_assigned:
                all_assigned[idx] = net

    for idx, net in all_assigned.items():
        segments[idx]['net'] = net


def _merge_segments(segments):
    """Merge colinear same-net same-layer segments."""
    if not segments:
        return segments

    # Group by (net, layer)
    groups = {}
    for i, s in enumerate(segments):
        key = (s['net'], s['layer'])
        groups.setdefault(key, []).append(i)

    merged = []
    used = set()
    for (net, layer), indices in groups.items():
        # Sort by x then y
        items = [(segments[i], i) for i in indices]
        items.sort(key=lambda x: (x[0]['x1'], x[0]['y1']))

        i = 0
        while i < len(items):
            si, oi = items[i]
            if oi in used:
                i += 1
                continue
            # Try to extend this segment with subsequent colinear ones
            best_j = i
            j = i + 1
            while j < len(items):
                sj, oj = items[j]
                if oj in used:
                    j += 1
                    continue
                # Check colinear
                is_h = abs(si['y1'] - si['y2']) < 0.01
                sj_h = abs(sj['y1'] - sj['y2']) < 0.01
                if is_h != sj_h:
                    j += 1
                    continue
                if is_h:
                    # Both horizontal: same y, overlapping or adjacent x
                    if abs(si['y1'] - sj['y1']) < 0.1 and abs(si['width'] - sj['width']) < 0.1:
                        si_x2 = max(si['x1'], si['x2'])
                        sj_x1 = min(sj['x1'], sj['x2'])
                        if si_x2 >= sj_x1 - 0.1:
                            best_j = j
                            # Merge
                            si['x2'] = max(si_x2, max(sj['x1'], sj['x2']))
                            si['x1'] = min(si['x1'], si['x2'], sj['x1'], sj['x2'])
                            if si['x1'] == sj['x1']:
                                si['x1'] = min(si['x1'], sj['x1'])
                            used.add(oj)
                else:
                    # Both vertical: same x, overlapping or adjacent y
                    if abs(si['x1'] - sj['x1']) < 0.1 and abs(si['width'] - sj['width']) < 0.1:
                        si_y2 = max(si['y1'], si['y2'])
                        sj_y1 = min(sj['y1'], sj['y2'])
                        if si_y2 >= sj_y1 - 0.1:
                            best_j = j
                            si['y2'] = max(si_y2, max(sj['y1'], sj['y2']))
                            si['y1'] = min(si['y1'], si['y2'], sj['y1'], sj['y2'])
                            used.add(oj)
                j += 1
            merged.append(si)
            used.add(oi)
            i += 1

    # Also include unmerged segments
    for i, s in enumerate(segments):
        if i not in used:
            merged.append(s)

    return merged


# ── Write IO format ─────────────────────────────────────────────────
def write_io(devices, out_dir, circuit_name, routing=None, nets=None):
    """Write modules.txt, pins.txt, placement.txt, routing.txt to out_dir."""
    inp = os.path.join(out_dir, 'input')
    out = os.path.join(out_dir, 'output')
    os.makedirs(inp, exist_ok=True)
    os.makedirs(out, exist_ok=True)

    # modules.txt
    with open(os.path.join(inp, 'modules.txt'), 'w', encoding="utf-8") as f:
        f.write(f'# {circuit_name}\n')
        f.write('# id          w       h       active(x1,y1,x2,y2)         # info\n')
        f.write('\n')
        for d in devices:
            a = d['active']
            f.write(f'{d["id"]:<12s} {d["w"]:<7.3f} {d["h"]:<7.3f} '
                    f'{a[0]:.3f},{a[1]:.3f},{a[2]:.3f},{a[3]:.3f}  '
                    f'# {d["type"]} {d["cell"]} ox={d["ox"]:.3f} oy={d["oy"]:.3f}\n')

    # pins.txt
    with open(os.path.join(inp, 'pins.txt'), 'w', encoding="utf-8") as f:
        f.write('# module      pin      x        y        layer\n')
        f.write('\n')
        for d in devices:
            for pn in sorted(d['pins'].keys()):
                px, py, pl = d['pins'][pn]
                f.write(f'{d["id"]:<12s} {pn:<8s} {px:<8.3f} {py:<8.3f} M{pl}\n')

    # placement.txt
    with open(os.path.join(out, 'placement.txt'), 'w', encoding="utf-8") as f:
        f.write('# module       x           y           angle  orient\n')
        f.write('\n')
        for d in devices:
            f.write(f'{d["id"]:<14s} {d["x"]:<10.2f} {d["y"]:<10.2f} '
                    f'{d["angle"]:<6d} {d["orient"]}\n')

    # nets.txt
    with open(os.path.join(inp, 'nets.txt'), 'w', encoding="utf-8") as f:
        f.write('# net          priority    terminals\n')
        f.write('\n')
        if nets:
            for net_name in sorted(nets.keys()):
                terms = nets[net_name]
                priority = 'critical' if net_name.upper() in (
                    'VDD', 'AVDD', 'VCC', 'GND', 'AGND', 'VSS', 'OUT', 'VOUT'
                ) else 'normal'
                f.write(f'{net_name:<14s} {priority:<10s} {" ".join(terms)}\n')

    # constraints.txt (template)
    with open(os.path.join(inp, 'constraints.txt'), 'w', encoding="utf-8") as f:
        f.write('# SYMMETRY_PAIR / SYMMETRY_SELF / FLOW / WIRE_WIDTH\n')
        f.write('\n')

    # routing.txt
    with open(os.path.join(out, 'routing.txt'), 'w', encoding="utf-8") as f:
        f.write('# net         layer  x1       y1       x2       y2       width\n')
        f.write('\n')
        if routing:
            for s in routing:
                f.write(f'{s["net"]:<14s} {s["layer"]:<6s} '
                        f'{s["x1"]:<8.3f} {s["y1"]:<8.3f} '
                        f'{s["x2"]:<8.3f} {s["y2"]:<8.3f} '
                        f'{s["width"]:.3f}\n')

# ── Visualization ───────────────────────────────────────────────────
def render(devices, out_path, circuit_name, show_labels=True, show_pins=True, dpi=200, routing=None):
    """Render device BBs and routing wires using gdstk to compute global positions.

    Uses gdstk's flatten() to transform cell-local polygon coordinates
    to global coordinates — zero custom transform math, guaranteed correct.
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import numpy as np

    DEV_COLORS = {
        'nmos': '#2196F3', 'pmos': '#F44336',
        'capacitor': '#4CAF50', 'resistor': '#FF9800',
    }
    LAYER_COLORS = {
        'M1': '#2196F3', 'M2': '#F44336', 'M3': '#4CAF50',
        'M4': '#FF9800', 'M5': '#9C27B0', 'M6': '#00BCD4', 'M7': '#795548',
    }

    all_x, all_y = [], []

    for d in devices:
        ref = d['_ref']

        # Use gdstk to get the global BB of this reference.
        # Create a Reference with the same transform and let gdstk flatten it.
        # This guarantees the transform is correct — no custom math.
        ref_copy = gdstk.Reference(
            ref.cell,
            origin=(d['x'], d['y']),
            rotation=math.radians(d['angle']),
            magnification=1.0,
            x_reflection=(d['orient'] in ('MX', 'MY', 'MXR90', 'MYR90'))
        )
        tmp = gdstk.Cell('_tmp')
        tmp.add(ref_copy)
        flat = tmp.flatten()

        pts = [(pt[0], pt[1]) for p in flat.polygons for pt in p.points]
        if pts:
            gx = [p[0] for p in pts]; gy = [p[1] for p in pts]
            corners = np.array([
                [min(gx), min(gy)], [max(gx), min(gy)],
                [max(gx), max(gy)], [min(gx), max(gy)],
            ], dtype=float)
        else:
            corners = np.array([
                [d['x'], d['y']], [d['x']+d['w'], d['y']],
                [d['x']+d['w'], d['y']+d['h']], [d['x'], d['y']+d['h']],
            ], dtype=float)

        d['_corners'] = corners
        all_x.extend(corners[:, 0])
        all_y.extend(corners[:, 1])

        # Compute global pin positions using gdstk (same approach as BB)
        d['_pin_global'] = {}
        if show_pins:
            for pn, (px, py, pl) in d['pins'].items():
                # Convert pin coords (relative to BB origin) to cell-local
                lx, ly = px + d['ox'], py + d['oy']
                # Create a tiny marker cell at the pin position, transform with gdstk
                pin_cell = gdstk.Cell('_p')
                pin_cell.add(gdstk.rectangle((lx-0.005, ly-0.005), (lx+0.005, ly+0.005), layer=1))
                pin_ref = gdstk.Reference(
                    pin_cell,
                    origin=(d['x'], d['y']),
                    rotation=math.radians(d['angle']),
                    magnification=1.0,
                    x_reflection=(d['orient'] in ('MX', 'MY', 'MXR90', 'MYR90'))
                )
                pin_tmp = gdstk.Cell('_pt')
                pin_tmp.add(pin_ref)
                pin_flat = pin_tmp.flatten()
                pin_pts = [(pt[0], pt[1]) for p in pin_flat.polygons for pt in p.points]
                if pin_pts:
                    gx_pin = sum(x for x,y in pin_pts) / len(pin_pts)
                    gy_pin = sum(y for x,y in pin_pts) / len(pin_pts)
                    d['_pin_global'][pn] = (gx_pin, gy_pin, pl)

    if not all_x:
        all_x, all_y = [0, 100], [0, 100]

    margin = max(max(all_x)-min(all_x), max(all_y)-min(all_y)) * 0.08
    xlim = (min(all_x) - margin, max(all_x) + margin)
    ylim = (min(all_y) - margin, max(all_y) + margin)

    fig_w = max(8, min(24, (xlim[1]-xlim[0]) / 20))
    fig_h = max(8, min(24, (ylim[1]-ylim[0]) / 20))
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=dpi)
    ax.set_aspect('equal')
    ax.set_xlim(xlim); ax.set_ylim(ylim)
    ax.grid(True, color='#EEEEEE', linewidth=0.5, alpha=0.7)

    # Draw devices
    for d in devices:
        corners = d['_corners']
        color = DEV_COLORS.get(d['type'], '#999')
        poly = mpatches.Polygon(corners, closed=True, facecolor=color,
                                edgecolor=color, linewidth=1.0, alpha=0.35, zorder=3)
        ax.add_patch(poly)

        # Label at center — always show, bold, on top
        cx = corners[:, 0].mean()
        cy = corners[:, 1].mean()
        short_id = d['id']
        ax.text(cx, cy, short_id, ha='center', va='center', fontsize=5,
                fontweight='bold', color='#111111', zorder=10)

        # Pins
        if show_pins:
            for pn, (gx, gy, pl) in d.get('_pin_global', {}).items():
                color = LAYER_COLORS.get(f'M{pl}', '#333')
                ax.plot(gx, gy, marker='x', color=color, markersize=4,
                        markeredgewidth=1.5, zorder=11)
                if show_labels:
                    ax.annotate(pn, (gx, gy), textcoords="offset points",
                                xytext=(3, 3), fontsize=4, color=color, zorder=12)

    # Legend
    types_seen = set(d['type'] for d in devices)
    legend = []
    for t in sorted(types_seen):
        legend.append(mpatches.Patch(facecolor=DEV_COLORS[t], edgecolor=DEV_COLORS[t],
                      alpha=0.35, label=t))
    # Show used metal layers
    if routing:
        layers_in_use = set(s['layer'] for s in routing)
        for l in sorted(layers_in_use):
            color = LAYER_COLORS.get(l, '#999')
            legend.append(mpatches.Patch(color=color, label=f'{l} routing'))
    ax.legend(handles=legend, loc='upper right', fontsize=7, framealpha=0.8)

    # Draw routing as filled polygons with actual physical width.
    # For large routing counts, thin aggressively to keep render fast.
    MAX_ROUTING_POLYS = 5000
    if routing:
        # Sort by length descending — keep long segments first
        sorted_segs = sorted(routing, key=lambda s: -max(
            abs(s['x2']-s['x1']), abs(s['y2']-s['y1'])))
        for s in sorted_segs[:MAX_ROUTING_POLYS]:
            color = LAYER_COLORS.get(s['layer'], '#999')
            x1, y1, x2, y2 = s['x1'], s['y1'], s['x2'], s['y2']
            hw = s['width'] / 2.0

            if abs(x2 - x1) < 0.001:
                rcorners = np.array([
                    [x1 - hw, y1], [x1 + hw, y1],
                    [x2 + hw, y2], [x2 - hw, y2]
                ])
            elif abs(y2 - y1) < 0.001:
                rcorners = np.array([
                    [x1, y1 - hw], [x1, y1 + hw],
                    [x2, y2 + hw], [x2, y2 - hw]
                ])
            else:
                dx, dy = x2 - x1, y2 - y1
                length = math.hypot(dx, dy)
                px, py = -dy / length * hw, dx / length * hw
                rcorners = np.array([
                    [x1 + px, y1 + py], [x1 - px, y1 - py],
                    [x2 - px, y2 - py], [x2 + px, y2 + py]
                ])

            rpoly = mpatches.Polygon(rcorners, closed=True, facecolor=color,
                                     edgecolor='none', alpha=0.55, zorder=1)
            ax.add_patch(rpoly)

    ax.set_title(f'{circuit_name} — {len(devices)} devices', fontsize=10, fontweight='bold')
    ax.set_xlabel('X (um)'); ax.set_ylabel('Y (um)')
    ax.tick_params(labelsize=7)
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return out_path

# ── Main ────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(description='gds2io.py — GDS → IO format')
    p.add_argument('gds_path', help='Path to .gds or .calibre.db file')
    p.add_argument('--name', '-n', required=True, help='Circuit name for output directory')
    p.add_argument('--io-dir', default='.', help='IO root directory')
    p.add_argument('--vis', action='store_true', help='Render PNG visualization')
    p.add_argument('--no-labels', action='store_true', help='Hide labels in visualization')
    p.add_argument('--no-pins', action='store_true', help='Hide pin markers')
    p.add_argument('--dpi', type=int, default=200, help='Output DPI for PNG')
    args = p.parse_args()

    # 1. Read GDS
    print(f'Loading: {args.gds_path}')
    lib = gdstk.read_gds(args.gds_path)
    top_cells = lib.top_level()
    if not top_cells:
        print('ERROR: No top-level cells found')
        sys.exit(1)
    top = top_cells[0]
    bb = top.bounding_box()
    tw = bb[1][0] - bb[0][0]
    th = bb[1][1] - bb[0][1]
    print(f'Top cell: {top.name}, {tw:.0f}x{th:.0f} um, {len(top.references)} refs')

    # 2. Extract devices
    devices = extract_all(top)
    types = Counter(d['type'] for d in devices)
    print(f'Devices: {len(devices)} ({dict(types)})')

    if not devices:
        print('ERROR: No devices extracted')
        sys.exit(1)

    # 3. Self-verify
    ok, errors = verify(devices)
    if errors:
        print(f'VERIFY: {ok}/{len(devices)} OK — {len(errors)} MISMATCHES:')
        for e in errors[:10]:
            print(f'  {e}')
        if len(errors) > 10:
            print(f'  ... and {len(errors)-10} more')
    else:
        print(f'VERIFY: {ok}/{len(devices)} OK ✓')

    # 4. Parse netlist (.sp) if available
    # The .sp files are in TO_AutoLayout/0A_SRC.NET/
    # For variant suffixes (_ILP, _LP, etc), fall back to base name
    sp_dir = os.path.join(os.path.dirname(args.gds_path), '..', '..', '..', '0A_SRC.NET')
    sp_dir = os.path.normpath(sp_dir)
    # Try exact name first, then strip variant suffixes
    for sp_name in [args.name,
                    re.sub(r'_(ILP|LP|MILP|MILP2|P|7947|3887)$', '', args.name)]:
        sp_path = os.path.join(sp_dir, f'{sp_name}.sp')
        if os.path.exists(sp_path):
            break
    sp_instances = parse_sp_netlist(sp_path) if os.path.exists(sp_path) else {}
    match_map = match_sp_to_gds(sp_instances, devices)
    nets = build_nets(sp_instances, match_map)
    if nets:
        print(f'Nets: {len(nets)} nets from {sp_path}')

    # 5. Write IO files
    out_dir = os.path.join(args.io_dir, args.name)
    routing = extract_routing(top, devices)
    write_io(devices, out_dir, args.name, routing=routing, nets=nets)
    assigned = sum(1 for s in routing if s['net'] != 'UNASSIGNED')
    print(f'Wrote: {out_dir}/input/{{modules,pins,nets,constraints}}.txt')
    print(f'Wrote: {out_dir}/output/{{placement,routing}}.txt ({len(routing)} segments, {assigned} net-assigned)')

    # 5. Render
    if args.vis:
        png_path = os.path.join(out_dir, 'output', f'{args.name}_layout.png')
        render(devices, png_path, args.name,
               show_labels=not args.no_labels,
               show_pins=not args.no_pins,
               dpi=args.dpi,
               routing=routing)
        print(f'Saved: {png_path}')

if __name__ == '__main__':
    main()

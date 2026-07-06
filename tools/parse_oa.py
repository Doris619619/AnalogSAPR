#!/usr/bin/env python3
"""
parse_oa.py — 解析 Cadence OA 版图数据 → IO 标准格式

输入：
    OA 数据库导出的文本数据（src.net + .sp），因为 .oa 本身是二进制格式，
    必须通过 Cadence SKILL/oaPython 或 Calibre LVS 导出为文本后才能解析。

    本脚本使用两个文件：
      - <circuit>.src.net   : auCdl 导出的原理图网表（器件参数 + 连接关系）
      - <circuit>.sp        : Calibre LVS 从版图提取的 SPICE 网表（含 $X $Y 坐标）

输出（写入 IO/<circuit>/ 目录）：
    input/modules.txt       : 器件物理尺寸和有源区
    input/pins.txt          : 每个 pin 的坐标和层
    input/nets.txt          : 连接关系
    input/constraints.txt   : 设计约束（模板，需人工补充）
    output/placement.txt    : 版图中的实际放置坐标
    output/routing.txt      : 空模板

用法：
    python parse_oa.py <circuit_name> [--src-net PATH] [--sp PATH]

示例：
    python parse_oa.py peak_detection
"""

import re
import sys
import os
import math
from pathlib import Path
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import argparse


# ============================================================
# 数据模型
# ============================================================

@dataclass
class Module:
    """一个器件 BB（Building Block）"""
    id: str                     # 器件名
    dev_type: str               # nmos, pmos, resistor, capacitor
    model: str                  # PDK 模型名
    w: float = 0.0              # BB 宽度 (μm)
    h: float = 0.0              # BB 高度 (μm)
    active: Tuple[float, float, float, float] = (0, 0, 0, 0)
    pins: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)  # pin→(x,y,layer_int)
    x: float = 0.0              # 版图坐标 X (μm)
    y: float = 0.0              # 版图坐标 Y (μm)
    orient: str = "R0"
    m: int = 1
    parameters: Dict[str, float] = field(default_factory=dict)
    # 连线签名（用于与 LVS 实例匹配）
    conn_signature: str = ""  # 如 "D:net1 G:net2 S:net3 B:net3"


@dataclass
class Net:
    name: str
    terminals: List[str] = field(default_factory=list)
    priority: str = "normal"


@dataclass
class LvsInstance:
    """LVS SPICE 中的一个版图实例"""
    ref: str        # 实例名 X0, M60 等
    model: str      # 子电路名或模型名
    dev_type: str   # mos, resistor, capacitor
    x: float        # μm
    y: float
    d: int          # orient code
    orient: str     # R0/R90/...
    nets: List[str] = field(default_factory=list)  # 连接的 net 名（按顺序）
    # 器件参数
    L: float = 0.0
    W: float = 0.0
    m: int = 1


# ============================================================
# 通用辅助函数
# ============================================================

def parse_value(s: str) -> float:
    """解析 SPICE 数值: 1u → 1e-6, 400n → 4e-7, 750.0n → 7.5e-7"""
    s = s.strip().lower()
    multipliers = {
        'f': 1e-15, 'p': 1e-12, 'n': 1e-9, 'u': 1e-6,
        'm': 1e-3, 'k': 1e3, 'meg': 1e6, 'g': 1e9
    }
    try:
        return float(s)
    except ValueError:
        pass
    for suffix, mult in sorted(multipliers.items(), key=lambda x: -len(x[0])):
        if s.endswith(suffix):
            try:
                return float(s[:-len(suffix)]) * mult
            except ValueError:
                pass
    num_match = re.match(r'([\d.]+)', s)
    if num_match:
        return float(num_match.group(1)) * 1e-6
    return 0.0


def orient_code_to_str(d: int) -> str:
    """Cadence orient code → 角度"""
    mapping = {0: "R0", 90: "R90", 180: "R180", 270: "R270",
               109: "MX", 203: "MY", 437: "MXR90", 887: "MYR90"}
    return mapping.get(d, f"D{d}")


def orient_to_angle(s: str) -> int:
    """orient 字符串 → 角度 0/90/180/270"""
    if "90" in s:
        return 90
    elif "180" in s:
        return 180
    elif "270" in s:
        return 270
    return 0


# ============================================================
# 解析器：auCdl 源网表
# ============================================================

def parse_src_netlist(filepath: str):
    """解析 auCdl 原理图网表，返回 modules, nets, top_cell。"""
    modules: Dict[str, Module] = {}
    nets: Dict[str, Net] = {}
    top_cell = ""

    with open(filepath, encoding="utf-8") as f:
        text = f.read()

    top_match = re.search(r'\.SUBCKT\s+(\S+)\s+(.*?)\n', text)
    if not top_match:
        raise ValueError("No .SUBCKT found in src netlist")
    top_cell = top_match.group(1)
    top_ports = set(top_match.group(2).split())

    for line in text.split('\n'):
        line = line.strip()
        if not line or line.startswith('*') or line.startswith('.'):
            if not (line.startswith('MM') or line.startswith('XR') or line.startswith('XC')):
                continue

        # MOSFET: MM<num> D G S B model L=... W=... m=...
        mm = re.match(
            r'(MM\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+.*?[lL]=(\S+)\s+[wW]=(\S+)',
            line
        )
        if mm:
            dev_id = mm.group(1)
            d, g, s, b = mm.group(2), mm.group(3), mm.group(4), mm.group(5)
            model = mm.group(6)
            L = parse_value(mm.group(7))
            W = parse_value(mm.group(8))
            m = int(re.search(r'\bm=(\d+)', line).group(1)) if 'm=' in line else 1
            dev_type = 'nmos' if model.startswith('n') else 'pmos'

            signature = f"D:{d} G:{g} S:{s} B:{b}"
            mod = Module(id=dev_id, dev_type=dev_type, model=model,
                         parameters={'L': L, 'W': W}, m=m,
                         pins={}, conn_signature=signature)
            modules[dev_id] = mod

            for pin_name, net_name in [('D', d), ('G', g), ('S', s), ('B', b)]:
                if net_name not in nets:
                    nets[net_name] = Net(name=net_name)
                nets[net_name].terminals.append(f"{dev_id}.{pin_name}")
            continue

        # 电阻: XR<num> PLUS MINUS model l=... w=...
        xr = re.match(
            r'(XR\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+.*?[lL]=(\S+)\s+[wW]=(\S+)',
            line
        )
        if xr:
            dev_id, plus, minus, model = xr.group(1), xr.group(2), xr.group(3), xr.group(4)
            L = parse_value(xr.group(5))
            W = parse_value(xr.group(6))
            m = int(re.search(r'\bm=(\d+)', line).group(1)) if 'm=' in line else 1

            signature = f"PLUS:{plus} MINUS:{minus}"
            mod = Module(id=dev_id, dev_type='resistor', model=model,
                         parameters={'L': L, 'W': W}, m=m,
                         pins={}, conn_signature=signature)
            modules[dev_id] = mod

            for pin_name, net_name in [('PLUS', plus), ('MINUS', minus)]:
                if net_name not in nets:
                    nets[net_name] = Net(name=net_name)
                nets[net_name].terminals.append(f"{dev_id}.{pin_name}")
            continue

        # 电容: XC<num> PLUS MINUS model nr=... lr=... w=...
        xc = re.match(
            r'(XC\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+.*?nr=(\S+)\s+lr=(\S+)\s+[wW]=(\S+)',
            line
        )
        if xc:
            dev_id, plus, minus, model = xc.group(1), xc.group(2), xc.group(3), xc.group(4)
            nr = float(xc.group(5))
            lr = parse_value(xc.group(6))
            W = parse_value(xc.group(7))
            m = int(re.search(r'\bm=(\d+)', line).group(1)) if 'm=' in line else 1

            signature = f"PLUS:{plus} MINUS:{minus}"
            mod = Module(id=dev_id, dev_type='capacitor', model=model,
                         parameters={'nr': nr, 'lr': lr, 'W': W}, m=m,
                         pins={}, conn_signature=signature)
            modules[dev_id] = mod

            for pin_name, net_name in [('PLUS', plus), ('MINUS', minus)]:
                if net_name not in nets:
                    nets[net_name] = Net(name=net_name)
                nets[net_name].terminals.append(f"{dev_id}.{pin_name}")
            continue

    # 推断优先级
    for name, net in nets.items():
        upper = name.upper()
        if upper in ('VDD', 'AVDD', 'VCC', 'GND', 'AGND', 'VSS', 'OUT', 'VOUT'):
            net.priority = 'critical'
        else:
            net.priority = 'normal'

    return modules, nets, top_cell


# ============================================================
# 解析器：LVS SPICE 网表
# ============================================================

def parse_lvs_spice(filepath: str, circuit_name: str = "peak_detection") -> Tuple[List[LvsInstance], Dict[str, Dict]]:
    """解析 Calibre LVS 提取的 SPICE，返回 (实例列表, 子电路定义参数)。

    子电路定义格式: {name: {W: float, L: float, ...}}
    """
    instances: List[LvsInstance] = []
    subckt_params: Dict[str, Dict] = {}

    with open(filepath, encoding="utf-8") as f:
        text = f.read()

    # 先解析所有子电路定义，提取参数
    for sub_match in re.finditer(
        r'\.SUBCKT\s+(\S+)\s+(.*?)\n(.*?)\.ENDS',
        text, re.DOTALL
    ):
        sub_name = sub_match.group(1)
        sub_body = sub_match.group(3)
        params = {}
        # 提取 W, L, nr, lr 等
        for key in ['W', 'L', 'nr', 'lr']:
            m = re.search(rf'\b[{key.lower()}{key.upper()}]=(\S+)', sub_body)
            if m:
                params[key] = parse_value(m.group(1))
        if params:
            subckt_params[sub_name] = params

    # 找到指定电路名的 SUBCKT
    top_match = re.search(
        rf'\.SUBCKT\s+{re.escape(circuit_name)}\s+(.*?)\n', text
    )
    if not top_match:
        print(f"      ⚠ 未在 SPICE 中找到 {circuit_name} 的 SUBCKT")
        return instances, subckt_params

    body = text[top_match.start():]

    # 用深度计数提取顶层实例（depth==1）
    depth = 0
    for line in body.split('\n'):
        line_stripped = line.strip()
        if not line_stripped:
            continue
        if line_stripped.startswith('.SUBCKT'):
            depth += 1
            continue
        if line_stripped.startswith('.ENDS'):
            depth -= 1
            if depth <= 0:
                break
            continue
        if depth != 1:
            continue
        if line_stripped.startswith('*') or line_stripped.startswith('.'):
            continue

        # 提取坐标
        coord_match = re.search(r'\$X=(\S+)\s+\$Y=(\S+)\s+\$D=(\S+)', line)
        t_match = re.search(r'\$T=(\S+)\s+(\S+)\s+(\S+)\s+(\S+)', line)
        if not coord_match and not t_match:
            continue

        if t_match:
            x = float(t_match.group(1)) / 1000   # LVS nm → μm
            y = float(t_match.group(2)) / 1000
            orient = f"T_{t_match.group(3)}_{t_match.group(4)}"
            d = 0
        else:
            x = float(coord_match.group(1)) / 1000  # LVS nm → μm
            y = float(coord_match.group(2)) / 1000
            d = int(coord_match.group(3))
            orient = orient_code_to_str(d)

        # MOSFET
        mos_match = re.match(
            r'(M\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+.*?[lL]=(\S+)\s+[wW]=(\S+)',
            line
        )
        if mos_match:
            inst = LvsInstance(
                ref=mos_match.group(1),
                model=mos_match.group(6),
                dev_type='nmos' if mos_match.group(6).startswith('n') else 'pmos',
                nets=[mos_match.group(2), mos_match.group(3),
                      mos_match.group(4), mos_match.group(5)],
                L=parse_value(mos_match.group(7)),
                W=parse_value(mos_match.group(8)),
                x=x, y=y, d=d, orient=orient
            )
            instances.append(inst)
            continue

        # 子电路实例: X<num> net1 net2 ... model params... $X=... $Y=... $D=...
        x_match = re.match(r'(X\d+)\s+(.*)', line_stripped)
        if x_match:
            ref = x_match.group(1)
            rest = x_match.group(2)

            # 先去掉 $X $Y $D $T 等坐标属性
            clean = re.sub(r'\$[A-Z]+=\S+', '', rest).strip()

            # 提取子电路名：在参数之前的最后一个非数字 token
            # 子电路名通常如 "rupolym", "cfmom_2t_CDNS_...", "nch_ulvt_mac_CDNS_..."
            parts = clean.split()
            # 找到参数声明之前的位置（l=... w=... nr=... 等）
            subckt_idx = -1
            for i, p in enumerate(parts):
                if re.match(r'^[a-zA-Z_][\w_]*$', p) and not re.match(r'^\d+$', p):
                    # 可能是子电路名 — 取最后一个非参数、非网表数字的 token
                    pass
            # 简单策略：找所有看起来像 model/subckt 名的 token（包含字母和下划线）
            model_candidates = [p for p in parts if re.match(r'^[a-zA-Z_][\w_]*$', p)]
            if model_candidates:
                subckt = model_candidates[-1]  # 最后一个符合条件的
                # 子电路名之前的 token 就是 nets
                subckt_pos = parts.index(subckt)
                nets = parts[:subckt_pos]
            else:
                # fallback: 最后一部分是子电路名
                subckt = parts[-1] if parts else 'unknown'
                nets = parts[:-1] if len(parts) > 1 else []

            # 确定器件类型
            dev_type = 'unknown'
            for kw, dt in [('rupolym', 'resistor'), ('cfmom', 'capacitor'),
                           ('nch_ulvt', 'nmos'), ('pch_ulvt', 'pmos')]:
                if kw in subckt:
                    dev_type = dt
                    break

            # 提取器件参数（优先从实例行，否则从子电路定义）
            inst_L = 0.0
            inst_W = 0.0
            l_match = re.search(r'\b[lL]=(\S+)', clean)
            w_match = re.search(r'\b[wW]=(\S+)', clean)
            if l_match:
                inst_L = parse_value(l_match.group(1))
            if w_match:
                inst_W = parse_value(w_match.group(1))
            # 如果实例行没有，查子电路定义
            if (inst_L == 0.0 or inst_W == 0.0) and subckt in subckt_params:
                sp = subckt_params[subckt]
                if inst_L == 0.0 and 'L' in sp:
                    inst_L = sp['L']
                if inst_W == 0.0 and 'W' in sp:
                    inst_W = sp['W']

            inst = LvsInstance(
                ref=ref, model=subckt, dev_type=dev_type,
                nets=nets, x=x, y=y, d=d, orient=orient,
                L=inst_L, W=inst_W
            )
            instances.append(inst)

    return instances, subckt_params


# ============================================================
# 版图实例 → 原理图器件的匹配
# ============================================================

def correlate_instances(
    src_modules: Dict[str, Module],
    lvs_instances: List[LvsInstance],
    nets: Dict[str, Net]
) -> Dict[str, List[LvsInstance]]:
    """
    将 LVS 实例匹配到原理图器件。

    策略：LVS 提取使用内部 net 编号（与原理图 net 名不同），无法直接按 net 名匹配。
    改用：按器件类型 + W/L 参数匹配。

    返回: {src_dev_id: [LvsInstance, ...]}
    """
    matched = defaultdict(list)

    # 按器件类型分组 LVS 实例
    lvs_by_type: Dict[str, List[LvsInstance]] = defaultdict(list)
    for inst in lvs_instances:
        lvs_by_type[inst.dev_type].append(inst)

    # 对每种类型，按参数分组
    def param_key(inst: LvsInstance) -> Tuple:
        return (round(inst.W, 8), round(inst.L, 8))

    # 对每个原理图器件，找参数最接近的 LVS 实例组
    used_lvs = set()  # 已匹配的 LVS 实例 ref

    for dev_id, mod in src_modules.items():
        candidates = lvs_by_type.get(mod.dev_type, [])

        # 过滤：相似参数
        src_W = mod.parameters.get('W', 0)
        src_L = mod.parameters.get('L', 0)
        tol = 1e-9  # 1nm 容差

        matching = []
        for inst in candidates:
            if inst.ref in used_lvs:
                continue
            w_match = abs(inst.W - src_W) < tol
            l_match = abs(inst.L - src_L) < tol or (src_L == 0 and inst.L == 0)
            if w_match and l_match:
                matching.append(inst)

        if matching:
            # 判断 LVS 实例是 flat (M前缀) 还是 subcircuit (X前缀)
            #   - subcircuit: 已将 m>1 打包，每个源器件对应 1 个 LVS 实例
            #   - flat: 每个 LVS 实例对应 m=1 的指，源器件的 m 个并联指对应 m 个实例
            is_subckt = all(inst.ref.startswith('X') for inst in matching)
            if is_subckt:
                take = 1  # 源器件的 m>1 已打包在子电路中
            else:
                take = mod.m  # 每个平行指一个 flat 实例

            take = min(len(matching), max(take, 1))
            matched[dev_id] = matching[:take]
            for inst in matching[:take]:
                used_lvs.add(inst.ref)

    # 报告
    total_lvs = len(lvs_instances)
    matched_lvs = len(used_lvs)
    matched_devs = len(matched)
    if total_lvs > 0:
        print(f"      → {matched_devs}/{len(src_modules)} 个器件匹配到版图实例 "
              f"({matched_lvs}/{total_lvs} 个 LVS 实例已关联)")
    if matched_lvs < total_lvs:
        remaining_lvs = [i for i in lvs_instances if i.ref not in used_lvs]
        print(f"      ⚠ {total_lvs - matched_lvs} 个 LVS 实例未匹配（多为电阻段/电容段被拆分）")
        for dt in set(i.dev_type for i in remaining_lvs):
            cnt = sum(1 for i in remaining_lvs if i.dev_type == dt)
            print(f"        {dt}: {cnt} 个")

    return dict(matched), used_lvs


# ============================================================
# BB 尺寸和 pin 位置估算
# ============================================================

def estimate_bb_from_layout(module: Module, lvs_group: List[LvsInstance]):
    """
    从 LVS 坐标推算 BB 尺寸。

    如果只有一个 LVS 实例，使用 PDK 典型尺寸估算。
    如果有多个并联实例，用 bbox + 间距推算。
    """
    if not lvs_group:
        _estimate_bb_from_params(module)
        return

    if len(lvs_group) == 1:
        inst = lvs_group[0]
        # 对于 mos，从 W/L 估算
        if module.dev_type in ('nmos', 'pmos'):
            _estimate_bb_from_params(module)
            module.x = inst.x
            module.y = inst.y
            module.orient = inst.orient
        else:
            _estimate_bb_from_params(module)
            module.x = inst.x
            module.y = inst.y
            module.orient = inst.orient
        return

    # 多实例：计算 bbox
    xs = [i.x for i in lvs_group]
    ys = [i.y for i in lvs_group]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # 为每个实例加上典型宽度
    typ_w, typ_h = _typical_bb(module)
    max_x += typ_w
    max_y += typ_h

    module.w = round(max_x - min_x, 2)
    module.h = round(max_y - min_y, 2)
    module.x = round(min_x, 2)
    module.y = round(min_y, 2)

    # Active region
    margin = min(module.w, module.h) * 0.15
    module.active = (round(margin, 2), round(margin, 2),
                     round(module.w - margin, 2), round(module.h - margin, 2))


def _typical_bb(mod: Module) -> Tuple[float, float]:
    """根据器件参数返回典型宽度和高度 (μm)。"""
    W = mod.parameters.get('W', 2e-6) * 1e6
    L = mod.parameters.get('L', 1e-6) * 1e6

    if mod.dev_type in ('nmos', 'pmos'):
        # TSMC 28nm 典型: 多指晶体管 BB
        return max(W * 1.8 + 1.0, 2.0), max(L * 3.0 + 1.5, 1.8)
    elif mod.dev_type == 'resistor':
        L_total = mod.parameters.get('L', 10e-6) * 1e6
        return L_total + 1.5, max(W * 5, 1.2)
    elif mod.dev_type == 'capacitor':
        nr = mod.parameters.get('nr', 100)
        lr = mod.parameters.get('lr', 10e-6) * 1e6
        return nr * 0.16 + 3, lr * 1.1 + 3
    return 3.0, 2.0


def _estimate_bb_from_params(mod: Module):
    """纯从参数估算 BB 尺寸（fallback）。"""
    tw, th = _typical_bb(mod)
    mod.w = round(tw, 2)
    mod.h = round(th, 2)
    margin = min(mod.w, mod.h) * 0.15
    if margin < 0.2:
        margin = 0.2
    mod.active = (round(margin, 2), round(margin, 2),
                  round(mod.w - margin, 2), round(mod.h - margin, 2))


def estimate_pin_positions(mod: Module):
    """估算 pin 在 BB 上的相对位置。"""
    bw, bh = mod.w, mod.h
    if mod.dev_type in ('nmos', 'pmos'):
        mod.pins = {
            'D': (round(bw * 0.15, 2), round(bh * 0.5, 2), 1),
            'G': (round(bw * 0.5, 2), round(bh * 0.78, 2), 1),
            'S': (round(bw * 0.85, 2), round(bh * 0.5, 2), 1),
            'B': (round(bw * 0.5, 2), round(bh * 0.22, 2), 1),
        }
    elif mod.dev_type == 'resistor':
        mod.pins = {
            'PLUS':  (round(bw * 0.05, 2), round(bh * 0.5, 2), 1),
            'MINUS': (round(bw * 0.95, 2), round(bh * 0.5, 2), 1),
        }
    elif mod.dev_type == 'capacitor':
        mod.pins = {
            'PLUS':  (round(bw * 0.3, 2), round(bh * 0.8, 2), 2),
            'MINUS': (round(bw * 0.7, 2), round(bh * 0.2, 2), 2),
        }


def layer_int_to_str(l: int) -> str:
    return f"M{l}" if 1 <= l <= 7 else "M2"


# ============================================================
# 输出写入
# ============================================================

def write_modules(modules: Dict[str, Module], path: str):
    with open(path, 'w', encoding="utf-8") as f:
        f.write("# IO/input/modules.txt\n")
        f.write("# 自动生成 — parse_oa.py (从 Cadence OA 导出数据提取)\n")
        f.write("# 器件物理属性，位置和角度在 output/placement.txt\n")
        f.write("#\n")
        f.write("# id          w       h       active(x1,y1,x2,y2)         # info\n")
        f.write("\n")
        for dev_id in sorted(modules.keys()):
            m = modules[dev_id]
            a = m.active
            info = (f"{m.dev_type} {m.model} "
                    f"L={m.parameters.get('L', 0):.2e} W={m.parameters.get('W', 0):.2e} "
                    f"m={m.m}")
            f.write(f"{m.id:<12s} {m.w:<7.2f} {m.h:<7.2f} "
                    f"{a[0]:.2f},{a[1]:.2f},{a[2]:.2f},{a[3]:.2f}  "
                    f"# {info}\n")


def write_pins(modules: Dict[str, Module], path: str):
    with open(path, 'w', encoding="utf-8") as f:
        f.write("# IO/input/pins.txt\n")
        f.write("# 自动生成 — parse_oa.py\n")
        f.write("# pin 坐标相对器件左下角，层为推断值\n")
        f.write("#\n")
        f.write("# module      pin      x        y        layer\n")
        f.write("\n")
        for dev_id in sorted(modules.keys()):
            m = modules[dev_id]
            for pin_name, (px, py, layer_int) in m.pins.items():
                layer = layer_int_to_str(layer_int)
                f.write(f"{m.id:<12s} {pin_name:<8s} {px:<8.2f} {py:<8.2f} {layer}\n")


def write_nets(nets: Dict[str, Net], path: str):
    with open(path, 'w', encoding="utf-8") as f:
        f.write("# IO/input/nets.txt\n")
        f.write("# 自动生成 — parse_oa.py\n")
        f.write("# 连接关系（网表），优先级别为合理推断\n")
        f.write("#\n")
        f.write("# net          priority    terminals\n")
        f.write("\n")
        for net_name in sorted(nets.keys()):
            net = nets[net_name]
            terms = " ".join(net.terminals)
            f.write(f"{net.name:<14s} {net.priority:<10s} {terms}\n")


def write_constraints(modules: Dict[str, Module], nets: Dict[str, Net], path: str):
    with open(path, 'w', encoding="utf-8") as f:
        f.write("# IO/input/constraints.txt\n")
        f.write("# 自动生成 — parse_oa.py\n")
        f.write("# ⚠ 约束为模板推断，请根据实际设计补充修正！\n")
        f.write("#\n")
        f.write("# 格式:\n")
        f.write("#   SYMMETRY_PAIR    <name>   vertical   <left>   <right>\n")
        f.write("#   SYMMETRY_SELF    <name>   vertical   <module>\n")
        f.write("#   FLOW     <net>    <out_pin>    <in_pin>\n")
        f.write("#   WIRE_WIDTH       <net>    <min_w>    <max_w>\n")
        f.write("\n")

        # 启发式发现对称对
        dev_ids = sorted(modules.keys())
        paired = set()
        for i, id1 in enumerate(dev_ids):
            if id1 in paired:
                continue
            m1 = modules[id1]
            for id2 in dev_ids[i + 1:]:
                if id2 in paired:
                    continue
                m2 = modules[id2]
                if (m1.dev_type == m2.dev_type and m1.model == m2.model
                        and m1.parameters.get('L') == m2.parameters.get('L')
                        and m1.parameters.get('W') == m2.parameters.get('W')
                        and m1.m == m2.m):
                    paired.add(id1)
                    paired.add(id2)
                    f.write(f"# 器件 {id1} 和 {id2} 参数相同，可能是对称对\n")
                    f.write(f"# SYMMETRY_PAIR    sg_auto   vertical  {id1}      {id2}\n")
                    break

        f.write("\n")
        for net_name in sorted(nets.keys()):
            net = nets[net_name]
            if net.priority == 'critical':
                f.write(f"# 关键线网 {net_name} 需指定线宽范围\n")
                f.write(f"# WIRE_WIDTH      {net_name}    ?    ?\n")

        f.write("\n# === 格式参考 ===\n")
        f.write("# SYMMETRY_PAIR    sg1   vertical  MM17      MM22\n")
        f.write("# SYMMETRY_SELF    sg3   vertical  MM29\n")
        f.write("# FLOW     OUT    MM26.D    XR25.PLUS\n")
        f.write("# WIRE_WIDTH      AVDD    2    8\n")


def write_placement(modules: Dict[str, Module], path: str):
    with open(path, 'w', encoding="utf-8") as f:
        f.write("# IO/output/placement.txt\n")
        f.write("# 自动生成 — parse_oa.py\n")
        f.write("# 从 Calibre LVS 提取的版图坐标（全局坐标，μm）\n")
        f.write("#\n")
        f.write("# module       x           y           angle  orient\n")
        f.write("\n")
        has_coords = any(m.x != 0.0 for m in modules.values())
        if has_coords:
            for dev_id in sorted(modules.keys()):
                m = modules[dev_id]
                angle = orient_to_angle(m.orient)
                f.write(f"{m.id:<14s} {m.x:<10.2f} {m.y:<10.2f} "
                        f"{angle:<6d} {m.orient}\n")
        else:
            f.write("# (无版图坐标数据 — 布局器将自动放置)\n")


def write_routing_template(path: str):
    with open(path, 'w', encoding="utf-8") as f:
        f.write("# IO/output/routing.txt\n")
        f.write("# 由布线器自动生成\n")
        f.write("#\n")
        f.write("# net    layer  x1    y1    x2    y2    width\n")
        f.write("\n")


# ============================================================
# 主转换
# ============================================================

def convert(src_net_path: str, sp_path: str, output_dir: str):
    print(f"[1/5] 解析原理图网表: {src_net_path}")
    modules, nets, top_cell = parse_src_netlist(src_net_path)
    print(f"      → {len(modules)} 个器件, {len(nets)} 条线网, 顶层: {top_cell}")

    lvs_instances = []
    if sp_path and os.path.exists(sp_path):
        print(f"[2/5] 解析 LVS 版图坐标: {sp_path}")
        lvs_instances, subckt_params = parse_lvs_spice(sp_path, top_cell)
        print(f"      → {len(lvs_instances)} 个版图实例")

    # 匹配
    correlated = {}
    used_lvs = set()
    if lvs_instances:
        print(f"[3/5] 匹配原理图器件 ↔ 版图实例...")
        correlated, used_lvs = correlate_instances(modules, lvs_instances, nets)

    # 对于未匹配到坐标的器件，从同类型剩余 LVS 实例中取 bbox
    # 如果是电阻/电容等被拆分的器件，多个源器件可能共享同一 bulk 结构
    unmatched_lvs = [i for i in lvs_instances if i.ref not in used_lvs]
    unmatched_by_type: Dict[str, List[LvsInstance]] = defaultdict(list)
    for inst in unmatched_lvs:
        unmatched_by_type[inst.dev_type].append(inst)

    print(f"[4/5] 估算 BB 尺寸和 pin 位置...")
    for dev_id, mod in modules.items():
        lvs_group = correlated.get(dev_id, [])
        if not lvs_group:
            fallback = unmatched_by_type.get(mod.dev_type, [])
            if fallback:
                lvs_group = fallback
        estimate_bb_from_layout(mod, lvs_group)
        estimate_pin_positions(mod)

    # 写入
    input_dir = os.path.join(output_dir, 'input')
    output_dir_full = os.path.join(output_dir, 'output')
    os.makedirs(input_dir, exist_ok=True)
    os.makedirs(output_dir_full, exist_ok=True)

    print(f"[5/5] 写入 IO 文件 → {output_dir}/")
    write_modules(modules, os.path.join(input_dir, 'modules.txt'))
    write_pins(modules, os.path.join(input_dir, 'pins.txt'))
    write_nets(nets, os.path.join(input_dir, 'nets.txt'))
    write_constraints(modules, nets, os.path.join(input_dir, 'constraints.txt'))
    write_placement(modules, os.path.join(output_dir_full, 'placement.txt'))
    write_routing_template(os.path.join(output_dir_full, 'routing.txt'))

    for fname in ['modules.txt', 'pins.txt', 'nets.txt', 'constraints.txt']:
        print(f"      ✓ input/{fname}")
    for fname in ['placement.txt', 'routing.txt']:
        print(f"      ✓ output/{fname}")

    print(f"\n完成! 输出目录: {output_dir}/")
    print(f"  下一步: 编辑 {output_dir}/input/constraints.txt 添加实际设计约束")


# ============================================================
# 主入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="parse_oa.py — 从 Cadence OA 导出数据生成 IO 标准格式"
    )
    parser.add_argument("circuit", help="电路名（如 peak_detection）")
    parser.add_argument("--src-net", help="auCdl 源网表路径")
    parser.add_argument("--sp", help="LVS SPICE 网表路径")
    parser.add_argument("--oa-dir", default=".",
                        help="OA 库根目录")
    parser.add_argument("--io-dir", default=".",
                        help="IO 输出根目录")

    args = parser.parse_args()

    src_net = args.src_net
    if not src_net:
        default_src = os.path.join(args.oa_dir, "0A_SRC.NET", "SRC.NET",
                                   f"{args.circuit}.src.net")
        if os.path.exists(default_src):
            src_net = default_src
        else:
            print(f"错误: 未找到源网表 {default_src}，请用 --src-net 指定")
            sys.exit(1)

    sp = args.sp
    if not sp:
        default_sp = os.path.join(args.oa_dir, "0A_SRC.NET", f"{args.circuit}.sp")
        if os.path.exists(default_sp):
            sp = default_sp

    output_dir = os.path.join(args.io_dir, args.circuit)
    convert(src_net, sp, output_dir)


if __name__ == "__main__":
    main()

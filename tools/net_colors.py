# 文件职责：为 layout / btree 渲染提供稳定的一 net 一色映射。
"""layout 与 btree 共用的 net 颜色表，保证两图对照时颜色一致。"""

from __future__ import annotations

# 固定色板；超出长度时按哈希回退，避免同名 net 在不同图上变色。
NET_COLOR_PALETTE = [
    "#D81B60",  # 品红
    "#00897B",  # 青绿
    "#5E35B1",  # 紫
    "#F9A825",  # 琥珀
    "#039BE5",  # 蓝
    "#6D4C41",  # 棕
    "#C0CA33",  # 黄绿
    "#EF6C00",  # 橙
]


# 按 net 名称分配稳定颜色；同名始终同色。
def color_for_net(net: str, known_nets: list[str] | None = None) -> str:
    if known_nets:
        ordered = list(dict.fromkeys(known_nets))
        if net in ordered:
            return NET_COLOR_PALETTE[ordered.index(net) % len(NET_COLOR_PALETTE)]
    digest = sum((index + 1) * ord(ch) for index, ch in enumerate(net))
    return NET_COLOR_PALETTE[digest % len(NET_COLOR_PALETTE)]


# 为一批 net 建立 name -> color 映射；按名称排序，保证 layout/btree 同色。
def net_color_map(nets: list[str]) -> dict[str, str]:
    ordered = sorted(dict.fromkeys(net for net in nets if net))
    return {net: color_for_net(net, ordered) for net in ordered}

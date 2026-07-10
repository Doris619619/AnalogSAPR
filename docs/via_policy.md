# Via 代价与统计口径

- A* 阶段把 via 作为局部换层动作计入搜索代价，并累加 `via_count`。
- Global/DP 阶段为保持论文目标函数口径，仅统计 `via_count`，不把 via 放入候选选择代价。
- Detailed 阶段不再通过整条金属换层进行免费合法化；无法用原候选或同逻辑线段替代候选避开短路时，应报告失败和惩罚。
- Detailed 合法化会在原候选、同逻辑线段替代候选之间按最终几何代价比较；若没有候选可合法输出，再用已布异网 detailed 金属作为障碍执行一次真实 A* reroute。
- 最终 `phi` 使用的 wirelength、bend 和 via 优先来自 detailed 输出线段的真实几何统计；仅当 detailed routes 为空时回退到 global 指标。

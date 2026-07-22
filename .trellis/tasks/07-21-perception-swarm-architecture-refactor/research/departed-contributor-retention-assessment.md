# Research: 离线成员的地图贡献生命周期

- Query: Phase 3 如何处理 stale source，下一阶段动态成员需要补什么语义？
- Scope: internal
- Date: 2026-07-21

## Findings

- `source_stale_timeout` 当前只改变 merger 诊断，不删除已经融合的 source contribution。
- `OctoMapMerger` 算法库已有 `removeSource(source_id)`，可显式撤销来源并重算受影响 voxel。
- 当前 allocator 要求所有静态 expected source 都 fresh；动态 membership 提交后，required-live source 集合必须随 membership/view epoch 改变，否则正常退出会永久阻塞协调。
- 新架构已有 contributor manifest、source session、alignment epoch 和显式 contributor 删除语义，可以把“成员是否在线”与“历史地图是否保留”分开表达。
- 当前洞穴 occupancy 模型是静态环境；动态障碍与动态 occupancy layer 不在首版范围内。本机路径安全始终使用当前无人机自己的 known-free map，而不是仅凭全局地图授权移动。

## Implication

父级已选择：departed source 默认转为 Frozen，保留静态 occupancy provenance 但退出 live health；Frozen-only frontier 不分配任务；alignment/资源/显式删除可撤销；新 session 必须 keyframe 替换。不能继续依赖 stale timeout 的隐式副作用。

## Evidence

- `docs/phases/phase-03-swarm.md:1901`
- `docs/phases/phase-03-swarm.md:2034`
- `ws/src/swarm_controller/include/swarm_controller/OctoMapMerger.hpp:74`
- `ws/src/swarm_controller/src/GlobalMapMergerNode.cpp:868`

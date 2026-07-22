# Research: 多雷达输入健康与地图降级

- Query: 单个传感器失效时，唯一 active mapper 和 authoritative local map 应如何降级？
- Scope: internal
- Date: 2026-07-21

## Findings

- 感知接口草案已经要求算法按 capability 接受、降级或拒绝输入，并要求回放记录每个传感器的能力状态。
- D-019 冻结 vehicle session 内的 sensor descriptor inventory，但允许已登记 sensor 的运行期健康状态变化。
- D-020 确定每个 vehicle session 只有一个 active mapper；因此传感器失效不能通过并行 mapper 或运行中算法切换解决。
- 当前文档没有定义 mapper 的最小可运行输入条件，也没有定义单个 sensor 掉线后 local map health、shared contribution 和 task gate 的联动。
- 将所有 sensor 一律视为必需会使可选补盲雷达的失效无条件停机；允许 mapper 任意继续则可能在关键 free-ray/视场能力丢失后仍把地图标为健康。

## Implication

推荐 active mapper 在启动时声明并校验 minimum viable input contract：所需 sensor ID/geometry/capability 组合以及允许的 degraded 组合。运行期只改变输入健康，不切换算法实现。健康输入仍满足 minimum contract 时 authoritative map 可继续推进，但必须发布 Degraded 状态和当前有效能力；不再满足时停止提交新的 authoritative revision，地图 freshness 到期，shared contribution 按既有 Frozen/health 规则处理，任务进入 Hold/LocalFallback 或撤销。

## Evidence

- `docs/decisions/perception-observation-interface.md:95`
- `docs/decisions/perception-observation-interface.md:261`
- `docs/decisions/perception-observation-interface.md:281`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:380`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:155`

# 子任务优化建议回溯记录

> 父任务：07-21-perception-swarm-architecture-refactor  
> 创建日期：2026-07-22  
> 来源：`ARCHITECTURE-REVIEW.md` §7.3 "建议修订项"

---

## 说明

本文档记录父任务评审中针对子任务（C2-C9）的优化建议。这些建议在父任务评审时提出，但未记录到具体子任务的规划文档中。

**使用方式**：
- 创建子任务时，从本文档提取相关建议
- 在子任务的 `prd.md` 或 `implement.md` 中引用建议编号
- 标记为已采纳后，在本文档中更新状态

---

## 未追踪的优化建议与后续 TODO（11 项）

### C2-OPT-1：pose reset 与 alignment invalidation 的优先级顺序

**目标子任务**：C2（地图对齐）

**建议**：
在 C2 子任务中明确 pose reset 与 alignment invalidation 的优先级顺序：
- 如果位姿 reset（frame_id 变化、时间回退、位置跳变），应该如何处理当前的 alignment？
- 是立即废弃当前 alignment？还是等待新的 alignment 重新计算？

**来源**：`ARCHITECTURE-REVIEW.md` §4.1

**状态**：📌 已引用到子任务 `07-27-c2-local-observation-map`

---

### C2-OPT-2：枚举必需能力和可选能力

**目标子任务**：C2（地图对齐）

**建议**：
在 C2 子任务中枚举 mapper 的必需能力（required capabilities）和可选能力（optional capabilities）：
- 必需能力：位姿输入、传感器输入、本地地图生成
- 可选能力：全局对齐、多机融合、地图保存/加载

**来源**：`ARCHITECTURE-REVIEW.md` §6.5

**状态**：📌 已引用到子任务 `07-27-c2-local-observation-map`

---

### C2-OPT-3：明确 capability 的表达方式

**目标子任务**：C2（地图对齐）、C6（多机协作）

**建议**：
在 C2 和 C6 的接口设计中明确 capability 的表达方式：
- 枚举（enum）？
- 位掩码（bitmask）？
- 结构化对象（struct with bool fields）？

**来源**：`ARCHITECTURE-REVIEW.md` §6.9

**状态**：📌 C2 部分已引用到 `07-27-c2-local-observation-map`；C6 仍待创建时引用

---

### C3-TODO-1：地图共享与静态 alignment 端到端验证

**目标子任务**：C3（地图状态与增量更新）定义协议证据；C8（总集成）完成多进程
与 RViz 验收。

**TODO**：
- 使用至少两个 source-local map 和版本化静态 alignment 聚合 shared/global view；
- 验证 pose reset/alignment revoke 后旧贡献失效，随后通过 keyframe/resync 重建；
- 验证跨 source occupied/free/unknown 对齐结果及 RViz 可视效果；
- 该项不阻塞 C2 本机地图验收，C2 只交付 alignment epoch/revision 绑定与失效边界。

**来源**：C2 规划期间用户明确决定“地图共享留待后续验证”。

**状态**：⏸️ 待 C3/C8 创建时引用

---

### C4-OPT-1：补充性能基线和目标

**目标子任务**：C4（通信数据面）、C9a（性能验证）

**建议**：
在 C4 和 C9a 子任务中补充性能基线和目标：
- 地图更新延迟目标（当前只有 merger 基线 p50: 0.561s）
- 任务分配延迟目标
- 内存/CPU 预算
- 最大支持无人机数量

**来源**：`ARCHITECTURE-REVIEW.md` §5.3

**状态**：⏸️ 待 C4/C9a 创建时引用

---

### C4-OPT-2：明确安全机制（身份验证、加密、授权）

**目标子任务**：C4（通信数据面）

**建议**：
在 C4 或独立安全子任务中明确：
- 身份验证：agent_id 如何分配？是否需要签名？
- 消息加密：是否加密地图数据？加密级别？
- 授权机制：哪些 agent 可以发送全局地图？哪些只能接收？

**来源**：`ARCHITECTURE-REVIEW.md` §6.3

**状态**：⏸️ 待 C4 创建时引用

---

### C5-OPT-1：拆分为 4 个子任务（C5a/b/c/d）

**目标子任务**：C5（拓扑与角色）

**建议**：
C5 是最复杂的子任务，建议拆分为 4 个独立子任务：
- **C5a**：拓扑发现与维护（Raft election、liveness）
- **C5b**：角色切换机制（Follower → Coordinator 转换）
- **C5c**：多 Coordinator 一致性（冲突解决、仲裁）
- **C5d**：拓扑容错测试（网络分区、节点崩溃）

**来源**：`ARCHITECTURE-REVIEW.md` §4.4

**状态**：⏸️ 待 C5 创建时考虑拆分

---

### C5-OPT-2：补充资源预算数值

**目标子任务**：C5（拓扑与角色）

**建议**：
在 C5 拓扑子任务中补充资源预算的具体数值或计算方法：
- 内存预算：每个 agent 的拓扑状态占用内存
- CPU 预算：拓扑维护（心跳、election）的 CPU 占用
- 网络带宽预算：拓扑消息的带宽占用

**来源**：`ARCHITECTURE-REVIEW.md` §5.6

**状态**：⏸️ 待 C5 创建时引用

---

### C5-OPT-3：提供 2 个 action adapter 示例

**目标子任务**：C5（拓扑与角色）

**建议**：
在 C5 子任务中提供至少 2 个 action 类型的 adapter 示例：
- **探索任务**：coordinator 分配探索区域，follower 执行探索并回传地图
- **角色迁移**：coordinator 检测到自身资源不足，将角色迁移给更强的 follower

**来源**：`ARCHITECTURE-REVIEW.md` §6.7

**状态**：⏸️ 待 C5 创建时引用

---

### C6-OPT-1：coordinator action 只传递意图，不控制 actuator

**目标子任务**：C6（多机协作）

**建议**：
在 C6 多机协作子任务中明确：
- Coordinator 的 action 只传递任务意图（探索区域、目标点）
- Local executor 保留 actuator authority（路径规划、避障、速度控制）
- Coordinator 不直接控制 follower 的电机、舵机等执行器

**来源**：`ARCHITECTURE-REVIEW.md` §4.2

**状态**：⏸️ 待 C6 创建时引用

---

### C9-OPT-1：补充量化验收指标

**目标子任务**：C9a（性能验证）、C9b（集成验证）

**建议**：
在 C9a/C9b 子任务创建时补充量化验收指标：
- **C9a**：地图更新延迟 p50/p99、CPU 占用百分位、内存占用峰值、支持无人机数量上限
- **C9b**：端到端任务完成率、网络分区恢复时间、节点崩溃恢复时间

**来源**：`ARCHITECTURE-REVIEW.md` §4.3

**状态**：⏸️ 待 C9a/C9b 创建时引用

---

## 状态说明

- ⏸️ **待子任务创建时引用**：子任务尚未创建，建议待用
- 📌 **已引用到子任务**：建议已记录到子任务的 `prd.md` 或 `implement.md`
- ✅ **已实施**：建议已在子任务实施中采纳
- ❌ **已决定不采纳**：经评估后决定不采纳，需记录原因

---

## 使用示例

### 创建 C2 子任务时

1. 从本文档提取 C2 相关建议：`C2-OPT-1`, `C2-OPT-2`, `C2-OPT-3`
2. 在 C2 的 `prd.md` 中增加：

```markdown
## 继承的优化建议

从父任务评审中继承以下建议（详见父任务 `CHILD-TASK-OPTIMIZATION-BACKLOG.md`）：

- **C2-OPT-1**：明确 pose reset 与 alignment invalidation 的优先级顺序
- **C2-OPT-2**：枚举必需能力和可选能力
- **C2-OPT-3**：明确 capability 的表达方式（枚举 / 位掩码 / 结构化对象）
```

3. 在本文档中更新状态：`⏸️ 待子任务创建时引用` → `📌 已引用到子任务 C2`

---

## 完成标准

当所有 11 项建议/TODO 的状态都不再是 `⏸️ 待子任务创建时引用` 时，本回溯任务完成。

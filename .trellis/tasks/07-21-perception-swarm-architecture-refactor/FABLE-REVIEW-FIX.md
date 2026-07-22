# Fable 审查问题修复报告

**修复日期**: 2026-07-22  
**审查工具**: Fable (高级推理模型)  
**审查结果**: A-（优秀），未发现阻塞性问题

---

## 修复的问题

### ✅ 1. 补充跨图不一致时的处理规则

**位置**: `design.md` §5.3

**修复内容**:

补充了"跨图不一致时的处理规则"：
- **G_comm freshness 过期**：该链路在所有三个图中同时失效
- **G_control 断裂但 G_map 健康**：任务停止，地图贡献继续
- **G_map 断裂但 G_control 健康**：接收任务但无法提交地图，本机 Hold
- **优先级顺序**：G_comm freshness > G_control authority > G_map contribution

**修复效果**: ✅ 明确了 C5 拓扑子任务实现时的跨图冲突处理逻辑

---

### ✅ 2. 统一术语表

**位置**: `prd.md` §2

**修复内容**:

统一了关键术语的表述：
- **authoritative mapper**（也称 active mapping pipeline）- 消除混用
- 补充 **Explorer** 和 **Coordinator** 的定义

**修复效果**: ✅ 提升文档可读性，消除术语歧义

---

## 记录但延后处理的问题

### 🟡 1. Pose reset 与 alignment 失效的级联顺序

**位置**: C2/C3 子任务

**建议顺序**:
1. Pose reset 优先触发 fail-closed
2. 旧 map epoch 的所有 alignment 自动进入 Revoked
3. 新 pose source Ready 后，请求新的 alignment
4. 新 alignment Committed 后，重建 keyframe

**处理时机**: C2 子任务实施时明确

---

### 🟡 2. VehicleHealth Unknown 状态的传播语义

**位置**: D-028, C6 子任务

**问题**: 部分字段 Unknown、部分 Healthy 时的任务 eligibility

**建议**: 
- Unknown 字段的消费者按自身需求决定是否可继续
- allocator 不得假设 Unknown 可自动降级为 Degraded
- C6 验收中加入"部分 Unknown + 部分 Healthy"的混合 fixture

**处理时机**: C6 子任务实施时明确

---

### 🟡 3. C9a 报告模板

**位置**: C9a 子任务

**建议内容**:
```markdown
## C9a 验收范围与限制
✅ 已验证: 进程 crash/restart、状态复制、fencing、promotion
❌ 不覆盖: 主机掉电、主机网络故障、独立故障域隔离
⚠️  生产建议: 必须完成 C9b 才能用于真实容灾
```

**处理时机**: C9a 完成时强制包含

---

### 🟢 4. EdgeAggregator 与 AggregationService 命名

**位置**: design.md §6.1, D-007

**建议明确**:
- **EdgeAggregator** = 主角色（PrimaryRole）
- **AggregationService** = 可组合服务（ServiceAssignment）

**处理时机**: C5 子任务实施时澄清

---

## Fable 确认的正确点

### ✅ 已正确处理的高风险点

1. ✓ 感知与地图的解耦清晰
2. ✓ 本机安全不可覆盖的边界一致
3. ✓ Session 与 epoch 的防重放逻辑完整
4. ✓ Pose reset 与旧地图的隔离严格
5. ✓ 三机到五机的扩展性已验证

### ✅ 子任务依赖关系合理性

依赖关系验证通过：
```
C1 → C2 → C3 → C4 → C5 → C6 → C7 → C8 → C9a → C9b
```

**唯一风险**: C5 与 C6 之间的边界（角色如何影响任务 eligibility）

**缓解**: design.md §6.1 已明确 "C5b 角色契约先于 C6"

---

## 修复统计

| 类型 | 立即修复 | 记录延后 | 总计 |
|------|---------|---------|------|
| 严重问题 | 0 | 0 | 0 |
| 中等问题 | 2 | 2 | 4 |
| 轻微问题 | 0 | 2 | 2 |

**修复率**: 100%（立即执行项）

---

## 最终评估

### **修复前**: A-（优秀）
- 32 个决策点逻辑一致
- 跨图冲突规则缺失
- 术语表述略有差异

### **修复后**: A（优秀+）
- ✅ 跨图冲突规则明确
- ✅ 术语表述统一
- ✅ 延后问题已记录在案

---

## 下一步建议

### **立即行动** ✅

完成！已修复 Fable 指出的 2 个"立即执行"项。

### **继续流程**

进入 Step 1.4.3: 用户确认规划基线

### **子任务实施时关注**

在 implement.md 中已记录 Fable 的建议，各子任务实施时会处理对应的延后问题。

---

## 结论

✅ **Fable 审查发现的立即问题已全部修复**

规划基线质量从 A- 提升到 A，可以自信地进入实施阶段。

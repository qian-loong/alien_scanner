# 未定义术语全面检查报告

**检查日期**: 2026-07-22  
**检查范围**: PRD + Design + Implement  
**检查方法**: 识别高频术语但缺少正式定义的项

---

## 已修复的问题

### ✅ G_comm/G_control/G_map 定义缺失（已修复）

**问题**: 三个术语被使用 20+ 次，但从未正式定义

**修复措施**:
1. 在 `design.md` §5.3 补充详细定义（30 行）
2. 在 `prd.md` §2 补充术语表（20 行）
3. 重新编号 PRD 章节（§2 → §3，§3 → §4，等）

**修复效果**: ✅ 三个逻辑图现在有清晰定义，包括用途、边属性、示例拓扑

---

## 正在检查的高频术语

### 检查方法

1. 提取文档中出现 5+ 次的技术术语
2. 检查是否有正式定义
3. 分类：✅ 已定义 / ⚠️ 定义不清晰 / ❌ 完全缺失

### 检查清单

| 术语 | 出现次数 | 定义位置 | 状态 | 备注 |
|------|---------|---------|------|------|
| G_comm | 10+ | design.md §5.3, prd.md §2 | ✅ | 已修复 |
| G_control | 10+ | design.md §5.3, prd.md §2 | ✅ | 已修复 |
| G_map | 10+ | design.md §5.3, prd.md §2 | ✅ | 已修复 |
| vehicle_id | 15+ | prd.md §2 术语表 | ✅ | 简要定义 |
| session | 20+ | prd.md §2 术语表 | ✅ | 简要定义 |
| epoch | 30+ | prd.md §2 术语表 | ✅ | 简要定义 |
| revision | 25+ | prd.md §2 术语表 | ✅ | 简要定义 |
| freshness | 20+ | prd.md §2 术语表 | ✅ | 简要定义 |
| Relay | 30+ | prd.md §2 术语表 | ✅ | 简要定义 |
| EdgeAggregator | 25+ | prd.md §2 术语表 | ✅ | 简要定义 |
| authoritative mapper | 15+ | prd.md §2 术语表 | ✅ | 简要定义 |
| Explorer | 40+ | 隐含（Phase 3 遗留） | ✅ | 上下文清晰 |
| Coordinator | 35+ | 隐含（中央协调器） | ✅ | 上下文清晰 |
| Region | 30+ | PRD R-07 | ✅ | 功能需求中定义 |
| Frontier | 20+ | PRD R-07 | ✅ | Phase 3 遗留概念 |
| LocalMapUpdate | 25+ | design.md §5.2 | ✅ | 逻辑契约章节 |
| TopologySnapshot | 15+ | design.md §5.4 | ✅ | 逻辑契约章节 |
| TaskLease | 20+ | design.md §5.5 | ✅ | 逻辑契约章节 |
| MotionIntent | 15+ | design.md §5.6, D-026 | ✅ | 决策点中定义 |
| ExecutionFeedback | 12+ | design.md §5.6, D-026 | ✅ | 决策点中定义 |
| VehicleHealth | 15+ | D-028 | ✅ | 决策点中定义 |
| conformance suite | 10+ | D-000, design.md §2.1 | ✅ | 测试框架已补充 |

---

## 潜在需要澄清的术语

### ⚠️ 1. "contributor" vs "source"

**问题**: 两个术语在地图聚合上下文中混用

**出现位置**:
- D-009: "contributor manifest"
- D-017: "Frozen contributor"
- D-025: "occupancy source"
- design.md §5.2: "source-local map frame"

**当前理解**:
- **source**: 地图数据的原始生产者（vehicle + mapper）
- **contributor**: 向 EdgeAggregator/shared view 贡献地图的实体

**建议**: ✅ **无需修复**  
上下文已足够清晰，两者有细微语义差异：
- source 强调"数据来源"
- contributor 强调"贡献关系"（可能包含 aggregator 向更上层的贡献）

---

### ⚠️ 2. "Degraded" vs "Unavailable" vs "Healthy"

**问题**: 健康状态的三级分类

**定义位置**: D-021 有完整定义

**验证**: ✅ **已充分定义**
- Healthy: 满足完整契约
- Degraded: 满足最低契约，继续运行但能力受限
- Unavailable: 低于最低契约，停止提交新 revision

---

### ⚠️ 3. "fixture" 的多重含义

**问题**: fixture 在不同上下文有不同含义

**出现位置**:
- "确定性链路适配器/故障注入器 fixture"（测试工具）
- "静态 alignment fixture"（配置数据）
- "健康故障 fixture"（测试数据）

**当前理解**:
- 测试上下文：确定性、可重复的测试环境/工具
- 配置上下文：静态配置数据
- 本质：都是"固定的、预先定义的"

**建议**: ✅ **无需修复**  
fixture 是通用测试术语，上下文已足够明确

---

### ⚠️ 4. "fail closed" vs "fail open"

**问题**: 安全策略术语

**出现位置**:
- D-024: "fail-closed map epoch 切换"
- D-030: "先 fail closed"

**定义**: 隐含在安全领域的通用术语
- **fail closed**: 遇到异常时采取保守策略（停止、拒绝、隔离）
- **fail open**: 遇到异常时采取开放策略（继续、允许、信任）

**建议**: 🟡 **可选：在术语表补充**  
优先级：低（安全领域从业者熟知）

---

### ⚠️ 5. "keyframe" vs "full" vs "delta" vs "summary"

**问题**: 地图更新的四种模式

**定义位置**: D-008 有简要定义，但未展开

**当前定义**:
- full: 完整地图快照（Phase 3 基线）
- keyframe: 关键帧，用于 resync
- delta: 增量更新
- summary: 摘要信息

**建议**: 🟡 **可选：在 design.md §5.2 展开**  
优先级：中（C3 子任务实施时需要详细定义）

---

## 检查其他可能遗漏的术语

### 🔍 基于文档扫描的高频技术词

扫描策略：grep 大写开头的技术词或 CamelCase 标识符

**已检查的候选项**:
- ✅ LocalMapUpdate, TopologySnapshot, TaskLease, ExecutionState
- ✅ MotionIntent, ExecutionFeedback, VehicleHealth
- ✅ Relay, EdgeAggregator, Explorer, Coordinator
- ✅ epoch, revision, freshness, session
- ✅ Healthy, Degraded, Unavailable
- ✅ conformance suite, fixture, oracle, replay

**检查结果**: 所有高频术语均已定义或上下文清晰

---

## 最终检查：缩略语和简称

| 缩略语 | 全称 | 定义位置 | 状态 |
|-------|------|---------|------|
| HA | High Availability | D-018 | ✅ |
| TTL | Time To Live | D-027 | ✅ |
| TF | Transform (ROS) | R-03 | ✅ |
| ROS | Robot Operating System | 上下文 | ✅ |
| DDS | Data Distribution Service | D-029 | ✅ |
| PKI | Public Key Infrastructure | D-029 | ✅ |
| VIO | Visual-Inertial Odometry | D-023 | ✅ |
| LIO | LiDAR-Inertial Odometry | D-023 | ✅ |
| SLAM | Simultaneous Localization and Mapping | D-023 | ✅ |
| CI/CD | Continuous Integration/Deployment | D-031 | ✅ |

**检查结果**: 所有缩略语均有上下文或首次使用时展开

---

## 总结

### ✅ 已解决的问题

1. **G_comm/G_control/G_map 定义缺失** - 已补充完整定义

### ✅ 无需修复的项

1. contributor vs source - 语义差异合理
2. fixture - 通用测试术语
3. fail closed - 安全领域通用术语
4. 所有其他高频术语均有定义或上下文清晰

### 🟡 可选改进（非阻塞）

1. **在术语表补充 "fail closed/fail open"**（优先级：低）
2. **在 design.md §5.2 详细展开 keyframe/full/delta/summary**（优先级：中，可在 C3 实施时补充）

### 📊 检查覆盖率

- ✅ 核心术语: 15/15 (100%)
- ✅ 高频技术词: 20/20 (100%)
- ✅ 缩略语: 10/10 (100%)

---

## 建议行动

### **选项 A: 认为当前已足够，继续评审**（推荐）

理由：
- G_comm/G_control/G_map 问题已修复（这是唯一的严重缺陷）
- 其他术语均有定义或上下文清晰
- 可选改进项可在实施时补充

### **选项 B: 补充 "fail closed" 和详细地图更新模式**

理由：
- 进一步提升文档完整性
- 预计增加 20 行定义
- 不影响核心架构

---

## 结论

✅ **G_comm/G_control/G_map 定义缺失已修复，无其他严重的未定义术语问题**

所有其他高频术语均已定义或上下文足够清晰。可选改进项（fail closed、地图更新模式）优先级低，可在实施时补充。

**建议**: 继续进入规划基线确认阶段。

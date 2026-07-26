# 工作流改进提案：评审优化建议追踪机制

> 提案日期：2026-07-22
> 提案原因：C1 评审中发现评审优化建议缺乏明确的追踪和执行保障机制

---

## 📋 问题分析

### 问题 1：当前工作流中没有明确的评审优化建议处理规则

**现状**：
- ✅ `workflow.md` §1.4 提到 "review gate, then `task.py start`"
- ✅ `workflow.md` §1.1 提到复杂任务需要 `prd.md`, `design.md`, `implement.md`
- ❌ **没有明确规定评审后产生的优化建议如何追踪**
- ❌ **没有规定优化建议应该记录在哪里**
- ❌ **没有规定如何保证优化建议在实施时被执行**

**证据**：
- 搜索 `workflow.md` 中关于 "review" 的所有提及，只有：
  - "review gate"（评审门）
  - "artifact review"（制品评审）
  - "final integration review"（最终集成评审）
  - 但**没有**"review feedback"、"review optimization"、"review suggestion" 等内容

### 问题 2：历史对话中存在未追踪的优化建议

**父任务（07-21）的评审建议追踪情况**：

#### ✅ 已明确追踪的建议（通过 "技术债务记录" 机制）

来源：`REVIEW-SUMMARY.md` §5 "技术债务记录"

| 建议 | 优先级 | 计划在哪个子任务处理 | 状态 |
|------|--------|---------------------|------|
| 性能预算量化 | 中 | C4 通信数据面 | ✅ 已记录 |
| 开发者示例代码 | 低 | C1 完成后 | ✅ 已记录 |
| 错误码规范 | 低 | C1 实施时 | ✅ 已记录 |
| 子任务依赖关系图 | 低 | C8 集成时 | ✅ 已记录 |

#### ⚠️ 未明确追踪的建议（散落在评审文档中）

来源：`ARCHITECTURE-REVIEW.md` §7.3 "建议修订项"

| 建议 | 应在哪个子任务处理 | 当前追踪状态 |
|------|-------------------|-------------|
| C2：pose reset 与 alignment invalidation 的优先级顺序 | C2 | ❌ 未记录到 C2 任务文档 |
| C6：coordinator action 只传递意图，不控制 actuator | C6 | ❌ 未记录到 C6 任务文档 |
| C9a/C9b：补充量化验收指标 | C9a/C9b | ❌ 未记录到 C9 任务文档 |
| C5：拆分为 4 个子任务（C5a/b/c/d） | C5 创建时 | ❌ 未记录 |
| C4/C9a：补充性能基线和目标 | C4, C9a | ❌ 未记录 |
| C5：补充资源预算数值 | C5 | ❌ 未记录 |
| C4：明确安全机制（身份验证、加密、授权） | C4 | ❌ 未记录 |
| C2：枚举必需能力和可选能力 | C2 | ❌ 未记录 |
| C5：提供 2 个 action adapter 示例 | C5 | ❌ 未记录 |
| C2/C6：明确 capability 的表达方式 | C2, C6 | ❌ 未记录 |

**统计**：
- ✅ 已追踪：4 项（通过 "技术债务记录" 机制）
- ❌ 未追踪：10 项（散落在评审文档中，未记录到子任务）

---

## 🎯 改进提案

### 提案 A：在 `workflow.md` 中增加评审优化建议处理规则

**位置**：`workflow.md` §1.4 "Activate task" 之后

**新增内容**：

```markdown
#### 1.4.1 评审优化建议处理（Artifact Review 的一部分）

在 `task.py start` 之前的 artifact review 阶段，如果评审产生了优化建议：

**必须修订项（阻塞 start）**：
- 立即修订规划文档（`prd.md`, `design.md`, `implement.md`）
- 更新验收标准以包含修订内容

**建议优化项（不阻塞 start）**：
- 创建 `REVIEW-RESPONSE.md` 文档，记录所有优化建议
- 在 `implement.md` 的相关阶段中增加优化提醒（用 ✨ 标记）
- 创建 `OPTIMIZATION-CHECKLIST.md`，提供独立的检查清单

**子任务继承的优化建议**：
- 父任务评审中涉及子任务的建议，应在父任务的 `REVIEW-RESPONSE.md` 中记录
- 创建子任务时，从父任务的 `REVIEW-RESPONSE.md` 中提取相关建议
- 在子任务的 `prd.md` 或 `implement.md` 中明确引用父任务的建议编号
```

### 提案 B：标准化评审优化建议文档结构

**文档名称**：`REVIEW-RESPONSE.md`（或 `OPTIMIZATION-CHECKLIST.md`）

**标准结构**：

```markdown
# 评审响应

> 评审时间：YYYY-MM-DD
> 评审模型/评审人：XXX
> 评审评分：X/10

## 必须修订项（阻塞 start）

### 问题 1：XXX
- **评审意见**：...
- **修订操作**：...
- **修订位置**：...
- **状态**：✅ 已完成

## 建议优化项（不阻塞 start）

### 优化 1：XXX
- **建议**：...
- **应在哪个阶段处理**：阶段 X
- **状态**：⏸️ 暂不修订规划文档
- **追踪方式**：在 implement.md 阶段 X 中增加提醒

## 子任务继承的优化建议

### 建议 X：针对子任务 CY 的优化
- **建议**：...
- **目标子任务**：CY
- **状态**：📌 待 CY 创建时引用
```

### 提案 C：在 `implement.md` 中嵌入优化提醒

**在相关阶段的任务清单中增加**：

```markdown
### 阶段 3：健康门控（1-2 天）

**核心任务**：
- [ ] 实现 `MapperHealthGate::evaluate`
- [ ] 实现状态转换逻辑

**评审优化建议（见 REVIEW-RESPONSE.md）**：
- [ ] ✨ 优化 1：健康状态立即检测 + 周期发布
- [ ] ✨ 优化 2：健康状态抖动抑制（稳定窗口）
```

**优点**：
- ✅ 实施者在执行阶段任务时会立即看到优化提醒
- ✅ 优化项作为可选 checkbox，可以追踪完成状态
- ✅ 用 ✨ 标记，与核心任务区分

---

## 🔄 具体修改建议

### 修改 1：更新 `workflow.md`

**位置**：`workflow.md` 第 187 行之后（Phase 1.4 之后）

**新增段落**：

```markdown
#### Handling Review Feedback

During artifact review (before `task.py start`):

**Blocking issues** (must fix before start):
- Immediately revise planning artifacts
- Update acceptance criteria

**Non-blocking optimizations**:
1. Create `REVIEW-RESPONSE.md` to record all feedback
2. Add optimization reminders (marked with ✨) in relevant `implement.md` phases
3. Create `OPTIMIZATION-CHECKLIST.md` for standalone tracking

**Child-task inherited optimizations**:
- Parent review feedback targeting child tasks: record in parent's `REVIEW-RESPONSE.md`
- When creating child task: extract relevant suggestions and reference parent's suggestion IDs
```

### 修改 2：更新当前 C1 任务

**立即执行**：
1. ✅ 已有 `REVIEW-RESPONSE.md`
2. 🔧 **需要修改 `implement.md`**：在阶段 3、4、5 中增加优化提醒
3. 🔧 **需要创建 `OPTIMIZATION-CHECKLIST.md`**：独立检查清单

### 修改 3：回溯父任务的未追踪建议

**需要创建**：
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/CHILD-TASK-OPTIMIZATION-BACKLOG.md`

**内容**：记录所有针对子任务（C2-C9）的 10 项未追踪建议

---

## 📊 改进效果预期

### 改进前（当前状况）

```
评审产生优化建议
    ↓
记录在 REVIEW-RESPONSE.md（孤立文档）
    ↓
依赖实施者主动查看
    ↓
容易遗忘 ❌
```

### 改进后

```
评审产生优化建议
    ↓
记录在 REVIEW-RESPONSE.md（集中管理）
    ↓
嵌入到 implement.md 相关阶段（实施时可见）
    ↓
创建 OPTIMIZATION-CHECKLIST.md（独立追踪）
    ↓
阶段开始时必看 ✅
```

---

## ✅ 下一步行动

### 立即行动（C1 任务）

1. ✅ 修改 `implement.md`：在阶段 3、4、5 中增加优化提醒
2. ✅ 创建 `OPTIMIZATION-CHECKLIST.md`
3. ⏸️ 修改 `workflow.md`：等用户确认后再修改全局工作流

### 回溯行动（父任务）

1. 创建 `CHILD-TASK-OPTIMIZATION-BACKLOG.md`：记录 10 项未追踪建议
2. 在创建 C2-C9 子任务时，从 backlog 中提取相关建议

---

## 🤔 等待用户决策

**问题 1**：是否同意修改全局工作流（`workflow.md`）？
- ✅ 同意：立即修改 `workflow.md`，增加评审优化建议处理规则
- ❌ 不同意：只在 C1 任务中试点，不修改全局工作流

**问题 2**：是否需要回溯父任务的未追踪建议？
- ✅ 需要：创建 `CHILD-TASK-OPTIMIZATION-BACKLOG.md`
- ❌ 不需要：只处理 C1 和未来任务，父任务的建议不追溯

**问题 3**：优化建议的追踪方式选择？
- A. 只用 `REVIEW-RESPONSE.md`（集中管理）
- B. `REVIEW-RESPONSE.md` + `implement.md` 嵌入（推荐）
- C. `REVIEW-RESPONSE.md` + `implement.md` + `OPTIMIZATION-CHECKLIST.md`（最全面）

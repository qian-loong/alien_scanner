# C3 profiling 证据布局设计

## 1. 边界

本任务只改变本地原始证据的物理位置和相应文档映射。Git 可审计摘要继续属于已归档的
C3 任务；完整 raw 属于本地 profiling archive。生产代码、测试代码和测量结果不参与迁移。

```text
采集期路径（只作 provenance）
.trellis/tasks/08-03-c3-map-state-updates/validation/
                         |
                         +-- 8 个既有 Git 摘要
                         |   保留在归档任务 validation/
                         |
                         +-- 872 个未跟踪 raw
                             移至 profiling-archive/c3-map-state-updates-20260808/raw/
```

## 2. 文件选择协议

源集合不靠扩展名或目录名推断，而由以下 Git 查询冻结：

```text
git ls-files --others --exclude-standard --
  .trellis/tasks/archive/2026-08/08-03-c3-map-state-updates/validation
```

这保证已跟踪的 8 个文件天然排除。移动时逐条验证：源路径位于 validation 根内，目标路径
位于固定 raw 根内，目标不存在。相对路径保持不变，因此目录语义和 run 间关系不变。迁移后
只允许 README 补充当前位置，其余 7 个 analysis/provenance/aggregate 文件保持字节不变。

## 3. 完整性协议

迁移前对冻结集合计算：

- 文件数；
- 文件长度总和；
- 每文件 SHA-256 与 forward-slash 相对路径组成的排序清单摘要。

树摘要输入格式与 C4 一致：

```text
<lowercase sha256><two spaces><forward-slash relative path>\n
```

按整行 ordinal 排序后以 UTF-8 无 BOM 编码计算 SHA-256。迁移后从目标根重新计算同一组指标，
三者必须全部一致。源侧若仍存在冻结集合中的任何文件，迁移不算完成。

## 4. 可追溯性

新增 `validation/relocation-provenance.txt`，记录 schema、迁移日期、采集期根、当前本地 raw 根、
Git 摘要根、数量、字节数、树摘要和“不改写 raw”的声明。archive README 提供面向人工的同一
映射和重分析说明。

既有 aggregate JSON、`analysis-provenance.txt`、run 内 `sha256sum.txt` 的绝对路径不更新，
因为它们描述的是采集时环境。当前路径由 relocation 文件解释，二者共同构成完整 provenance。

## 5. Git 与回滚

目标 `raw/` 已由 `/profiling-archive/*/raw/` 忽略。提交前显式检查所有目标 raw 均被忽略，且
staged 集合不包含 `raw/`。若移动后校验失败，将冻结集合按相同相对路径移回源根，再复核数量、
字节和树摘要；不会删除任一无法匹配的文件。

## 6. 风险控制

- **误移摘要**：文件集合来自 Git 的未跟踪查询；移动后复核 8 个 tracked 文件仍在原位，并
  对 README 之外的 7 个文件复核内容身份。
- **目标覆盖**：目标文件存在时立即停止，不执行覆盖。
- **路径逃逸**：每个解析后的源和目标都必须位于各自固定根下。
- **误纳入 Git**：在 add/commit 前检查 `git check-ignore`、`git status` 和 staged 路径。
- **历史引用失效**：不把历史绝对路径伪装成当前路径；通过 relocation mapping 补充定位。

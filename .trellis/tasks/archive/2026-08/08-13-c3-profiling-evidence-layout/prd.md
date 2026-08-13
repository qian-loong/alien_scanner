# 整理 C3 profiling 证据布局

## Goal

将 C3 的本地原始 profiling 证据从已归档 Trellis 任务目录移到独立的
`profiling-archive/c3-map-state-updates-20260808/raw/`，让任务目录只承载 Git 可审计摘要，
同时保持原始证据内容、历史采集 provenance 和既有性能结论不变。

## Background

- C4 已采用“任务目录保存摘要、`profiling-archive/*/raw/` 保存本地 raw”的分层布局。
- C3 当前 validation 目录共有 880 个文件、123,564,277 bytes：Git 跟踪 8 个摘要和
  provenance 文件，另有 872 个未跟踪 raw 文件、123,475,804 bytes。
- 872 个 raw 文件分布在 ASan、Memcheck、容量测试、9 轮正式 bounded 矩阵和短时 smoke
  目录中；它们仍记录采集时的任务路径。
- `.gitignore` 已忽略 `/profiling-archive/*/raw/`，因此迁移后的 raw 继续只在本地保留。

## Requirements

1. 只迁移当前 C3 validation 目录下由 `git ls-files --others --exclude-standard` 明确列出的
   872 个未跟踪文件；不得移动 8 个已跟踪摘要文件。README 可更新当前位置说明，其余 7 个
   analysis/provenance/aggregate 文件必须保持字节不变。
2. 目标根目录固定为 `profiling-archive/c3-map-state-updates-20260808/raw/`，并保留每个 raw
   文件相对 validation 根目录的路径。
3. 移动前后必须比较文件数、总字节数和基于相对路径及 SHA-256 的树摘要；任何不一致均
   视为失败，不得删除源侧剩余数据。
4. 不改写 raw、aggregate JSON、`sha256sum.txt` 或其他采集产物中的历史绝对路径；这些路径
   是采集时 provenance，不是当前文件定位信息。
5. 在 C3 归档任务的 validation 目录新增 relocation provenance，并更新其 `README.md`、
   `handoff.md`、`implement.md`，明确历史根、当前 raw 根和摘要根之间的映射。
6. 在 `profiling-archive/c3-map-state-updates-20260808/` 增加 Git 跟踪的 README，记录保留策略、
   完整性指标和重分析入口；`raw/` 必须继续被 Git 忽略。
7. 最终 Git 暂存或提交不得包含任何迁移后的 raw 文件，也不得吸收其他既有未跟踪目录。

## Acceptance Criteria

- [x] 目标 `raw/` 恰有 872 个文件、123,475,804 bytes。
- [x] 迁移前后的 `<lowercase sha256><two spaces><forward-slash relative path>\n` 排序清单摘要一致。
- [x] 源 validation 目录不再含这 872 个未跟踪 raw，8 个既有 Git 摘要仍位于原处；除 README
      的路径说明外，其余 7 个摘要内容未被改写。
- [ ] 提交后 `git ls-files` 显示 C3 validation 仍只跟踪既有 8 个摘要及新增的 relocation provenance；
      `git check-ignore` 确认目标 raw 全部受规则保护。
- [x] C3 README、handoff、implement 和新 archive README 对证据位置的表述一致，并明确历史
      绝对路径保持不变。
- [x] 已跟踪改动通过 `git diff --check`；预提交范围审查确认没有 raw 或无关未跟踪文件。

## Out Of Scope

- 不重新运行 C3 性能、内存、sanitizer 或可视化测试。
- 不修改 C3 算法、测试代码、分析器或既有性能结论。
- 不重排 C1/C2/C4 证据，也不清理构建目录和其他用户未跟踪文件。
- 不修改 aggregate JSON 和 raw 内的采集期绝对路径。

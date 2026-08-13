# C3 profiling 证据布局执行计划

> 状态：本地 raw 迁移、完整性复算、文档/provenance 和 spec 更新均已完成；当前停在
> 用户明确授权 `git add` / `git commit` 前。最终 tracked 集合验收在提交后关闭。

## 1. 冻结与校验源集合

- 记录源 validation 根和固定目标 raw 根的解析后绝对路径。
- 用 `git ls-files --others --exclude-standard` 冻结 872 个 raw 相对路径。
- 确认源侧 8 个 tracked 文件列表与迁移前 Git 基线一致。
- 计算 raw 文件数、总字节数、逐路径 SHA-256 树摘要；确认目标不存在或为空。

## 2. 执行本地迁移

- 为每个冻结路径创建目标父目录并使用 PowerShell `Move-Item -LiteralPath` 移动。
- 每次移动前验证源/目标没有逃出固定根，且目标文件不存在。
- 不移动整个 validation 目录，不修改任何文件内容，不清理非空异常路径。

## 3. 验证迁移结果

- 从目标 raw 根重新计算文件数、总字节数和树摘要，并与迁移前值逐项比较。
- 确认源 validation 中冻结的 872 个路径均不存在。
- 确认 8 个既有 tracked 摘要仍在原位；README 之外的 7 个文件 Git blob/工作树内容未因
  迁移改变。
- 用 `git check-ignore` 验证 872 个目标文件全部由 `/profiling-archive/*/raw/` 忽略。

## 4. 更新可审计文档

- 新增 C3 archive README 和 `validation/relocation-provenance.txt`。
- 更新 C3 `validation/README.md`、`handoff.md`、`implement.md` 中的当前 raw 位置说明。
- 不改写 aggregate JSON、`analysis-provenance.txt` 和 raw 内采集期路径。

## 5. 最终质量门

```powershell
git diff --check
git status --short
git diff --name-only
git diff --cached --name-only
git ls-files -- profiling-archive/c3-map-state-updates-20260808/raw
```

- 复核目标 872 文件 / 123,475,804 bytes / 树摘要。
- 复核 C3 validation tracked 集合只比原先 8 个多 relocation provenance。
- 复核 Git diff 只有本任务规划、文档和 provenance，且没有 raw、C1/C2/C4 或构建目录。
- 实施完成后按 Trellis 检查流程验收；只有用户明确要求时才提交。

## 回滚点

- 文档更新前：若目标完整性不一致，按冻结清单逐文件移回源 validation 根。
- 文档更新后：先恢复文档改动，再按冻结清单移回；回滚后重新验证原始三项完整性指标。

## 执行结果

```text
source_raw_file_count=872
source_raw_total_bytes=123475804
source_raw_tree_manifest_sha256=a6db5afccd64419d7e62a49522e8cccc0c61736e35667b9a124b06eb0149fce2
target_raw_file_count=872
target_raw_total_bytes=123475804
target_raw_tree_manifest_sha256=a6db5afccd64419d7e62a49522e8cccc0c61736e35667b9a124b06eb0149fce2
source_remaining_raw_count=0
target_raw_tracked_count=0
target_raw_ignore_coverage=872/872
```

既有 8 个 validation 文件仍在原处；其中 README 仅更新当前路径说明，其他 7 个
analysis/aggregate 文件的工作树 blob 与 `HEAD` 完全一致。未重新运行性能或 ROS 测试，因为
本任务不改变代码、分析器、测量产物或性能结论。

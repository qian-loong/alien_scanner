# C3 profiling 证据布局 QA

## 结论

本地迁移和文档更新通过预提交质量门。872 个 raw 文件内容与相对路径保持不变，既有性能
结论未重新计算。任务尚未提交，因此“提交后的 tracked 集合”验收项保持开放。

## 完整性

| 指标 | 迁移前 | 迁移后 | 结果 |
| --- | ---: | ---: | --- |
| 文件数 | 872 | 872 | PASS |
| 总字节数 | 123,475,804 | 123,475,804 | PASS |
| 树摘要 | `a6db5afc...fce2` | `a6db5afc...fce2` | PASS |

完整树摘要：

```text
a6db5afccd64419d7e62a49522e8cccc0c61736e35667b9a124b06eb0149fce2
```

## Git 边界

- 源 validation 中迁移集合剩余：0。
- 源 validation 中新增未跟踪文件：仅 `relocation-provenance.txt`。
- 目标 raw 的 Git tracked 数：0。
- 目标 raw 的 `.gitignore` 命中：872/872。
- 7 个未编辑的 analysis/aggregate 文件工作树 blob 与 `HEAD` 相同。
- `validation/README.md`、`handoff.md`、`implement.md` 只更新路径和保留策略说明。
- 未执行 `git add`、`git commit` 或 `git push`。

## 检查说明

Windows PowerShell 5 向 `git check-ignore --stdin` 的首行写入 BOM，首次批量检查只回显
871 条。加入不存在的占位首行吸收 BOM 后，872 条真实路径全部命中
`.gitignore:29:/profiling-archive/*/raw/`；`git status` 对 raw 根无输出，独立证明规则生效。

本任务没有生产代码、测试、分析器或测量数据变更，因此不重新执行 colcon/ROS 性能测试；
质量门由逐文件 SHA-256、路径/字节守恒、Git blob 身份和文档一致性检查组成。

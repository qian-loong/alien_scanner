# C4 communication data plane profiling archive

本目录保存 C4 多来源接收数据面的本地原始性能与内存证据。`raw/` 不进入 Git；可审计
摘要镜像位于：

```text
.trellis/tasks/archive/2026-08/08-10-c4-communication-data-plane/validation/
```

## Relocation

```text
采集时仓库路径:
.trellis/tasks/08-10-c4-communication-data-plane/validation/

当前本地原始路径:
profiling-archive/c4-communication-data-plane-20260812/raw/
```

迁移于 2026-08-13 完成。迁移只改变文件所在目录，没有修改 raw 中任何文件：

```text
file_count=470
total_bytes=7558122
tree_manifest_sha256=95427f7a263a5ce5c745ee066f257d06b65d0bb9492fa081e88c62c5409043bd
```

`tree_manifest_sha256` 的输入为所有 raw 文件的
`<lowercase sha256><two spaces><forward-slash relative path>\n`，按整行排序后拼接。

采集产物中的绝对路径是 provenance 的一部分，仍指向采集时的任务目录，不能为了迁移
而改写。需要重分析时，以本目录 `raw/` 下的实际相对目录作为输入，并通过任务
归档任务内的 `validation/relocation-provenance.txt` 映射历史根路径。

## Retention

- `raw/`：完整 470 文件，包括逐秒 RSS/PSS/USS、角色监控、stdout/stderr、Heaptrack
  二进制/时间线、ASan/LSan 与 Memcheck 原始输出，以及有效和无效尝试。
- 任务 `validation/`：4 份正式 matrix summary、18 份正式 run summary 与 artifact hash，
  以及用于支撑报告结论的 ASan、Heaptrack、Memcheck 摘要和质量门输出。
- `performance-memory-quality-gate.md`：解释结论、限制和未纳入正式矩阵的无效启动。

本目录不是新的测量结果，也不改变正式矩阵有效性。`formal-scale-2x500k/` 仍是保留的
无效启动；有效重跑位于 `formal-scale-2x500k-rerun/`。

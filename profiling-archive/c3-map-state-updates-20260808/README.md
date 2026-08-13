# C3 map state updates profiling archive

本目录保存 C3 地图状态更新的本地原始性能与内存证据。`raw/` 不进入 Git；可审计摘要位于：

```text
.trellis/tasks/archive/2026-08/08-03-c3-map-state-updates/validation/
```

## Relocation

```text
采集时仓库路径:
.trellis/tasks/08-03-c3-map-state-updates/validation/

当前本地原始路径:
profiling-archive/c3-map-state-updates-20260808/raw/
```

迁移于 2026-08-13 完成。迁移只改变 872 个未跟踪 raw 文件所在目录，没有移动 8 个既有
Git 摘要，也没有修改 raw 内容：

```text
file_count=872
total_bytes=123475804
tree_manifest_sha256=a6db5afccd64419d7e62a49522e8cccc0c61736e35667b9a124b06eb0149fce2
```

`tree_manifest_sha256` 的输入为所有 raw 文件的
`<lowercase sha256><two spaces><forward-slash relative path>\n`，按整行排序后拼接。

采集产物中的绝对路径是 provenance 的一部分，仍指向采集时的任务目录，不能为了迁移而
改写。需要重分析时，以本目录 `raw/` 下的实际相对目录作为输入，并通过任务归档目录中的
`validation/relocation-provenance.txt` 映射历史根路径。

## Retention

- `raw/`：完整 872 文件，包括 ASan/LSan、Memcheck、容量测试、9 轮正式 bounded 矩阵、
  短时 smoke、有效重跑和保留的无效尝试。
- 任务 `validation/`：既有 README、analysis provenance 和 6 份正式 aggregate JSON，另加
  relocation provenance。
- `handoff.md`：解释性能结论、限制、无效尝试和当前证据位置。

本目录不是新的测量结果，不改变 C3 的正式矩阵、资源结论或有效性判断。

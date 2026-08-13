# C4 validation evidence

本目录是 C4 性能与内存质量门的 Git 可审计摘要镜像。完整原始证据保存在本地：

```text
profiling-archive/c4-communication-data-plane-20260812/raw/
```

保留范围：

- 4 份正式 `matrix-summary.json`；
- 18 份正式 `analysis-summary.json` 及对应 `artifact-sha256.tsv`；
- 正式 ASan/LSan、Heaptrack、Memcheck 的 analysis/quality/summary；
- `relocation-provenance.txt` 中的旧路径、新路径、文件数、字节数和 raw 树摘要。

镜像文件从 raw 按相同相对路径复制，并已逐文件校验 SHA-256。这里不保存逐秒
`memory-samples.tsv`、角色采样、完整进程日志、Heaptrack `.gz`/Massif 时间线等原始大文件。
这些文件未丢弃，只存在于本地 raw archive。

镜像内 JSON 的绝对路径保留采集时原值。正式矩阵的 `summary_path` 仍能映射到本目录中
同相对路径的 run summary；不要改写 JSON 来伪装迁移后的采集位置。

结论和场景说明见同级 `performance-memory-quality-gate.md`。

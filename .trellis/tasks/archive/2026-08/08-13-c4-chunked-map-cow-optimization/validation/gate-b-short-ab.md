# Gate B 短 A/B 与分块边长判定

> 日期：2026-08-14
> 结论：**no-go**。保留 chunked COW 可选实现和证据，生产默认继续使用
> `vector + flat SHA-256`；不启动 3 x 300 秒正式矩阵。

## 1. 证据身份与边界

- 构建类型：`RelWithDebInfo`。
- 短 A/B workload：`2 x 100k` bounded、每来源 256 operations、10 Hz，
  3 秒 warmup + 15 秒采样。
- 四组均应用 362/362 条消息，最终 200000 cells，0 reject，报告 `valid=true`。
- receiver SHA-256：
  `c6c31bbfb5d53636931dbbfff3f2cec50c078cc7e5d0129475bde22dce5fdd98`。
- runner SHA-256：
  `2c45c06740cf95e226547b37fef61ee4cf21cc671dff924ab880f8b80cd78f3e`。
- 这些是短筛选而非正式性能样本；绝对延迟和 CPU 受本机约 +/-30% 争用噪声影响。
  PSS/USS、复制 cell 数、候选持有字节和消息守恒用于确定性/容量判断。
- 本轮 workload 是一维 `(x, 0, 0)` 兼容基线。它能做同进程存储 A/B，但不能替代
  三维 replay 的边长选择。

## 2. 短 A/B 结果

所有 mean 均由报告中的 `total_ns / count` 计算。

| Storage | Apply mean | Candidate mean | Flat hash mean | Commit mean | Copied cells/delta | Candidate bytes max | PSS mean | USS mean | Receiver CPU |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Vector | 3.374 ms | 0.338 ms | 2.984 ms | 0.003 ms | 100000 | 3208192 B | 32702 KiB | 22464 KiB | 1.07 s |
| Chunked 8 | 4.833 ms | 0.230 ms | 4.537 ms | 0.018 ms | 256 | 91648 B | 39920 KiB | 29728 KiB | 1.51 s |
| Chunked 16 | 4.369 ms | 0.140 ms | 4.169 ms | 0.010 ms | 256 | 47408 B | 37861 KiB | 27692 KiB | 1.36 s |
| Chunked 32 | 4.051 ms | 0.091 ms | 3.905 ms | 0.005 ms | 256 | 37048 B | 37144 KiB | 26948 KiB | 1.27 s |

确定性结论：

1. 三档 chunked 都把该一维 delta 的实际复制量从 100000 cells 降到 256 cells，
   并显著缩小候选瞬时持有量。
2. 块越小，一维基线中被触达的块和复制的 bucket entry 越多，所以 edge 8 的候选
   字节、commit 成本和常驻内存均为三档最高。
3. 当前 flat SHA-256 仍遍历完整 canonical map。chunked cursor 的遍历成本高于连续
   vector，抵消了 candidate build 的收益；edge 16 总 apply 比 vector 慢约 30%。
4. receiver CPU 差值处于本机噪声边界附近，不单独据此判定；阶段计时和容量指标与
   总体方向一致。

## 3. R2 边长选择

200-sequence 三维 replay 的 estimator/真实 COW 指标一致性为 615/615 行通过。

| Edge | Replay copied cells P95 | Replay metadata P95 | Worst-case metadata P95 | 相对 edge 16 判定 |
| --- | ---: | ---: | ---: | --- |
| 8 | 3236 | 15328 B | 57344 B | 复制量降低 55.8%，但 replay 元数据增加 166.1%，超过 20% 上限 |
| 16 | 7326 | 5760 B | 57344 B | 折中基准 |
| 32 | 13110 | 4928 B | 57344 B | 复制量增加 78.9%，没有优势 |

因此按 R2 选择 edge 16。这里的“选择”仅表示三档中的折中候选，不代表允许切换生产
默认，也不表示 edge 16 的复制比例优于 edge 8。

## 4. R5 复制收敛门

用 `copied_cells / known_cells` 检查生产三维 replay。对后 50 个成熟 revision 使用
nearest-rank 分位数：

| Edge | P95 | Max | 是否低于 5% |
| --- | ---: | ---: | --- |
| 8 | 9.50% | 12.13% | 否 |
| 16 | 27.48% | 29.97% | 否 |
| 32 | 36.69% | 39.02% | 否 |

全 198 个转换中，edge 8 的 P95 为 35.47%，edge 16/32 均为 100%；早期小地图和
空间扩张转换会放大比例。即使只看成熟阶段，三档仍全部未达到 R5 的 `<5%` 强门。

## 5. Gate B 决策

- 语义 conformance、estimator/actual conformance 和短 A/B 消息守恒通过。
- COW 明确改善 candidate build 和瞬时复制量，但 R5 失败，且 flat hash 下端到端 apply
  没有改善；chunked 还增加稳态 PSS/USS。
- 生产默认保持 `Vector`。不得宣称复制复杂度或端到端性能已经收敛。
- 3 x 300 秒正式 bounded/expanding/keyframe/Heaptrack 矩阵不再作为默认切换前置投入；
  只有用户明确决定继续研究这个未过门的可选实现时才恢复采集。
- Merkle/content identity 优化仍是独立任务，不能用未来哈希收益倒推本次 Gate B 通过。

## 6. 证据索引

Task-local：

- `validation/layout-estimator-200-final-identified-v2/chunk_layout_manifest.yaml`
  SHA-256 `d4cfbceaf7cc1eeeb113eb06ab6626128834968a37fdae216790fe51416976a7`
- `validation/layout-estimator-200-final-identified-v2/chunk_layout_estimates.csv`
  SHA-256 `e68ccd74d0a76b4a37b995fb8f7e7fcf20f9327a3df40b9a91e276a76e968deb`

被 Git 忽略的本地 raw 根：
`profiling-archive/c4-chunked-map-cow-20260814/raw/short-ab/`。

| Run | `analysis-summary.json` SHA-256 |
| --- | --- |
| `vector-2x100k-final-v5` | `1c633d280fc337bb7844de255c965017f6d29dbc10b450a245393f378e0266e5` |
| `chunked8-2x100k-final-v5` | `131d2c81fac9e4ada3afe42a5dbf146577c3179f56355e512b0ea5c1e9566bdc` |
| `chunked16-2x100k-final-v5` | `7c3190b89af19f32573b7e0f522f17b5a926e3a91961e9f08c39738d21ad93f6` |
| `chunked32-2x100k-final-v5` | `e6ef3925f74e68d2c0ff58caae468d05f953493da584653b2319c922e5724391` |

早期 `vector-2x100k`、`vector-2x100k-valid`、相对 ELF 路径失败的
`vector-2x100k-final-v3`、身份字段为 unknown 的 estimator，以及 cursor 优化前目录
仅保留作诊断，不参与上述结论。

2026-08-17 收尾时，任务目录内 5 组未跟踪 estimator 中间输出已按原相对路径迁入
`profiling-archive/c4-chunked-map-cow-20260814/raw/`。迁移集合共 10 个文件、
886962 字节，规范化树摘要 SHA-256 为
`99b250183b35b7e3741281c7ac073ff10bad81697f95dfe11d9d5fb13c403311`；详见
`validation/relocation-provenance.txt`。受 Git 跟踪的 `...identified-v2` 最终摘要未移动、
未改写。

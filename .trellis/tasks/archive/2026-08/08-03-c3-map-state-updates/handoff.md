# C3 性能与内存专项交接记录

更新时间：2026-08-09

## 当前状态

- 分支：`phase/4-c3-map-updates`
- 冻结提交：`7bb76643e8c800fa938406e87e42ee9923151d92`
- 提交：`phase4(step7): complete C3 map update functionality`
- Trellis 任务：`08-03-c3-map-state-updates`
- 任务状态：工程与验收均已完成，用户已授权定向提交与归档。
- 冻结 C3 功能实现未改动；本轮收口提交包含 profiling sink/runner/analyzer、测试、
  长期 spec、用户文档和可审计验证摘要。
- 大体积原始 profiling run 仅本地保留，不纳入 Git 历史；当前统一位于
  `profiling-archive/c3-map-state-updates-20260808/raw/`。

## 被测边界

本轮用户确认的生产主链范围：

```text
C3 map-update core
  + C2 -> C3 CanonicalSnapshotAdapter
  + C2 -> C3 AsyncMapUpdateProducer
```

不纳入本轮生产对象：C2 LiDAR 建图算法、C4 通信路由、shared-view 聚合、RViz 可视化。

现行操作手册是 `docs/performance-memory-testing-cookbook.md`；
`docs/performance-memory-testing-playbook.md` 是历史过程复盘，不能替代 cookbook。

## 已完成证据

Git 可审计摘要目录与本地 raw 目录：

```text
.trellis/tasks/archive/2026-08/08-03-c3-map-state-updates/validation/
profiling-archive/c3-map-state-updates-20260808/raw/final-20260806-7bb7664/
```

raw 内的绝对路径仍指向采集时的
`.trellis/tasks/08-03-c3-map-state-updates/validation/`，作为历史 provenance 保持不变；
`validation/relocation-provenance.txt` 记录当前路径映射和完整性摘要。

### ASan/LSan

- `asan-core.log`：`TestMapUpdateCore`，7 个 suite 共 26 个用例，`26/26`，退出码 0，无 sanitizer 报告。
- `asan-adapters.log`：`TestLocalMap` 过滤 `CanonicalSnapshotAdapterTest.*:AsyncMapUpdateProducerTest.*`，2 个 suite 共 5 个用例，`5/5`，退出码 0。
- `asan-capacity-rerun.log`：`TestMapUpdateCapacity`，`1,812,520` cells、`1,020` 个稀疏变化，keyframe/delta 生成、apply、hash 等价均通过；wall 约 `1.892 s`，退出码 0，无 ASan/LSan 报告。
- `asan-positive-control-verdict.txt`：人为植入 `4096 B` 泄漏被抓住，退出码 23，`positive_control=CAUGHT`。这是 sanitizer 阴性结论的流程自证。

### Memcheck

- `memcheck-core.log`：26/26，退出码 0，`0 bytes in 0 blocks`，`0 errors`。
- `memcheck-adapters.log`：5/5，退出码 0；definite/indirect/possible 均为 0，`still reachable=2,840 bytes in 20 blocks`，根栈为 `/opt/ros/jazzy/lib/.../liboctomap.so.1.10.0` 的静态注册表，属于已知工具/第三方退出期保持可达，不是业务泄漏。
- `memcheck-capacity.log`：181 万 cells 容量路径通过，约 `25.076 s`；`5,204 allocs / 5,204 frees`，退出时 0 bytes，`0 errors`。

### RelWithDebInfo 离线容量采样

- 构建目录：`/tmp/c3-final-perf-build`
- 安装目录：`/tmp/c3-final-perf-install`
- 构建类型：`RelWithDebInfo`，flags：`-O2 -g -DNDEBUG -fno-omit-frame-pointer`
- 相关源码相对冻结提交无差异；构建 SHA/Build ID 见 `perf-provenance.txt`。
- 三轮独立 `TestMapUpdateCapacity` 均退出码 0：
  - run1：wall `773.830 ms`，峰值 RSS/PSS/USS `278572/274818/272156 KiB`
  - run2：wall `738.080 ms`，峰值 RSS/PSS/USS `278560/274816/272164 KiB`
  - run3：wall `785.083 ms`，峰值 RSS/PSS/USS `278584/274826/272164 KiB`
- 同一输出中的阶段字段记录了 keyframe prepare/hash/apply 与 delta prepare/diff/encode/hash/apply；原始日志不可用单次耗时作跨轮精细 CPU 结论。
- 这些是一次性 expanding 容量测试进程峰值，不是长期运行节点的 bounded 稳态 RSS，也不是 C3 enabled/disabled A/B 结论。

## 正式 bounded 性能矩阵

同一 RelWithDebInfo 构建、同一 bounded 10 Hz workload 下，完成三种模式各 3 轮、每轮
300 秒：

```text
profiling-archive/c3-map-state-updates-20260808/raw/final-bounded-20260808-disabled-run1..3
profiling-archive/c3-map-state-updates-20260808/raw/final-bounded-20260808-enabled-run1..3
profiling-archive/c3-map-state-updates-20260808/raw/final-bounded-20260808-keyframe-only-run1..3
```

9 轮均正常结束并通过 raw analyzer、角色/计数/资源门。每轮包含 3,001 observations、
3,001-3,002 revisions；enabled 与 keyframe-only 均在 drain window 内收敛 latest revision。

| Mode | CPU 均值 | RSS 峰值 KiB | RSS 斜率 KiB/min |
| --- | ---: | ---: | ---: |
| disabled | 11.88-12.50% | 62,648-63,032 | 0-289.7 |
| enabled | 66.91-67.54% | 102,392-104,516 | 0-448.0 |
| keyframe-only | 76.04-77.50% | 102,728-104,472 | 520.3-866.6 |

三种 aggregate 均为 `suspected_sustained_growth=false`。keyframe-only 的三轮正斜率低于
当前 1,024 KiB/min 门，作为后续长时 soak 观察项保留，不表述为“绝对零增长”。enabled
相对 disabled 的 CPU 差异明显高于本机 30-50% 分辨率，可报告额外成本；keyframe-only
相对 enabled 约 15% 的差异只报告，不下优劣结论。

enabled 每轮发布约 3,803-3,805 个 MapUpdate，其中 30 个 keyframe、3,773-3,775 个
delta；materialize P95 约 54 ms，diff P95 约 2.6-2.8 ms。keyframe-only 每轮发布
3,804-3,805 个 keyframe、0 个 delta。

## 分析器来源

- 原始 9 轮证据、capture-time `analysis-summary.json` 和原始 `*-aggregate.json` 由
  run manifest/source diff 固定的采集期分析器生成；
- `*-aggregate-cpu.json` 是在不改动原始目录的前提下，用增加 pidstat CPU 汇总后的
  分析器重新计算；
- 当前 `scripts/lib/local_map_profile_analysis.py` SHA-256 为
  `66e6ff4f0eeeb862ddf6fc82fea3e3c3eb5387ac629825afed56ae6beb671cae`；
- 完整 provenance 与 aggregate hash 见 `validation/analysis-provenance.txt`。

## 需要注意的无效/观察项

- `asan-build.log`：首次 ASan 构建因 `--log-base` 参数位置错误立即失败；随后 `asan-build-rerun.log` 已成功，前者不要当测试失败。
- `asan-capacity.log`：首次容量命令因容器没有 `/usr/bin/time` 未启动；随后 `asan-capacity-rerun.log` 已成功。
- 首次 enabled 25 秒 expanding smoke 出现 5 个 revision skew，作为 invalid 原始证据
  保留；独立重跑通过，因此只记录为调度抖动观察项，不能单独作为稳定容量拐点。
- adapter/async producer 的 sanitizer/Memcheck 覆盖为 5 个定向用例；181 万 cell capacity
  harness 覆盖 core 路径，不把它描述为 adapter/async 大容量测试。

## 收口边界

最终 `trellis-check`、验证命令和 spec 更新均已完成。收口只提交本记录列出的 C3
profiling、文档、spec、父/子任务状态和小型验证摘要；不清理或回退父任务以外的脏改动，
不纳入 `.agents/.claude/.codex/.cursor`、Playwright 临时文件、其他 `profiling-archive`
或构建目录。C3 raw 后续只在上述本地 archive 维护；任务 validation 仅保留 Git 摘要与
迁移 provenance。

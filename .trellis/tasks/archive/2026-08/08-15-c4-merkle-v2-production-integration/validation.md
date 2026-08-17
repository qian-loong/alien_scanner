# C4.3 当前验证记录

日期：2026-08-16

分支：`phase/4-merkle-v2-production-integration`

容器：`alien-scanner-dev` / `alien-scanner-jazzy:latest`
验证构建：`build-c43-cross2` + `install-c43-cross2`；生产隔离构建：`build-c43-prod-off` + `install-c43-prod-off`

## 已通过

统一跨包构建（`perception_interfaces`、`perception_map_update`、
`perception_local_map`、`perception_profiling`、`swarm_data_interfaces`、
`swarm_data_plane`）：成功。

带 ROS 环境初始化的完整 CTest：

| 包 | 结果 |
| --- | --- |
| `perception_map_update` | 11/11 passed |
| `perception_local_map` | 8/8 passed |
| `perception_profiling` | 4/4 passed |
| `swarm_data_plane` | 9/9 passed |

合计 32 个测试全部通过，包含 Merkle/容量/receiver rejection、full-ray、closed-loop、
resource bounded/expanding/keyframe-replacement、routed resync、可视化和真实洞穴场景。
CTest 运行前显式加载了 `/opt/ros/jazzy/setup.bash` 与本次独立 install，避免容器默认
shell 未加载 `ament_cmake_test` 的环境误报。

`BUILD_TESTING=OFF` 生产隔离构建成功（11 个依赖闭包包）。在
`swarm_data_plane` 的 CMake target 与安装前缀中未发现 `c4_resource_profile_*`、
`C4ResourceProfile` 或测试 fixture 目标。

## Cadence 对齐后的 V1/V2 短对照

此前的短对照出现 V1 `2` 个、V2 `6` 个 keyframe。原因不是 V1 处理速度较慢，而是
V2 `MapUpdateProducer` 默认的 `max_delta_chain_length=128` 在约 331 条/来源的窗口内
触发了两次自动基线重建；冻结 V1 fixture 只发送每个来源的初始 keyframe。

本次只调整 V2 profiling fixture 的 `bounded` 模式：将其 producer 专用
`max_delta_chain_length` 设为 `0`（无限制），并在源摘要写入
`keyframe_policy=initial-only`。生产 `MapUpdateLimits` 默认的 128 没有修改；
`keyframe-replacement` 仍显式使用每 revision keyframe，其他 workload 保持原策略。
runner 现在校验摘要中的 keyframe policy，避免后续 A/B 又混入不同 cadence。

两次使用同一参数：`bounded`、`source_count=2`、`cells_per_source=10000`、
`delta_operations=256`、`rate_hz_per_source=10`、`qos_depth=4`、预热 3 秒、窗口 30 秒、
`domain_id=196`。原始证据目录：

- `profiling-archive/c4-merkle-v2-production-20260816/raw/short-v1-6865c08-cadence-aligned/`
- `profiling-archive/c4-merkle-v2-production-20260816/raw/short-v2-4033bd1-cadence-aligned/`

| 指标 | V1 frozen `6865c08` | V2 candidate `4033bd1` | 说明 |
| --- | ---: | ---: | --- |
| messages applied | 663 | 662 | 定时器边界相差 1 条，约 0.15% |
| keyframes applied | 2 | 2 | cadence 已一致 |
| receiver CPU (single-core %) | 1.200 | 0.767 | 记录值，主机竞争不确定度约 ±30% |
| PSS mean (KiB) | 20,344 | 21,452 | V2 +1,108 KiB（约 +5.4%） |
| USS mean (KiB) | 10,220 | 11,388 | V2 +1,168 KiB（约 +11.4%） |
| apply total (ms) | 251.704 | 101.623 | V2 短测下降，需正式矩阵复核 |
| flat canonical hash / Merkle total (ms) | 203.082 | 32.061 | 算法阶段下降；两边阶段语义不同 |
| candidate build total (ms) | 25.647 | 73.601 | V2 的 chunk/Merkle candidate 成本增加 |

V1/V2 的 payload 分别为 `4,737,693 B` 和 `4,731,282 B`，application CDR 分别为
`5,151,405 B` 和 `5,154,962 B`；这两个总量受 V2 少处理一条消息和 schema 字段影响，
不作为独立带宽结论。两边均 `valid=true`、无 rejection/duplicate，最终 known cells 均为
20,000。该短对照只能证明 cadence 已对齐并给出方向性筛选，仍不能替代正式 `3 x 300 s`
矩阵、sanitizer、Memcheck、Heaptrack 或 rollback Gate。

旧 v1 identity 字段扫描结果：

- `content_hash_duration_ns` 等旧 profiling 字段只存在于冻结 archive；
- `ContentHasher::content_hash` 只存在于 flat oracle、benchmark 和 correctness tests；
- production `perception_map_update/src`、`perception_local_map/src`、
  `swarm_data_plane/src` 没有 flat identity 读取。

## 正式资源 Gate 证据

### 3 x 300 秒 candidate v2 矩阵

有效证据目录：
`profiling-archive/c4-merkle-v2-production-20260816/raw/formal-v2-4033bd1-sources-2x100k-20260816-rerun/`。
首次使用 install symlink 的目录因 runner 要求 `/proc/cmdline[0]` 与真实 ELF 路径一致而
`valid=false`，已保留但不纳入 Gate。

三次运行均使用 `sources-2x100k`、2 个 source、每源 100,000 cells、每源 10 Hz、每次
256 个 delta operations、warmup 3 s、正式窗口 300 s，domain 230/231/232；candidate
build 为 `build-c43-v2-rel`，镜像为
`sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52`。
三次均 `valid=true`、`normal_completion=1`、无 rejection/duplicate/endpoint anomaly，
最终 known cells 为 200,000；receiver PID/starttime 唯一。摘要 runner SHA-256 为
`c8f7540b816f836e0eb24ac609eee357f937d08eae137ade42a38bcf96de33a9`。

| repetition | messages / keyframes / deltas | PSS mean (KiB) | USS mean (KiB) | PSS slope (KiB/min) | apply (ms) | Merkle (ms) | candidate build (ms) |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 6063 / 2 / 6061 | 42833.64 | 32952.64 | 10.57 | 1263.00 | 298.64 | 994.22 |
| 2 | 6063 / 2 / 6061 | 42777.92 | 32873.92 | 10.25 | 1260.88 | 295.34 | 994.75 |
| 3 | 6064 / 2 / 6062 | 42864.53 | 32979.53 | 82.28 | 1245.01 | 294.41 | 980.10 |

apply 占 callback 比例为 93.68%–95.23%，队列峰值为 0；三轮 source/receiver ELF 的
SHA-256 和 build-id 完全一致。PSS 斜率远低于当前 `1024 KiB/min` 的资源增长告警线，
但这不抵消内存工具的严格门结果。

### Sanitizer、Memcheck 和 Heaptrack

- C4 受影响包的 ASan/UBSan/LSan 精确重跑中，`perception_map_update`、`swarm_data_plane`
  全部通过；`perception_local_map` 只有 full-ray 在 LSan 开销下发生时序失败，未出现业务
  leak 报告。专用 v2 2x10k/30 s sanitizer smoke 为 `valid=true`、`sanitizer.gate_pass=true`、
  报告文件为空。2x100k 失败样本因 LSan overhead 只应用 keyframe，不能纳入 Gate。
- 有效 Memcheck 业务链为
  `raw/memcheck-v2-c43-1x100-10s/`：keyframe 1、delta 12、messages 13、无拒绝/非法访问、
  definitely lost 0 B、indirectly lost 0 B、possibly lost 768 B、still reachable 194097 B。
  768 B 栈来自第三方 `liblttng-ust` 初始化线程；原始严格 Memcheck `gate_pass=false`，
  但该 finding 已按下文的已知第三方例外单独归因，不计为 C4.3 业务泄漏。
- 有效 Heaptrack 证据为 `raw/heaptrack-v2-c43-2x10k-20s/`：parse/target verified/gate
  均为 true，peak heap 7.70 MiB、peak RSS（含工具开销）39.36 MiB、216261 allocation
  calls；Heaptrack 的 366356 B `total_memory_leaked` 不等同于 Memcheck definite leak，
  未发现业务 mismatch/rejection。

### 历史复现与回归判断

本次 `768 B possibly lost` 不是 C4.3 新引入的回归：

- 已归档的 C4 通信数据面 Gate 在 2026-08-10 的真实 ROS receiver 中记录了同样的
  `768 bytes possibly lost`，两个栈均为 `allocate_dtv -> pthread_create -> liblttng-ust`
  初始化线程 TLS，且没有经过 C3/C4 业务代码（见归档
  `08-10-c4-communication-data-plane/performance-memory-quality-gate.md` §8.4）。
- `profiling-archive/retest-20260802/memcheck/verdict.txt` 中的 C1、C2、C2-v2 和
  C2-scaled 四份日志也都记录 `possibly lost: 768 bytes in 2 blocks`。
- C2 专项归因记录进一步指出该 TLS 残留量与地图规模无关，属于 ROS/glibc/LTTng 退出期
  行为，不能据此分类为 C2 业务泄漏。该历史测试另有 rcutils/glibc 的 128 B definite
  finding，与当前 C4.3 的 768 B LTTng finding 分开统计，不能混为一个结论。

因此，当前证据支持“固定的第三方 runtime 残留、不是本次代码回归、未观察到 C4.3 业务
definite/indirect leak”。报告明确分列：`strict_memcheck_gate=false`、
`business_memory_gate=pass`、`business_memory_gate_exception=known-third-party-lttng-ust-768B`。
该例外只覆盖已复现且不经过业务栈的 768 B LTTng TLS finding，不覆盖任何业务 definite/indirect
leak、非法访问或未归因 finding。

### Rollback dry-run

在临时 worktree `.tmp-rollback-v1` 对 `4033bd1` 的父提交 `d479967` 做了隔离回退，使用
独立 `build-rollback-v1` / `install-rollback-v1` 前缀构建成功。四个受影响包的回退测试中，
`perception_map_update` 11/11、`perception_profiling` 4/4 通过；`swarm_data_plane` 首轮
9 个测试中 closed-loop 因启动时序失败，单测重跑 1/1 通过；`perception_local_map` 的
full-ray integration 在两次运行均报告既有 `no same-clock usable pose` / `mapper recovery
stability gate`，其余 7/8 通过。该结果证明 change set 可移除并恢复 v1 构建/测试入口，
但不能把 baseline 的 full-ray 时序问题写成全包回退测试通过。临时 worktree 已在记录后移除，
当前分支未执行回退。

## 最终结论

当前结论为：**v2-only 生产集成实现与自动化回归通过；正式生产资源 Gate 为 GO（含已知第三方
runtime 例外）**。3 x 300 秒矩阵正确、PSS 增长有界、业务 definite/indirect leak 和非法
访问均为零；唯一严格工具差异是已在 C1/C2/C4 历史证据中复现的 `liblttng-ust` 768 B
`possibly lost`。该 finding 保留在原始 Memcheck 结果中，不伪称严格工具 gate 通过，但按
明确的业务 Gate 例外不阻塞本次生产集成。v2-only 可作为当前分支生产默认，C5d `EdgeAggregator`
前置阻塞解除；失败 raw 样本和工具开销仍不进入 Gate 聚合。

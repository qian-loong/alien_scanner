# C1/C2 真实链路资源重测报告（2026-08-02/03）

> 任务：`.trellis/tasks/08-01-c1-c2-perception-perf-memory-retest`
> 方法论：[`performance-memory-testing-cookbook.md`](performance-memory-testing-cookbook.md)
> 数据：**全新采集**（源码 rev `95577cf`+工作区 diff，镜像 `6eb20770ab23`，
> 容器 Jazzy）。上轮任何测量数据未作为本轮证据。
> 原始证据：`profiling-archive/retest-20260802/`（每 run 含 raw、门记录、sha256）。

## 0. 两个核心问题的答案

**Q1 有没有内存泄漏？——两个维度均无业务泄漏，各带边界：**

- **经典泄漏（真实路径）**：无。ASan/LSan 真实 bag 单圈 + epoch 重置路径
  （2 份拼接）对 C1、C2 共 4 个 run 全部零报告；Memcheck 零访问/未初始化错误，
  definite lost 全部归因 glibc `dl-open`（128 B/64 B，与上轮同签名）。
  **阳性对照通过**：人造 4096 B 泄漏被 LSan 清点、栈帧命中、exit 23——
  检测流程自身可信（闭合上轮未验证缺口）。
- **无界增长（30 min 周期负载）**：周期谷值斜率 C2 **+38.3 KiB/min**、
  C1 **+32.0 KiB/min**，为业务阈值 1024 KiB/min 的 **3.7% / 3.1%**。
  锯齿形态完好（每 epoch 重置后回落）。
  **如实标注**：漂移统计上非零（灵敏度下限 32.2 / 6.6 KiB/min），两节点量级
  一致；Heaptrack 归因见 finding F1。更慢的泄漏需更长窗口才能排除。

**Q2 CPU 占用多少、瓶颈在哪？——（本机观测值 ±30%，仅排序级结论）**

真实链路 6 节点争用下（3 轮一致）：

| 节点 | 档 | CPU 均值% (3 轮) |
| --- | --- | --- |
| `cave_laser_scan`（scanner） | 夹具 | **3.9（最大户）** |
| `perception_local_map`（C2） | **产品** | **1.9** |
| `pose_gate` | 夹具 | 1.3 |
| `perception_input`（C1） | **产品** | **1.0** |
| `fake_odom` | 夹具 | 0.8 |
| `cave_publisher`（truth） | 夹具 | 0.1 |

产品档合计 ≈2.9%，夹具档 ≈6.3%——**该场景 CPU 大头在仿真侧，产品节点余量
巨大**。C2 callback p99 = 2.60 ms，占 10 Hz 周期预算的 ~2.6%。

## 1. 集成直采（3 轮 × 50 s，scene-record，全部 valid）

- **饱和度（显式呈现）**：10 Hz 下三级链路计数守恒 197/197/197（run1）、
  零丢帧、revision 连续无缺口；`观测数 − 末 revision = 3` 为恢复稳定门
  启动期合法消耗（实测 0–3 随竞态波动，对账容差 5）。
- **每节点内存（run1，KiB）**：

| 节点 | PSS 均值 | PSS 峰值 |
| --- | --- | --- |
| C2 | 17 759 | 18 379 |
| C1 | 12 907 | 12 912 |
| fake_odom | 12 570 | 12 587 |
| scanner | 12 195 | 12 199 |
| truth | 12 164 | 12 175 |
| pose_gate | 12 058 | 12 078 |
| **系统总计（PSS 求和，6 节点）** | **79 653** | **80 323** |
| 录制器（support，单独记账） | 38 697 | 38 782 |

- 50 s 窗口内的 PSS 斜率（C2 ~3.3 MB/min）是运动段建图的合法增长，
  **不是泄漏证据**；泄漏判定见 §4。
- **端到端延迟（记录型，不作判据）**：scan 戳 → state 落 bag，
  p50 = 104.7 ms，p99 = 106.3 ms，极差 <2 ms——由 pose gate 100 ms 调度
  周期主导的恒定传输时延。

## 2. 回放隔离基线（replay，全部 valid）

| 项 | C2 | C1 |
| --- | --- | --- |
| 等价核对 | 终局 epoch 1=1，revision **194↔194（零偏差）** | 录制输出 **195↔195（零偏差）** |
| 机制 | 终局 state 快照（窗口外单查） | 支援录制器计数（`bag-replay_out`） |

**C2 阶段延迟分解**（stage 探针构建，LTTng UST，194 callback 全捕获、
3904 事件零丢失、`gate_pass=true`）：

| 阶段 | p50 (µs) | p95 (µs) | p99 (µs) | max (µs) |
| --- | ---: | ---: | ---: | ---: |
| callback（总） | 1 718.7 | 2 401.4 | 2 600.4 | 2 648.6 |
| mapper_apply | 1 303.0 | 1 822.2 | 2 011.3 | 2 185.8 |
| snapshot_total | 301.1 | 578.3 | 680.7 | 800.8 |
| snapshot_serialization | 270.2 | 549.5 | 650.6 | 765.5 |
| state_publication | 27.9 | 48.1 | 53.5 | 56.3 |
| read_transaction | 1.7 | 2.8 | 3.4 | 17.4 |

`mapper_apply` 占 callback ~76%——真实洞穴几何下的主耗时段。同 run 内阶段
占比可信（cookbook §6）；绝对值带 ±30% 环境不确定度。

## 3. Sanitizer 路径覆盖（ASan install，全部 valid）

| Run | 输入 | 等价 | ASan/LSan 报告 |
| --- | --- | --- | --- |
| c2-asan | 真实 bag 单圈 | 194↔194 | **零** |
| c1-asan | 真实 bag 单圈 | 195↔195 | **零** |
| c2-asan-x2 | 2 份拼接（epoch 重置路径） | 周期门过 | **零** |
| c1-asan-x2 | 2 份拼接 | 390=2×195 精确 | **零** |
| **阳性对照** | 人造 4096 B 泄漏 shim | — | **CAUGHT**（exit 23，栈帧命中） |

ASan 下 `--rate 1.0` 即可跟上（等价零偏差），未动用降频。

**Memcheck**（rel 构建，`--rate 0.1`，裁剪单圈）：

| 节点 | 逻辑工作量 | 访问/未初始化错误 | definite lost | possibly lost |
| --- | --- | --- | --- | --- |
| C2 | revision 194（完整） | **0** | 128 B ×2 → glibc `dl-open resize_scopes` | 768 B → glibc `allocate_dtv`（TLS） |
| C1 | 195/195 | **0** | 64 B ×1 → 同上 | 768 B → 同上 |

全部归因基础设施，**无业务代码 loss**。

## 4. 泄漏长跑（70 周期 × 26 s ≈ 30 min/节点，rel 构建，全部 valid）

裸 `--loop` 不可行（时间戳高水位门全量拒收，实测 revision 冻结）；
使用重打戳拼接 bag（`scripts/restamp-concat-bag.py`，机制见 cookbook §4a）。

| 项 | C2 | C1 |
| --- | --- | --- |
| 周期门 | 终局 epoch **=70**（每周期恰一次重置）+ 末周期 revision 194=基准 | 输出 **13 650 = 70×195 精确** |
| 谷值斜率（去首尾 68 周期） | **+38.33 KiB/min**（stderr 16.08） | **+32.03 KiB/min**（stderr 3.31） |
| 灵敏度下限（2×stderr） | 32.2 KiB/min | 6.6 KiB/min |
| 对业务阈值 1024 | **3.7%** | **3.1%** |
| 30 min 累计漂移 | +997 KiB | +1 180 KiB |
| 锯齿均值（峰−谷） | 239 KiB | 99 KiB |

**结论表述**：在该周期负载下，两产品节点均无超过阈值的持续增长；存在统计上
可分辨的 ~35 KiB/min 缓慢漂移（见 F1）。比该灵敏度更慢的泄漏本轮不可排除。

## 5. Findings（均不阻塞核心结论，另行跟进）

- **F1 ~35 KiB/min 内存漂移与 OcTree 实例挂账**。Heaptrack 20 周期归因
  （epoch=20、revision 193 等价）：退出期仍持有 8.17 MB，其中 Fast DDS 池
  1.69 MB、**liboctomap 1.07 MB——含 `OcTree` 构造 ~35 KB × 17 个实例仍在
  持有（20 个 epoch）**。怀疑各 epoch 的树/快照被读事务持有链保留。
  LSan 判定全部可达（非经典泄漏）。建议：审查 epoch 退役后旧树的释放路径；
  若为设计内缓存，补文档并给出上界。
- **F2 `pose_timeout_s` 是契约成员，降频重放不得缩放**。它参与 mapper
  contract fingerprint——改动后录制的 HealthState 因指纹不匹配被
  "rejected upstream health contract/session" 全量拒绝（实测 revision=0）；
  且其语义在时间戳域，本就无需缩放。降频重放**只缩放到达域看门狗**：
  `sensor_timeout_s`、`health_timeout_s`、`pose_receive_timeout_s`、
  `map_freshness_s`（×降频倍数，v2 参数实测 194 完整提交）。
- **F3 录制器开销**：`ros2 bag record` PSS ≈ 38.7 MB，为链路中最大进程
  （单独记账，未计入被测节点）。带录制的 CPU 数字略高于无录制，干净数字
  以回放隔离轮为准。
- **F4 场景 CPU 由夹具主导**：scanner 3.9% 是产品节点 C2（1.9%）的 2 倍。
  评估产品容量时不得把夹具占用摊入。

## 6. 与真机的已知差异（不变项引用 design.md §8）

仿真传感器替代真实驱动；bag 回放时序对延迟分位数有影响（E2E 为记录型）；
pose gate 使 C1 面对已排序输入；无 PMU、CPU ±30% 污染（**环境属性，
非指标制约**——换目标硬件后同套脚本可做精确预算验收）。

## 7. 复现

```bash
# 全部驱动脚本存档于 profiling-archive/retest-20260802/drivers/
#（step0-builds.sh 三构建, step1-scene-runs.sh, step2-*.sh, step3-*.sh, step4-leak-runs.sh）
# 集成直采 3 轮（录制 + 7 节点采样）
bash scripts/profile-graph.sh scene-record <install-rel> <out> 50
# 回放隔离（C2 例）
ALIEN_GRAPH_REPLAY_BAG=<bag-all> ALIEN_GRAPH_REPLAY_TOPICS="obs pose health /tf /tf_static" \
ALIEN_GRAPH_REPLAY_NODES="perception_local_map:perception_local_map_node:cave_full_ray_local_map" \
ALIEN_GRAPH_REPLAY_PARAMS=<params> ALIEN_GRAPH_EQUIVALENCE_BASELINE=<terminal-state.txt> \
bash scripts/profile-graph.sh replay <install-rel> <out> 55
# 泄漏长跑素材与运行
python3 scripts/restamp-concat-bag.py <bag-all> <out-bag> --copies 70 --trim-s 25
ALIEN_GRAPH_CYCLE_COPIES=70 ALIEN_GRAPH_CYCLE_BASELINE_REVISION=194 ... \
bash scripts/profile-graph.sh replay-loop <install-rel> <out> 1800
```

## 8. AC 对照

| AC | 判定 | 依据 |
| --- | --- | --- |
| AC1 3 轮直采 + 7 节点 + 录制器采样 + raw/sha256 | **通过** | §1，`scene-run{1,2,3}` 全 valid |
| AC2 CPU 排序/内存画像/PSS 求和/饱和度/E2E 记录型 | **通过** | §0/§1 |
| AC3 回放隔离基线 + 等价 + C2 阶段分解 | **通过** | §2（等价零偏差、194 callback 零丢失） |
| AC4 sanitizer 真实路径 + 阳性对照 + 长跑双维度 + 灵敏度 | **通过** | §3/§4（阳性对照 CAUGHT；漂移如实标注） |
| AC5 表述纪律 + 真机差异 + 覆盖边界 | **通过** | 全文；§6 |

测量完成时间：2026-08-03。

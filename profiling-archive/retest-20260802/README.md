# retest-20260802 原始证据档案清单

> 报告：`docs/perception-real-chain-retest.md`（commit `1197b35`）
> 本目录**不只包含有效证据**——按门纪律，invalid run 的 raw 保留但不计入基线；
> 诊断/冒烟证据单独归类。每个条目的身份如下，不要混用。

## 一、正式有效证据（全部 `valid=true`，报告结论的依据）

| 目录 | 身份 | 报告章节 |
| --- | --- | --- |
| `scene-run1/2/3` | 集成直采 3 轮（CPU/内存画像 + bag-all 素材源） | §1 |
| `graph-summary.txt` | 三轮汇总（analyze-graph-profile.py 输出） | §0/§1 |
| `replay-c2` `replay-c1` | 回放隔离基线（等价零偏差） | §2 |
| `stage-c2` | C2 阶段延迟分解（194 callback 零丢失） | §2 |
| `c2-asan` `c1-asan` `c2-asan-x2` `c1-asan-x2` | Sanitizer 真实路径 + epoch 重置路径 | §3 |
| `positive-control` | 阳性对照（CAUGHT，exit 23） | §3 |
| `memcheck/`（其中 **c1 与 c2-v2** 为有效证据，见下） | Memcheck 双节点 | §3 |
| `leak-c2` `leak-c1` | 30 min 泄漏长跑（周期门全过） | §4 |
| `heaptrack-c2` | 漂移归因（20 周期，epoch=20/rev=193 等价） | §5 F1 |

## 二、invalid raw（按纪律保留，不计入任何结论）

| 目录/文件 | 为什么 invalid | 教训归档处 |
| --- | --- | --- |
| `scene-run{1,2,3}-invalid-gatecal` | 对账器 trailing allowance=1 过严（恢复稳定门合法消耗启动期 0–3 帧被误判）；修门后全新重录 3 轮 | reconcile-graph-bags.py 注释 |
| `memcheck/memcheck-c2.log` + `c2-node.log`（首跑） | 0.1 倍速下到达域看门狗全部超时 → fail-closed，revision=0，仅覆盖启动/关闭路径 | 报告 F2 |
| `memcheck/memcheck-c2-scaled.log` + `params-c2-rate01.yaml`（v1 缩放） | 错误地把契约成员 `pose_timeout_s` 一并缩放 → 契约指纹不匹配 → 全量拒收 | 报告 F2、design.md §4 |

memcheck 目录内**有效**件：`memcheck-c1.log`、`memcheck-c2-v2.log`、
`params-c2-rate01-v2.yaml`、`verdict.txt`（含全部尝试的时间顺序记录）。

## 三、诊断与冒烟证据（`diagnostics/`，方案可行性判定的依据，非资源基线）

| 目录 | 证明了什么 |
| --- | --- |
| `loop-smoke-a-20260802` | 裸 `--loop` 不可行：高水位门全量拒收（观测拒 350、位姿拒 1660、revision 冻结 173） |
| `loop-smoke-b-20260802` | 重打戳拼接可行：零拒收，两段各完整入图（173+172），边界位姿跳变触发 epoch 重置 |
| `memcheck-diag` | 首个 memcheck 失败的现场（health=2、active_sensor=0、诊断空） |
| `memcheck-scaling-bisect` | 参数二分全过程：`case-*.log`（8 个对照）+ `diag-only-*.log` + 4 份单变量参数文件，锁定 `pose_timeout_s` 为破坏源 |

## 四、脚本（`drivers/`）

**编排驱动**（本轮每一步的实际执行脚本，含失败后修正的最终版）：
`step0-builds.sh` ~ `step4-leak-runs.sh`、`loop-smoke-{a,b}.sh`、
`memcheck-diag.sh`、`step3-memcheck-c2-scaled.sh`、`restamp-concat.py`
（工具转正前的原型，正式版在仓库 `scripts/restamp-concat-bag.py`）。

**分析脚本**（报告数字的复算入口）：
- `cycle-trough-analysis.py` — §4 周期谷值斜率（用法见文件头）
- `stage-percentiles.py` — §2 阶段分位数表
- `e2e-latency-from-bag.py` — §1 端到端延迟

**正式工具链在仓库、不在本目录**（commit `1197b35`，各 run 的
`run-manifest.txt` 记录了运行时的 sha256 可比对）：
`scripts/profile-graph.sh`、`scripts/analyze-graph-profile.py`、
`scripts/restamp-concat-bag.py`、`scripts/reconcile-graph-bags.py`、
`scripts/resolve-graph-pids.py`、`scripts/lib/graph_*.py`。

## 五、不在本目录的内容

- v3 改版**之前的方案分析与讨论**：在任务工件
  （`.trellis/tasks/08-01-.../{prd,design,implement}.md` 的修订历史）与
  会话记录中，不属于测量证据。
- 上轮（07-25~07-31）的测量数据：与本轮无关（全新采集原则），
  其结论文档在 `docs/perception-resource-profiling.md`、
  `docs/local-map-resource-profiling.md`。
- 中间构建产物（build-rel/stage/asan）：在容器 `/tmp/alien-retest-20260802/`，
  体积大且可由 `drivers/step0-builds.sh` + provenance 复现，不入档案。

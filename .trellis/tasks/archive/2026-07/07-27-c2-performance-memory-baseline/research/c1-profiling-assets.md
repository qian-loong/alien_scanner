# Research: C1 profiling assets for the C2 baseline

- Query: 调查 C1 profiling 资产中可直接复用、应参数化和不应复制的部分，覆盖 runner、analyzer、测试、Docker/Compose、归档设计、当前工具和旧 raw evidence。
- Scope: internal（含当前开发容器的只读实测）
- Date: 2026-07-27

## Files Found

- `scripts/profile-perception.sh`：C1 九模式统一 runner；包含 PID/PGID、同窗 workload、profiler 控制、退出状态和报告 gate。
- `scripts/analyze-perception-profile.py`：三轮 300 秒 plain-sample 的 RSS/PSS/USS/CPU 复算器。
- `scripts/test_analyze_perception_profile.py`：analyzer 与 runner 内嵌 parser 的合成正反例测试。
- `scripts/perception-profile-mixed.yaml`：C1 两路 2D + 一路 3D 的固定 mixed workload。
- `scripts/perception-profile-asan.cmake`：独立 ASan/LSan build 的 CMake initial cache。
- `scripts/perception-profile-lsan.supp`：只匹配 Python 3.12 launch-testing 宿主栈的 LSan suppression。
- `.devcontainer/Dockerfile`：profiling 工具包安装入口。
- `.devcontainer/docker-compose.yml`：开发容器 capability 和 seccomp 配置。
- `docs/perception-resource-profiling.md`：C1 结果、复现流程、raw evidence 生命周期和兼容性说明。
- `.trellis/tasks/archive/2026-07/07-25-c1-perception-resource-profiling/{prd.md,design.md,implement.md}`：C1 profiling 的需求、PID/退出协议和最终复核记录。
- `.trellis/spec/backend/quality-guidelines.md`：已沉淀的 External Resource Profiling Contracts。
- `ws/src/alien_perception/perception_local_map/config/cave_full_ray_scene.yaml`：C2 已验收的 360-beam、10 Hz、20 秒、10 m FullRay 场景。
- `ws/src/alien_perception/perception_local_map/launch/cave_full_ray_scene.launch.py`：现有 C2 多进程正确性场景编排。
- `ws/src/alien_perception/perception_local_map/launch/cave_full_ray_scene_config.py`：场景 schema 与冻结值校验器。
- `ws/src/alien_perception/perception_local_map/src/PerceptionLocalMapNode.cpp`：C2 target 的 observation/apply/state/OctoMap 路径。
- `ws/src/alien_perception/perception_local_map/test/TestLocalMap.cpp`：现有单帧 `ResourceBaseline`，只适合作为正确性/数量级 oracle。

## Findings

### 1. 总结矩阵

| 资产/能力 | C2 处理 | 结论 |
| --- | --- | --- |
| PID/starttime/PGID 身份与隔离 | 直接复用核心协议，扩展 role registry | 算法正确，C2 只是角色更多 |
| perf/trace/Heaptrack/Valgrind/ASan/LSan 模式和退出 gate | 直接复用工具状态机 | 不应重写第二套工具语义 |
| provenance、双状态 `valid`/`normal_completion`、SHA 清单 | 直接复用并补强 C2 workload schema | 已进入项目 spec |
| `pidstat` + `smem` + `smaps_rollup` 采样 | 直接复用采样格式 | C2 analyzer 需关联 revision/map size |
| ASan initial cache、Python-host LSan suppression | 原样复用 | build package closure 改为 C2 全链路 |
| Dockerfile/Compose 工具与权限声明 | 文件无需复制；当前容器需重建/核验 | 运行实例尚未获得 `PERFMON` |
| C1 target/fixture/YAML/topic/sensor count | 不可复用语义；必须换成 C2 scenario adapter | 当前 runner 明确不是通用 runner |
| C1 analyzer 的三传感器 `27/9 Hz` gate 与固定 60 秒稳态窗 | 只复用解析/统计原语 | bounded/expanding 必须分开分析 |
| C1 runner/analyzer 整文件复制 | 禁止 | 会复制 73 KiB 状态机并形成两套不一致门禁 |
| 2026-07-25 C1 raw artifacts | 仅用于 C1 复算/兼容回归 | 不能作为 C2 target 的正式证据 |

### 2. 可直接复用的 runner 核心

1. **输入和输出安全门。** Runner 关闭 mode 集合、要求正整数时长、拒绝已存在输出目录（`scripts/profile-perception.sh:4`, `:20`, `:29`, `:37`）。C2 应保留“每次 run 新目录”的合同。
2. **进程身份与隔离。** `process_starttime`、`process_identity_matches`、`wait_for_process_identity`、`wait_for_isolated_pgid` 和 signal 前复核集中在 `scripts/profile-perception.sh:139`, `:151`, `:167`, `:189`, `:220`。C1 设计要求按工具分别解析 launcher/tool/tracee，而不是把 wrapper PID 当 target（归档 `design.md:139`；spec `quality-guidelines.md:301`）。
3. **正式窗口监控。** Runner 每 250 ms 检查必需 role 的 PID、`/proc/<pid>/stat` starttime 和 zombie，并监视 LTTng session（`scripts/profile-perception.sh:1080`, `:1125`, `:1147`）。这比重新写 wall-clock sleep 可靠，应直接保留。
4. **perf 同窗协议。** control FIFO/ACK 入口在 `scripts/profile-perception.sh:930`；正式窗口外层在 `:1830`，停止后再验证 ACK 包围关系在 `:1849`。冻结并复核 TID 集的逻辑在 `:956` 和 `:1190`。归档设计 `design.md:154` 与 spec `quality-guidelines.md:314` 已把该行为定为合同。
5. **tool-specific normal stop。** C1 明确区分 perf、Heaptrack、Valgrind/Sanitizer、ROS trace 的退出语义（归档 `design.md:152`）；runner 最终只在无强制停止和 role exit failure 时设置 `normal_completion=true`（`scripts/profile-perception.sh:1952`）。C2 不应重新解释 Memcheck finding exit code 或接受被强杀报告。
6. **工具报告 parser/gate。** perf lost samples/symbol quality、Heaptrack 完整汇总、Massif stack-aware peak、Memcheck target/summary 均已由 `generate_reports` 统一处理（`scripts/profile-perception.sh:1317`）。合成测试覆盖 lost samples（`scripts/test_analyze_perception_profile.py:185`）、Heaptrack peak（`:217`）、Massif stack peak（`:254`）和 Memcheck target command（`:293`）。
7. **provenance。** Runner 已记录 host-provided `sha256:<64 hex>` image ID、source revision/dirty count、runner/YAML/analyzer hash、kernel/RMW/affinity/cgroup/capabilities、target ELF SHA/build-id（`scripts/profile-perception.sh:399`, `:454`, `:485`），并保存 `git-status.txt`、`ldd`、ELF notes/dynamic section（`:489`）。C2 只需换 target/package/workload 内容，不应另建 manifest 格式。
8. **raw artifact integrity。** cleanup 对所有 raw files 生成 `sha256sum.txt`，并把 `valid` 与 `normal_completion` 分开写入 manifest（`scripts/profile-perception.sh:365`, `:372`）；这与 spec `quality-guidelines.md:323`、`:334` 一致。
9. **plain sampler。** `pidstat` 每秒、`smem`/`smaps_rollup` 每 10 秒的目标 PID 采样实现可直接复用（`scripts/profile-perception.sh:1033`）。

### 3. 必须参数化/替换的 C1 硬编码

1. **runner 目标边界完全写死。** 固定 YAML 是 `perception-profile-mixed.yaml`（`scripts/profile-perception.sh:44`）；固定 package prefix/ELF 是 `perception_input_node` 与 `perception_fixtures`（`:414`, `:426`）；fixture 只能启动一个 publisher（`:512`）；target 只接受同一 params file（`:639`）。C1 手册也明确声明该 runner 不是通用 ROS runner（`docs/perception-resource-profiling.md:240`）。C2 target 必须改为 `perception_local_map_node`，辅助角色至少包括 cave publisher、odom、scanner、pose gate、C1 input 和 TF。
2. **单 fixture PID 模型不够。** C1 正式窗口只监控 `fixture/tracee/sink` 加 profiler/sampler（`scripts/profile-perception.sh:1082`）。C2 现有 launch 包含 cave publisher、fake odom、scanner、pose gate、perception input、local map、static TF 和可选 RViz（`cave_full_ray_scene.launch.py:36`, `:51`, `:75`, `:98`, `:133`, `:163`, `:171`, `:189`）。C2 必须把所有必需辅助进程作为独立 role 记录和监控；只记录 launch parent PID 会漏掉 child crash。
3. **现有 monolithic launch 不能直接交给所有 profiler。** Heaptrack/Valgrind/Sanitizer 必须从 target 进程启动插桩，而 `cave_full_ray_scene.launch.py` 会自行启动 local-map target（`:163`）。C2 profiling 编排应能“启动辅助栈但不启动 target”，再由 runner 在工具下启动唯一 target，防止双 target 和 PID 歧义。
4. **参数/graph gate 写死 C1 节点和四 topic。** `wait_for_nodes` 只认两个 C1 节点（`scripts/profile-perception.sh:785`）；参数 dump 只检查 fixture/input（`:803`）；graph 只检查三输入加 observation output（`:868`）。C2 需要精确验证 observation、pose、health、state、OctoMap、diagnostics 及所有 producer/subscriber cardinality，并拒绝同 domain 外来 endpoint。
5. **sink 和 workload parser 不能复用语义。** C1 sink 只 echo `LidarObservation.sensor_id`（`scripts/profile-perception.sh:539`），并强制出现 front/rear/top（`:896`）；parser 固定三个 ID 和 `duration*27/9`（`:1275`, `:1291`, `:1304`）。C2 应至少同时证明唯一 observation stamp、10 Hz 输入、distinct revision、单一 epoch、fingerprint/diagnostics、known bounds/map size；map state heartbeat 的重复 revision 不能冒充 applied throughput。
6. **C1 YAML 不可复制为 C2 workload。** 其固定值是 front/rear/top、`requires_pose=false` 和 C1 mixed topics（`scripts/perception-profile-mixed.yaml:1`, `:32`, `:34`）。C2 已验收场景是单个 pose-gated FullRay sensor、360 beam、10 Hz、seed 42、0.2 m resolution（`cave_full_ray_scene.yaml:20`, `:43`, `:51`, `:66`, `:80`）。
7. **现有 C2 场景只能作为 oracle/seed。** Loader 强制 10 m/20 s 直线和冻结 360-beam geometry（`cave_full_ray_scene_config.py:49`, `:53`, `:62`）。因此它不能直接覆盖 300-1500 秒 bounded/expanding 正式窗口；需要 profiling-specific scenario schema，且保持相同 beam/range/resolution/seed 合同。
8. **bounded 与 expanding 需要不同 gate。** C2 PRD 要求 bounded 在 known voxel 收敛后判断 RSS/PSS/heap，expanding 要关联 revision/known/bounds 与内存增量（当前任务 `prd.md:33`）。不能把“地图正常扩张”交给 C1 的统一 1 MiB/min leak 阈值。

### 4. Analyzer 和测试的复用边界

- 可复用原语：`parse_manifest`（`scripts/analyze-perception-profile.py:11`）、线性斜率（`:20`）、percentile（`:36`）、smem/smaps block 解析（`:42`）、pidstat 解析（`:83`）、三目录/三 evidence identity 校验（`:165`, `:172`）。
- 必须参数化：稳态起点固定为 `t0 + 60 s`（`:46`），run mode 必须是 `plain-sample` 且至少 300 秒（`:111`），workload 固定 front/rear/top 与 `27/9 Hz`（`:124`, `:129`），最终只给统一 `1024 KiB/min` sustained-growth 判断（`:201`）。C2 bounded 应依据“map-size 已收敛”的机器证据选择稳态段；expanding 应回归 `memory ~ known_voxel/revision/bounds`，而不是应用 leak boolean。
- C2 还需要与 state/OctoMap 采样按 monotonic timestamp 对齐，输出 revision rate、distinct revision、map epoch、known/free/occupied、bounds 体积和每新增 known voxel 的内存成本。现有 `LocalMapState` 只暴露 epoch/revision、known bounds、fingerprint 和 last changed-cell count（`PerceptionLocalMapNode.cpp:615`, `:627`, `:636`, `:655`），known/free/occupied 总数需要从 OctoMap snapshot 或专用 profiling collector 复算，不能从 heartbeat 行数猜测。
- 现有测试并非独立 parser module 测试：`run_embedded_parser` 通过字符串 anchor 从 Bash heredoc 抽取 Python（`scripts/test_analyze_perception_profile.py:96`）。若抽取 shared runner/parser，必须同步把这些 parser 变成可导入/可调用单元，避免复制 heredoc 和复制测试 fixture。
- 当前测试已经覆盖三 run distinct path/evidence（`:146`, `:151`）、低 workload（`:134`）和 incomplete pidstat（`:117`）。C2 应保留这些负例，并新增 bounded 未收敛、expanding map 未增长、重复 revision/stamp、epoch 变化、fingerprint/diagnostic 错误、collector 缺样、role child crash。

### 5. ASan/LSan cache

- `scripts/perception-profile-asan.cmake:1` 到 `:15` 只设置通用 C/C++ RelWithDebInfo、frame pointer 和 address sanitizer 编译/链接选项，没有 C1 package 名或 target 名，可直接用于 C2 独立 sanitizer prefix。
- 变化只在 colcon package closure：C2 target 与辅助 workload packages 必须全部来自该独立 install；不能把日常 install 或 profiling install 混入。
- `scripts/perception-profile-lsan.supp:1` 只 suppress `python3.12` launch-testing host。归档设计限定它不适用于独立 C++ target/fixture（归档 `design.md:227`）。C2 使用同类 Python test host 时可直接复用；若新 collector 是 Python，也不能因此全局 suppress target 或 collector 的业务栈。
- Sanitizer sink 的 `libasan:libstdc++` preload 处理已存在（`scripts/profile-perception.sh:543`），可复用于任何会加载 ASan 化 ROS message library 的 Python host。

### 6. Docker/Compose 与当前工具实测

- Dockerfile 已声明 `babeltrace2`、Heaptrack、linux-tools、smem、sysstat、Valgrind 和 `ros-jazzy-ros2trace`（`.devcontainer/Dockerfile:8` 到 `:16`, `:32`）；Compose 已声明最小 `PERFMON`、`SYS_PTRACE` 和 `seccomp:unconfined`（`.devcontainer/docker-compose.yml:31`）。C2 不应再复制 Dockerfile 或扩大为 privileged。
- 2026-07-27 live container：22 CPUs，kernel `6.10.14-linuxkit`，`perf_event_paranoid=2`；image 为 `sha256:76321bde799cb94b4e38ce61feb9836b968b2ff875ae0899fa518ea154e77a13`。
- 当前运行实例的 Docker inspect 只有 `CAP_SYS_PTRACE`，没有 Compose 文件里已声明的 `CAP_PERFMON`；这表明它是未按当前 Compose 重建的旧实例。正式 C2 运行前必须重建并再次记录 capability，但先保护旧 `/tmp` evidence。
- `/usr/bin/perf` wrapper 因 host kernel 版本不匹配而报错；runner 的 fallback 找到 `/usr/lib/linux-tools/6.8.0-136-generic/perf`（对应 `scripts/profile-perception.sh:439`）。直接 `perf 6.8.12 stat -e task-clock:u true` smoke 成功；硬件 PMU 仍不可假定可用。
- 已实测工具：Heaptrack/heaptrack_print 1.5.0、Valgrind 3.22.0、sysstat/pidstat 12.6.1、smem 1.5、LTTng 2.13.11、Babeltrace 2.0.5、`ros-jazzy-ros2trace` package 8.2.6。`ros2 trace --list` 能列出 35 个 UST event；本次没有启动正式 trace session。

### 7. 旧 raw evidence 的可用范围

- 当前 live container 中 `/tmp/alien-perception-profile-20260725-c1-03` 仍存在，含 C1 正式/探针/故障注入 runs；`29a/29b/29c-plain-300` 均有 `sha256sum.txt`。
- 只读运行当前 `scripts/analyze-perception-profile.py` 成功复算 29a/29b/29c，三 run evidence identity 不同，结果仍为 `suspected_sustained_growth=false`。这证明现有 analyzer 对 C1 raw 的兼容性，应作为重构回归 oracle。
- 它们不能进入 C2 baseline：target ELF、source revision、workload、topics、map metrics 和 runner version 都不同。C1 最终复核也明确说明加固 runner 后没有重跑正式 profiler，旧数值只按旧 raw 已有字段交叉核对（归档 `implement.md:81`；`docs/perception-resource-profiling.md:245`）。
- `/tmp` 不是 bind mount/volume；重建容器可能永久丢失旧 raw（`docs/perception-resource-profiling.md:251`, `:490`）。如果要保留 C1 回归 oracle，应在 C2 rebuild 前按 `sha256sum.txt` 导出到仓库外，不能复制进 C2 run 目录或 Git。

### 8. 不应复制，建议文件面

**不应复制：**

- 不复制整份 `profile-perception.sh` 为 C2 runner。该文件约 73 KiB，复制会让 PID、退出、perf 和 parser gate 演化成两套。
- 不复制 `perception-profile-mixed.yaml`；它表达的是 C1 三 sensor workload，不是通用模板。
- 不复制旧 raw 或 manifest 作为新 run；每次 C2 run 必须由新的 target PID/ELF/t0/t1 产生。
- 不复制 analyzer 中的 smem/pidstat parser；抽为共享 Python helper或由 C2 analyzer import。

**建议文件面（供后续设计，不是本研究中的实施）：**

- 保留 `scripts/profile-perception.sh` 的 C1 四参数兼容入口和当前行为。
- 从 runner 抽出 `scripts/lib/profile-runner-common.sh`（命名可调整）：PID/PGID、role monitor、provenance、perf/trace/tool lifecycle、raw hashing 和 report gates。
- 新建薄的 `scripts/profile-local-map.sh`：只定义 C2 target、辅助 role 启停、scenario、graph/parameter/workload collector gate，并调用 shared core。
- 将内嵌工具 parser 抽成可导入的 `scripts/profile-report-parsers.py`，让 C1/C2 runner 和合成测试共用；否则 `test_analyze_perception_profile.py:96` 的字符串 anchor 会阻碍安全重构。
- 保留 `scripts/analyze-perception-profile.py` 作为 C1 compatibility wrapper；新建 `scripts/analyze-local-map-profile.py`，复用 shared memory/pidstat/statistics helper，分别实现 bounded convergence 与 expanding attribution。
- 新增两个 profiling-only workload manifests（bounded/expanding）和一个 machine-readable collector。Collector 必须同窗记录 observation、state、OctoMap/diagnostics，并输出可由 analyzer 重算的数据；不要把这些 profiling 字段加入 C2 public ROS interface。
- C2 辅助 stack 应有 profiling-specific launch/adapter，能关闭 target 和 RViz；target 始终由 runner/tool 启动。现有 `cave_full_ray_scene.launch.py` 继续作为 20 秒正确性 oracle，不改造成长时 profiler launch。
- `scripts/perception-profile-asan.cmake`、`scripts/perception-profile-lsan.supp`、`.devcontainer/Dockerfile` 和 `.devcontainer/docker-compose.yml` 预计无需内容变更，只需在 C2 build/run 文档中复用并实测。
- 新增 C2 baseline 文档，不覆盖 `docs/perception-resource-profiling.md` 的 C1 结果和复现说明。

### 9. 关键风险

1. **AC3 不能仅靠 C1 外部 trace 达成。** C1 trace 只给 callback/take/publish 聚合（归档 `design.md:200`）；C2 PRD 要分别给 mapper apply、state publication、read transaction 和 OctoMap snapshot 分位数（当前 `prd.md:43`）。这些内部阶段位于 `PerceptionLocalMapNode.cpp:534`, `:542`, `:545`, `:570`, `:598`。后续设计必须明确非语义 instrumentation/tracepoint 方案；仅参数化 runner 会产生虚假满足。
2. **多进程辅助栈的退出语义更复杂。** 20 秒正确性场景中的 trajectory/producer 会自然结束，正式 300-1500 秒窗口若角色提前退出必须 invalid，而不是把停止后的 heartbeat 当 workload。
3. **map-size 归因数据源未完全现成。** `LocalMapState` 有 bounds/revision，但没有 known/free/occupied 总数；OctoMap snapshot 解析成本本身又是 C2 要测的 visualization-only 路径。Collector 必须区分 target serialization cost 与外部解析 cost。
4. **dirty provenance 仍可能歧义。** Runner 记录 dirty count/path（`scripts/profile-perception.sh:458`, `:489`），但正式 baseline 若在 dirty tree 上构建，单靠 path 不足以重建源码。后续设计应选择“formal run 必须 clean”或保存完整 source tree/patch hash，不能只写 `dirty_count`。
5. **当前容器 capability 与仓库配置不一致。** software event smoke 可用不等于 perf record/control 全矩阵已通过；重建后仍需按 C1 规则重新 smoke。

## External References

- 本研究未依赖网页资料；版本和能力均来自 2026-07-27 当前 `alien-scanner-dev` 的只读命令输出以及仓库冻结文档。
- Ubuntu packages（live container）：`babeltrace2 2.0.5-3build2`、`heaptrack 1.5.0+dfsg1-2ubuntu3`、`linux-tools-generic 6.8.0-136.136`、`smem 1.5-2`、`sysstat 12.6.1-2`、`valgrind 3.22.0-0ubuntu3`、`ros-jazzy-ros2trace 8.2.6-1noble.20260615.173133`。

## Related Specs

- `.trellis/spec/backend/quality-guidelines.md:279`：External Resource Profiling Contracts 的 scope。
- `.trellis/spec/backend/quality-guidelines.md:287`：当前 C1 runner/analyzer signature。
- `.trellis/spec/backend/quality-guidelines.md:297`：provenance、role PID、同窗 workload、perf ACK、normal completion、artifact completeness 合同。
- `.trellis/spec/backend/quality-guidelines.md:369`：runner fault injection 和 parser regression 的最低测试矩阵。
- `.trellis/spec/backend/local-observation-map-contract.md:35`：C2 epoch/revision 语义。
- `.trellis/spec/backend/quality-guidelines.md:233`：性能 baseline 必须使用超过 100 个 unique frame，并分开 conversion/health timing；C2 collector 同理不能计重复 heartbeat/stamp。

## Caveats / Not Found

- `python ./.trellis/scripts/task.py current --source` 在本研究 session 返回 `Current task: (none)`；输出路径来自父任务显式指定的 `.trellis/tasks/07-27-c2-performance-memory-baseline`，未修改 active-task 状态。
- Trellis researcher 禁止 Git 操作，因此本研究没有运行 `git status`，不能权威分类 dirty files。只读目录观察到根目录的 `tmp_precut_work.py`、`tmp_precut_work_out.txt`、`.tmp-bag-analysis/`；它们不在 `.gitignore:1` 到 `:25` 的明确规则中，属于潜在无关工作文件，所有权未知，后续不得自动加入 C2 提交。`rosbags/` 则由 `.gitignore:6` 明确忽略。
- 本次只执行了 `bash -n scripts/profile-perception.sh` 和旧 raw 的只读 analyzer 复算。未运行 `scripts/test_analyze_perception_profile.py`，因为该测试把临时目录创建在仓库根（`scripts/test_analyze_perception_profile.py:19`），超出 researcher 仅可写当前 task `research/` 的权限。
- 未执行任何 C2 build、profiler、ASan/LSan、Heaptrack、Valgrind 或长时 workload；当前工具结论仅表示 availability/smoke，不是 C2 baseline 通过证据。

## 2026-07-27 Execution Preflight

- Source baseline: branch `phase/4-perception-swarm-refactor`, commit
  `06a5d7fe66f13316fa76eba41daf58f5694509eb`; the dirty worktree was kept intact.
- Live container image:
  `sha256:76321bde799cb94b4e38ce61feb9836b968b2ff875ae0899fa518ea154e77a13`.
  `docker inspect` still reports only `CAP_SYS_PTRACE`, `Privileged=false`;
  `/sys/kernel/tracing/uprobe_events` is unavailable. Formal stage probes remain blocked
  until the container is rebuilt from the current Compose file and re-smoked.
- The C1 report evidence runs `21-perf-stat-600` through `28-massif-180`, plus the
  three `29a`/`29b`/`29c-plain-300` compatibility runs, were copied without deleting
  the originals to `D:\WorkDir\alien-scanner-profile-raw\c1-20260725` outside the
  repository. Linux FIFO control files were intentionally not copied because they are
  runtime IPC objects and are not members of any checksum manifest.
- An independent read-only container mounted each of the eleven copied run directories at
  its original absolute path. Every `sha256sum -c sha256sum.txt` invocation exited zero and
  reported all listed artifacts as `OK`. The invalid sample-count conclusion recorded for
  `22-perf-record-1200` is preserved as evidence history; export integrity does not make that
  run a valid baseline.
- Copied checksum-manifest SHA-256 values:
  - `29a-plain-300`: `f58b6442a5331649b0d1aefbfb6d2b47d63c359abdf93bb56e68346e356b0f88`
  - `29b-plain-300`: `57054af9c3eae7e4eec2822e0d7625410c503e9bc531f71907603dc5fb82560f`
  - `29c-plain-300`: `cc5491f0d0d17162bdbc905e742291ff8debe54972ed43edd4133ca68a9a0627`

## 2026-07-27 Rebuilt Environment And RelWithDebInfo Closure

- The current `alien-scanner-jazzy:latest` image and live container both resolve to
  `sha256:e688730d49c37f4d5bc4b58f85701022dffa335280fcc254806945abae9873d1`.
  `docker inspect` reports `CAP_PERFMON`, `CAP_SYS_PTRACE`,
  `seccomp:unconfined`, and `Privileged=false`; PID 1 exposes both capabilities.
- The live kernel remains `6.10.14-linuxkit` with `perf_event_paranoid=2`. The installed
  fallback `/usr/lib/linux-tools/6.8.0-136-generic/perf` is version `6.8.12` and its
  `task-clock:u` smoke succeeds. Tool versions rechecked in the rebuilt container are
  LTTng `2.13.11`, Babeltrace `2.0.5`, Heaptrack `1.5.0`, Valgrind `3.22.0`, and
  sysstat/pidstat `12.6.1`; the image also retains `smem` and `ros2trace`.
- External LTTng ELF probing was retried against the installed
  `PerceptionLocalMapNode::on_observation` symbol using the kernel-domain
  `--userspace-probe=elf:<target>:<mangled-symbol>` form. Event enable failed with
  `Kernel tracer not available`; `/sys/kernel/tracing/uprobe_events` is absent. The
  temporary LTTng session was destroyed normally. This is an environment capability
  result, not permission to substitute ROS callback tracing for internal stage timing;
  the design's default-OFF UST fallback is required.
- Independent performance prefix: `/tmp/alien-c2-relwithdebinfo-20260727`. Its target
  `perception_local_map_node` has SHA-256
  `f02968d03ce72f00e52bb11cef5633695cea480762db83ce295189d6b4b7ab70`, GNU build ID
  `030edef5070342648ed32144d43d1a519140cd40`, and retained `.debug_info` plus `.symtab`.
  Compile command inspection confirmed `-O2 -g -DNDEBUG -fno-omit-frame-pointer`.
- The complete nine-package closure test in that prefix finished normally. Recomputing
  the stored xUnit results with `colcon test-result --all --verbose` reports exactly
  `266 tests, 0 errors, 0 failures, 0 skipped`. This is the RelWithDebInfo functional
  closure result only; it does not cover the still-pending latency and sanitizer prefixes.

## 2026-07-27 LTTng UST Stage Capture Diagnosis

- The residual `c2-stage-all-20260727` session was stopped and destroyed normally before
  further testing. Its per-CPU stream files were zero bytes while the session was active,
  but normal finalization flushed them. Babeltrace then parsed 1321 events: 1291 UST
  statedump records and 30 real `perception_local_map_stage` records (10 mapper-apply
  begin/end pairs and 5 read-transaction begin/end pairs). No `ros2:*` event was present.
  An active trace file size is therefore not an emptiness gate; evidence is parsed only
  after normal stop/destroy.
- `TRACEPOINT_PROBE_DYNAMIC_LINKAGE` was not the failure boundary. The provider and all
  emission calls are centralized in `libperception_local_map_stage_latency.so`; the target
  has one dynamic dependency on that library. A provider-specific wildcard session
  recorded a `TestLocalMap` mapper/read pair, and a real node-only run recorded 25
  state-publication pairs. The provider was both registered and enabled in those runs.
- The failed runner configured a per-user channel with 8 MiB sub-buffers and 8 sub-buffers
  per CPU. On the live 22-CPU container that requests about 1,476,395,008 bytes before
  control/metadata overhead, while `/dev/shm` is a 67,108,864-byte Docker tmpfs. Repeating
  the real-node run with that exact channel produced only a 4096-byte metadata file and
  zero events; changing only the channel to 512 KiB x 4 produced real streams/events.
- `sessiond not accepting connections to local apps socket` is an informational UST debug
  message from the optional per-user sessiond listener. The root/global sessiond was alive,
  `/var/run/lttng/lttng-ust-sock-8` existed with mode 0666, and the same process registered
  with the global daemon. It was not the capture failure.
- The focused fix uses 256 KiB x 4 for the low-rate stage provider (about 1 MiB per allowed
  CPU) and reserves an explicit 8 MiB `/dev/shm` margin. Before session creation the runner
  records and checks `allowed_cpu_count * subbuf_size * num_subbuf + margin`; insufficient
  headroom invalidates the run before a metadata-only session can be mistaken for evidence.
- The first real runner smoke after the channel fix completed every role and the LTTng
  stop/destroy path normally. Its headroom record was 22 CPUs, 262144-byte sub-buffers,
  4 sub-buffers, an 8388608-byte margin, 31457280 bytes required, and 62701568 bytes
  available. The resulting CPU-0 stream was 1110016 bytes and contained 21711 target stage
  events with zero discarded events. The run was initially invalid only because the runner
  converted CTF with `--fields=all --names=all`, which emits `timestamp = ...`, while the
  pairing parser intentionally accepts the default `[seconds.fraction]` pretty contract.
- Re-converting that unchanged CTF with `babeltrace2 convert --clock-seconds` and rerunning
  the parser produced 221 complete applied callbacks and 1326 samples: 221 each for callback,
  mapper apply, state publication, read transaction, snapshot serialization, and snapshot
  total. It reported zero in-window unmatched entries/returns, nesting mismatches, incomplete
  callbacks, duplicates, invalid durations, and lost events; `gate_pass=true`. The runner now
  uses that exact conversion command and retains a separate detail-format file for diagnosis.
  The post-processing correction was first validated against the already normally finalized
  CTF artifact, then the complete runner was repeated in a new directory to close the
  end-to-end gate.
- The second end-to-end smoke used image
  `sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52`,
  target SHA-256 `fa04068f3e55362a41029b624e4c2bcce07aa2748f6da46cee01cd8650b0e903`,
  and build ID `abceefec42aab355910c06736f61f17121faf1cf`. It reached warmup revision
  801, recorded a 22-second formal window, and exited with `valid=true`,
  `normal_completion=true`, `forced_stop=false`, and no remaining LTTng session. The trace
  contained 21708 target events, 220 complete applied callbacks, and 1320 stage samples
  (220 for each required stage), with zero lost events, unmatched entries/returns, nesting
  mismatches, incomplete callbacks, duplicate samples, or invalid durations.

## 2026-07-27 ASan Checkpoint Alignment Diagnosis

- The sanitizer prefix `/tmp/alien-c2-asan-20260727` completed the full nine-package
  closure with `266 tests, 0 errors, 0 failures, 0 skipped` under
  `detect_leaks=0`, the ASan/`libstdc++` preload, and the sequential single-worker
  constraint. The canonical launch fixture now delays only its scanner by two seconds;
  the shared scene launch retains a zero-delay default, so the production pose gate and
  normal launch behavior are unchanged.
- The first 60-second bounded ASan runner directory,
  `/tmp/alien-c2-asan-smoke-20260727`, reached a normal role shutdown and a complete
  601-observation/601-revision formal window. It was nevertheless marked invalid because
  the checkpoint sampler started after warmup revision 802, read all historical rows in
  `states.csv`, and attached current memory samples to historical revisions 100 through
  800. The resulting revision-100 state/resource skew was about 70 seconds.
- The corrected sampler records its monotonic start boundary and ignores state rows whose
  receipt predates that boundary. The analyzer treats a memory checkpoint as formal only
  when both its state receipt and resource receipt lie in `[t0, t1]`, while still requiring
  every oracle-backed `revision % 100 == 0` checkpoint inside the formal revision range.
  Thus pre-window rows cannot trigger a false skew, and filtering them cannot hide a
  genuinely missing formal checkpoint.
- A temporary `/tmp` copy removed only the cleanup-appended `valid=false` for diagnosis;
  the original manifest remained unchanged with SHA-256
  `8f86730479efe4d5f6749d3985b8a38d416cbc5206c368ba364673a98a06f315`.
  The corrected analyzer then validated the underlying `revision 803..1403` window with
  six real memory checkpoints (900 through 1400) and 59 resource samples. This proves the
  checkpoint fix against the preserved raw data, but the original run remains invalid and
  must not count as ASan evidence. A new 60-second ASan run with the corrected sampler is
  required before the independent LSan run.

## 2026-07-27 Valid ASan And LSan Evidence

- The corrected ASan run is
  `/tmp/alien-c2-asan-smoke-checkpointfix2-20260727`. It used target SHA-256
  `02a67716ab8ec7d1a8f466905e7ed5f60579ed01e0a78063c237c8195a668dbc` and build ID
  `69f24e474daa6ef3063f4d35e58718a4c9624c10`, recorded a 60.092310587-second formal
  window with 601 observations, 602 contiguous revisions, 601 snapshots, six exact memory
  checkpoints, and 59 resource samples. Every role exited normally; the final manifest is
  `valid=true`, `normal_completion=true`, `forced_stop=false`, and
  `role_exit_failure=false`. No ASan error report was emitted.
- The independent LSan run is
  `/tmp/alien-c2-lsan-smoke-checkpointfix-20260727`, against the same sanitizer ELF and
  frozen workload but a fresh target PID/session/window. It recorded 60.092093042 seconds,
  601 observations/revisions/snapshots, six exact memory checkpoints, and 59 resource
  samples. Its final validity and role-exit gates are all clean. The only `lsan.42068` text
  is the requested thread-processing log; it contains no `ERROR: LeakSanitizer` or
  `SUMMARY: LeakSanitizer` section and the target exited zero.
- These runs close the sanitizer closure/workload portion of implementation step 5. They
  do not close the remaining runner-mode smokes/fault injection or the formal
  long-duration matrices.

## 2026-07-27 Current Production-Like Closure

- A fresh nine-package production-like closure was built sequentially at
  `/tmp/alien-c2-relwithdebinfo-current-20260727` with `MAKEFLAGS=-j1`,
  `CMAKE_BUILD_PARALLEL_LEVEL=1`, colcon's sequential executor and one worker, using
  `scripts/perception-profile-relwithdebinfo.cmake`. All nine packages finished in about
  489 seconds. Package caches for workspace dependencies resolve to this prefix.
- The installed `perception_local_map_node` SHA-256 is
  `48ddeaa99cd02c73d474d048092a550be1db0def808b0f68d7e47daa3b32e1da`; its GNU build ID
  is `ad6d08ff2f180f56e1ba2c86276d1caf66f73759`. The exact compile command contains
  `-O2 -g -DNDEBUG -fno-omit-frame-pointer`, and the ELF retains `.debug_info` and
  `.symtab`. The sourced-prefix `ldd` has no missing or sanitizer libraries, `nm -C`
  retains `PerceptionLocalMapNode::on_observation`, and the custom
  `perception_local_map_stage` dependency/symbol is absent.
- `PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS:BOOL=OFF` is recorded in the
  package CMake cache. Generic LTTng libraries still appear transitively through ROS 2
  `libtracetools`; they are not the profiling-only stage provider and are not evidence
  that the option is enabled. Full `ldd`, `nm -C`, compile command, ELF sections/dynamic
  metadata, option value, and hashes are under the prefix's `provenance/`; that directory's
  checksum-manifest SHA-256 is
  `7eff4e64a7a8f6a4e0d73379e996d1f15b9da4babf1fab94046decf3310b66c0`.
- The first full test invocation accidentally used colcon's default install base, so its
  command logs injected `/workspaces/alien-scanner/ws/install`; despite passing, it is not
  counted as independent closure evidence. The complete test was rerun from `env -i`,
  sourcing only `/opt/ros/jazzy` and the new `/tmp` install and explicitly passing the new
  `--install-base`. Its logs contain zero main-workspace install hits, all nine package
  install bases point to the new prefix, and clean-environment `ldd` resolves workspace
  interface libraries there. The final machine result is exactly
  `266 tests, 0 errors, 0 failures, 0 skipped`.

## 2026-07-27 Plain And Perf Stat Runner Smokes

- The bounded plain smoke at `/tmp/alien-c2-smoke-plain-20260727` completed a
  10.091877419-second formal window with 101 observations, 101 revisions, 100 snapshots,
  one exact memory checkpoint, and 10 resource samples. Its final manifest is
  `valid=true`, `normal_completion=true`, `forced_stop=false`, and
  `role_exit_failure=false`; every recorded role exited normally and no role PID remained.
- The first bounded perf-stat attempt is preserved unchanged at
  `/tmp/alien-c2-smoke-perf-stat-20260727`. Its enable, disable, and stop commands all
  received ACKs and the software-event report itself passed the parser gate, but the run is
  correctly `valid=false`, `normal_completion=false`, and `forced_stop=true`. The runner
  waited for attached `perf -t <TIDs>` to exit after the stop ACK while the target was still
  alive; perf in turn waited for the attached target, creating a lifecycle deadlock. The
  runner eventually forced the tool down and recorded
  `INVALID: perf did not finalize after controlled stop`. This raw directory remains
  invalid evidence and was not overwritten or repaired in place.
- Both the C1 and C2 wrappers now retain the disable and stop ACKs, close the control FIFO,
  verify perf PID/starttime/PGID through `signal_process`, send a planned SIGINT, and then
  wait/reap. Exit codes 0 and 130 are accepted; any other code sets both `VALID=false` and
  `ROLE_EXIT_FAILURE=true`. Only a post-SIGINT timeout sets `forced_stop=true` and escalates
  through TERM/KILL before reaping. `wait_child` is the sole normal-path exit-code writer,
  so a planned code is recorded once. A standalone perf 6.8 lifecycle experiment produced
  all three ACKs, exit code 130, and parseable task-clock/context-switch rows. Shell syntax
  and both 17-test Python suites passed after the repair.
- The new bounded perf-stat smoke is
  `/tmp/alien-c2-smoke-perf-stat-sigintfix-20260727`. It used the current production-like
  target SHA-256 `48ddeaa99cd02c73d474d048092a550be1db0def808b0f68d7e47daa3b32e1da`
  and build ID `ad6d08ff2f180f56e1ba2c86276d1caf66f73759`. The formal window was
  10.140940232 seconds with 101 observations, revisions 807 through 908, 101 snapshots,
  one exact checkpoint, and 10 resource samples. Enable occurred before `t0`; disable and
  stop ACKs occurred after `t1`; `perf-window-quality.txt` and
  `perf-stat-quality.txt` both report `gate_pass=true`.
- The repaired perf process exited 130 after planned SIGINT and that code appears exactly
  once in `exit-codes.txt`. The final manifest is `valid=true`,
  `normal_completion=true`, `forced_stop=false`, `role_exit_failure=false`, and
  `script_exit_code=0`. All software events were supported with 100% running time; hardware
  PMU events remained explicitly `not_supported`. The C2 analyzer accepted the raw data,
  every checksum verifies, and all six recorded role PIDs were absent after cleanup. The
  checksum-manifest SHA-256 is
  `8033741173a2193c06c86d4c0b1257ff12c09b5669434a83b0e7e7db398fff08`.
- The bounded perf-record smoke at
  `/tmp/alien-c2-smoke-perf-record-sigintfix-20260727` exercised the same repaired
  lifecycle for a 20.104037057-second formal window. It produced 201 observations,
  revisions 808 through 1009, 201 snapshots, two exact checkpoints, and 20 resource
  samples. All three control commands received ACKs in the required order around the
  window, perf exited 130 after the planned SIGINT, and the final manifest is
  `valid=true`, `normal_completion=true`, `forced_stop=false`, and
  `role_exit_failure=false`.
- The 2.042 MiB `perf.data` is parseable and contains 9555 samples with zero lost samples,
  14.767138% unknown samples, and six workspace samples. The target build ID
  `ad6d08ff2f180f56e1ba2c86276d1caf66f73759` is present in the build-ID list, and the
  symbolized report includes workspace symbols such as
  `PerceptionLocalMap::OctoMapBackend::known_bounds() const`. The short-window parser sets
  `gate_enforced=false` and `gate_pass=true`; this proves the tool/report pipeline only,
  not the formal `>=120 s` and `>=1000` symbol-quality acceptance gate. The analyzer and
  all checksums pass, every recorded role PID is absent after cleanup, the `perf.data`
  SHA-256 is `232286f69fd5cae638eeaeabbc4274a0d500fdb3408d651194d8d52127f73f93`,
  and the checksum-manifest SHA-256 is
  `2eef727711e3c867ca7cf90b25aa44970b9defb8367d1437b18041a2864ca79d`.
- The bounded ROS trace smoke at `/tmp/alien-c2-smoke-ros-trace-20260727` completed a
  10.102942692-second formal window with 101 observations/revisions/snapshots and one exact
  checkpoint. The session `alien-local-map-198-68112` was active before the window and at
  every one-second monitor sample. It stopped normally, left no available LTTng recording
  session, and produced non-empty CTF metadata and CPU stream/index files.
- The target-vpid trace conversion is parseable and reports 1108 callback, 1006 take, and
  1308 publish events in the formal window. The final manifest is `valid=true`,
  `normal_completion=true`, `forced_stop=false`, and `role_exit_failure=false`; the C2
  analyzer and all checksums pass, and every recorded role PID is absent after cleanup. The
  checksum-manifest SHA-256 is
  `3b86879e89665844d5bcda8d4c072168cdc78fd3da5d68139b8db1fc1eb4a94f`.
- The first bounded Heaptrack attempt is preserved at
  `/tmp/alien-c2-smoke-heaptrack-20260727`. It failed before the workload because the
  runner required `$!` to expose a `heaptrack` ELF. A minimal Heaptrack 1.5.0 process-tree
  experiment at `/tmp/alien-heaptrack-tree-SiT5H7` proved the real model: the isolated PGID
  leader is `/usr/bin/dash` executing `/usr/bin/heaptrack`, with three same-PGID children
  (`heaptrack_interpret`, `gzip`, and one exact target ELF). The old startup cleanup then
  produced secondary identity/finalization findings because no tracee identity had been
  recorded. That directory remains `valid=false`, `normal_completion=false`, and
  `forced_stop=true` and is not counted as evidence.
- C1 and C2 now search the launcher-rooted tree, including the root itself, for exactly one
  target ELF. An exec-in-place target is accepted directly; a descendant target additionally
  requires the launcher PID/starttime/PGID and exact NUL-separated `heaptrack` and target
  argv, plus the same isolated PGID. The manifest records the resulting process model,
  launcher executable, launcher cmdline, and tracee identity. An incomplete startup uses a
  separate verified-launcher TERM/timeout-KILL/reap path rather than applying normal tracee
  semantics to an identity that was never established.
- `/tmp/alien-c2-smoke-heaptrack-identityfix-20260727` proved that identity repair through a
  normally completed 10.097-second formal window and complete Heaptrack report, but remains
  `valid=false`, `normal_completion=true`. Its sole invalid reason is the old
  `heaptrack_print -M <raw>` command: Heaptrack 1.5.0 interprets the argument after `-M` as
  the output path and then reports the required input file missing. The raw directory was
  not edited. Running the corrected `-f <raw> -M <separate-output>` externally against that
  gz produced 8337 snapshots through 93.331 seconds and passed the new dedicated timeline
  parser.
- Both wrappers now write the Heaptrack Massif-compatible timeline to a separate file and
  validate contiguous snapshots, monotonic seconds, target command, positive peak, and
  requested-window coverage. The shared parser has synthetic positive/negative coverage;
  shell syntax, C1's 19 tests, and C2's 17 tests pass with the identity and timeline fixes.
- The valid bounded Heaptrack smoke is
  `/tmp/alien-c2-smoke-heaptrack-massiffix-20260727`. Its launcher PID 79136 is the verified
  `dash` wrapper, target PID 79158 is the unique exact-ELF descendant, both use PGID 79136,
  and both exit zero. The 10.099377232-second formal window contains 101 observations,
  revisions 803 through 904, 101 snapshots, one exact checkpoint, and 10 resource samples.
  The final manifest is `valid=true`, `normal_completion=true`, `forced_stop=false`, and
  `role_exit_failure=false`.
- The 1,224,105-byte Heaptrack gz and text report pass with total runtime 93.39 seconds,
  4,921,705 allocations, 34,969 temporary allocations, 28.96 MiB peak heap, and 70.90 MiB
  peak RSS including profiler overhead. The independent 1,573,526-byte timeline has 8317
  contiguous snapshots, final time 93.393 seconds, a 28,959,994-byte peak, target command
  verification, and `gate_pass=true`. The analyzer and every checksum pass; all six recorded
  role PIDs are absent after cleanup. The gz SHA-256 is
  `9fe90bbcd533df08cfc62679b22552c166a4ac7db2483a8a0603b8526b618d5d`, the timeline
  SHA-256 is `b3912b07c331277084b526ed1e7e7dda588d0de68e70178e0e1d86a34bd3f029`,
  and the checksum-manifest SHA-256 is
  `836f4a7411c37ff94a1b85d8024c8d232884add87b1a1d4d6fcc33248b251f7d`.
- The bounded Valgrind Massif smoke at
  `/tmp/alien-c2-smoke-valgrind-massif-20260727` correctly failed before the formal window:
  the fixed warmup deadline required revision 802, but the target reached only revision 103
  while the sink received 1834 observations and state sequence reached 792. The target's
  last committed observation stamp was 186.8 seconds while input had reached 193.3 seconds,
  directly exposing backlog rather than an empty workload. The sole invalid reason is
  `workload did not reach its required warmup revision`; the manifest is `valid=false`,
  `normal_completion=false`, `forced_stop=false`, and `role_exit_failure=false`, with
  Valgrind/target, fixture, and sink all exiting zero.
- The preserved 8,779,862-byte raw Massif artifact is independently valid tool output but
  is diagnostic only. An external `ms_print` plus the unchanged Massif parser found 120
  complete stack-aware snapshots, final time 184457 ms, a 33,223,072-byte peak at 182185 ms,
  the exact target command, and `gate_pass=true`. The raw SHA-256 is
  `c1c88c94b1a1ab6040fab21d74d00d0139dd7945abc1613032334228e18565d8`; the original run
  directory and manifest were not edited. This is a Valgrind perturbation candidate finding:
  the tool cannot maintain the frozen 10 Hz workload in the current configuration. The
  evidence does not justify lowering the rate, reducing beams, relaxing warmup, or counting
  the raw timeline as a valid C2 smoke/formal baseline.
- These are short runner/toolchain smokes only. They do not close the implementation-plan
  item requiring a valid smoke and fault injection for every mode, and none of their
  measurements are part of the formal long-duration baseline.

## 2026-07-27 Memcheck Throughput And Runtime Findings

- The bounded Memcheck attempt is preserved unchanged at
  `/tmp/alien-c2-smoke-valgrind-memcheck-20260727`. The frozen fixture delivered 1825
  unique observations with no duplicates, ordering errors, sequence gaps, schema errors,
  or digest mismatches, but the target reached only revision 231 and state sequence 995
  before the fixed warmup deadline required revision 800. Its only runner invalid reason
  is `workload did not reach its required warmup revision`; the run remains
  `valid=false`, `normal_completion=false`, `forced_stop=false`, and
  `role_exit_failure=false`. The fixture and sink exited zero, while Valgrind and the
  target exited with the configured finding code 42.
- The complete 316,912-byte raw Memcheck log has SHA-256
  `2a8eebf2786718d96141b41d83f341493d19eb80cadfb017780a0bb380948e3e`. It reports
  128 bytes definitely lost in two blocks, zero indirectly lost, 384 bytes possibly lost
  in one block, and 211,047 bytes still reachable. `ERROR SUMMARY` is one context and
  there is no invalid read, write, free, or other memory-access error.
- The unchanged shared parser was run offline against the exact production-like target
  ELF. It wrote only new diagnostic files outside the original run:
  `/tmp/alien-c2-memcheck-diagnosis-20260727.summary.txt` and
  `/tmp/alien-c2-memcheck-diagnosis-20260727.quality.txt`, with SHA-256 values
  `03a6562c62b46b7c40adee4a3b7830a90ad86be231f0deee7e28b03ca3fb9a92` and
  `cbb75654d51956d8d1b12592d72199671ad3aae645f67985dde34635bc7b3b56`.
  The quality output confirms `target_verified=true`, `finding=true`,
  `invalid_access=false`, and `other_error=false`.
- The definite-loss stack is glibc loader state from `resize_scopes` /
  `dl_open_worker_begin`, reached through `dlopen` and
  `rcutils_load_shared_library`. The possible-loss stack is glibc pthread TLS allocation
  (`allocate_dtv` / `_dl_allocate_tls`) for a thread created by `liblttng-ust`. Neither
  stack establishes ownership by the C2 mapper or occupancy code. They remain a
  runtime/upstream attribution finding rather than a declared C2 business leak.
- Historical PIDs 89489, 89520, and 89554 were each checked explicitly after parsing and
  were absent from `/proc`. The offline parser therefore did not depend on or leave a live
  target, Valgrind process, fixture, or sink.
- Two non-started child findings now own the unresolved boundaries:
  `.trellis/tasks/07-27-c2-valgrind-throughput-blocker` preserves the frozen workload and
  investigates why Massif/Memcheck cannot keep up; and
  `.trellis/tasks/07-27-c2-memcheck-runtime-leak` isolates the rcutils/glibc and LTTng
  findings with minimal ROS controls or narrowly justified upstream suppressions. These
  findings do not make either invalid Valgrind run a valid smoke or formal baseline.

## 2026-07-28 Stage Callback Event Set And Smoke

- `profile-local-map.sh` now accepts an optional sixth argument for `stage-latency` only:
  `callback` or `full`. Omitting it preserves the previous CLI as `full`; non-stage modes
  reject a sixth event-set argument. The selected value is written as
  `stage_event_set` in the run manifest. `callback` creates exact LTTng event rules only
  for `callback_begin` and `callback_end`; `full` creates exact rules for all existing 12
  events instead of relying on a provider wildcard.
- Provider availability and session enablement remain separate gates. The runner always
  requires all 12 events in `lttng list --userspace`, then checks both the active and final
  session evidence against the exact selected set. A callback session containing any
  mapper/state/read/snapshot event rule is rejected.
- `stage_latency_analysis.py` writes `event_set` to its quality file and uses the selected
  stage set for complete-callback, duplicate, duration, nesting, unmatched, and output
  checks. It additionally reports and rejects `unexpected_event_set_events`. The C2
  analyzer requires manifest and quality event sets to agree and emits only callback
  quantiles for callback evidence. Old full raw remains re-readable: a missing manifest
  or quality event-set field defaults to `full`, and the new unexpected-event counter
  defaults to zero only for old quality files.
- The existing latency prefix was reused without rebuilding. Its target SHA-256 is
  `fa04068f3e55362a41029b624e4c2bcce07aa2748f6da46cee01cd8650b0e903`, build ID is
  `abceefec42aab355910c06736f61f17121faf1cf`, the stage CMake option remains ON, and the
  compile database retains `-O2 -g -DNDEBUG -fno-omit-frame-pointer`. Every current CMake,
  stage instrumentation, mapper, bridge, and node source timestamp predates the installed
  ELF; the intervening changes are runner/parser assets rather than C2 stage business code.
- The callback-only smoke is
  `/tmp/alien-c2-stage-callback-smoke-20260728`. It recorded a 10.099798483-second formal
  window with 101 observations, 102 contiguous revisions, 101 snapshots, and 101 complete
  applied callback samples. The latency CSV has 101 data rows and only the `callback`
  stage; its p50/p95/p99/max values are 18,475,950 / 22,291,939 / 26,191,234 /
  29,679,039 ns. These short-run timings are toolchain smoke data, not calibration or
  formal baseline numbers.
- The final LTTng session lists exactly callback begin/end event rules, while the provider
  availability artifact contains all 12 events. Quality reports `lost_events=0`, zero
  unmatched entries/returns, nesting mismatches, incomplete callbacks, duplicate samples,
  invalid durations, and unexpected event-set events; `gate_pass=true`. The C2 analyzer
  accepted the run with `latency.event_set=callback` and no non-callback metrics.
- The final manifest is `valid=true`, `normal_completion=true`, `forced_stop=false`,
  `role_exit_failure=false`, and `script_exit_code=0`. Launcher/tracee PID 97739, sink PID
  97784, fixture PID 97806, pidstat PID 1102, and sampler PID 1123 were all absent after
  cleanup, and `lttng list` reported no remaining session. Every run checksum verifies;
  the checksum-manifest SHA-256 is
  `08b2b0b503fab95ea44e9028028b7ca0b70125b7fe3257bdb48fe658d9c8e533`.
- Container regression after the event-set change passed Bash syntax and Python compile
  checks, C1's 19 tests, and C2's 18 tests. The required unprobed/callback/full runs of at
  least 120 seconds remain intentionally pending; this 10-second smoke does not close or
  alter that implementation-plan checkbox.

## 2026-07-28 Stage Calibration Aggregator

- `scripts/analyze-local-map-stage-calibration.py` and
  `scripts/lib/stage_latency_calibration.py` now consume exactly three ordered roles:
  unprobed `plain-sample`, callback-only `stage-latency`, and full nested
  `stage-latency`. Each input first passes the existing C2 analyzer, then the aggregator
  requires bounded workload, at least 120 seconds, and at least 1200 complete callback
  samples in each probed run.
- The CPU gate is a bidirectional absolute percentage-point comparison, not a one-sided
  slowdown check: `abs(full - unprobed) <= max(abs(unprobed) * 0.05, 0.5)`. The callback
  p50/p95/p99 gates likewise use bidirectional absolute differences:
  `abs(full - callback_only) <= max(abs(callback_only) * 0.10, 50000 ns)`.
- Calibration provenance fails closed on source inputs, workload/config hashes, RMW,
  CPU assignment, role PID/starttime/PGID, target build identity, and independent run
  identities. The production target must record the stage option OFF; callback/full must
  use the same stage-enabled SHA/build ID and a distinct install prefix from production.
  `pidstat` must contain only the tracee PID at a one-second interval and bracket the
  formal window on both sides within two seconds.
- The CLI writes machine-readable JSON plus key/value quality output. A structurally
  invalid run exits through argparse with code 2; complete evidence outside a disturbance
  threshold writes both artifacts and exits 1. Synthetic positive and negative coverage
  now passes all 25 C2 analyzer tests, including the 1200-sample fixture's required middle
  memory checkpoint.
- The runner now resolves `install_prefix`, infers the corresponding package build tree
  (or accepts an explicit build base), parses its real `CMakeCache.txt` and
  `compile_commands.json`, and requires the build-tree target GNU build ID to equal the
  installed target build ID. It records verified `RelWithDebInfo`, the mode-appropriate
  O2 or sanitizer flags, and the actual stage option. Direct validation passed the existing
  production O2/OFF, latency O2/ON, and sanitizer O1/OFF prefixes.
- After the tracee PID/starttime/PGID stabilizes, the runner reads normalized
  `Cpus_allowed_list` from `/proc/<pid>/status`, requires the target's actual list to be
  `0`, and records it as `target_affinity`. `pidstat` now starts only after profiler/session
  preparation; its monotonic start is recorded after PID identity confirmation. Its stop
  timestamp is recorded immediately after formal `t1` and before signaling pidstat. The
  runner independently enforces the same two-second bracket and one-second interval later
  recomputed by the calibration analyzer.
- Untracked-source archives now use sorted names, zeroed mtime/ownership, GNU format, and
  `gzip -n`. Rebuilding the real smoke archive from its unchanged file list reproduced
  SHA-256 `28105c1aeb2c33a608a38284c24a74f1134b41b044acbd8963e1a6a08dff621b`
  exactly, so three sequential calibration runs can compare this field without archive
  metadata noise.
- The non-baseline runner smoke is `/tmp/alien-c2-provenance-smoke-20260728`. Although the
  initiating client timed out after five seconds, the isolated runner continued and
  finalized through its own state machine. Its 10.094742722-second formal window recorded
  101 observations, 102 revisions, normalized target affinity `0`, and pidstat start/stop
  margins of 14,299,733 ns and 14,404,078 ns. It ended `valid=true`,
  `normal_completion=true`, `script_exit_code=0`; every artifact checksum passed, no role
  PID remained, and the checksum-manifest SHA-256 is
  `83e1e138e142a4ddca1e08f6f709ea3c63367c84ee88ebf456aada2255f3b52a`.
- Regression now passes C2's 28 tests and C1's 19 tests in the container, plus shell syntax
  and Python compile checks. At that point no 120-second unprobed/callback/full calibration
  run had started; the later runs and their rejected comparison are recorded below.

## 2026-07-28 Formal Stage Calibration Evidence Rejection

### Preserved evidence and formal result

- This attribution is offline-only. It did not rerun any workload, rebuild an ELF, relax a
  threshold, or change C2 production semantics. The preserved run directories are
  `/tmp/alien-c2-calibration-unprobed-20260728`,
  `/tmp/alien-c2-calibration-callback-20260728`, and
  `/tmp/alien-c2-calibration-full-20260728`; all three `sha256sum.txt` manifests verify.
  Their checksum-manifest SHA-256 values are respectively
  `05228132a251cb7d254170074972c020080cc9c84115b5509bc2a7fc2e72287d`,
  `2db8673a823579c04668008a6673c52c120712f55aabaecead4aaaad80dcdaa4`, and
  `a85cc5f0f45441e03bb527e9dff83ffdd1c49fd718d0e2207ce4239035675afa`.
  The aggregate JSON is `/tmp/alien-c2-stage-calibration-20260728.json`, SHA-256
  `40e9d7a9444d8a9a57b98649d0ac018183b07865bfcbae8d4e5ba9a0185884ff`.
- Each run is recorded as `valid=true`, `normal_completion=true`, `forced_stop=false`, and
  `role_exit_failure=false` by the current per-run analyzer. Each formal window is about
  120.099 seconds with 1201 unique observations, 1202 revisions, and 120 target-only
  one-second `pidstat` samples. The pidstat start/stop margins are 14-16 ms, all target
  affinities are CPU 0, helper affinity is CPU 1, and no workload/backlog or role-
  finalization gate failed. This per-run validity does not make the cross-build calibration
  comparison admissible; the analyzer did not yet validate transitive dependency identity.

| Role | target CPU mean | `%usr` mean | `%system` mean | CPU population stdev | target RSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| unprobed | 35.6230% | 35.3730% | 0.2500% | 0.9173 pp | 62,308 KiB |
| callback-only | 37.7041% | 37.3793% | 0.3248% | 1.8115 pp | 91,084 KiB |
| full nested | 37.7203% | 37.4538% | 0.2665% | 1.9398 pp | 90,884 KiB |

- Full versus unprobed is `+2.097333 pp`, or `+5.887582%` relative. The frozen threshold
  is `max(35.623 * 5%, 0.5 pp) = 1.781150 pp`, so the existing aggregator emits
  `gate_pass=false`. The measured difference is not rounded into a pass, but it is also not
  accepted as a formal probe-overhead failure because the required same-closure A/B
  prerequisite is false.
- Callback-only and full each contain 1201 complete applied callbacks. Their callback
  p50/p95/p99 values differ by `425927 / 530038 / 1231744 ns`, or
  `2.2294% / 2.2228% / 4.5638%`, and all three frozen callback gates pass. Callback-only
  parsed 4020 target events while full parsed 42326, yet their target CPU means differ by
  only `0.016250 pp` (`0.0431%` of callback-only). Both traces report zero lost events,
  unmatched entry/return pairs, nesting mismatches, incomplete callbacks, duplicate stage
  samples, invalid durations, and unexpected event-set events. The ten additional internal
  event kinds in the full session are therefore not the observed 2.097 pp difference.

### Per-second distribution and order uncertainty

| Role | CPU p5/p25/p50/p75/p95 | Four consecutive 30 s means | first/second 60 s | slope per minute | lag-1 |
| --- | --- | --- | --- | ---: | ---: |
| unprobed | 34.00 / 35.00 / 36.00 / 36.00 / 37.00 | 35.298 / 35.697 / 36.265 / 35.232 | 35.498 / 35.748 | +0.017 pp | 0.050 |
| callback-only | 35.00 / 36.00 / 37.62 / 39.00 / 41.00 | 38.563 / 37.429 / 36.629 / 38.196 | 37.996 / 37.413 | -0.383 pp | 0.372 |
| full nested | 35.00 / 36.63 / 37.00 / 39.00 / 41.05 | 37.363 / 37.097 / 39.725 / 36.697 | 37.230 / 38.211 | +0.242 pp | 0.553 |

- The stage-enabled distributions are elevated across the window rather than by one
  isolated spike, and the increase is almost entirely user CPU. However, the 30-second
  blocks move by up to about 3 pp, the two stage runs have materially higher variance and
  serial correlation than unprobed, and their slopes have opposite signs.
- The fixed execution order was unprobed around 16:55 UTC, callback-only around 17:01, and
  full around 17:07. The evidence records target affinity and cgroup limits, but not CPU
  frequency, thermal/throttling state, host-wide CPU/load/steal, or a repeated randomized
  or counterbalanced order. One sequential observation per role cannot estimate this
  environmental/order component.

### Confirmed build-closure invalidity

- All three runtime manifests agree on source revision `06a5d7fe66f13316fa76eba41daf58f5694509eb`,
  dirty diff SHA, untracked-list SHA, reproducible untracked archive SHA, runner/helper/
  analyzer/workload hashes, image/kernel/RMW, CPU assignment, and workload parameters.
  Callback-only and full use the same stage target SHA/build ID
  `fa04068f...` / `abceefec...`; unprobed uses production target
  `48ddeaa9...` / `ad6d08ff...`.
- Both target packages report `RelWithDebInfo` and
  `-O2 -g -DNDEBUG -fno-omit-frame-pointer`. The expected target-level differences are the
  stage option `OFF` versus `ON`, the stage macro on instrumented translation units, the
  two provider translation units, and the provider library link. Relevant target source
  mtimes precede both target builds and have not changed afterward, which supports but does
  not cryptographically prove a common target-source snapshot.
- The comparison is not a same-build full-closure A/B. Production resolves
  `perception_core` and `perception_interfaces` from its independent
  `/tmp/alien-c2-relwithdebinfo-current-20260727/install` closure. The stage build's CMake
  cache, compile database, link command, and runtime `ldd` instead resolve them from
  `/workspaces/alien-scanner/ws/install`.
- In particular, the stage target statically links the workspace `perception_core` built
  as `Release -O3 -DNDEBUG`, while production links a
  `RelWithDebInfo -O2 -g -DNDEBUG -fno-omit-frame-pointer` copy. Their archive SHA-256
  values are `c13b32...` and `17cdf6...`, their sizes are 144,602 and 4,158,378 bytes, and
  normalized relocation-aware disassembly hashes differ. The generated
  `perception_interfaces` shared objects also have different whole-file hashes, although
  the checked typesupport `.text` section is identical. Thus stage enablement is not the
  only changed build variable. This is a confirmed evidence-contract failure, not merely a
  possible source of statistical noise.
- Runtime worktree hashes describe the tree at run time, not the exact inputs used when
  either older prefix was built. Source/object/ELF timestamp ordering makes a source
  content mismatch less likely, but the current manifest has no build-time source bundle
  digest or dependency-closure content attestation. It cannot close that gap after the
  fact.

### Bayesian decision and break-loop result

- The discriminating build evidence is predicted by H3 (non-equivalent closure), but not
  by a clean H1 (stage fixed overhead) versus H2 (host/order variance) experiment. H3 is
  therefore confirmed as an **evidence-validity defect**. It does not prove that the
  `Release -O3` dependency caused any particular portion of the 2.097 pp observation; it
  proves that the experiment cannot allocate that observation to the probe.
- H1 and H2 remain unidentifiable from these runs. Callback/full equality strongly rejects
  the narrower claim that recording the ten extra internal event kinds caused the delta,
  while the fixed order and missing host telemetry leave environmental variance open.
  Assigning normalized cause probabilities after discovering a perfectly confounded build
  variable would add false precision, so no probe-overhead posterior is used for action.
- H4 (pidstat/parser error) is strongly disfavored: independent raw recomputation exactly
  matches the aggregator, and PID, interval, bracket, sample count/range, and checksums all
  pass. The defect is the missing transitive build-provenance gate, not arithmetic.
- Primary root-cause categories are **B, cross-layer contract** and **E, implicit
  assumption**, with a **D, test coverage gap**: runner/analyzer provenance stopped at the
  target package instead of the linked workspace closure, and synthetic tests did not
  reject a stage prefix that silently consumed a different underlay.
- The formal calibration checkbox remains open. The original threshold is unchanged, the
  three raw runs remain preserved, and neither their old `valid=true` fields nor the
  aggregate `gate_pass=false` may be cited as formal probe-overhead evidence. No formal
  300-second stage-latency baseline may proceed from this comparison.
- Follow-up is isolated in the planning finding
  `.trellis/tasks/07-28-c2-stage-calibration-build-equivalence`. It must first add
  transitive dependency provenance and rejection tests, then rebuild the stage overlay on
  the production profiling prefix so all non-stage dependencies are identical. Only after
  that gate passes may the unchanged calibration be rerun. A paired build that still
  exceeds the frozen CPU gate would be preserved as a new valid failure and would require
  separate overhead/order attribution before instrumentation changes.

## 2026-07-28 Paired Build Equivalence And Short Smoke Gate

- The child finding used fresh pair root
  `/tmp/alien-c2-stage-pair-20260728-v3` in image
  `sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52`.
  Its source revision remained `06a5d7fe66f13316fa76eba41daf58f5694509eb` before,
  between, and after both builds; the canonical paired source identity is
  `c9feb80814bf7b5cbfbb0f5a69480da235d5dbd1af7fac53e022382c9fc3ffe9`.
- The OFF production nine-package closure was built from an `env -i` ROS-only underlay
  with the sequential executor and one worker. The ON overlay then selected only
  `perception_local_map` over that production underlay. Isolated test reruns report
  `266 tests, 0 errors, 0 failures` for production and
  `47 tests, 0 errors, 0 failures` for the stage package. No stage build/install artifact
  for `perception_profiling` exists, and the main-workspace-install scan has zero hits.
- Production and stage closure JSON are byte-identical with SHA-256
  `bdca0207bf89fecdc884a8d3564eb05779923c8ee3b15e5e851b4cf847d0f495`.
  The dependency comparison digest is
  `c1c369a937e175c11f0c0317791e2bb813c30f24b798849fad5185a236c2e54e`, and the helper
  set digest is `14509d238d4f39bfc02a59977f4991d6d136966423724ac558007442edb04b73`.
  Live verification independently rehashed 11 `perception_core` artifacts,
  98 `perception_interfaces` artifacts, all three helper ELFs, and the workload YAML.
- Three fresh 10-second runs completed sequentially at
  `/tmp/alien-c2-paired-smoke-unprobed-20260728-v3`,
  `/tmp/alien-c2-paired-smoke-callback-20260728-v3`, and
  `/tmp/alien-c2-paired-smoke-full-20260728-v3`. Every run is `valid=true`,
  `normal_completion=true`, has the frozen approximately 10 Hz workload without backlog
  or rejection, and passes its complete raw checksum manifest. Callback/full use the same
  ON target SHA/build ID; unprobed uses the distinct OFF target. Both stage traces report
  zero lost, unmatched, nesting, incomplete, duplicate, invalid-duration, and unexpected
  event-set counters. Every recorded PID is absent and no LTTng session remains.
- The three short-run checksum-manifest SHA-256 values are respectively
  `d7803ce5e00908237f325071df4bb7c7c9c1904096afbdf0020b803f82baa354`,
  `dbda22206a1b0035bbd2b195e3e3ecc93868616ca2b3f9ad14f56415f4ab7710`, and
  `3e680ec5cb212565a132a23f1827d37fe0289d267073b474d137d1dd933ac592`.
  The pair-level evidence manifest is
  `/tmp/alien-c2-stage-pair-20260728-v3/pair-evidence-sha256.txt`, SHA-256
  `788fe13b4113f8787291c7e30de29033cfa0e1b8fcf86d9ab4660f3a39d2669f`.
- This closed only the child finding's pre-review build/plumbing smoke gate. No schema-2
  formal calibration aggregator or new 120-second calibration run was executed, and no
  short-run CPU or latency value is adopted as a baseline. Independent code review then
  fixed source/raw binding, build-path containment, closure workload binding, fixed-domain
  dependency rejection, and stale aggregate output handling, reaching 0 Blocking / 0 High
  with 36 C2 and 19 C1 tests passing.
- Those review fixes changed files under `scripts/`, which is part of the canonical paired
  source identity. The v3 closure and pair checksums remain valid historical evidence, but
  v3 no longer authorizes a run from the current source identity. A fresh v4 pair and three
  new 10-second smokes are required before formal authorization. The parent calibration
  checkbox remains open, and the three rejected old raw directories and old aggregate
  remain unchanged.

# C2 stage calibration build equivalence

## Goal

补齐 C2 stage calibration 的传递 workspace 依赖 provenance 合同，构建只改变
stage instrumentation 的 production/stage 配对前缀，并在任何 CPU 扰动结论被采纳前
机器证明 target、依赖、helper 和 source identity 的可比性。

## Background

- 2026-07-28 的 unprobed/callback/full 三组 raw 均通过旧单 run validity 门，但
  stage target 从 `/workspaces/alien-scanner/ws/install` 链接 `Release -O3` 的
  `perception_core`，production target 从独立 `/tmp` closure 链接
  `RelWithDebInfo -O2 -fno-omit-frame-pointer` 的副本。旧 aggregate
  `gate_pass=false` 因此不是可采纳的 probe-overhead 证据。
- `perception_profiling` 的 fixture、sink、oracle 会静态链接 local-map、cave 和 lidar
  算法。stage prefix 若重建 helper，会同时改变 CPU 1 上的输入生成/证据角色，形成第二
  个变量。
- `perception_local_map_node` 实际消费的非系统 workspace 编译/链接依赖只有
  `perception_core` 与 `perception_interfaces`。production 九包 closure 仍负责提供
  `cave_world`、`drone_scanner`、`perception_adapters`、`perception_input_node`、
  `perception_fixtures`、`perception_local_map` 和 `perception_profiling`，但 formal
  target A/B 只需审计实际消费的两个依赖包，并以 helper ELF identity 覆盖 CPU 1 角色。
- 现有 `scripts/lib/local_map_build_provenance.py` 只检查 local-map node 的一个 compile
  entry、目标 build type 和 stage option；runner 又要求 target/helper 同 prefix。应在
  这些现有资产上做定向扩展，不建立通用 ROS/CMake 依赖解析框架。

## Requirements

### R1: 旧证据与 source identity

- 保留以下 raw 与旧 aggregate 原样，不覆盖、不修补 manifest、不重新标记为通过：
  `/tmp/alien-c2-calibration-unprobed-20260728`、
  `/tmp/alien-c2-calibration-callback-20260728`、
  `/tmp/alien-c2-calibration-full-20260728`、
  `/tmp/alien-c2-stage-calibration-20260728.json`。
- 新 analyzer 必须因缺少 closure provenance 而拒绝这组三组比较；单 run 原有
  `valid=true` 与跨 build calibration 不可采纳必须继续区分。
- 配对构建前后复用现有 source revision、binary diff、untracked list 与 deterministic
  untracked archive 合同生成同一 canonical source identity。任一 hash 在 OFF closure
  与 ON overlay 构建之间变化，整组 pair 无效。

### R2: 最小 closure manifest

- 定向扩展 `local_map_build_provenance.py`，每个新 run 生成 schema 固定、键和数组稳定
  排序的 `target-workspace-closure.json`，并把完整 artifact SHA 与 comparison digest
  写入 `run-manifest.txt`。
- manifest 只枚举本 C2 calibration 实际需要机器比较的内容：
  - `perception_core`：production build cache/profile、全部 installed public headers、
    `libperception_core.a`；
  - `perception_interfaces`：production build cache/profile、全部 installed generated
    headers、`libperception_interfaces*` artifacts；
  - production `perception_profiling` 的 fixture/sink/oracle ELF 及 workload YAML。
- 每个文件记录 canonical realpath、相对 closure install base 的路径、类型、size、
  SHA-256，并在 ELF 可用时记录 GNU build ID。两个 dependency package 的非测试 compile
  entries 必须验证 `RelWithDebInfo -O2 -g -DNDEBUG -fno-omit-frame-pointer` 且无冲突
  optimization/sanitizer/frame-pointer flag。
- canonical dependency digest、helper-set digest 和 paired source identity 必须在
  unprobed/callback/full 三组完全一致。target ELF SHA/build ID 继续遵循 OFF 与 ON 不同、
  callback 与 full 相同的原合同。
- 任何缺字段、路径越出声明 closure、artifact 缺失/重复、hash/build ID 无法重算、
  workspace dependency 集合不是精确的 `perception_core`/`perception_interfaces`，或发现
  `/workspaces/alien-scanner/ws/install`，都必须在 workload 启动前 fail closed。

### R3: target/helper prefix 分离

- 保持 `profile-local-map.sh` 现有位置参数和 stage event-set CLI 兼容。非 stage mode
  默认继续要求 target/helper 来自同一 requested install base。
- stage mode 必须显式接收 verified production closure install/build base。runner 先
  source production `setup.bash`，再 source stage overlay 的 `local_setup.bash`，不得通过
  overlay `setup.bash` 静默继承构建时 underlay。
- stage mode 只允许 `perception_local_map` 从 overlay 解析；`perception_profiling`、
  fixture、sink、oracle、workload、`perception_core` 和 `perception_interfaces` 必须从
  同一 production closure 解析。三个 helper ELF SHA/build ID 和 workload SHA 必须与
  unprobed 完全一致。
- source 前后都扫描 `AMENT_PREFIX_PATH`、`CMAKE_PREFIX_PATH`、`COLCON_PREFIX_PATH`、
  `LD_LIBRARY_PATH`、`PYTHONPATH`，并扫描 target CMake cache、compile database、link
  command、RPATH/`ldd`。出现主工作区 install 即拒绝；workspace source path
  `/workspaces/alien-scanner/ws/src` 允许存在。

### R4: production-underlay paired build

- 使用全新外部 pair root、`env -i`、仅 `/opt/ros/jazzy` underlay、colcon sequential
  executor、`MAKEFLAGS=-j1` 与 `CMAKE_BUILD_PARALLEL_LEVEL=1` 构建 OFF production 九包
  closure。不得复用或清理现有 raw/prefix 目录。
- source 新 production install 后，在第二组 build/install/log base 中仅
  `--packages-select perception_local_map`，stage option ON；不得重建
  `perception_profiling` 或其他 dependency/helper 包。
- 两次构建使用同一 compiler/toolchain、
  `scripts/perception-profile-relwithdebinfo.cmake` 和冻结 source identity。除 stage
  macro/call sites、provider translation units/provider library 与 target ELF 外，所有
  workspace dependency/helper artifacts 必须是 production prefix 的同一文件。
- 旧 production prefix 只有在新 schema 能反算完整 source/closure identity 并通过全部
  门时才可作为 underlay；缺 build-time source identity 时不得用时间戳替代 hash，必须
  新建 paired production closure。

### R5: 验证层次

- 合成测试覆盖 canonical manifest 稳定性、缺失/重复/越界 artifact、dependency
  Release/O3/omit-frame-pointer、混入主工作区 install、helper hash/build ID mismatch、
  closure/source digest mismatch、callback/full target mismatch，以及正向 paired fixture。
- C1 runner/analyzer 行为与 C2 非 stage mode CLI 必须保持回归通过；不得复制 runner 或
  修改 C2 C++/CMake 业务实现。
- paired build 先直接通过 closure validator，再分别执行新的 10 秒 unprobed、callback、
  full smoke。smoke 只验证解析路径、closure/helper identity、trace 配对、10 Hz、正常退出
  和 raw checksum，不作为 120 秒 calibration 数字。

### R6: 正式三组重跑条件

- 只有代码审核完成、合成/C1/C2 tests 通过、paired build 门通过、三组短 smoke 均
  `valid=true`/`normal_completion=true`、closure/helper/source digests 完全一致且用户
  另行授权后，才允许创建三个新的 120 秒 output 目录。
- 正式顺序、120 秒/1200 callback、10 Hz、CPU 0 target/CPU 1 helpers、event sets、
  pidstat bracket、trace integrity、CPU 门和 callback percentile 门全部保持父任务冻结值。
- paired aggregate 若通过，只关闭 build-equivalence finding 的证据门；父任务仍按其
  review/验收流程决定 stage calibration checkbox。若数值门仍失败，保留为新的有效失败
  并建立独立 overhead/order-variance finding，本任务不优化 instrumentation。

## Acceptance Criteria

- [ ] AC1: 新 closure manifest 可由 raw/build/install artifacts 独立重算，精确覆盖
  `perception_core`、`perception_interfaces` 和三个 production helper/workload，且 old
  three-run evidence 被新 calibration analyzer 明确拒绝。
- [ ] AC2: 合成正反例证明 mixed-prefix、Release/O3 dependency、main-workspace install、
  helper/source/closure mismatch 均在 CPU 比较前失败；现有 CPU/latency threshold tests
  保持通过。
- [ ] AC3: fresh OFF production closure 与 ON local-map-only overlay 由冻结 source
  identity、单 worker 命令构建，所有非 stage workspace artifacts 字节相同，且 build/
  compile/link/RPATH/runtime 证据中无主工作区 install。
- [ ] AC4: unprobed/callback/full 短 smoke 使用同一 production fixture/sink/oracle/YAML，
  各自正常退出、10 Hz 无 backlog、trace 完整、checksum 通过，并报告相同 closure/
  helper/source digests；smoke 数字未写作 baseline。
- [ ] AC5: 只有 R6 前置门与用户授权同时满足才执行正式三组重跑；新 aggregate 无论
  pass/fail 都保留原阈值、完整 raw 与可重算 provenance。
- [ ] AC6: `profile-local-map.sh` 非 stage CLI、C1 wrapper/analyzer 与 per-run old raw
  readability 无回归；task 保持 profiling evidence scope，C2 生产语义和公共接口零改动。

## Out Of Scope

- 降低 10 Hz/360 beams、缩短正式窗口、放宽 5%/0.5 pp 或 callback percentile 门。
- 优化 stage tracepoint、C2 occupancy/map/snapshot、ROS 接口或生产默认值。
- 建立通用 colcon dependency graph/SBOM 框架，或审计与 formal target/helper 无关的
  所有 ROS/system shared libraries。
- 在本 planning phase 构建、运行 smoke/calibration、修改代码、启动 task 或提交。

## Notes

- 父任务归因：
  `.trellis/tasks/07-27-c2-performance-memory-baseline/research/c1-profiling-assets.md`。
- 本任务是复杂 finding；`design.md`、`implement.md` 和 curated JSONL 通过评审后，仍需
  用户明确授权才能 `task.py start` 和执行长时三组重跑。

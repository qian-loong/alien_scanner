# C4 通信数据面实施计划

## 1. 当前状态

- 分支：`phase/4-c4-communication-data-plane`
- 父任务：`07-21-perception-swarm-architecture-refactor`
- 子任务状态：功能、自动化测试和可视化人工验收已完成；用户已授权提交功能基线，
  独立性能/内存质量门留待提交后执行，本次不 `push`。
- 前置：C3 `MapUpdate`、canonical codec、`MapUpdateApplier`、`ResyncRequestLedger` 和 exact-revision oracle 已稳定。
- 本文只规划 C4；C5 拓扑/角色和 C8 多进程总集成不提前实现。

## 2. 实施顺序

### Step 1：接口与兼容边界（已完成）

- [x] `perception_interfaces` 保留 C3 消息，C4 新增独立 `swarm_data_interfaces` 包。
- [x] 建立 routed map、delivery ack、resync intent/ack、link diagnostic 和 aggregate manifest ROSIDL。
- [x] 为 identity/session、epoch/revision/sequence、origin clock、validity budget、route、hash 和 bounded 字段实现显式校验。
- [x] 用标准 `rclcpp::QoS` 定义 map/state/diagnostic profile；生产代码不依赖 Fast DDS/Cyclone 私有字段。
- [x] C3 `MapUpdate` 经 envelope 编解码 conformance 测试后 source、revision、hash 和 payload 保持一致。
- [x] topic 名、header stamp 和参数不承担隐式协议字段。

### Step 2：ROS-free 数据面核心（已完成）

- [x] 新建 ROS-free 值对象、协议版本校验、immutable payload holder 和 checked size accounting。
- [x] 实现 source/session/epoch/revision ledger 以及 duplicate/conflict/gap/old-route admission。
- [x] 用本地 monotonic clock 扣减 validity budget，并记录 origin/forward 诊断字段。
- [x] 仅保留有界 resync ledger/backlog；可靠传输、history 和重传交给 RMW/DDS。
- [x] 缺口、过期、资源上限和 route/TTL 拒绝均返回明确 ingress 状态，不实现通用传输队列。
- [x] 地图 mutation 统一委托 `perception_map_update::MapUpdateApplier`。

### Step 3：Resync 与 aggregate 原子边界（已完成）

- [x] 短 service request 通过 requester session + client id 幂等映射到 `correlation_id`。
- [x] initial/gap/conflict/epoch-change 进入 barrier；只接受关联 keyframe，恢复后再放行 delta。
- [x] aggregate update 与 contributor manifest 实现 canonical SHA-256、同 revision 校验和原子提交。
- [x] 提供 aggregate/contributor contract fixture；正式 EdgeAggregator 角色留给 C5。

### Step 4：测试链路与 trust adapter（已完成）

- [x] 在独立测试 target 实现固定 seed 的 delay/loss/bandwidth/queue/partition 调度；生产 launch 不加载该 target。
- [x] 注入 clock offset/skew/回跳和新 clock session，验证 payload/origin 顺序不变。
- [x] 实现 permissive/static trust validator，覆盖 spoof/replay/authority epoch/expired/revoked/unknown producer。
- [x] 拒绝路径返回有界 diagnostic，且 trust rejection 前后 C3 状态 fingerprint 不变。

### Step 5：端点 adapter 与最小纵向链（已完成）

- [x] 薄 source/receiver 节点完成 C3 `MapUpdate` 与 C4 envelope 转换。
- [x] 地图流使用 topic，resync 使用短 service，keyframe 通过异步地图流返回，QoS 全部走标准 `rclcpp` API。
- [x] 默认 `rmw_fastrtps_cpp` 的真实 ROS 进程闭环通过；RMW/DDS 负责生产传输。
- [x] LogicalLinkAdapter 只在测试边界插入，验证丢 delta、gap、correlated keyframe 恢复，不实现 ACK/NACK/重传。
- [x] launch 闭环验证 receiver 最终接收 revision 1、3，resync correlation 非空，且 link output sequence 4 可见。

### Step 6：测试与性能证据（当前质量门）

详细执行方案见 [`performance-memory-quality-gate.md`](performance-memory-quality-gate.md)；
C4 之后、C5 之前的分块+COW决策门见
[`follow-up-chunked-map-optimization.md`](follow-up-chunked-map-optimization.md)。

- [x] ROS-free/ROS conversion/link gtest 全部通过。
- [x] launch/integration 闭环通过，包含丢包、gap、resync correlation 和旧 sequence 拒绝。
- [x] 固定 seed link trace 在单测中重复一致；launch 闭环连续 3 次通过。
- [x] 四场景可视化 fixture 的 launch test 通过，验证单 RViz 配置对应的四个 MarkerArray topic、阶段标题、sequence/revision/correlation 和红色 gap/TTL、绿色恢复状态。
- [x] 洞穴验收 fixture 使用真实 A1/A2 C1→C2→C3 地图更新；headless launch test 验证 B 落后悬停、A2 地图冻结/恢复和 A1 独立前进，RViz 人工目检确认地图、轨迹与通信状态显示正常。
- [x] 源码、CMake 和 package manifest 未发现 Fast DDS/Cyclone 私有依赖；当前镜像仅验证已安装的默认 RMW。
- [x] Release/RelWithDebInfo 资源性能矩阵完成：6 个 bounded 场景、每场景 3 轮、
  每轮 300 秒，共 18/18 个有效样本；bytes、CPU、freshness、RSS/PSS/USS、身份和
  业务守恒证据见 `performance-memory-quality-gate.md` §8。
- [x] ASan/LSan 覆盖 bounded、expanding 和 keyframe replacement 并通过；ROS-free
  core Memcheck 严格通过。真实 ROS receiver Memcheck 的 768 bytes possible-lost 已
  定位为 `liblttng-ust` 线程 TLS 第三方退出行为并如实披露，未使用 suppression。

## 3. 预计文件边界

### 新增

- `ws/src/swarm_data_interfaces/`：ROSIDL msg/srv、CMake、package manifest。
- `ws/src/swarm_data_plane/`：ROS-free core、bounded ledger/backlog、resync、trust adapter、薄 ROS node、标准 QoS profile、tests 和仅测试使用的 logical-link fixture。

### 可能修改

- `ws/src/alien_perception/perception_interfaces/`：仅在 adapter 需要补充兼容字段时修改；不得破坏 C3 字段语义。
- `ws/src/alien_perception/perception_map_update/`：只增加明确的 C4 conversion hook 或测试 helper，不改变 map apply/producer 逻辑。
- 根 CMake/colcon 配置：按仓库 CMake 约定加入新包，保持本地 target 与安装 target 一致。

### 明确不修改

- C3 地图核心 revision/hash/applier 语义。
- `swarm_controller` 的 GlobalMapMerger 逻辑和旧中央启动入口；它们作为基线保留。
- C5 拓扑/角色、C6 任务、C7 执行安全和 C8 总集成。

## 4. 验证命令草案

以下命令在 `alien-scanner-dev` 容器内执行，具体包名确认后替换：

```bash
docker exec alien-scanner-dev bash -lc \
  'cd /workspaces/alien-scanner/ws && \
   colcon build --symlink-install --packages-up-to swarm_data_plane'

docker exec alien-scanner-dev bash -lc \
  'cd /workspaces/alien-scanner/ws && \
   source install/setup.bash && \
   colcon test --packages-select swarm_data_plane && \
   colcon test-result --verbose'
```

质量门还包括：

- `rg`/编译检查确保 ROS-free core 不 include `rclcpp`、生成消息或 ROS node 头。
- `rg`/包依赖检查确保生产源码不 include Fast DDS/Cyclone 等厂商私有 API；RMW 只由环境和部署配置选择。
- 运行固定 seed 的 fault/replay fixture，保存 seed、schema version、payload hash、fault trace 和资源摘要。
- 运行 `launch_testing` 确保至少有一个真实测试方法，且进程不会空跑或依赖跨 topic callback 顺序推断因果。
- 用 `git diff --check`、CMake configure、Release build、ASan/LSan/Memcheck 和定向 gtest 作为提交前质量门。

## 4.1 本次验证记录

在 `alien-scanner-dev` 中先加载 `/opt/ros/jazzy/setup.bash`，再加载工作区
`install/setup.bash`，避免 `ament_cmake_test` 的 Python 模块缺失：

```text
colcon build --symlink-install --packages-select swarm_data_plane
colcon test --packages-select swarm_data_plane
colcon test-result --verbose
Summary: 346 tests, 0 errors, 0 failures, 0 skipped
```

随后使用 `-DCMAKE_BUILD_TYPE=RelWithDebInfo` 构建并再次执行同一测试集，结果仍为
`346 tests, 0 errors, 0 failures, 0 skipped`。`test_test_swarm_data_plane_closed_loop.py`
单独连续运行 3 次均通过；活动测试等待 receiver 的 revision 3 delivered ack 与
`/c4/routed_after_fault` sequence 4 同时到达，避免跨 topic callback 到达顺序造成竞态。

加入 C4 可视化 fixture 后，完整 `swarm_data_plane` 测试结果为
`349 tests, 0 errors, 0 failures, 0 skipped`。可视化 launch test 使用 `show_rviz=false`
验证 ROS topic、Marker 阶段和进程退出。

2026-08-11 新增的真实洞穴验收使用 A1/A2 的正式 C1→C2→C3 链和 C4 receiver fixture。
`test_c4_cave_visual_validation.py` 的两个用例均通过，分别断言 A2 的确定性 drop、gap、
correlated keyframe 恢复不阻塞 A1，以及全部 launch 进程正常退出。随后通过 VcXsrv 启动
`c4_cave_visual_validation.launch.py show_rviz:=true`，人工目检确认洞穴真值、A1/A2/B
轨迹、B accepted maps 和通信状态显示正常。该场景只作为功能与可视化验收，不作为 C4
性能或内存容量证据。

2026-08-12 在提交功能基线前重新执行完整验证：`colcon build --symlink-install
--packages-up-to swarm_data_plane` 构建 11 个依赖包成功；`colcon test --packages-select
swarm_data_plane` 的 6 个 CTest target 全部通过，`colcon test-result
--test-result-base build/swarm_data_plane --verbose` 汇总为 `29 tests, 0 errors, 0 failures,
0 skipped`。同时确认生产源码和 package manifest 无 Fast DDS/Cyclone DDS 私有 API，
`LogicalLinkAdapter` 仍只在 `BUILD_TESTING` 内编译。本次结果是功能基线证据；独立 workload
的资源矩阵和 sanitizer/memcheck 仍按计划在提交后执行。

## 4.2 协议回归与洞穴人工验收

原四场景 fixture 改为 headless 协议回归，覆盖 `A -> B` gap/resync、
`A1/A2 -> B` aggregate、`B -> C` aggregate recovery 和 `B1 -> B2 -> C` TTL。
人工验收另用真实洞穴场景：A1/A2 平齐出发并前进约 8 m，B 前进 3～4 m 后悬停；
A2 的一个 delta 被丢弃后，B 的 A2 远端地图冻结，相关 keyframe 到达后追到最新 revision，
A1 的接收地图始终继续增长。详细判定见同目录 `qa.md`。

## 5. 风险与回滚点

| 风险 | 预防/证据 | 回滚 |
|---|---|---|
| C4 复制 C3 MapUpdate 语义 | adapter/conformance 对比 source/revision/hash/payload | 删除 envelope adapter，保留现有 C3 topic/service |
| 重复实现 DDS 可靠传输 | 生产路径只用标准 RMW/DDS；应用只保留 bounded ledger/backlog | 删除自研调度逻辑，回到标准 QoS + resync |
| RMW history 或应用 backlog 耗尽 | 有界 QoS/resource limit、gap 检测和 resync 测试 | 关闭 delta，使用低频 keyframe |
| stamp 被重写掩盖延迟 | origin/forward 双时间字段和 clock fault fixture | 拒绝该指标，回到每跳 duration |
| Relay/aggregator 语义提前耦合 | C4 只定义 envelope/manifest，真实角色留到 C5 | 回退单 source/receiver vertical slice |
| trust rejection 改变领域状态 | rejected fixture 前后 fingerprint 对比 | 禁用 adapter 生产路径，只保留诊断 |
| 性能结论受宿主争用污染 | 同条件固定 seed，分别报告 bytes/RSS/阶段比例并遵守边界 | 不给绝对 CPU 容量结论 |

## 6. 启动前检查与当前限制

- [x] 已采用独立 `swarm_data_interfaces` + `swarm_data_plane` 包边界。
- [ ] 第二种 RMW smoke test 未执行：当前容器未安装 Cyclone DDS；切换接口仍保持标准 API，可在安装第二种 RMW 后复用同一 fixture。
- [x] 方案与实现已按 PRD/design/本仓库 C3 规范核对。
- [x] 当前任务保持阶段分支；功能基线按用户授权提交，独立性能/内存质量门不混入本次提交，
  且不执行 `git push`。

## 7. 2026-08-12 性能/内存质量门记录

- 正式 bounded 矩阵：`2 x 10k`、`1/2/4/8 x 100k`、`2 x 500k`，每项三轮
  300 秒，18/18 `valid=true`。18 个 receiver PID/starttime 唯一，source/receiver
  ELF SHA-256 与 build-id 全矩阵一致，消息/hash/revision/cell/bytes 全部守恒。
- 固定容量下最大 PSS/USS 斜率 `117.44 KiB/min`，低于 `1024 KiB/min` 门槛；没有
  观察到时间相关无界增长。
- `2 x 10k / 100k / 500k` 平均 PSS 为 `19.76 / 31.72 / 84.70 MiB`，apply p95
  为约 `0.52 / 4.19 / 16.78--33.55 ms`。apply 占 callback `97--99.9%`，地图规模
  相关效应远大于本机噪声，已触发 C5 前分块+COW独立任务决策。
- Heaptrack：`2 x 100k bounded` peak heap `19.16 MiB`；`2 x 500k keyframe`
  peak heap `90.75 MiB`，退出后均回落约 421 KiB。
- ASan/LSan 三类 workload 通过；ROS-free core Memcheck 退出 0 bytes/0 errors。
  真实 ROS receiver 严格 Memcheck 未通过的唯一 possible-lost 为
  `liblttng-ust` 初始化线程 TLS 的 768 bytes，未经过业务栈。
- 当前 profiling 代码、报告和 validation 产物尚未提交；没有新的 `git add`、
  `git commit`、`git push` 或任务归档授权。
- 2026-08-13 按历史证据保留策略完成分层：完整 470 文件 raw 树迁移到本地
  `profiling-archive/c4-communication-data-plane-20260812/raw/`，任务 `validation/`
  只保留正式矩阵/run、sanitizer、Heaptrack 和 Memcheck 的可审计摘要。迁移前后 raw
  文件数、字节数和树摘要一致，采集时 JSON 路径不改写。

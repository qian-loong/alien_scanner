# C3 地图状态与增量更新：实施计划

> 状态：Step 1-7 的功能实现、文档、单元/集成测试、replay/oracle、RViz 可视化以及
> 冻结版本的性能与内存专项验收均已完成。用户已授权本轮收口，任务将在定向提交后
> 归档；shared-view alignment 消费验收由后续聚合路径完成。

## 1. 实施边界与依赖

本任务保持为父级 C3 的一个 Trellis 子任务，不再拆空子任务。原因是 core codec、wire
接口、C2 exact-revision adapter 和 receiver 的兼容性共同构成一个版本化协议，单独交付
任一部分都不能完成 C3 验收。下面各 Step 仍保持独立可构建、可测试、可回滚，并在用户
明确要求提交时按 `phase4(stepK): ...` 粒度提交。

前置依赖：

- C2 `LocalObservationMapper`、`CommitReceipt`、`MapReadTransaction` 已实现并通过验收；
- `perception_interfaces` 已提供 C1/C2 消息；
- 开发容器 `alien-scanner-dev` 内 OpenSSL 3 headers/library 可用；
- 既有 `perception_profiling` 固定场景可驱动真实 C2 mapper；历史 Phase 3 bag 资产不可用。

明确不在本任务实施：

- C4 routed envelope、跨机 retry/TTL/QoS/背压；
- C5 Relay、EdgeAggregator、Frozen contributor；
- native dirty-region/change journal；
- 在线 alignment、压缩或真实无线性能优化。

## 2. Step 1：建立 core 类型、canonical codec 与 hash

### 目标

创建 `perception_map_update` 包和 ROS-free core 基础，使相同逻辑输入产生相同 bytes/hash，
所有异常和容量路径可在无 ROS 环境下测试。

### 主要文件

```text
ws/src/alien_perception/perception_map_update/
  CMakeLists.txt
  package.xml
  include/perception_map_update/MapUpdateTypes.hpp
  include/perception_map_update/MapUpdateLimits.hpp
  include/perception_map_update/CanonicalCodec.hpp
  include/perception_map_update/ContentHasher.hpp
  src/MapUpdateTypes.cpp
  src/CanonicalCodec.cpp
  src/ContentHasher.cpp
  test/TestCanonicalCodec.cpp
```

### 工作项

1. 定义 `SourceIdentity`、geometry、voxel、snapshot、delta op、update、hash 和 result enums。
2. 实现所有结构校验：finite geometry、session、revision、strict ordering、duplicate index。
3. 实现 checked add/multiply 和 canonical encoded-size 预检。
4. 实现 encoding v1：大端整数、normalized double、bounded UTF-8 string、array count。
5. 使用 OpenSSL `EVP` 实现流式 SHA-256；固定 geometry/content/update hash 范围。
6. 添加 golden byte/hash fixtures，覆盖空状态、边界 index、`-0.0`、非法 NaN/Inf。
7. 按 `docs/ament-cmake-conventions.md` 导出 core target，并保证根 CMake/CLion 构建树别名可用。

### 验收门

- core public headers 不 include ROS、OctoMap 或 `rclcpp`；
- 相同 snapshot/update 在重复运行中 bytes 与 SHA-256 完全一致；
- decoder 拒绝 truncated、trailing、overflow、重复/乱序 index；
- `content_hash` 不含 revision，`update_hash` 覆盖 reconstruction 语义但不含 correlation/header stamp；
- `colcon test --packages-select perception_map_update` 的 Step 1 gtest 全绿。

### 风险与回滚

- 风险：codec 字段顺序后改会形成兼容负担。
- 控制：在实现业务 producer 前固定 v1 golden vector；字段变化必须推进 encoding version。
- 回滚：删除新包即可，不影响现有 C2。

## 3. Step 2：实现 diff、producer 决策、applier 与 resync core

### 目标

完成不依赖 ROS/C2 的状态复制算法，并用 deterministic conformance suite 固定行为。

### 主要文件

```text
include/perception_map_update/SnapshotDiffer.hpp
include/perception_map_update/MapUpdateProducer.hpp
include/perception_map_update/MapUpdateApplier.hpp
include/perception_map_update/ResyncStateMachine.hpp
src/SnapshotDiffer.cpp
src/MapUpdateProducer.cpp
src/MapUpdateApplier.cpp
src/ResyncStateMachine.cpp
test/TestSnapshotDiffer.cpp
test/TestMapUpdateProducer.cpp
test/TestMapUpdateApplier.cpp
test/TestResyncStateMachine.cpp
test/MapUpdateConformanceSuite.hpp
```

### 工作项

1. 用 sorted-vector 双指针比较生成 upsert/remove operations。
2. 明确支持 operations 为空但 revision 前进的 revision-only delta。
3. 实现 producer baseline、publish commit、cross-revision coalescing 和 keyframe trigger matrix。
4. 实现 `delta_enabled=false`、chain/cell/byte/resource fallback。
5. 实现 receiver `Empty/Ready/ResyncRequired/Removed` 状态机与显式 expected-source admission。
6. 实现 scratch vector merge + content hash 验证 + 单次 swap 原子提交。
7. 实现 duplicate/stale/gap/conflict/old session/epoch/remove/summary 结果矩阵。
8. 实现 bounded resync request ledger 和 correlation idempotency。
9. 增加 fixed-seed property-style fixture：snapshot A -> delta -> B 与 keyframe B 完全等价。

### 验收门

- added、removed、Free/Occupied flip 和 revision-only delta 均通过；
- 任意拒绝路径不改变 cells/revision/hash/freshness；
- 相同 revision/content 重复幂等，不同 content 冲突并进入 resync；
- keyframe 可从 Empty/ResyncRequired 恢复；
- 所有 retained/candidate/payload/resync ledger 上限有 overflow/limit tests；
- Delta 可跨多个 revision，receiver 只要求 current 等于 base，不要求 `new=base+1`。

### 风险与回滚

- 风险：结果 hash 每次全状态计算在大地图上昂贵。
- 控制：首版保留该正确性 oracle并分阶段计时；后续只能增加等价的增量 hash 快速路径。
- 回滚：producer 配置为 keyframe-only，applier 仍使用同一原子完整替换路径。

## 4. Step 3：增加 ROS 接口与唯一 ROS adapter

### 目标

把已冻结的 core 语义映射到 ROS 进程边界，不让 generated message 泄漏进算法库。

### 主要文件

```text
ws/src/alien_perception/perception_interfaces/msg/MapUpdate.msg
ws/src/alien_perception/perception_interfaces/srv/RequestMapResync.srv
ws/src/alien_perception/perception_interfaces/CMakeLists.txt
ws/src/alien_perception/perception_interfaces/package.xml
ws/src/alien_perception/perception_map_update/include/perception_map_update/ros/MapUpdateConversions.hpp
ws/src/alien_perception/perception_map_update/src/MapUpdateConversions.cpp
```

### 工作项

1. 添加 version、kind、source、geometry、revision、provenance、计数、hash、correlation、payload 字段。
2. service 支持 exact 与 bootstrap-latest 请求，response 不含地图 payload。
3. 实现唯一 message encode/decode；固定 hash array 长度和 bounded strings/payload。
4. 保证 ROS header stamp 不进入 core ordering/hash/freshness。
5. 增加 build-time/static assertions 或转换错误测试，防止常量值漂移。
6. 验证 rosbag/CLI 可识别新消息和 service definition。

### 验收门

- `perception_interfaces` 与 `perception_map_update_ros` 可独立构建；
- core target 仍无 ROS 依赖；
- message -> core -> message round-trip 保留全部语义字段；
- malformed array/string/payload 在进入 applier 前被拒绝；
- service response schema 无 `payload`/cell array。

### 风险与回滚

- 风险：接口字段变化影响后续 C4。
- 控制：以 protocol/encoding/hash version 拒绝未知版本；C4 只包裹，不重解释 payload。
- 回滚：新接口尚无既有消费者，不替换 `LocalMapState` 或 OctoMap。

## 5. Step 4：接入 C2 exact transaction 与有界异步 producer

### 目标

在不改变 C2 权威写入、安全和现有可视化输出的前提下，将提交 receipt 异步转换为 C3 update。

### 主要文件

```text
ws/src/alien_perception/perception_local_map/include/perception_local_map/LocalObservationMapper.hpp
ws/src/alien_perception/perception_local_map/src/LocalObservationMapper.cpp
ws/src/alien_perception/perception_local_map/include/perception_local_map/CanonicalSnapshotAdapter.hpp
ws/src/alien_perception/perception_local_map/src/CanonicalSnapshotAdapter.cpp
ws/src/alien_perception/perception_local_map/include/perception_local_map/AsyncMapUpdateProducer.hpp
ws/src/alien_perception/perception_local_map/src/AsyncMapUpdateProducer.cpp
ws/src/alien_perception/perception_local_map/src/PerceptionLocalMapNode.cpp
ws/src/alien_perception/perception_local_map/CMakeLists.txt
ws/src/alien_perception/perception_local_map/package.xml
ws/src/alien_perception/perception_local_map/config/*.yaml
ws/src/alien_perception/perception_local_map/test/TestMapUpdateProducerIntegration.cpp
```

### 工作项

1. 扩展 `MapReadMetadata`，在 transaction lock 下捕获 exact `last_commit` 与 contract fingerprint。
2. 实现 transaction -> C3 canonical snapshot adapter，拒绝 Unknown/duplicate/out-of-range/limit fault。
3. 实现单 pending + 单 in-flight worker、latest-wins、bounded retry 和 shutdown join。
4. `on_observation()` 成功 receipt 路径只 enqueue，不执行 C3 traversal/diff/hash/encode。
5. worker 处理 `Superseded` 时不混用 metadata/content，并收敛到最新 pending receipt。
6. publisher callback 成功后才推进 producer baseline。
7. 添加参数：enable、delta enable、各容量上限、chain limit、周期 interval 默认 0。
8. 在 `/diagnostics` 增加 pending/in-flight/published/superseded/timing/bytes/cell counters。
9. 保留并回归现有 synchronous visualization OctoMap；不改变 header stamp/freshness/safety gate。

### 验收门

- 静态/代码检查确认 C3 全图工作不在 mutation callback 调用栈；
- callback enqueue 路径严格有界；worker 无 detach、无无界容器；
- old receipt 返回 `Superseded`，不产生错误 revision update；
- 两个 backend 通过同一 exact snapshot conformance fixture；
- C2 现有 gtest、launch tests、state/OctoMap 行为不回归；
- C3 disabled 时运行行为与 C2 基线一致。

### 风险与回滚

- 风险：transaction read lock 的 O(n) 遍历可能阻塞下一次 writer。
- 控制：10 Hz backlog/drain 是阻塞门；expanding capacity 只报告 knee。若失败，回到独立
  immutable snapshot/dirty journal 设计评审，不在本步偷偷修改 backend 语义。
- 回滚：`map_update_enabled=false`，现有 C2 路径完整保留。

## 6. Step 5：实现 reference receiver、resync service 和直接闭环

### 目标

交付 C3 的单 source 可运行纵向链，覆盖 late join、gap、乱序、损坏与恢复。

### 主要文件

```text
ws/src/alien_perception/perception_map_update/include/perception_map_update/OctoMapViewAdapter.hpp
ws/src/alien_perception/perception_map_update/src/OctoMapViewAdapter.cpp
ws/src/alien_perception/perception_map_update/src/PerceptionMapUpdateReceiverNode.cpp
ws/src/alien_perception/perception_map_update/launch/map_update_reference.launch.xml
ws/src/alien_perception/perception_map_update/config/map_update_reference.yaml
ws/src/alien_perception/perception_map_update/test/test_map_update_closed_loop.py
ws/src/alien_perception/perception_map_update/test/test_map_update_resync.py
ws/src/alien_perception/perception_map_update/test/test_map_update_rejection.py
```

并修改 `PerceptionLocalMapNode.cpp` 以提供 `RequestMapResync` service。

### 工作项

1. receiver node 订阅 update 与 metadata-only `LocalMapState`，所有 apply 交 core applier。
2. 发布 reconstructed visualization-only OctoMap 与结构化 diagnostics。
3. Empty/ResyncRequired 通过 async service client 请求 keyframe，不阻塞 executor callback。
4. producer service callback 只校验/登记 intent 并返回 correlation，不做 materialization。
5. keyframe topic response 绑定 correlation；重复 service request 幂等。
6. 增加 test injector 发布 duplicate、乱序、future base、旧 epoch、损坏 hash/payload。
7. 验证 last valid revision/content 在所有拒绝路径保持不变。
8. 验证 new epoch keyframe 恢复和 retired session/epoch replay fence。
9. reference QoS 使用 reliable/volatile/keep-last(1)，不声称为 C4 网络策略。

### 验收门

- 直接 topic 闭环可从 C2 revision 重建 source-local occupancy；
- 周期 keyframe 关闭时，晚启动 receiver 通过 service + async keyframe 恢复；
- service response 不含大 payload，callback 时延不包含全图工作；
- duplicate 不刷新 freshness，gap/conflict 进入 resync 并保留 last valid state；
- keyframe 原子恢复后继续接受 delta；
- reconstructed OctoMap 仅用于验证/RViz，不成为新的权威查询契约。

### 风险与回滚

- 风险：reference receiver 与 C4 最终端点职责混淆。
- 控制：C3 node 只处理直接 topic 和本机 service；不实现 route/QoS 策略/重试/背压。
- 回滚：停止 receiver node；C2 producer 和原 OctoMap 不受影响。

## 7. Step 6：Replay、性能、容量与内存安全验收

> 当前状态：replay/oracle、容量正确性、资源边界、冻结版本 CPU/RSS、绝对延迟、
> ASan/LSan 与 Memcheck 专项验收均已完成。

### 目标

用现有 oracle 和工具证明重建正确、有界、可收敛，并记录方案 A 的真实成本。

### 工作项

1. 复用真实 `LocalObservationMapper + OctoMapBackend`，从每个已提交 receipt 获取 exact
   transaction，并通过生产 `CanonicalSnapshotAdapter` 生成 oracle snapshot。
2. 对每个 revision checkpoint 执行 canonical snapshot -> 正式 keyframe/delta producer ->
   正式 applier -> reconstructed snapshot 等价比较；历史 Phase 3 bag 留作未来额外输入。
3. 固定 seed 扰动顺序：duplicate、swap、drop、corrupt、epoch reset、resync。
4. 在 bounded reference 场景持续 10 Hz 输入，采集 observation processed/backlog、worker
   pending/in-flight、published latest 和 drain time。
5. 稀疏变化 fixture 比较 canonical delta bytes 与 keyframe bytes；结果必须确定性更小。
6. expanding 场景扩到约 1.81M cells，报告 materialize/diff/hash/apply、callback wait、CPU、RSS。
7. 运行 ASan/LSan、Valgrind/memcheck；区分业务泄漏、工具缺失和 ROS/DDS 已知噪声。
8. C3 enabled/disabled 与 keyframe-only 三种模式使用同输入、同构建、同采样窗口。
9. 保存原始报告、manifest、参数、build ID/SHA-256，禁止只保留人工摘要。

### 阻塞门

- 10 Hz bounded 输入无 observation backlog；
- 停止输入后在配置 drain window 内 published/reconstructed revision 收敛 latest；
- pending/in-flight/retained/payload/receiver memory 不越界；
- sparse delta bytes < matching keyframe bytes；
- replay 每个检查点内容等价；
- sanitizer/memcheck 无业务非法访问或泄漏；
- 所有正确性/拒绝/恢复测试通过。

### 只报告、不阻塞的指标

- generate/materialize/sort/diff/hash/encode/apply 绝对延迟；
- expanding capacity knee；
- C3 enabled 与 baseline 的 CPU/RSS 差异；
- 相对当前完整 OctoMap snapshot 的序列化/处理成本。

禁止使用 `<10%` CPU/延迟门；只有效应明显高于 30-50% 环境分辨率下限时才下方向性结论。

2026-08-08 的正式矩阵使用 disabled、enabled、keyframe-only 三种模式各 3 轮、每轮
300 秒。9 轮均正常结束并通过 raw analyzer；enabled 每轮发布约 3,803-3,805 个更新并
完成 drain，keyframe-only 每轮发布约 3,804-3,805 个 keyframe 并完成 drain。三组内存
aggregate 均未触发持续增长门。ASan/LSan、Memcheck 和 1,812,520-cell capacity harness
同时通过；完整数值、原始证据和后处理来源见 `validation/README.md`。任务归档后的
Git 摘要仍在 `validation/`，872 个本地 raw 文件已于 2026-08-13 完整迁移到
`profiling-archive/c3-map-state-updates-20260808/raw/`；历史采集路径不改写，映射与树摘要见
`validation/relocation-provenance.txt`。

### 风险与回滚

- 风险：方案 A 在目标容量下无法满足 10 Hz 结构性门。
- 控制：先定位 transaction lock、materialize、sort、hash、diff、apply 各阶段；只有证据指向
  O(n) compare 才创建 native dirty-region 独立优化任务，方案 A 保留 oracle。
- 回滚：keyframe-only 或 C3 disabled，不放宽安全/freshness。

## 8. Step 7：文档、spec 与最终全量复核

### 目标

让协议、运行方法、参数、功能验证以及冻结版本性能/内存证据成为后续 C4 可直接消费
的稳定输入。

### 工作项

1. 新增/更新 C3 使用文档：消息语义、topic/service、启动命令、诊断与回滚参数。
2. 更新 `.trellis/spec/backend/`，记录 map-update contract、revision-only delta、hash/version、
   callback isolation、resync 与资源边界。
3. 在父任务 PRD/design/implement 中只同步 C3 已完成状态和 C4 输入，不提前修改 C4 实现。
4. 运行 `trellis-check` inline 全量复核：spec、CMake、测试、跨包消息流、错误路径、复用。
5. 列出全部受影响 package，并逐一复核其 Quality Check 要求。
6. 汇总 diff、功能测试、性能与内存结果及剩余风险。

### 验收门

- `prd.md` 所有 acceptance criteria 可映射到测试/报告；
- design、code、wire constants、参数默认值和文档无双重真相；
- 全量 build/test 无失败、错误或 skip；
- C4 可把 `MapUpdate` 当作 opaque semantic payload，并复用 resync intent/correlation；
- 未经用户明确要求不执行 `git add`、`git commit` 或 `git push`。

## 9. 验证命令基线

实施时在容器 `/workspaces/alien-scanner/ws` 中执行。命令可根据新增测试 target 微调，
但不得省略等价门。

### 9.1 定向构建

```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/alien-scanner/ws
colcon build --symlink-install \
  --packages-up-to perception_map_update perception_local_map
source install/setup.bash
```

### 9.2 定向测试

```bash
colcon test \
  --packages-select perception_interfaces perception_map_update perception_local_map \
  --event-handlers console_direct+
colcon test-result --verbose
```

### 9.3 全量回归

```bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

### 9.4 Profiling 与内存检查

优先复用：

```text
scripts/profile-perception.sh
scripts/profile-local-map.sh
scripts/analyze-perception-profile.py
scripts/analyze-local-map-profile.py
scripts/perception-profile-asan.cmake
docs/performance-memory-testing-playbook.md
```

需要为 C3 增加 stage/counter 时扩展现有 runner 与 parser，不另建不可比较的一次性脚本。

## 10. `task.py start` 前检查

- [x] `prd.md` 已完成 convergence，无重复/已解决开放问题。
- [x] `design.md` 与 D-01 至 D-07 一致。
- [x] `implement.md` 每个 Step 有目标、依赖、验收、风险和回滚。
- [x] Snapshot/Keyframe/Delta/Summary/Remove 语义均有 owner。
- [x] revision-only empty delta 已进入需求、设计和测试计划。
- [x] C2 exact metadata 不通过非事务 `state()` 拼接。
- [x] C3/C4/C5 边界无冲突。
- [x] 资源门与性能报告门区分明确。
- [x] 用户已审核并明确批准进入实施。

上述检查已在实施开始前完成；功能提交、冻结版本性能与内存专项验收、最终质量门和
spec 同步均已完成。用户已授权本轮收口，剩余生命周期操作是定向提交后归档本任务并
记录 session journal。

# C2 本机观测消费与局部地图实施计划

## 0. 开始条件

- [x] `prd.md`、`design.md` 已完成方案审核且无 blocking/high finding。
- [x] PRD convergence pass 完成；无临时 brainstorm 段、重复要求或已解决问题。
- [x] 用户明确批准进入实现。
- [x] `implement.jsonl`、`check.jsonl` 至少各有一个真实 context entry。
- [x] `python .trellis/scripts/task.py validate .trellis/tasks/07-27-c2-local-observation-map`
  通过。
- [x] 执行 `task.py start` 后才允许修改 C++、ROS 接口、launch 或测试。

## 1. 预计影响范围

### 新增

- `ws/src/alien_perception/perception_local_map/`
  - `package.xml`、`CMakeLists.txt`
  - ROS-free map types、backend interface、OctoMap backend、mapper engine
  - thin `PerceptionLocalMapNode`
  - XML launch、参数、RViz 配置
  - backend conformance、engine gtest、launch tests
- `docs/local-observation-map.md`：运行、topic、RViz 目检和 legacy 边界。
- `ws/src/drone_scanner/include/drone_scanner/CaveLaserScanNode.hpp`、
  `src/CaveLaserScanNode.cpp`、`src/CaveLaserScanMain.cpp`：薄标准 LaserScan
  publisher，复用 `FakeLidar`/`ICaveField`，不包含地图累积逻辑。
- `ws/src/drone_scanner/include/drone_scanner/LaserScanProjection.hpp`、对应 `src/`
  与 `test/`：ROS-free ranges/index/return validation helper 和 golden tests。
- `ws/src/alien_perception/perception_fixtures/src/PoseGatedLaserScanRelay.cpp` 及测试：
  validation-only PoseEstimate watermark/lead-delay gate；perception-specific 依赖留在
  fixture 包，不引入 `drone_scanner`。
- `ws/src/alien_perception/perception_local_map/launch/cave_full_ray_scene.launch.py`、
  `config/cave_full_ray_scene.yaml`、独立 RViz config、ROS-free scene gtest、launch test
  与真实 RViz smoke。

### 修改

- `ws/src/alien_perception/perception_interfaces/msg/LocalMapState.msg`
  （新增文件）、`HealthState.msg` 的 producer source/session 与 canonical contract
  fingerprint 字段，以及该包的 CMake/package manifest。
- `perception_core` 的 canonical contract fingerprint utility，以及
  `perception_input_node` 的 HealthState session/fingerprint 映射和对应 C1 回归测试。
- 必要时为 C1 fixture 增加可复用的 C2 launch 组合；不得改变 C1 已冻结的
  observation/ray-evidence 语义。
- `drone_scanner`、`perception_fixtures` 与 `perception_local_map` 的 CMake/package
  manifest：注册新节点、fixture gate、scene/测试资产及必要依赖；不得引入新的
  自定义 ROS interface，`drone_scanner` 不得新增 `perception_interfaces` 依赖。
- Phase H 实现完成后定向补充 `docs/local-observation-map.md` 的新场景运行命令和
  目检项；本轮规划收敛不修改该文档。
- 根 CMake 仅在自动 package discovery 无法识别新包时定向修复；不维护手写
  package 顺序。

### 明确不修改

- 旧 `swarm_controller::OctoMapBuilder`、GlobalMapMerger、allocator 和 fleet
  topology 的业务逻辑。
- 现有 swarm_controller single/multi-drone public launch 的默认 legacy mapper
  选择；新链切换留到 C3/总集成。
- C3 LocalMapUpdate/keyframe/delta/summary 协议。
- C1 `LidarObservation`、`PoseEstimate` 的既有 wire 数值和 ray-evidence 枚举。
- Phase H 不修改 `PerceptionInputNode.cpp`、`PerceptionLocalMapNode.cpp`、
  `LocalObservationMapper`、`SensorExtrinsicRegistry` 或既有 C1/C2 debug 资产；真实链
  的跨 topic 时序由 validation-only fixture gate 解决。

## 2. 实施阶段

### Phase A：包骨架与公共契约

- [x] 新建 `perception_local_map` ament_cmake 包，划分 ROS-free core、ROS-free
  OctoMap backend 和薄 node targets；C++17、warnings、export/install rules 遵循
  `docs/ament-cmake-conventions.md`。
- [x] 定义 `OccupancyState`、map geometry/identity/epoch/revision、bounds/cell、
  `BackendCapabilities`、`EffectiveMapCapabilities`、`LocalMapState`。
- [x] 固定 canonical voxel lattice：`floor((coord-origin)/resolution)`、负坐标数学
  floor、cell/region `[min,max)` 与明确 Invalid/OutOfRange。
- [x] 定义只读 `IOccupancyMapView` 与写入侧 `ILocalOccupancyBackend`；安全/查询
  consumer 只能获得 view，公共 headers 禁止 include rclcpp、ROS message 或 OctoMap。
- [x] 增加 `LocalMapState.msg`，包含 resolution/lattice origin、remaining validity
  budget、known bounds、state sequence、contract schema version/fingerprint、
  combination identity、ray-presence 与按类型/证据等级的 sensor counts；message
  声明闭合 health/ray 常量，wire enum/字段由节点显式 decode/encode，C2 不输出
  content hash/delta payload。
- [x] 在 `perception_core` 实现带 schema version、稳定排序/编码和小写 SHA-256 wire
  表达的 canonical `MapperContractFingerprint`；descriptor inventory、同一 combination
  内 requirements/specific IDs 的不同插入顺序结果必须相同，degraded combination
  priority 重排或任一其他语义字段变化必须改变摘要。
- [x] 为 `HealthState.msg` 增加 producer source/session 与 contract fingerprint 并让
  C1 publisher 填充；C1 message/launch tests 证明重启后 session 改变、contract 漂移
  或 malformed fingerprint 时旧 health 不能绑定/授权新输入。
- [x] 添加 compile-only/public-header smoke，确认 standalone colcon 与根 superbuild
  均解析 `perception_local_map` 导出的 package-scoped targets。

**回滚点 A**：除删除新包与 `LocalMapState.msg` 注册外，还必须恢复
`HealthState.msg` schema、C1 publisher/fixture/tests 和 fingerprint utility，随后重建
`perception_interfaces` 的全部反向依赖；不得留下新旧 message type support 混用。

### Phase B：后端 conformance 与 OctoMap adapter

- [x] 实现 `OctoMapBackend`：参数校验、reset、atomic evidence apply、point query、
  bounded-region query、deterministic cell ordering。
- [x] 在 test support 中实现 `DeterministicVoxelBackend`，不安装为生产 runtime。
- [x] 建立同一 conformance suite，两个 backend 必须通过 required capability。
- [x] 覆盖初始 unknown、free/occupied、同批 occupied precedence、reset、非法
  bounds/resolution、负坐标/半开边界/OutOfRange、empty/non-empty known bounds、
  optional unsupported 行为。
- [x] ordinary Rejected/NoEvidence 必须通过逐 cell 快照证明 strong no-change；增加
  mutation fault injection，证明 mapper 销毁 faulted backend、封禁所有旧 guarded
  transaction/token、关闭旧 revision 链并推进新 epoch，从空 backend 恢复且不复制
  整棵 OctoMap。
- [x] 实现 mapper-owned `MapReadTransaction`：绑定 identity/epoch/revision/geometry/
  known bounds，持有有界 shared read lock，并以 move-only value 自身提供 query/
  value-visitor/close；不返回 backend/view/raw reference。apply/reset/fault 使用 exclusive
  lock；并发、close、move-from 与 revoke tests 证明无混合 revision、悬空引用或复活。
- [x] public-header/API static assertions 证明 consumer 不能取得或长期保存 backend/
  `IOccupancyMapView`/`octomap::OcTree` 引用；package-private snapshot bridge 只返回
  materialized ROS message value。
- [x] Applied 返回 `CommitReceipt`；exact-revision transaction 在并发推进后返回
  Superseded，snapshot worker coalesce/retry latest。故障注入证明旧 receipt 不会标记
  新内容，且不依赖 mutually-exclusive ROS callback group 才正确。
- [x] 确认领域 public headers 不暴露 `octomap::OcTree`；只允许 backend-specific
  adapter/节点访问只读 snapshot。

**回滚点 B**：保留公共类型，移除具体 backend；旧 builder 不受影响。

### Phase C：射线 evidence 投影与 mapper engine

- [x] 实现完整 ROS-free observation validation，复用
  `Perception::is_valid_ray_evidence()` 和 `Scan2D::return_kind()`。
- [x] 实现 Scan2D `HitOnly/HitRay/FullRay` 矩阵；NoReturn 只在 FullRay 写到
  `range_max` 的 free，Invalid 永不写图。
- [x] 实现 Cloud3D HitOnly endpoint；高 capability/non-finite/cross-type 输入整批
  fail closed。
- [x] 组合 `map <- body` 与冻结 `body <- sensor`，支持多个 sensor 的独立 origin。
- [x] 实现 ROS-free `SensorExtrinsicRegistry`：node 只提供 TF sample，registry 负责
  有限性、四元数、sensor/frame identity、首次冻结、容差和 drift/session 拒绝。
- [x] NoEvidence batch 不推进 revision/freshness；相同
  `(sensor,session,origin_stamp)` 的传输重放不进入 apply；更高 stamp 的新 batch 即使
  evidence 相同且 `changed_cell_count=0` 仍返回 Applied，推进一次并刷新真实 commit。
- [x] gtest 覆盖能力边界、min/max、NaN/inf/out-of-range、同批冲突和固定 replay。

**回滚点 C**：engine 与 backend 解耦，可单独退回到 conformance-only 状态。

### Phase D：pose/session/health/map epoch 状态机

- [x] 实现有界 pose history，只匹配 acquisition stamp 之前且未过期的最近 pose。
- [x] observation/pose 只在相同 `clock_domain` 配对；callback receive time 使用可注入
  monotonic clock，wire origin/freshness 不充当持续时钟。
- [x] 每 sensor 独立维护 producer session 与 last accepted stamp；拒绝旧 session
  replay，不跨 sensor 建立 timestamp 全序。
- [x] C2 使用冻结 inventory/contract 独立运行 C1 `MapperHealthGate`；校验 C1 health
  producer session 与 canonical contract fingerprint，并仅把上游三态作为不可抬高
  ceiling。最低 degraded 允许一台 LiDAR + HitOnly，但必须来自启动时声明的组合。
- [x] 固定 combination identity：`minimum` 或 `degraded/<priority-index>`，只与 contract
  version/fingerprint 一起解释；测试 upstream ceiling 降级时 ID 仍报告本地实际满足
  组合，禁止使用 description 作为 identity。
- [x] 实现 pose source/session/frame/reset epoch、stamp rollback、position/orientation
  jump 检测。
- [x] 保存 active/retired pose lineage 与每 lineage high-water stamp；旧 session、重复
  stamp、reset epoch 回退和 DDS replay 只拒绝，不重复推进 map epoch。
- [x] discontinuity 顺序：close old chain -> revoke alignment -> reset backend/history
  -> new epoch/revision 0 -> recover。
- [x] 提供 ROS-free `submit_alignment(reference)` fixture/C3 adapter API，覆盖 map
  source/session/epoch、provider session、单调 revision 校验和 reset 后 Revoked；不新增
  C2 alignment ROS topic，不应用 shared transform。
- [x] 证明 alignment 未 Ready/fleet 断开不影响本机 map health。
- [x] gtest 覆盖旧 observation/pose/alignment 跨 epoch replay 与 freshness 到期。
- [x] `tick(monotonic_now)` 只允许 freshness/health 到期，不得推进 epoch/revision 或
  改写 last-commit provenance。

**回滚点 D**：状态机测试先固定；若 reset 失败，禁止继续 ROS 接线。

### Phase E：薄 ROS 节点与标准输出

- [x] 节点仅订阅 C1 `perception/observations`、`perception/pose`、
  `perception/health`；显式 switch 解码所有 wire enum。
- [x] 节点只在 acquisition stamp 查询并转换 `body <- sensor` TF sample，冻结、容差和
  drift 判定由 ROS-free registry 完成；不得以 `map <- sensor` TF 代替 pose。
- [x] 参数加载后冻结 backend、identity、frames、geometry、health contract、pose
  tolerance；非法值启动失败。
- [x] 发布 `local_map/state`、`local_map/octomap` 和 diagnostics；map revision/freshness
  只响应真实状态/commit，state heartbeat 仅推进 state sequence，OctoMap 使用
  transient-local。
- [x] 按 `design.md` 固定 schema 发布 geometry、last provenance、contract/
  combination identity、sensor counts、backend capability 和仅作版本门控的 alignment
  摘要；`validity_remaining_ns` 由发布时本机 monotonic budget 投影，接收端按本机
  receive monotonic time 重建 deadline，不得增加 content hash/map cell/delta payload。
- [x] `local_map/state` 使用 KeepLast(1)、reliable、volatile、有限 lifespan/deadline 和
  steady-timer 固定 heartbeat；每次 emission 严格推进 state sequence，heartbeat 不
  推进 map revision。测试 ROS clock pause/rollback、late join、延迟/重复 sequence、
  publisher crash、首次 session probation 和 remaining-budget 到期。
- [x] callback 捕获输入/TF/backend 错误并 fail closed；日志节流但状态拒绝立即生效。
- [x] launch test 覆盖 invalid enum/parameter、TF drift、late subscriber、单 publisher
  cardinality 和无 `scan_returns`。

**回滚点 E**：运行期停止 C2 launch 即可切回隔离的 legacy builder；若回滚源码或
wire，必须继续执行回滚点 A，恢复已修改的 C1 HealthState producer/type support。

### Phase F：C1 fixture、legacy oracle 与 RViz

- [x] 复用 C1 隧道环切面/岔口 fixture，不复制其 beam/angle/range 常量。
- [x] 提供 `local_map_debug.launch.xml`（或在无法无损复用既有 Python fixture
  组合时保留最小 Python wrapper，并在评审中说明原因）。
- [x] RViz 显示 C1 occupied hit、hit free、no-return free、Invalid、右手坐标轴与
  C2 实际 OctoMap；固定 frame 与网格 display 命名清晰。
- [x] 自动断言 HitOnly/HitRay/FullRay 的 occupancy 差异、岔口 no-return、Invalid
  不写图和截面方向。
- [x] 使用旧 `TestOctoMapBuilder` 的代表查询作为 oracle；不要求二进制 OctoMap
  snapshot 一致。
- [x] graph 断言新 launch 不存在 FakeLidar、OctoMapBuilder、`scan_returns`，且新旧
  mapper 不写同一 authoritative topic。
- [x] 补充 `docs/local-observation-map.md` 的容器命令与目检清单。

**回滚点 F**：删除新 demo/测试资产不影响核心 mapper。

### Phase G：全量验收与代码审核

- [x] Release 构建相关 package dependency closure。
- [x] 运行所有 ROS-free gtest、launch tests 和 fixed replay。
- [x] 记录固定 fixture 的 wall time、峰值 RSS、known/occupied cell 数作为 C2
  baseline；此任务不宣称 C9 性能容量目标。
- [x] 使用 `trellis-check` subagent 做全 scope 代码审核并直接修复；复核到无
  blocking/high-confidence finding。
- [x] 运行 `git diff --check`，检查没有无关工作树文件混入。
- [x] 用户执行 RViz 目检并明确验收。

原 C2 范围验收记录（2026-07-27，不含下述 Phase H）：Release build 与根
superbuild 通过；六包累计
`121 tests, 0 failures`；固定 fixture 基线约为 `1763 us`、峰值 RSS
`20280 KiB`、known `1567`、occupied `113`；full-scope `trellis-check` 无剩余
blocking/high finding；用户已完成 RViz 目检并明确通过。

### Phase H：连续 FullRay 洞穴场景扩展

- [x] 完成 R-C2-09/Design §13 的规划审核；修复全部 blocking/high finding，并取得
  用户对冻结场景、标准 LaserScan FullRay 和坐标/束序契约的确认后再修改源码。
- [x] 新增 canonical structured scene config/loader，唯一保存 cave、trajectory、scan、
  timing 和 descriptor/contract。Python launch 只读取一次、验证并计算派生的
  `angle_max`、四元数参数顺序与 delay budget，再向 cave publisher、scanner、gate、
  C1/C2 fan-out；ROS-free/launch tests 读取同一 config，不复制运行时常量。
- [x] 在 `drone_scanner` 实现 ROS-free `LaserScanProjection` 与薄
  `CaveLaserScanNode`/main：复用 `FakeLidar` 与 `ICaveField`，按 odometry acquisition
  stamp 以 `10 Hz` 输出 raw scan。固定 `N=360`、`angle_min=-pi`、
  `increment=2*pi/N`、`angle_max=angle_min+(N-1)*increment`、ranges 长度 N，
  `time_increment=0`、`scan_time=0.1 s`、空 intensities、`scan_link` frame；完成
  `j=(i+N/2)%N` remap。仅合法有限 hit 写 range、no-return 写 `+inf`，其他 hit
  异常整帧 fail closed；首个终点零速度 odometry 停止创建 acquisition。不恢复
  `/scan_returns`，也不向 `drone_scanner` 引入 `perception_interfaces`。
- [x] 在 `perception_fixtures` 实现 validation-only `PoseGatedLaserScanRelay`：缓存 raw
  scan，等待同 stamp/frame/source/clock-domain/current-session 的 C1 PoseEstimate
  watermark，之后施加 `pose_lead_delay=100 ms`（至少两个 odom period）再释放给 C1；
  超时、stamp 回退、lineage drift 和 overflow 丢弃并诊断。终点只排空此前 pending，
  不创建 acquisition；不修改 C1/C2 核心。
- [x] 增加独立 `cave_full_ray_scene.launch.py`/RViz：冻结 tree seed/radius、20 s 直线
  轨迹、频率/range/noise、唯一静态 `body <- scan` TF，并从同一参数组生成 C1/C2
  descriptor/contract。descriptor 用 `mounting_position/mounting_orientation` 作为
  期望 metadata/fingerprint；实际变换只来自 acquisition stamp 的 TF sample。保留原
  `local_map_debug` 入口不变，唯一 `LD_PRELOAD=liboctomap.so` 只挂在 RViz 进程。
- [x] 增加 ROS-free helper golden tests：锁定四个正交 index/direction、全部 360 束、
  LaserScan metadata/首末样本、finite hit/`+inf`、非法 hit 整帧拒绝、空 intensities
  与无 deskew；C1 adapter round trip 必须保持相同 FOV/range/resolution。
- [x] 增加 ROS-free scene gtest：读取 canonical config，以
  `TreeCaveField + FakeLidar + LocalObservationMapper` 的 read transaction 在固定 map
  点/区域断言 occupied/free/unknown；不得把真值发布或注入 mapper，也不解析
  visualization-only OctoMap 二进制来冒充 query。
- [x] 增加约 `40-45 s` timeout 的正式 launch test：真实运行 `20 s` C1/C2 链，要求
  至少 `100` 个唯一 accepted observation、同一 map epoch 内 revision 单调、known
  bounds X span 至少 `9 m`、C1/C2 fingerprint 一致，且 diagnostics 无 pose
  missing/stale rejection。终点等待 lead delay + transport drain 后 revision/
  provenance 固定，heartbeat sequence 可继续且 freshness 最终到期；同时断言禁止
  legacy/accumulator/第二 mapper 的 graph cardinality。
- [x] 增加 RViz 结构与真实进程 smoke：解析 config 锁定 `/cave/points`、Path、
  released LaserScan、TF、OctoMap displays，解析 launch 锁定 process-scoped preload；
  独立 domain 启动 RViz，检查 `/proc/<pid>/maps` 已加载
  `liboctomap_rviz_plugins.so`、日志无 undefined symbol、ROS graph 中 RViz 订阅预期
  topics。该 smoke 不替代 GUI 目检。
- [x] 在隔离 Release prefix 构建 `cave_world`、`drone_scanner`、
  `perception_fixtures` 与完整 C1/C2 dependency closure，运行新增和原 C2 全套测试、
  确定性 replay、RViz smoke、`colcon test-result --verbose` 与定向
  `git diff --check`，记录通过证据。
- [x] 使用 full-scope `trellis-check` subagent 审核 Phase H 代码与跨层配置，直接修复并
  复核至无 blocking/high-confidence finding。
- [x] 用户使用独立 RViz 场景目检洞穴真值、轨迹、当前 LaserScan、TF 与 C2 累积
  OctoMap，明确验收后才可勾选 Phase H。

Phase H 自动化验收记录（2026-07-27）：canonical loader/RViz/launch 静态测试
`3 passed`；`TestLaserScanProjection` `5/5`；Pose gate launch test `3/3`；
ROS-free scene transaction gtest `1/1`；真实 20 秒 C1/C2 scene launch test
约 `22.3 s`；独立 domain 真实 RViz plugin/maps/subscription smoke 约 `0.9 s`。
最终在全新隔离的 Release build/install/log prefix 中以 sequential executor 构建
8 包 dependency closure，构建约 `2m22s`；同一 prefix 的顺序全回归约 `2m7s`，结果为
`242 tests, 0 errors, 0 failures, 0 skipped`。首次并行全回归中旧
`fake_odom` 频率测试受资源争用只收到 9 帧，独立重跑与顺序全回归均通过；因此
最终 gate 固定使用 sequential executor。full-scope 审核进一步修复 recovery bridge
提前提交 revision、pose fault 后旧 history 继续授权、新 pose lineage 无法激活，以及
缺少并发 transaction/fault 覆盖；修复后 Release 8 包构建通过，全回归为
`248 tests, 0 errors, 0 failures, 0 skipped`，审核结论为 `0 Blocking / 0 High`。
RViz smoke 只创建测试 publishers 并验证
plugin/maps/subscriptions/QoS，不发布场景消息，因此其短暂 GUI 窗口空白是预期行为，
不能替代完整场景目检；完整场景中首帧 scan 可能在 TF cache 建立前被 RViz 丢弃，
后续链与自动验收正常。2026-07-27 用户确认独立完整场景 RViz 显示正常，Phase H
人工外观验收通过。

**回滚点 H**：停止新 scene launch 即可回到原 A-G 运行路径。源码回滚必须移除
`LaserScanProjection`、`CaveLaserScanNode`/main、validation-only gate、各自 CMake/
package/test 注册、canonical config/loader、scene launch/RViz、ROS-free scene test、
launch/RViz smoke 和 `docs/local-observation-map.md` 的 Phase H 段落；不得回退既有
C1/C2 wire/core、mapper、原测试或静态 debug 资产。

Phase H 功能、自动化、full-scope 审核和人工 RViz 外观验收均已完成；本任务已满足
功能验收，但仍需用户明确授权后才可提交或归档。本计划本身不授权自动提交。

## 3. 验收映射

| PRD acceptance | 实施阶段 | 自动化证据 |
| --- | --- | --- |
| 多 sensor、单 revision 链 | C、D、E | engine gtest + publisher cardinality launch test |
| HitOnly/HitRay/FullRay 上限 | B、C、F | evidence matrix + fixture occupancy assertions |
| 两 backend conformance | B | typed/parameterized gtest |
| lattice/known bounds/原子失败 | A、B | boundary conformance + fault injection + fenced-transaction gtest |
| revision-locked read transaction | B、E | API leak/static test + concurrent close/move/revoke + receipt/superseded/coalesced snapshot test |
| Healthy/Degraded/Unavailable 与 capability schema | A、D、E | gate/combination/count/presence gtest + health launch test |
| contract fingerprint/clock/receive age | A、D、E | canonical hash gtest + mismatch/malformed/session/clock launch test |
| state heartbeat/freshness QoS | D、E | sequence/late-join/delayed/crash/deadline launch test |
| pose reset/map epoch/alignment 隔离 | D | lineage high-water + alignment API + discontinuity/replay gtest |
| fleet 断开本机自治 | D、E | no-alignment/no-fleet integration test |
| legacy 对照且不双写 | F | query oracle + ROS graph assertions |
| Release/test/resource/diff | G | commands and recorded result |
| 连续标准 FullRay、多 revision、pose-before-scan、终点停扫 | H | helper gtest + watermark/diagnostics + scene launch test |
| known bounds 沿 X 扩展与三态 oracle | H | launch state snapshots + ROS-free mapper transaction scene gtest |
| C1/C2 contract/fingerprint/TF metadata 一致 | H | canonical scene config + fingerprint + extrinsic freeze assertion |
| RViz overlay 与 graph 隔离 | H | config/preload/plugin/subscription smoke + cardinality test + 用户目检 |

## 4. 验证命令

在 Windows 仓库根目录执行；实际构建发生在 `alien-scanner-dev`：

```powershell
docker exec alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  set -euo pipefail
  cd /workspaces/alien-scanner/ws
  colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to perception_local_map perception_fixtures
  source install/setup.bash
  colcon test --event-handlers console_direct+ \
    --packages-select perception_core perception_interfaces perception_adapters \
      perception_input_node perception_local_map perception_fixtures
  colcon test-result --verbose
"
```

无 GUI smoke：

```powershell
docker exec alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_local_map local_map_debug.launch.xml show_rviz:=false
"
```

RViz 目检：

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_local_map local_map_debug.launch.xml show_rviz:=true
"
```

Phase H Release 与无 GUI scene 验证（实现后执行）：

```powershell
docker exec alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  set -euo pipefail
  cd /workspaces/alien-scanner/ws
  colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to drone_scanner perception_fixtures perception_local_map
  source install/setup.bash
  colcon test --event-handlers console_direct+ \
    --packages-select cave_world drone_scanner perception_core perception_interfaces \
      perception_adapters perception_input_node perception_local_map perception_fixtures
  colcon test-result --verbose
"

docker exec alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_local_map cave_full_ray_scene.launch.py show_rviz:=false
"
```

Phase H RViz 目检（实现后执行）：

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_local_map cave_full_ray_scene.launch.py show_rviz:=true
"
```

仓库检查：

```powershell
python .trellis/scripts/task.py validate .trellis/tasks/07-27-c2-local-observation-map
git diff --check
git status --short
```

## 5. 审核关注点

- C1 wire enum 是否全部显式解码，Cloud3D 是否始终保持 HitOnly。
- Node 是否仍然过厚，TF/message/OctoMap conversion 是否泄漏进领域算法。
- map epoch/revision/freshness 是否只有一个写入点。
- reset 是否先 fence 旧链和 alignment，再允许新 epoch。
- sensor session reconnect 与 pose session reset 是否被错误地同等处理。
- conformance 是否真的运行两个 backend，而非两个测试复制品。
- 可选 backend capability 为 false 时，消费者是否仍调用了它。
- 新 launch 是否意外恢复 `scan_returns` 或同时启动旧 builder。
- `LocalMapState` 是否被误用为 C3 map-content update。
- CaveLaserScan 的 angle/index remap、static TF 和 C1/C2 mounting quaternion 是否
  表达同一旋转；实际变换是否只来自 TF，descriptor 是否只作期望 metadata/fingerprint，
  是否出现双重旋转或束序反向。
- pose-before-scan 是否由 fixture watermark + lead delay 提供，并以 C2 diagnostics
  证明，而不是只比较 stamp；`drone_scanner` 是否保持无 `perception_interfaces` 依赖。
- launch test 是否误把 OctoMap 二进制当 authoritative query；三态 oracle 是否在
  ROS-free mapper transaction 上执行。
- scene 数值是否只有一个真源；终点 drain 后 scan/revision/provenance/freshness 是否
  真实停止，heartbeat 是否只推进 state sequence。
- RViz 是否既有结构断言，也有真实 plugin load/subscription smoke，且 preload 未泄漏
  到 mapper/fixture 进程。

## 6. 提交边界

本计划不授权自动提交。实现、测试、审核和目检完成后只汇报 diff 与证据；仅在
用户明确要求 `提交/commit` 时按当前 Phase 分步提交规范执行，不 push。

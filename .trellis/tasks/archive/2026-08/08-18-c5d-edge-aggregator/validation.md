# C5d 验证记录

当前状态：实现、质量检查和运行期验证完成，等待用户明确提交授权。

## 核心与跨层覆盖

- `EdgeAggregator` 为每个 contributor 保留一份有界 `MapUpdateIngress` 状态；候选
  contributor、aggregate cells、manifest 和 producer baseline 仅在完整验证成功后原子交换。
- aggregate cell limit、geometry、manifest 或 producer prepare/commit 失败时，正式
  contributor、aggregate revision、`last_update` 和 producer baseline 不变。
- 同 voxel 使用 `Occupied` 优先，manifest 按 vehicle id 排序；Remove 保留
  `active=false` contributor 条目。
- route epoch/sequence gap 等异常进入 resync barrier；EdgeAggregator 按输入来源使用
  独立 resync service client，ack 必须匹配 producer/source，只有关联 correlation 的
  keyframe 才恢复。
- C5d 在 C4 ingress 之上额外拒绝未由 EdgeAggregator 请求的 route-epoch keyframe；这
  保证聚合器拥有来源恢复 correlation，而不改变 C4 的通用 ingress 语义。
- RuntimeAuthority 只以 Relay evidence 刷新 Relay heartbeat；EdgeAggregator capability
  evidence 不再误触发 Relay 心跳续期。
- RViz 配置使用单个 MarkerArray，显示聚合源、contributor 数量、aggregate revision 和
  degraded/available/resync 状态，并逐项显示 contributor active/inactive；resync 可见性
  由可靠 topology map-route epoch 前进驱动，避免丢失瞬态诊断。

## 构建与测试

执行环境：Docker `alien-scanner-dev`，ROS 2 Jazzy，2026-08-20。

```text
docker exec alien-scanner-dev bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /workspaces/alien-scanner/ws && \
   colcon build --symlink-install --packages-select perception_map_update swarm_data_plane swarm_controller'
Summary: 3 packages finished [43.2s]

docker exec alien-scanner-dev bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /workspaces/alien-scanner/ws && \
   colcon test --packages-select swarm_data_plane swarm_controller --event-handlers console_direct+'
100% tests passed, 0 tests failed out of 32
Summary: 2 packages finished [2min 10s]

docker exec alien-scanner-dev bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /workspaces/alien-scanner/ws && \
   colcon test-result --verbose'
Summary: 718 tests, 0 errors, 0 failures, 0 skipped
```

定向结果包括：`TestEdgeAggregator` 7/7、`TestRuntimeAuthority` 7/7、C5d N=5
launch test 2 个业务场景通过，包含双来源聚合、Relay 故障切换、旧 route 拒绝、两来源
correlated keyframe 恢复、resync Marker 状态和全进程干净退出。C5d launch test 连续
独立运行 3 次均通过。

## 证据文件

- `validation/colcon-build.log`
- `validation/colcon-test.log`
- `validation/colcon-test-result.log`
- `validation/edge-aggregator-unit.log`
- `validation/c5d-focused-test.log`
- `validation/c5d-launch-run-1.log`
- `validation/c5d-launch-run-2.log`
- `validation/c5d-launch-run-3.log`

以上日志与当前工作区代码同一轮生成；没有覆盖既有 `profiling-archive` 证据。

## 质量检查结论

- `git diff --check`：通过。
- `python -m py_compile ws/src/swarm_controller/test/test_c5d_edge_aggregator_runtime.py`：通过。
- 受影响边界已核对：`swarm_data_plane` 核心/ROS adapter、`swarm_controller` launch/RViz/test、
  `perception_map_update` 候选状态复制，以及 backend runtime spec。
- 复核期间发现并修复一次可视化时序问题及一次 C5d route-epoch barrier 缺口；修复后
  定向测试、三次 launch 复跑和全量回归均通过。

## 可视化复验

2026-08-20 在 Docker `alien-scanner-dev` 中使用宿主机 X11 转发重新启动：

```text
DISPLAY=host.docker.internal:0.0
ros2 launch swarm_controller c5d_edge_aggregator.launch.xml \
  show_rviz:=true show_cave_truth:=false
```

- RViz2 正常启动（`rviz2-23`），OpenGL 4.5 初始化成功，并加载
  `c5d_edge_aggregator.rviz` 的 `/swarm/runtime/markers` MarkerArray 配置。
- 运行期 Marker 样本显示 `aggregate r90 | active 2/2 | degraded no | resync no`，
  并包含 `explorer-0 active`、`explorer-1 active` contributor 明细。
- 发现并修复 C5d XML 顶层 `show_rviz` 与内部 sensing include 同名导致的条件变量覆盖：
  顶层先保存 `show_c5d_rviz`，内部 sensing RViz 仍固定关闭，C5d RViz 可按参数启动。
- 修复后定向重跑 `test_test_c5d_edge_aggregator_runtime.py`：2 个业务场景和 shutdown
  检查均通过，`100% tests passed`。

随后人工打开 RViz 时发现画面为空；检查 `ros2 topic info /swarm/runtime/markers -v`
显示原配置的订阅数为 `0`。原因是 C5d RViz 文件使用了非标准的 `Marker Topic` 键，
RViz 没有创建 MarkerArray 订阅。已改为标准 `Topic`（含可靠、Transient Local QoS）配置。
重启后同一检查显示：

```text
Publisher count: 1  (swarm_runtime_visualizer)
Subscription count: 1  (c5d_runtime_rviz)
```

RViz2 日志同时确认 OpenGL 4.5 初始化成功，MarkerArray 已接入运行期可视化链路。

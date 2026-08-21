# C5c Explorer 与纯 Relay 运行期验收记录

## 1. 验收身份

- 分支：`phase/5c-explorer-relay-runtime`
- 工作区基线：`99bc9f31236dfa89e6ed964157a08df4bff0c117`（C5c 改动保持未提交）
- 容器：`alien-scanner-dev`
- 镜像：`alien-scanner-jazzy:latest`
- 镜像 ID：`sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52`
- 验收时间：2026-08-18（Asia/Shanghai）

## 2. 构建与回归

执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/alien-scanner/ws
colcon build --symlink-install \
  --packages-up-to swarm_controller swarm_data_plane \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select swarm_data_plane swarm_controller
colcon test-result --verbose
```

结果：

- `swarm_data_plane` 和 `swarm_controller` 构建成功。
- 全量回归：`705 tests, 0 errors, 0 failures, 0 skipped`。
- C5c 定向 launch test：`675 tests, 0 errors, 0 failures, 0 skipped`。
- C5c launch test 使用 CTest `--repeat until-fail:3` 连续 3 次通过，每次约 7.4 秒。
- Python launch test 通过 `py_compile`；XML/YAML/package manifest 静态解析通过。

## 3. C5c 自动化场景

`test_c5c_explorer_relay_runtime.py` 覆盖：

1. 两个 source identity 经 Relay 0 到达两个中央 receiver；source/session/revision/hash/payload 保持不变，route epoch=1、hop=1。
2. 调用 `/c5c/relay_0/set_faulted` 仅注入 Relay 0 失联；authority 根据 heartbeat timeout 自动提交 topology/role/route epoch，Relay 1 接管。
3. 延迟的 route epoch=1 消息由 Relay 1 确定性拒绝，拒绝不刷新 origin freshness。
4. 新 route 产生 gap/resync；两个来源通过带 correlation ID 的 keyframe 收敛到 route epoch=2。
5. 调用 `/swarm/runtime/quiesce_explorer_0`，Explorer 进入 RuntimeHold，完成 Quiesced/HandoffReady ack，authority 提交 Draining role。
6. 所有 launch 进程退出码为 0。

## 4. RViz / runtime 证据

执行：

```bash
export ROS_DOMAIN_ID=224
export LD_PRELOAD=/opt/ros/jazzy/lib/x86_64-linux-gnu/liboctomap.so.1.10
ros2 launch swarm_controller c5c_explorer_relay.launch.xml show_rviz:=true
```

观察结果：

- `rviz2` 成功启动并订阅 `/swarm/runtime/markers`。
- `swarm_runtime_visualizer` 发布 transient-local `MarkerArray`，包含 `runtime_members`、`runtime_labels`、`active_map_routes`、`standby_map_edges`、`central_receiver` 和 `runtime_epoch`。
- marker 内容来自真实 topology/role/route snapshot；样例显示 2 Explorer、2 Relay、Active/standby 路径以及 `topology e18 | role e2`。
- 容器默认动态库搜索路径下，`octomap_rviz_plugins/OccupancyGrid` 会因 `OcTreeStamped` 符号冲突加载失败；使用上述 Jazzy `LD_PRELOAD` 后插件加载正常。这是容器运行时库路径问题，不影响 C5c core、launch test 或 runtime MarkerArray。该例外在人工验收记录中保留，不能表述为 OctoMap 显示已在默认环境无条件通过。

## 5. 修复记录

- `MapUpdateRouteProducerFixture` 的相关 keyframe 现在写入 resync service 返回的 `correlation_id`，避免接收端在错过首个 keyframe 后无法恢复。
- launch test 的跨测试场景状态改为类级保存，避免 `unittest` 为每个方法创建新实例导致旧 route 样本丢失。
- C5c fixture 显式等待 2500 ms 后再发布首个 Volatile keyframe，避免 DDS discovery 未完成时丢失唯一首帧；生产 QoS 与 wire 契约不变。
- C5c RViz 配置路径改为从 `swarm_controller` 包解析，和实际安装位置一致。

## 6. 验收结论

- C5c N=4（2 Explorer + 2 Relay；中央 receiver 不计入 fleet member）自动化 Gate：**通过**。
- Explorer runtime、纯 Relay 转发、route failover、旧 route 拒绝、correlated resync、role transition adapter 和 runtime visualization：**通过**。
- C5d EdgeAggregator、N=5（2 Explorer + 2 Relay + 1 EdgeAggregator）总验收：**未开始，不在本记录中提前宣称完成**。
- 生产默认 C4 Vector/Merkle v2 wire 不被 C5c 改写；C5c 只消费稳定的 topology/role/C4 opaque payload 契约。

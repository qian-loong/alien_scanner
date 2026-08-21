# C5b 验证记录

日期：2026-08-17
分支：`phase/5b-role-contract`
基线 HEAD：`6125023ca6904fb026b9105b5b96bc9aa0d94466`（C5b 改动尚未提交）
容器：`alien-scanner-dev`
镜像：`alien-scanner-jazzy@sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52`

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/alien-scanner/ws
colcon build --symlink-install \
  --packages-up-to swarm_data_interfaces swarm_data_plane \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select swarm_data_plane --return-code-on-test-failure
colcon test-result --verbose
```

结果：

- 构建通过：11 个依赖/目标包完成，0 失败。
- 测试通过：428 tests，0 errors，0 failures，0 skipped。
- `TestRoleState`：11/11 通过。
- `TestRoleConversions`：4/4 通过。
- `TestTopologyState`：9/9 通过。
- `TestTopologyConversions`：2/2 通过。
- `TestRoutedMapConversions`：5/5 通过，C4 MapUpdate v2 opaque 回归通过。
- C4 三个资源 smoke、closed-loop、visualization launch 和真实洞穴链路 launch 全部通过。

## 质量审阅与修正

- declared/effective capability、session 冻结、Joining/Resyncing 登记、Ready 门和旧 session
  拒绝均有固定 fixture。
- Explorer、Relay、EdgeAggregator、Reserve 和 Explorer + RelayService 组合已覆盖；Relay 与
  Aggregation graph prerequisite 分别有正反例。
- 双成员 quiesce/handoff、幂等 ack、unknown ack、rollback、Lost owner 强制撤销和 transition
  ID 有界重放窗口均通过。
- 审阅发现已提交 assignment 在 effective capability 收缩后仍可能通过新工作门；实现已改为
  每次读取最新 evidence，并补充 capability/health 回归。
- 审阅发现高频 evidence revision 会使完整 candidate 中未变化 assignment 被误判 stale；实现
  只要求新增/修改 assignment 引用当前 revision，未变化 assignment 保留原授权 revision。
- role candidate 现在拒绝 topology epoch 回退；ROS conversion 同时夹紧运行时 limit 和 wire
  schema 的 128-byte/sequence 上限。
- topology 与 role conversion 共用唯一 `VehicleIdentityConversions`，避免 session 字段转换漂移。

## Trellis 与静态检查

- `python ./.trellis/scripts/task.py validate .trellis/tasks/08-17-c5b-role-contract`：通过。
- `git diff --check` 与新增文件 trailing-whitespace 检查：通过；仅有既有父任务 JSON 的 CRLF/LF
  转换提示。
- `ros2 interface show`：`RoleSnapshot`、`CapabilityEvidence` 及嵌套消息的 bounded schema 与
  设计一致。
- ROS-free 边界：`RoleTypes` / `RoleState` 不包含 `rclcpp` 或 ROS message 依赖。
- backend code-spec 已同步到 `.trellis/spec/backend/swarm-role-contract.md`。

## 未执行项

- 未新增或打开 RViz 人工验收界面。C5b 只冻结权限契约，没有真实 Explorer/Relay/Aggregator
  运行期行为；角色可视化留给 C5c fixture。
- 未执行性能/内存矩阵。C5b 只增加有界小型状态和消息转换，没有建立性能隐门；真实 Relay 和
  多来源聚合负载分别在 C5c/C5d 后测量。
- 未实现 ROS action、任务 allocator、真实 Relay forwarding 或 EdgeAggregator map merge。

## 结论与后续边界

C5b 自动化质量门通过。C5a topology/session 之上已形成 declared/effective capability、一个
主角色、可组合服务、全局 role epoch、资源健康门和原子 transition barrier；C4 数据面回归
未受影响。

C5c 可直接消费 `RoleSnapshot` 实现 Explorer 与纯 Relay runtime，并把运行期 action/event 映射
到现有 transition API。C5d 可消费 EdgeAggregator + AggregationService 契约实现真实多来源
聚合。C6 只能把 role/effective capability 当 eligibility 输入，不得绕过 role epoch 自行授权。

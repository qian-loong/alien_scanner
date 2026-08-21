# C5a 验证记录

日期：2026-08-17
分支：`phase/5a-topology-foundation`
基线 HEAD：`16dcbcd3c517cc92cb413ee1953e14a4ce735f72`（C5a 改动尚未提交）
容器：`alien-scanner-dev`
镜像：`alien-scanner-jazzy@sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52`

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/alien-scanner/ws
colcon build --symlink-install --packages-up-to swarm_data_interfaces swarm_data_plane
source install/setup.bash
colcon test --packages-select swarm_data_plane
colcon test-result --verbose
```

结果：

- 构建通过：11 个依赖/目标包完成，0 失败。
- 测试通过：411 tests，0 errors，0 failures，0 skipped。
- `TestTopologyState`：9/9 通过。
- `TestTopologyConversions`：2/2 通过。
- `TestRoutedMapConversions`：5/5 通过，C4 opaque MapUpdate v2 转换回归通过。
- `test_swarm_data_plane_closed_loop.py`：2/2 通过。
- `test_c4_visualization.py`：2/2 通过。
- `test_c4_cave_visual_validation.py`：2/2 通过。

首次执行时发现两个测试夹具断言问题：component canonical 排序后不能依赖 `front()`，且
ROS bounded sequence 会在构造 65 个 member 时先于 conversion 抛出。测试分别改为按
`component_id` 查找，以及用 `TopologyLimits.max_members=1` 验证 conversion 的运行时上限；
修正后的完整包测试通过，未修改生产行为。

质量审阅还发现原实现只对当前 snapshot 保存 link/route epoch；稳定 ID 被移除后可能丢失
高水位，使相同旧 epoch 有机会重新加入。实现已增加有界 link/route epoch history，并补充
移除后重放、pending endpoint、资源上限、freshness 耗尽和单图改路回归；最终 411 项测试
全部通过。

## Trellis 与静态检查

- `python ./.trellis/scripts/task.py validate .trellis/tasks/08-17-c5a-topology-foundation`：通过。
- `git diff --check`：通过；仅输出既有父任务 JSON 的 CRLF/LF 转换提示。
- ROS-free 边界：`TopologyState` / `TopologyTypes` 不包含 `rclcpp`、Merkle 或 chunk 解析。
- ROS message 使用 bounded sequence；conversion 在 reserve 前应用 schema 与运行时双重上限。
- candidate 失败保持 committed snapshot；canonical 排序、session fence、descriptor drift、
  resync barrier、三图独立性、link/route epoch 高水位、TTL/freshness 均有单元测试覆盖。
- backend code-spec 已同步到 `.trellis/spec/backend/swarm-topology-contract.md`。

## 未执行项

- 未重新打开 RViz 做人工目检。C5a 不改变地图内容或 RViz 配置；既有 visualization 与真实
  洞穴链路 launch regression 已通过，因此人工界面不是本任务的阻塞门。
- 未执行 C5 性能/内存矩阵。C5a 只冻结拓扑契约，任务计划没有建立性能隐门；后续角色和
  EdgeAggregator 进入真实多来源负载后再单独设计测量。

## 结论与后续边界

C5a 自动化质量门通过。稳定身份、registration、membership 生命周期、`G_comm/G_control/G_map`
以及 link/route epoch 已形成可供 C5b 消费的确定性边界；C4 MapUpdate v2 仍保持 opaque。

回滚方式是关闭 topology adapter，继续使用 C4 单 source direct/routed path。C5b 可以基于
committed topology snapshot 设计 role/capability contract；C5c Relay/Explorer、C5d
EdgeAggregator 和 C6 task/lease 尚未实现，不在本结论中宣称完成。

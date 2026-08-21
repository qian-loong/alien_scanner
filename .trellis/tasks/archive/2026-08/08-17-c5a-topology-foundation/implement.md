# C5a 拓扑基础：实施计划

## 0. 规划与启动门

- [x] 从 `main` 创建 `phase/5a-topology-foundation`。
- [x] 创建本任务并关联父任务 `07-21-perception-swarm-architecture-refactor`。
- [x] 读取父 PRD/design/implement、C4 route/data-plane contract 和性能测量边界。
- [x] 完成 PRD convergence pass、方案评审和用户确认。
- [x] 运行 `task.py validate` 与 `task.py start`，进入实现阶段。

## 1. 身份与注册契约

- [x] 在 ROS-free 层定义稳定 fleet/vehicle/source/component/sensor identity、session 和 registration generation 的值约束。
- [x] 定义配置白名单、descriptor inventory、资源声明和 duplicate/old-session rejection。
- [x] 为身份排序、编码、长度/字符集/空值和 deterministic fixture 添加单元测试。

回滚点：只保留 C4 现有 source vehicle identity 和 route fixture，不改变已发布的 C4 MapUpdate v2。

## 2. Membership 生命周期

- [x] 实现 Absent/Joining/Resyncing/Ready/Draining/Lost/Quarantined 的 candidate transition。
- [x] 实现 membership/topology epoch 原子提交和失败不变式。
- [x] 明确 Frozen/Remove source 状态的拓扑表达，但不实现 C5d 聚合存储。
- [x] 覆盖 late join、rejoin、graceful leave、loss、duplicate ID、descriptor drift 和新 session resync。

回滚点：关闭 topology state adapter，保留 C4 单 source direct/routed path。

## 3. 三图与 link/route

- [x] 定义 G_comm/G_control/G_map 的节点、边、独立成员、link_id/link_epoch 和健康/资源字段。
- [x] 定义 route descriptor、route_epoch、hop、TTL/freshness 和 snapshot revision。
- [x] 实现旧 route/link epoch、未知成员、TTL/hop/资源超限的 fail-closed 规则。
- [x] 验证三图可独立变化，Relay 只透明转发而不修改 MapUpdate identity/payload。

回滚点：保留 C4 原有 route epoch ingress validator，新增 topology graph 只作为旁路候选。

## 4. 接口与转换

- [x] 根据设计评审确定 `swarm_data_interfaces` topology/membership/link/route 消息或服务的最小字段。
- [x] 在 `swarm_data_plane` 增加 ROS-free candidate/commit 与 ROS conversion；未知值、长度、epoch 和 budget 在分配前拒绝。
- [x] 不让算法库 include `rclcpp`，不让 Relay 解析 Merkle/chunk 内部。
- [x] 保持 C4 routed MapUpdate v2 的 payload、descriptor、digest、revision 和 hash 不变。

## 5. 确定性 fixture 与验证

- [x] 建立固定 seed 的多成员稀疏拓扑 fixture，覆盖至少两条不同逻辑路径、断链改路和重连。
- [x] 编写 ROS-free gtest：identity、registration、membership、graph、route、limits、atomicity、deterministic ordering（待容器执行）。
- [x] 运行现有 C4 opaque MapUpdate v2 透传回归，并执行 C5a conversion/integration tests：旧 epoch 拒绝、resync barrier 和诊断计数。
- [x] 验证与现有 C4 closed-loop/visualization fixture 的 route 行为不回归；本任务不改变 RViz 地图内容。

## 6. 质量门与收尾

- [x] 运行受影响包的 build/test，并记录精确命令、构建身份和结果。
- [x] 运行 `trellis-check`，完成跨层数据流、边界、测试覆盖和 spec 同步检查。
- [x] 运行 `trellis-update-spec`，只写入已稳定的 topology/membership/route contract。
- [x] 生成 C5a validation 报告，明确通过项、未执行项、回滚路径和 C5b 前置条件。
- [ ] 用户验收后按 `phase5(stepC5a): ...` 提交；不自动 push/merge。

## 7. 验证入口（规划值）

```bash
cd /workspaces/alien-scanner/ws
colcon build --symlink-install --packages-up-to swarm_data_interfaces swarm_data_plane
source install/setup.bash
colcon test --packages-select swarm_data_plane
colcon test-result --verbose
```

若实现新增独立 ROS-free 包，先更新本节和任务范围，再运行对应的精确包测试；不以全工作区通过替代受影响包结果。

## 8. 主要风险文件

- `ws/src/swarm_data_interfaces/msg/` 与必要的 `srv/`：跨进程 topology contract。
- `ws/src/swarm_data_plane/include/swarm_data_plane/`：ROS-free data-plane/topology 边界。
- `ws/src/swarm_data_plane/src/`：candidate/commit、route validation 和 conversion。
- `ws/src/swarm_data_plane/test/`：确定性 membership/graph/route fixture 与回归。
- `.trellis/spec/backend/swarm-data-plane-contract.md`：稳定 C4/C5 路由边界。

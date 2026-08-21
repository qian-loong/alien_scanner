# C5b 角色契约：实施计划

## 0. 规划与启动门

- [x] 从 C5a 收尾 HEAD 创建 `phase/5b-role-contract`。
- [x] 创建子任务并关联父任务 `07-21-perception-swarm-architecture-refactor`。
- [x] 读取父 PRD/design/implement、C5a archived artifacts 和 topology/data-plane code-spec。
- [x] 用户确认 AggregationService 首版只允许 EdgeAggregator 主角色。
- [x] 完成 PRD convergence pass 与方案自审，无未解决产品问题。
- [x] 用户确认最终 C5b 规划并授权进入实现。
- [x] 运行 `task.py validate` 与 `task.py start`，进入实现阶段。

## 1. Capability 契约

- [x] 定义 ROS-free capability kind、declared registration、effective evidence revision 和
  canonical ordering。
- [x] 实现同 session capability 冻结、effective subset、旧 session/revision 和 duplicate 拒绝。
- [x] 添加 Joining/Resyncing 可登记、仅 Ready 可获角色的固定 fixture。

回滚点：不创建 role state，仅保留 C5a membership/topology。

## 2. 主角色、服务和策略矩阵

- [x] 定义 PrimaryRole、ServiceKind、ServiceBudget、ServiceAssignment 和 RoleAssignment。
- [x] 实现一个主角色、service kind 唯一、required capability、graph prerequisite 和首版组合矩阵。
- [x] 覆盖 Explorer、Relay、EdgeAggregator、Reserve 和 Explorer + RelayService。

回滚点：保留 capability registration，不产生 committed RoleSnapshot。

## 3. Role snapshot 与原子 candidate

- [x] 定义全局 role epoch、topology epoch 引用、RoleSnapshot/RoleCandidate 和 RoleLimits。
- [x] 实现 stale topology/base role epoch、同 epoch conflict、非 Ready、资源超限和引用不闭合拒绝。
- [x] candidate 全部验证通过后一次提交，失败保持 snapshot 内容不变。

回滚点：使用静态 fixture assignment，不接入 coordinator runtime。

## 4. Transition 生命周期

- [x] 实现 Prepared/Quiescing/HandoffReady/Committed/RolledBack 状态和单 active transition 上限。
- [x] 实现 transition ID、changed member set、required ack、幂等 ack 和错误 session/ack 拒绝。
- [x] 实现 commit barrier 与 rollback，验证双成员 handoff 无双 owner、无半提交。

## 5. 健康与资源门控

- [x] 定义结构化 VehicleHealth/ResourceHealth evidence 与单调 revision。
- [x] 校验 effective capability、Unknown、LowPower、Failsafe、ComputeOverBudget 和 LinkDegraded。
- [x] 验证服务预算/健康独立，健康变化不静默获得新角色权限。

## 6. ROS 接口与转换

- [x] 在 `swarm_data_interfaces` 固定最小 capability/role/service/transition/snapshot 消息字段。
- [x] 使用 bounded sequence；conversion 在 reserve 前做 schema + runtime limit 双重预检查。
- [x] 保持 core ROS-free，不新增尚无 runtime owner 的 role action。

## 7. 测试与回归

- [x] 添加 `TestRoleState` 和 `TestRoleConversions`。
- [x] 构建 `swarm_data_interfaces` / `swarm_data_plane` 并运行完整 `swarm_data_plane` 测试。
- [x] 复跑 C5a topology/conversion 与 C4 opaque route/closed-loop regression。
- [x] 记录精确容器、镜像、HEAD、命令和结果；本步无独立性能隐门或 RViz 人工门。

## 8. 质量门与收尾

- [x] 运行 `trellis-check`，复核 cross-layer contract、limits、atomicity 和测试覆盖。
- [x] 运行 `trellis-update-spec`，新增稳定 role/capability/service code-spec。
- [x] 生成 validation 报告，明确 C5c/C5d 前置与未实现能力。
- [ ] 用户验收并明确授权后按 `phase5(stepC5b): ...` 提交；不自动 push/merge。

## 9. 验证入口（规划值）

```bash
source /opt/ros/jazzy/setup.bash
cd /workspaces/alien-scanner/ws
colcon build --symlink-install --packages-up-to swarm_data_interfaces swarm_data_plane
source install/setup.bash
colcon test --packages-select swarm_data_plane
colcon test-result --verbose
```

## 10. 主要风险文件

- `ws/src/swarm_data_interfaces/msg/`：跨进程 role/capability snapshot。
- `ws/src/swarm_data_plane/include/swarm_data_plane/`：ROS-free role/topology 边界。
- `ws/src/swarm_data_plane/src/`：candidate/transition/validation/conversion。
- `ws/src/swarm_data_plane/test/`：组合、迁移、资源和回归 fixture。
- `.trellis/spec/backend/`：稳定角色契约。

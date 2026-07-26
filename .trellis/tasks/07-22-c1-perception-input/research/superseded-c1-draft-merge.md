# 早期 C1 草案迁移记录

## 来源与结论

来源目录：`.trellis/tasks/07-22-perception-observation-model/`。

该目录是正式任务 `07-22-c1-perception-input` 之前留下的重复 C1 规划草案，
未完成 Phase 1，也未被激活。正式 C1 已使用自己的 `prd.md`、`design.md` 和
`implement.md` 进入实施。旧目录不能作为已完成任务归档；有效内容迁移后应从
父任务解绑并删除，避免同名任务继续污染 Trellis 状态。

## 本轮迁移内容

以下内容仍属于 C1 输入契约，并已合并到正式任务产物：

- 观测必须区分 `Hit`、`NoReturn`、`Invalid`；
- hit-only 点云不得静默解释为完整 free ray；
- 只有 descriptor 明确声明可靠 origin/direction/range 时才能产生相应
  free-space 证据；
- Q-C1-01 采用“原生 payload + 显式能力 + 按需查询”，不强制统一 ray array；
- capability 必须跨 ROS-free descriptor、单批 observation、健康门和 ROS 消息
  完整传播；
- PointCloud2 当前公共 schema 只有 XYZ/intensity，能力提升必须 fail closed；
- 使用确定性 contract tests 锁定数值分类与降级行为。

## 明确不在本轮迁移的内容

以下旧草案候选项不属于 AC-09 收口，不反向扩大已经实施的 C1：

- `ObservationWindow`、近似同步和多批缓存策略：由实际 mapper 消费需求驱动，
  在 C2 规划时决定；
- Phase 3 `scan_returns` 到新 mapper 的地图数值 replay：需要 C2 occupancy
  consumer，由 C2 conformance gate 承担；
- 大型点云缓存与远程 raw observation：分别属于 C2 资源设计和后续扩展；
- map epoch、旧 session 拒绝和 local map freshness：保持为 C2 边界。

这些内容不是被认定为“已完成”，而是保持在父任务/C2 的后续边界中。

## AC-09 设计决议

射线证据等级单调有序：

```text
HitOnly < HitRay < FullRay
```

- `HitOnly`：只能使用 occupied hit endpoint；
- `HitRay`：还可使用 origin 到 hit 的 free-space ray；
- `FullRay`：还可使用明确 no-return 到最大量程的 free-space ray。

`LaserScan` 原生 ranges 的分类规则：

- 量程内有限值：`Hit`；
- 正无穷：`NoReturn`；
- NaN、负无穷、量程外有限值：`Invalid`。

payload 分类与使用权限分离：即使某个 `LaserScan` batch 出现 `+inf`，只有
`FullRay` descriptor 才允许 C2 把该 beam 用作最大量程 free-space 证据。

## 方案审核修订

`gpt-5.6-sol / xhigh` 方案审核指出并已修订三项高置信问题：

1. PointCloud2 的安全门不能只依赖调用方先 `validate()`；`convert()` 直接调用
   也必须拒绝高于 `HitOnly` 的 descriptor。
2. `SensorRequirement.minimum_ray_evidence` 必须接入节点的 minimum/degraded
   运行时配置，默认 `hit_only`，非法值启动失败。
3. `HitRay`/`FullRay` LaserScan 必须验证 range、FOV、angular resolution 与冻结
   descriptor 一致，避免运行中元数据漂移扩大 free-space。

审核确认其余总体方向自洽，且没有把 occupancy 写入、窗口缓存、map epoch 或
旧 session 消费拒绝混入 C1。

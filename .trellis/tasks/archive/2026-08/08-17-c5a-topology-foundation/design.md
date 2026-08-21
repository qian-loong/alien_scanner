# C5a 拓扑基础：技术设计

## 1. 设计边界

C5a 只建立 topology/membership/link/route 的 ROS-free 领域边界和必要的 ROS 转换。它位于 C4 routed data plane 之上、C5b role contract 之下：

```text
C4 routed envelope + opaque MapUpdate v2
    -> C5a identity / registration / membership
    -> C5a G_comm / G_control / G_map + route epoch
    -> C5b capability / responsibility / lifecycle
    -> C5c Relay / Explorer
    -> C5d EdgeAggregator
```

业务算法库不 include `rclcpp` 或 ROS message；薄节点/转换层负责序列化、参数和收发。具体公开类名和 ROS 字段在实现前由接口评审决定，避免把父设计尚未固定的编码细节提前锁死。

## 2. 分层与文件所有权

- `swarm_data_interfaces`：承载跨进程 topology/membership/link/route 快照和事件所需的 ROS 接口；不放算法状态机。
- `swarm_data_plane`：继续拥有 C4 route ingress/egress，并新增 ROS-free topology state、candidate/commit、校验和测试适配边界；Relay 不解析 MapUpdate v2 内部。
- 若实现证明 C5a 状态库与 C4 data plane 产生不可接受耦合，再拆分独立 ROS-free 包；在此之前不提前新建包。
- `swarm_controller` 本步只提供测试或观察适配，不实现 role allocator、EdgeAggregator 或 C6 task lifecycle。

## 3. 身份模型

领域身份按以下层级区分：

```text
fleet_id
  -> stable vehicle_id
       -> vehicle_session_id (每次整机启动)
            -> component/source_id + component_session_id
                 -> sensor_id + frozen descriptor
```

规则：

1. stable ID 是业务主键；namespace/node/PID 仅用于寻址或诊断。
2. session 是运行实例，不可跨重启继承 map delta base、route、role 或 lease。
3. descriptor inventory 在 vehicle session 内冻结；descriptor 变化要求新 vehicle session。
4. 任何 ID/descriptor 都经过长度、字符集、空值和重复检查；fixture 可提供固定 session。
5. registration generation 只推进协调器接受的登记版本，不能降低 session/epoch 的拒绝门。

## 4. Registration 与 membership 状态机

注册以配置白名单为准，不进行开放式发现。一个 candidate registration 先完成身份、session、descriptor inventory、能力/资源声明的结构校验，再进入 membership 状态机：

```text
Absent
  -> Joining       identity/session/whitelist accepted
  -> Resyncing     link/health/clock/map prerequisites satisfied
  -> Ready         eligible for committed route/service view
  -> Draining      graceful revoke/handoff in progress
  -> Absent

Ready/Resyncing -> Lost or Quarantined -> Joining with a new accepted session
```

实现采用 committed snapshot + candidate transition：

1. 对 candidate 做 checked limits、旧 session、descriptor 和 epoch 校验。
2. 生成下一 membership/topology epoch 的完整 candidate view。
3. 只有所有关联 graph/link/route 引用闭合后才原子替换 committed view。
4. 失败只丢弃 candidate，保留最后合法成员、拓扑、路由和 source state。

Frozen contribution 不是 C5a 的地图汇聚实现，但 membership snapshot 必须能表达 source 已离线、不可接受 delta、仍保留 provenance 的状态，供 C5d 使用。

## 5. 三张逻辑图

统一的 graph snapshot 使用节点身份引用和 epoch 版本，但每张图拥有独立 edge 集合、健康字段和路由用途：

| 图 | 允许表达 | 本步禁止 |
| --- | --- | --- |
| `G_comm` | link_id/link_epoch、邻居、延迟/丢包/带宽、link health | 由通信连通性推导任务资格 |
| `G_control` | 控制目标、route epoch、hop/TTL、控制路径 | 修改地图 payload 或执行角色分配 |
| `G_map` | LocalMapUpdate 的 source/session 路径、map route epoch、resync prerequisite | 聚合 contributor、计算 Merkle、解释 chunk store |

图快照必须保留 source identity，不把 Relay 当成新的地图 source。不同图可以有不同成员、边、epoch 和 route；统一 topology epoch 只表示该 snapshot 的原子提交版本。

## 6. Link 与 route descriptor

每条逻辑链路至少包含 `link_id`、`link_epoch`、source/target、健康和资源预算引用。每条 route 至少包含 source/session、target、`route_epoch`、当前 hop、TTL/freshness budget 和拓扑 snapshot epoch。

- link 重建或连接属性改变推进 link epoch；路径切换推进 route epoch。
- link/route 从 committed snapshot 移除后，稳定 ID 的 epoch 高水位仍保留在有界历史中；
  后续重新加入必须使用更高 epoch，不能通过先删除再重放绕过单调性校验。
- receiver/ingress 只接受当前允许的 epoch 和正向 budget；旧 route、未知成员、TTL/hop/资源超限 fail closed。
- route forwarding 只更新 route metadata（如 hop/本地 receive/send provenance），不改 MapUpdate payload、source identity、map revision、content digest 或 Merkle descriptor。
- route snapshot 的排序、编码和诊断输出必须确定性，避免依赖 unordered container 迭代顺序。

## 7. C4 数据流与兼容性

```text
registration event
  -> membership candidate
  -> committed topology snapshot
  -> graph/route candidate
  -> C4 ingress admission
  -> opaque MapUpdate v2 forwarding
```

C5a 只增加 topology identity/route 边界，不改变 C4 的 v2 wire 内容身份。MapUpdate v2 的 descriptor、base/result digest、revision 和 update hash 完全由 C4 producer/receiver 负责；Relay 和 topology state 不复制或重新计算它们。

必要的 ROS conversion 必须在分配前验证 protocol/version、长度、ID、epoch、hop/TTL 和资源上限；未知枚举或版本原子拒绝。ROS-free core 可被固定 fixture 和后续 C5b/C5c 直接调用。

### 7.1 C5a ROS 接口字段冻结

本步只把 topology snapshot 和其中的身份、成员、链路、边、路径表达为消息，不把
状态机或 C4 `MapUpdate` payload 放进接口：

```text
VehicleIdentity
ComponentRegistration
SensorDescriptorIdentity
MemberRecord
LinkDescriptor
GraphEdge
RouteHop
RouteDescriptor
TopologySnapshot
```

`MemberRecord` 内嵌 registration generation、component/source session、冻结的 sensor
descriptor hash、membership state、availability 和四项 resync prerequisite。`RouteDescriptor`
只允许 `G_control` / `G_map`，`TopologySnapshot` 以 `topology_epoch` 作为原子提交版本。
消息数组使用与默认 `TopologyLimits` 对齐的 bounded sequence（member 64、link 256、edge
512、route 256、hop 64、每成员 component/sensor 32）；转换层在任何 vector reserve 前
再次检查运行时上限，并在返回 snapshot 前复用 ROS-free canonical 校验。C4 v2 地图内容
仍只能通过现有 routed envelope 透传。

## 8. 资源、诊断与确定性

所有 member/link/route/queue/contributor 上限使用 checked arithmetic；candidate 估算失败或超限时不分配不受控容器。诊断使用固定枚举和有界样本，至少覆盖：duplicate identity、old session、descriptor drift、registration rejection、stale link/route epoch、unknown member、resync barrier、TTL/hop/resource rejection 和 membership transition。

同一输入、同一 registration 顺序和同一 seed 必须产生相同 snapshot revision、排序、拒绝原因和 route digest（若为诊断摘要）。性能测量只在本任务验收后设计，不建立未经测量的隐含 CPU 门。

## 9. 回滚与迁移

- C5a 可通过关闭 topology adapter、保留现有 C4 direct/routed fixture 回退；C4 MapUpdate v2 路径不被覆盖。
- C5b/C5c/C5d 只能在 C5a committed snapshot/route contract 上实现；若 C5a Gate 失败，保留 C4 单链路和静态配置，不伪造多机 topology 能力。
- 不为兼容旧发布协议增加 v1/v2 双读或 runtime downgrade。

## 10. 风险

- 将三张图合并成一张会让通信健康、控制路径和地图路径隐式耦合；测试必须证明图可独立变化。
- 将 vehicle_id 当作 session 会导致重启旧消息被接受；所有 core rejection 测试必须覆盖旧 session。
- 把 registration generation 当成 inventory 热更新会污染 source/map epoch；descriptor drift 必须新 session。
- 在 C5a 引入角色字段或 EdgeAggregator 状态会提前锁死后续设计；本任务只保留引用和扩展点。

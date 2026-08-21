# C5a 拓扑基础：身份、成员与路由

## 1. 目标

在已完成的 C4 通信数据面之上，建立多无人机协作所需的第一层拓扑基础：稳定身份、启动会话、成员登记、成员生命周期、三张逻辑图和可版本化路由。C5b 角色契约、C5c Explorer/Relay、C5d EdgeAggregator 和 C6 多机任务只能消费本任务定义的稳定边界，不能各自发明身份或路由语义。

本任务的结果是可确定性验证的 topology/membership snapshot 与 route descriptor；它不要求本步完成具体角色行为或地图汇聚。

## 2. 已确认背景

- C4 routed envelope 已有 source identity、`route_epoch`、`hop_count`、`ttl_hops`、去重、断链和 resync 语义。
- C4.3 已将 v2 content descriptor/root 固定为生产契约；C5a 必须把 `MapUpdate` 当作 opaque payload，不读取 chunk store、Merkle Patricia 或其他地图内部结构。
- 父架构要求区分 `G_comm`（通信图）、`G_control`（控制图）和 `G_map`（地图图）；Relay 透传，只有后续 C5d 才在 `G_map` 聚合。
- 身份必须区分稳定 `vehicle_id` 与每次启动的新 `vehicle_session`；ROS namespace、node name、topic 和 PID 不能作为业务主键。
- fleet membership 使用配置白名单和受控动态 join/rejoin/leave；不实现开放式 ad-hoc discovery。

## 3. 需求

### R1 稳定身份与 session

- 定义 fleet、vehicle、component/source、sensor 的稳定身份层级。
- 每次整机启动生成唯一 `vehicle_session`；具体 producer 重启推进独立 source/component session。
- 同一 stable vehicle/source ID 的旧 session 不能被新 session 静默覆盖；旧 session 的消息、贡献和 owner 状态必须可拒绝或进入显式撤销流程。
- 身份字段有长度、字符集和空值校验，并能在固定 fixture 中显式注入确定性 ID。

### R2 registration 与 descriptor inventory

- registration 必须校验白名单、稳定身份、session、能力声明占位和资源预算。
- 同一 vehicle session 内冻结 sensor descriptor inventory；相同 descriptor 的 producer 可用新 component/source session 重连。
- 新增、移除或改变 sensor descriptor 必须启动新的 vehicle session，不能通过 registration generation 伪装成热变更。
- registration generation 只表示协调器接受的登记版本，不替代 session 或 topology epoch。

### R3 成员生命周期与原子拓扑视图

- 至少支持 `Absent -> Joining -> Resyncing -> Ready -> Draining -> Absent`，并能表达 `Lost`、`Quarantined` 和重新加入。
- Joining/Resyncing 成员不得进入当前 committed topology view；Ready 前必须完成身份登记、健康链路和必要 resync 前置。
- 成员变化必须原子推进 membership/topology epoch；不得出现半提交 membership、route 或 source contribution。
- 优雅退出先停止新任务和服务并完成撤销/交接；意外失联先使关联 route/service/task 不健康，再提交 Frozen/Remove 或恢复决策。

### R4 三张逻辑图

- `G_comm` 描述节点与物理/逻辑链路的连通、延迟、丢包、带宽和健康。
- `G_control` 描述控制对象的路由路径、route epoch、hop 和 TTL。
- `G_map` 描述 LocalMapUpdate、共享地图和未来 EdgeAggregator 的路径；地图 payload 仍从 LocalMapUpdate 开始。
- 三张图的成员、边和 epoch 可以不同；Relay 在三张图中保持透传，不因存在于 `G_map` 而获得聚合权限。

### R5 link/route epoch

- 链路具有稳定 `link_id` 和单调 `link_epoch`；路径变化推进 `route_epoch`。
- route descriptor 必须能表达 source/session、目标、hop/TTL/freshness 预算和当前 epoch。
- 旧 route、旧 link epoch、过期 TTL、未知成员和超出 hop/资源上限的消息必须 fail closed，并产生有界诊断。
- route 更新不得改变 source payload 的 identity、map epoch/revision、content digest 或 v2 descriptor。

### R6 确定性快照与资源边界

- topology snapshot、membership snapshot 和 route descriptor 必须有稳定排序、版本和可重复编码。
- 对 active members、pending registrations、links、routes、queue bytes 和 contributor 数量建立上限；超限确定性拒绝。
- 候选更新先校验身份、epoch、上限和引用完整性，全部通过后再原子替换 committed snapshot。

### R7 诊断与验证

- 诊断至少能区分 unknown identity、duplicate session、descriptor drift、stale epoch、route rejection、membership transition 和 resync prerequisite。
- 使用固定 seed 的 Level 0 fixture 验证重复 ID、旧 session、namespace remap、断链、改路、重连、late join、graceful leave 和失联。

## 4. 验收标准

- [x] 稳定身份、vehicle session、source/component session 和 sensor inventory 的层级及校验通过 ROS-free 单测。
- [x] 重复 stable ID、旧 session、未知/撤销成员和 descriptor drift 均被确定性拒绝，且不改变已提交 membership/topology/source state。
- [x] 成员生命周期和 registration/resync barrier 在固定 fixture 中原子收敛，Joining/Resyncing 不进入 Ready view。
- [x] `G_comm`、`G_control`、`G_map` 可以表达不同成员/边/epoch；快照排序和版本化编码可重复。
- [x] link epoch/route epoch、hop、TTL/freshness、旧 route 拒绝和路径切换通过 core/conversion 测试。
- [x] C4 MapUpdate v2 payload 在 route 透传前后保持字节和 identity 语义不变；C5a 不读取 Merkle 内部结构。
- [x] 资源上限、checked arithmetic、candidate failure atomicity 和 bounded diagnostics 通过测试。
- [x] 固定稀疏拓扑 fixture 至少覆盖多成员、断链/改路/重连和新 session resync；结果具备 provenance。
- [x] 任务文档、父任务依赖和相关 backend spec 在验收后同步；未实现的 C5b/C5c/C5d/C6 能力没有被提前宣称完成。

## 5. 非目标

- 不定义 Explorer、Relay、EdgeAggregator 的完整角色职责或角色迁移状态机；这些属于 C5b/C5c/C5d。
- 不实现跨来源地图聚合、contributor manifest 的生产汇聚或多 Region shared view。
- 不实现任务分配、Region matching、owner/lease 生命周期（C6）。
- 不实现 C9a/C9b 高可用、共识、主备接管或外部 quorum/fencing。
- 不改变 C4 MapUpdate v2 content identity、Merkle 算法、chunk store 或 receiver apply 逻辑。
- 不引入开放式 ad-hoc discovery；首版使用配置白名单和确定性 fixture。

## 6. 规划状态

实现与自动化质量门已完成，当前无阻塞；任务仍保持 `in_progress`，等待用户验收与明确提交授权。

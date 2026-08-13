# C4 通信数据面协议回归与洞穴人工验收

## 1. 验收分层

C4 使用两层验收：

1. 原四条确定性通信轨迹以 headless launch test 运行，不要求打开 RViz；
2. 一个真实洞穴场景用于 RViz 人工目检，显示 A1/A2/B 的位姿、轨迹、源地图、B 已接受
   的远端地图以及通信状态。

可视化必须把地图状态和通信状态分开：地图显示最后一次合法快照，通信异常显示在
独立的时间线/状态 Marker 中，不能把未知的丢失 delta 误画成空间障碍物。

## 2. Gap 与丢包判定

接收端已经接受 `sequence = N`，随后收到同一 producer/session/route epoch 的
`sequence = N + K`（`K > 1`）时，可以确定接收侧存在 gap：`N + 1 ... N + K - 1`
在当前接收顺序中缺失。该事实不等同于物理链路已证明丢包；原因也可能是乱序、延迟、
DDS history 淘汰或 endpoint 重启。

如果只有 `sequence = N`，之后没有任何消息，则必须由 freshness watchdog/validity timeout
判定静默过期。当前 C4 fixture 的主场景使用后续高序号到达触发 gap；静默超时属于后续
watchdog 验收，不改变本场景的 barrier 语义。

Gap 发生后：

1. 保留最后合法地图和其 revision；
2. 发布 `RejectedGap`/`resync_required` 诊断；
3. 发送幂等 `ResyncIntent`，请求当前完整 keyframe，而不是请求旧 sequence 的单个包；
4. barrier 期间拒绝普通 delta 和未关联的 keyframe；
5. 只接受匹配 `correlation_id` 的 keyframe；
6. keyframe 完整解析并原子提交后，以该 keyframe 的 sequence/revision 建立新基线。

发送方不保留无限历史。它保留当前权威快照和有界 resync ledger；如果发送方已推进到
revision 7，恢复 keyframe 应尽量直接携带最新 revision 7。恢复期间到达的普通 delta 可以
被拒绝，恢复 keyframe 成功后从其新 sequence 继续；迟到的旧 delta 不回滚地图。

## 3. 四项场景

### 3.1 Edge Recovery：A -> B

固定 source/receiver 链路：

```text
A source -> deterministic link -> B receiver
```

阶段：`READY BASELINE`、`DELTA DROPPED`、`GAP REJECTED`、`RESYNC RECOVERED`。

断言：

- gap 阶段地图 revision 与 baseline 相同；
- gap Marker 显示 expected sequence、received sequence 和 `RESYNC_REQUIRED`；
- 未关联 delta 不会产生地图 Marker；
- correlated keyframe 后 revision 增大，状态恢复为 READY；
- 恢复期间不等待或重放缺失的旧 delta。

### 3.2 Edge Aggregation：A1/A2 -> B

两个独立 contributor 发送到静态 aggregate fixture：

```text
A1 ─┐
    ├──> B aggregate
A2 ─┘
```

显示 contributor map 和 aggregate map；阶段至少覆盖 contributor 正常、单 contributor
gap、单 contributor resync、manifest/map 同 revision 原子恢复。

断言：

- A1 与 A2 的 source/session/revision 独立记录；
- A2 gap 不阻塞 A1 的合法状态；
- aggregate 不提交跨 revision manifest；
- 删除、epoch 切换和重新加入均以完整 aggregate 对象可见。

### 3.3 Upstream Recovery：B -> C

B 将 aggregate update + contributor manifest 发送给 central sink C：

```text
A1/A2 -> B aggregate -> C central
```

断言：

- C 只显示 revision 与 manifest 同步的 aggregate；
- B->C delta 丢失时 C 保留上一版完整 aggregate；
- correlation keyframe 到达后 C 原子恢复；
- service response 不携带大地图 payload。

### 3.4 Multi-hop TTL：B1 -> B2 -> C

固定两跳 relay，不实现动态拓扑发现：

```text
source/aggregate -> B1 relay -> B2 relay -> C
```

断言：

- relay 只改变 hop/forwarding metadata，不改变 payload、source revision 和 hash；
- hop count 逐跳增加，TTL 逐跳消耗；
- TTL 耗尽显示红色 route fault，并不提交地图；
- B1/B2 是同级 relay fixture，不暗示 C5 角色层级。

## 4. Headless 协议回归

以下四个 topic 继续由自动化 fixture 发布并由 launch test 断言：

- `Edge Recovery`：/c4/visualization/edge_recovery
- `Edge Aggregation`：/c4/visualization/edge_aggregation
- `Upstream Recovery`：/c4/visualization/upstream_recovery
- `Multi-hop TTL`：/c4/visualization/multihop_ttl

它们不再作为主要人工界面。测试直接检查阶段、sequence、revision、correlation、颜色语义
和 contributor/aggregate marker，保留快速回归价值。

## 5. 洞穴人工验收场景

固定布局：

```text
A1: (0, +1, 1.5) -> (8, +1, 1.5)
A2: (0, -1, 1.5) -> (8, -1, 1.5)
B : (0,  0, 1.5) -> (3.5, 0, 1.5)，随后悬停
```

A1/A2 使用真实 `fake_lidar -> perception_input_node -> perception_local_map_node` 管道，
各自发布独立 `LocalMapUpdate`。B 的 A1/A2 远端地图必须从 C4 `MapUpdateIngress` 的
已提交状态生成，不能直接复用源端 OctoMap。

人工观察顺序：

1. A1/A2 本地地图和 B 的两个接收副本正常增长；
2. B 停在 3～4 m，A1/A2 继续深入；
3. A2 首个 delta 被测试链路丢弃，B 的 A2 副本保持最后合法 revision；
4. 后续 A2 sequence 触发 gap，A2 链路/状态显示红色，A1 保持绿色；
5. resync 等待阶段显示黄色，并保留可配置的目检时间；
6. 相关 keyframe 到达后，B 的 A2 副本追到最新 revision，状态转绿；
7. source/receiver difference 只作为测试 oracle 单独显示，恢复后应消失或显著收敛。

## 6. 范围边界

- C4 只提供固定测试拓扑、数据面状态和可视化 evidence；不实现 C5 的成员发现、选举、
  动态角色迁移或路径规划。
- A/B/C 是 fixture capability，不是固定无人机类型；同一核心可被后续 C5 组合为 source、
  relay、aggregator 或 sink。
- RViz 视觉结果不是唯一正确性依据；sequence、revision、hash、correlation、ack 和
  状态 fingerprint 必须由 launch test 同时断言。

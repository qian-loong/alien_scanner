# 下一阶段感知与多机协作架构重构：总体决策

> 文档状态：已确认规划基线  
> 评审时间：2026-07-22  
> 评审评分：8.0/10 — 可以进入实施  
> 详细规划：`.trellis/tasks/07-21-perception-swarm-architecture-refactor/`

---

## 1. 概述

本文档记录了下一阶段感知与多机协作架构重构的**已确认架构决策**（D-000 至 D-031）。这些决策形成了后续子任务（C1-C8, C9a, C9b）的稳定基础，在实施过程中不应随意修改。

### 目标

建立下一阶段可评审、可分步实施的功能架构基线，使以下能力可以在不互相隐式耦合的前提下演进：
- 二维、三维及混合多雷达感知输入
- 本机状态、局部地图和本机安全闭环
- 地图 full/keyframe/delta/summary 更新语义
- 通信数据面、稀疏拓扑和角色模型
- 多 Region 任务分配及完整任务生命周期
- 诊断、确定性回放、性能比较和 N > 3 验收

### 范围约束

**本次包含**：
- 感知、地图、通信、拓扑、协作、执行和诊断的逻辑架构
- 稀疏拓扑与 Explorer/Relay/EdgeAggregator 的实际实现
- N=5 (2 Explorer + 2 Relay + 1 EdgeAggregator) 验收
- 恢复 Phase 3 的 3-9/3-10 延期需求

**本次不包含**：
- 选择具体真实 LiDAR 品牌和型号
- 本轮直接接入 Gazebo 或真实硬件
- 实现真实 VIO/LIO/SLAM、IMU/相机融合、飞控
- 自研分布式共识、quorum 存储或 fencing 基础设施

---

## 2. 架构原则

- **先逻辑语义，后传输绑定**：先定义契约，再选择 ROS/共享内存/网络编码
- **来源和时间可追溯**：任何转发、聚合或分配结果都保留 source、epoch/revision
- **本地优先安全**：全局协调只提供任务意图，不能授权穿越本机 unknown/occupied
- **控制面与数据面分离**：地图吞吐优化不隐式改变任务所有权
- **角色与部署分离**：Explorer、Relay、Aggregator 是逻辑职责，不绑定固定进程
- **一次只改变一类语义**：感知、地图、拓扑、任务分别建立可回滚验证门
- **基线可比较**：性能结论需要同输入、同参数、同构建类型和同采样窗口

---

## 3. 已确认架构决策（D-000 至 D-031）

### 3.1 基础决策（D-000 至 D-007）

#### D-000：算法替换机制
**决策**：主要算法在**进程启动时按配置选择**实现。

**内容**：
- 二次开发者实现固定 ROS-free C++ 端口并注册
- 由 launch/config 选择，在节点初始化阶段冻结
- 未知实现、能力不满足或配置无效必须启动失败并输出明确诊断
- 首版不支持运行中有状态热切换

**影响范围**：所有可替换算法端口（观测融合、mapper、聚合、Frontier、allocator、探索策略）

---

#### D-001：部署策略
**决策**：第一条可运行纵向链采用**中央协调迁移**，中央部署不属于逻辑接口。

**内容**：
- 中央 merger/shared-map view/allocator 是首个部署实现
- 逻辑接口允许后续替换为 Relay、EdgeAggregator 或分布式实现
- Phase 3 oracle、replay 和性能基线继续作为对比参考

**影响范围**：C5 拓扑、C6 多机协作

---

#### D-002：实施范围
**决策**：本次整体重构**必须实现稀疏拓扑**以及至少 Explorer/纯 Relay 纵向链。

**内容**：
- 不能只固定接口而不实现
- 必须验证 source 身份、三个逻辑图、route epoch/hop/ttl、断链与恢复
- 至少一条 `Explorer -> Relay -> 中央协调实现` 的完整数据路径

**影响范围**：C5 拓扑与角色、C8 集成验收

---

#### D-003：生产实现角色
**决策**：生产实现 **Explorer/Relay/EdgeAggregator**；Reserve 只定义稳定契约和测试替身。

**内容**：
- Relay 先以纯透传独立验收
- EdgeAggregator 作为独立实现验证地图聚合
- 不允许用一个"智能 Relay"同时冒充两种角色

**影响范围**：C5c Explorer + pure Relay 实现、C5d EdgeAggregator 实现

---

#### D-004：验收拓扑
**决策**：最低 **N=5**，角色为 2 Explorer + 2 Relay + 1 EdgeAggregator。

**内容**：
- 验证单 Relay 失效切换，中央协调器不计入 N
- N=3 中央直连保留为回归对照
- 两条 Relay 路径用于验证 route epoch 切换

**影响范围**：C8 集成验收

---

#### D-005：一致性策略
**决策**：按 source 有序、按视图原子、按健康门控。

**内容**：
- 每个 source 独立维护 session epoch/revision
- shared/global view 只发布已原子提交的 source revision 向量
- 中央 allocator 只基于健康视图发放或续约任务
- 首版不引入分布式共识和 Region 级部分可用

**影响范围**：C3 地图更新、C4 通信数据面、C6 多机协作

---

#### D-006：动态重配置
**决策**：中央协调器控制受限动态角色重配置，以 **role_epoch** 管理。

**内容**：
- 节点启动时发布能力声明
- 协调器以单调 role_epoch 发布角色分配和生效边界
- 不做节点自选举或算法热切换

**影响范围**：C5b 角色契约

---

#### D-007：角色组合模型
**决策**：节点采用**一个主角色 + 可组合服务能力**。

**内容**：
- 主角色决定探索/待命等任务资格
- Relay/Aggregation 服务分别进入相关逻辑图
- 服务具有独立资源预算和健康状态
- Explorer 可在预算允许时提供纯 Relay 服务

**影响范围**：C5b 角色契约、C5c/C5d 实现

---

### 3.2 感知与地图决策（D-008、D-013、D-020 至 D-025）

#### D-008：后端无关地图契约
**决策**：后端无关 occupancy 公共契约；**OctoMap 是默认参考实现**和兼容 adapter。

**内容**：
- 逻辑算法面向 OccupancyMapView/Update 工作
- 替代后端通过同一 conformance suite 验证
- 无法提供某项查询能力时必须显式声明

**影响范围**：C2 本机地图、C3 地图更新

---

#### D-013：坐标对齐
**决策**：每机保留 **source-local map frame**，以独立 alignment epoch/revision 对齐 shared frame。

**内容**：
- 首版使用静态/fixture alignment provider
- 在线地图配准作为后续扩展
- Alignment 变化会失效旧贡献并触发 keyframe 重建

**影响范围**：C2 本机地图、C3 地图更新、C5 拓扑

---

#### D-020：唯一 active mapper
**决策**：每个 vehicle session 只运行**一个 active mapping pipeline**，产生唯一 authoritative local occupancy source。

**内容**：
- 多雷达作为独立 observation 输入同一 pipeline
- 替代 mapper 在启动时选择，通过 replay/conformance suite 比较
- 首版不运行实时 shadow mapper

**影响范围**：C1 感知输入、C2 本机地图

---

#### D-021：Mapper 输入契约
**决策**：Active mapper 启动时声明 **minimum viable input** 和允许 degraded 组合。

**内容**：
- 仍满足最低能力时以 Degraded + 有效 capability 继续提交地图
- 低于最低能力时停止 revision、freshness 到期并触发 Frozen/Hold/任务撤销
- 恢复后重新通过健康门

**影响范围**：C1 感知输入、C2 本机地图、C6 多机协作

---

#### D-022：分层健康门控
**决策**：Degraded 按 **mapper、shared-view、role/task 和 local-execution** 分层门控。

**内容**：
- 合法地图证据与任务资格分开判断
- 中央协调器不得用全局布尔状态覆盖本机 known-free 安全拒绝

**影响范围**：C2 本机地图、C6 多机协作、C7 本机执行

---

#### D-023：位姿输入边界
**决策**：本轮不实现定位算法；以标准 **Odometry + TF** 绑定可替换的 ROS-free pose estimate。

**内容**：
- 保留 source/session、frame、quality/covariance、freshness 和 reset epoch
- 验证异常对地图、任务和本机安全的门控

**影响范围**：C1 感知输入、C2 本机地图

---

#### D-024：Pose reset 处理
**决策**：Pose source session/frame/reset epoch 不连续时 **fail closed**。

**内容**：
- 停止旧 map revision 和任务
- 旧贡献按旧 pose/alignment epoch Frozen/Removed
- 新 pose/alignment Ready 后建立新 map epoch 并 keyframe/resync
- 首版不原地重投影

**影响范围**：C2 本机地图、C3 地图更新

---

#### D-025：Mapper 部署位置
**决策**：C1-C8 的 authoritative mapper 位于 **vehicle-local compute domain**。

**内容**：
- Raw LiDAR/pose 在本机处理
- Fleet G_map 从 LocalMapUpdate 开始
- EdgeAggregator 只聚合 map update，不处理 raw LiDAR
- 远程 raw analytics/建图卸载延期

**影响范围**：C1 感知输入、C2 本机地图、C5 拓扑

---

### 3.3 通信与拓扑决策（D-009 至 D-017）

#### D-009：EdgeAggregator 契约
**决策**：EdgeAggregator 输出 **aggregate update + 有界 contributor manifest**。

**内容**：
- 内部保留 contributor 级贡献状态
- Aggregate update 与 manifest 原子一致
- 中央验证后直接应用，不重新构建每个 source 的完整地图

**影响范围**：C5d EdgeAggregator 实现

---

#### D-010：链路验收工具
**决策**：使用**确定性逻辑链路适配器/故障注入器**驱动真实 Relay/EdgeAggregator 进程。

**内容**：
- 固定 seed 下控制延迟、丢包、乱序、带宽、队列上限和断链/重连
- 验证应用数据面和恢复逻辑
- 不冒充 DDS 实现或无线物理仿真

**影响范围**：C4 通信数据面、C5 拓扑

---

#### D-011：跨进程接口
**决策**：独立 **`*_interfaces` ROS 包**承载跨进程领域契约。

**内容**：
- 原始 LiDAR 保持标准 sensor_msgs 消息
- 算法核心保持 ROS-free
- 节点负责消息转换

**影响范围**：C1-C8 所有子任务

---

#### D-012：交互模式
**决策**：稀疏链路传 **revisioned 领域消息**，端点 adapter 提供 topic/service/action。

**内容**：
- 地图/状态使用 topic
- 重同步使用短 service（返回 correlation ID）+ 异步 keyframe
- 任务/角色迁移使用 action
- 纯 Relay 不代理 action 内部协议

**影响范围**：C4 通信数据面、C5 拓扑

---

#### D-014：时间契约
**决策**：Origin 携带 **clock domain/session** 仅用于溯源。

**内容**：
- 逻辑顺序由 epoch/revision/sequence 保证
- 每跳用本地 monotonic validity budget
- Lease 使用单调 renewal sequence
- 不依赖跨域绝对时间排序

**影响范围**：C3 地图更新、C4 通信数据面、C6 多机协作

---

#### D-015：身份与 session
**决策**：用户配置 **stable vehicle/source ID**，每次启动生成新 session。

**内容**：
- 协调器登记 capability
- Namespace 仅寻址，不作为业务主键
- 重复活跃 ID 隔离，旧 session 全面失效

**影响范围**：C5a Identity/session 基础

---

#### D-016：成员管理
**决策**：配置**白名单**允许受控动态 join/rejoin/leave。

**内容**：
- Ready 前完成 resync
- 成员变化推进 topology/view epoch
- 开放式 ad-hoc discovery 不在首版范围

**影响范围**：C5a Identity/session、C5b 拓扑

---

#### D-017：离线贡献
**决策**：离线 source 默认保留为 **Frozen contributor**。

**内容**：
- 不计 live health、不接 delta
- Frozen-only frontier 不单独分配任务
- 按 alignment/资源/删除策略管理

**影响范围**：C5 拓扑、C6 多机协作

---

### 3.4 控制与安全决策（D-026 至 D-029）

#### D-026：控制接口解耦
**决策**：本机规划/安全使用 ROS-free **MotionIntent/ExecutionFeedback**。

**内容**：
- 具体 FakeOdom、Gazebo controller 或真实 autopilot 经 adapter 接入
- 本轮不实现飞控
- PoseStamped motion_goal 仅为 FakeOdom 兼容绑定

**影响范围**：C7 本机执行

---

#### D-027：单一执行权威
**决策**：每 vehicle session 一个 **local execution authority**。

**内容**：
- Coordinator 只发布任务/角色
- Local safety 与 external failsafe 可抢占
- Control_authority_epoch/TTL 拒绝旧命令和反馈
- Relay 不获得 actuator authority

**影响范围**：C6 多机协作、C7 本机执行

---

#### D-028：健康状态契约
**决策**：VehicleHealth/ResourceHealth 统一表达 battery、failsafe、actuator、compute、storage、link 状态。

**内容**：
- Unknown 不等于 Healthy
- Adapter/fixture 驱动角色、服务、任务和本机执行门控
- 首版不做能源规划

**影响范围**：C5 拓扑、C6 多机协作、C7 本机执行

---

#### D-029：消息认证
**决策**：领域 envelope 验证 identity、session、authority epoch、sequence、credential state 和 replay window。

**内容**：
- 未知/未认证/撤销/过期/replay 消息只能诊断，不能改变控制状态
- 真实 PKI 下放部署子任务

**影响范围**：C4 通信数据面、C5 拓扑

---

### 3.5 高可用决策（D-018、D-030）

#### D-018：HA 三层演进
**决策**：HA 分三层推进。

**C1-C8**：单 Active 完成 N=5 核心验收
- 不依赖 standby 或外部 quorum
- 控制对象已携带 authority 元数据

**C9a**：同机 Warm Standby
- 在同一主机验证状态复制、fencing、promotion 和 resync
- 明确标注"不宣称整机容灾"

**C9b**：跨机协调器 HA
- 将主备部署到独立故障域
- 接入成熟的外部 quorum/fencing
- 形成主机级 HA

**共同约束**：
- 三层均不增加无人机数量（N=5 保持不变）
- 业务包不手写共识

**影响范围**：C9a 同机 HA、C9b 跨机 HA

---

#### D-030：状态持久化
**决策**：权威运行状态默认 **ephemeral**。

**内容**：
- 进程重启推进新 session/epoch 并通过 resync 重建
- 不跨重启保留 lease/intent
- C9a 复制控制前缀验证接管
- C9b 外部持久保存 authority term
- 持久化地图/日志作为后续扩展

**影响范围**：C9a 同机 HA、C9b 跨机 HA

---

### 3.6 其他决策（D-019、D-031）

#### D-019：传感器 inventory
**决策**：同一 vehicle session 内**冻结传感器 descriptor inventory**。

**内容**：
- 相同 ID/descriptor 的 producer 可在 component/source session 更新后重连
- 增删传感器或改变 geometry/capability/安装 descriptor 必须建立新 vehicle session 并重新 registration/resync

**影响范围**：C1 感知输入、C5a Identity/session

---

#### D-031：仿真策略
**决策**：分级仿真验收策略。

**Level 0（纯软件）**：
- FakeLidar/FakeOdom/确定性链路适配器
- 完全可重复
- C1-C8 在此验收

**Level 1（软件仿真）**：
- 可选 Gazebo 用于噪声注入和可视化
- 非核心依赖

**Level 2/3（半物理/实物）**：
- 真实硬件集成
- 留作独立扩展

**影响范围**：C1-C8 所有子任务

---

## 4. 子任务顺序与依赖

```text
C1 (感知输入与观测数据模型)
  └─> C2 (本机观测消费与局部地图)
       └─> C3 (地图 snapshot/keyframe/delta 模型)
            └─> C4 (通信数据面)
                 └─> C5 (拓扑与角色)
                      ├── C5a: Identity/session/link 基础
                      ├── C5b: 三个逻辑图与 route epoch
                      ├── C5c: Explorer + pure Relay 实现
                      └── C5d: EdgeAggregator 实现
                      ├─> C6 (多 Region 与任务生命周期)
                      │    └─> C7 (本机恢复与执行状态)
                      │         └─> C8 (集成与 N>3 总验收)
                      │              └─> C9a (同机 HA 协议验证)
                      │                   └─> C9b (跨机协调器 HA)
```

### 关键路径
C1→C2→C3→C4→C5→C6→C7→C8 是关键路径。C9a 和 C9b 是可选的 HA 扩展，不阻塞核心功能。

### 并行机会
无。由于每个子任务都改变核心语义，必须顺序验证。

---

## 5. 验收门槛

### 父任务最终验收（C8 完成后）

**必须通过的验收维度**（30 项）：
1. 感知来源替换：FakeLidar 与标准 2D/3D fixture 进入同一逻辑边界
2. 多雷达：不同 origin 的 batch 不被错误合并
3. 本机地图权威性：每 vehicle session 唯一 active mapper
4. 输入能力降级：mapper Healthy/Degraded/Unavailable 转换正确
5. 分层健康门：mapper、shared view、role/task、local execution 独立门控
6. 位姿输入：FakeOdom 与 Odometry/TF fixture 进入同一契约
7. Pose reset：source/session/frame/reset 变化关闭旧链并推进新 epoch
8. Mapper 部署：authoritative mapper 在 vehicle-local domain
9. 飞控边界：MotionIntent/ExecutionFeedback 与具体协议隔离
10. 控制权威：单一 local authority、control epoch、抢占和停止确认
11. Vehicle health：按字段门控，Unknown 不视为 Healthy
12. 消息信任：spoof/replay/过期 credential 被拒绝
13. 地图等价：keyframe + delta 重建与 snapshot 内容一致
14. 数据面恢复：重复、乱序、缺 delta 可恢复
15. 链路故障：确定性链路适配器驱动真实 Relay/Aggregator
16. 跨进程契约：标准 LiDAR 消息与独立 interfaces 包边界明确
17. 交互映射：topic/service/action 映射到领域协议
18. 坐标对齐：source-local map 经 alignment 进入 shared view
19. 时间语义：origin clock domain 与每跳 duration 分离
20. 身份/session：stable ID 与 namespace 分离，旧 session 全部拒绝
21. 动态成员：white list session 可 late join/rejoin/leave/loss
22. 离线贡献：最后合法 source 可 Frozen 保留
23. 协调器权威基线：C1-C8 单 Active 不依赖 standby
24. 同机 HA 协议：C9a standby 不计 fleet N
25. 跨机 HA：C9b 主备位于独立故障域
26. 拓扑：稀疏拓扑和纯 Relay 链可运行
27. 任务：多 Region、唯一 owner、lease/revoke/reassign
28. 本机安全：断链时 body/segment known-free 契约仍成立
29. 扩展性：N=5 异构角色完成单 Relay 故障切换
30. 性能：与 Phase 3 Release 基线同条件对比

---

## 6. 风险与控制

| 风险 | 控制措施 |
|------|---------|
| 同时改感知和地图导致 free-space 回归 | C1/C2 分开；固定 ray fixture 和地图重建 oracle |
| Delta 节省网络但中央仍全量重建 | 接收端直接应用 delta；分别测传输与处理成本 |
| Relay 与 Aggregator 职责混合 | 第一版 Relay 纯透传；Aggregator 单独子任务 |
| 拓扑变化造成旧数据冒充新鲜数据 | Origin stamp、source epoch/revision 与 forward stamp 分离 |
| 分布式任务出现重复 owner | 首版采用中央协调迁移，明确一致性协议 |
| 为产生多 Region 放宽安全或阈值 | Detector/Component 单独行为评审；本机安全不变 |
| N > 3 只验证节点能启动 | 加入资源上限、断链、角色变化和任务唯一性判据 |

---

## 7. 相关文档

- **详细规划**：`.trellis/tasks/07-21-perception-swarm-architecture-refactor/`
  - `prd.md`：需求、约束和验收标准
  - `design.md`：技术设计和决策理由
  - `implement.md`：实施计划和子任务顺序
  - `ARCHITECTURE-REVIEW.md`：系统性方案评审报告（8.0/10）
  - `CONVERGENCE-REPORT.md`：PRD 收敛清理报告（40 条 → 20 条）
  - `research/architecture-decision-inventory.md`：决策清单

- **历史决策**：
  - `docs/decisions/phase-03-swarm-qa.md`：Phase 3 收口与遗留问题
  - `docs/phases/phase-03-swarm.md`：Phase 3 实施记录

---

## 8. 变更记录

| 日期 | 版本 | 变更内容 |
|------|------|---------|
| 2026-07-22 | v1.0 | 初版，确认 32 个架构决策（D-000 至 D-031） |

---

**文档状态**：✅ 已确认规划基线，可以创建子任务并进入实施阶段

# PRD §2 "已确认事实" 与决策点重复分析

## 📊 完全重复的事实（建议删除）

### 🔴 事实 #10 vs D-000

**PRD §2 事实 #10**:
> 主要算法扩展采用"进程启动时按配置选择实现"：二次开发者实现固定 ROS-free C++ 端口并注册实现，由 launch/config 选择；首版不支持运行中有状态热切换。

**Design D-000**:
> `Accepted(D-000)`：主要算法在**进程启动时按配置选择**实现。二次开发者实现固定 ROS-free C++ 端口并注册，由 launch/config 选择；选择在节点初始化阶段冻结。未知实现、能力不满足或配置无效必须启动失败并输出明确诊断。首版不支持运行中有状态热切换，也不定义热切换所需的状态迁移、并发切换或回滚协议。

**重复度**: 95% - 几乎逐字相同  
**建议**: 删除事实 #10，D-000 更详细且有诊断要求

---

### 🔴 事实 #11 vs D-001

**PRD §2 事实 #11**:
> 第一条新架构纵向链采用中央协调迁移：中央 merger/shared-map view/allocator 可以作为首个实现继续存在，但中央部署不属于逻辑接口；感知、地图更新、通信与任务契约必须允许后续替换为 Relay、EdgeAggregator 或分布式实现。

**Design D-001**:
> `Accepted(D-001)`：采用"中央协调迁移，接口先可替换"方案。中央 merger/shared-map view/allocator 是首条纵向链的部署实现，不是逻辑接口；后续可替换为 Relay、EdgeAggregator 或分布式实现。

**重复度**: 90% - 表达同一决策  
**建议**: 删除事实 #11，D-001 已完整表达

---

### 🔴 事实 #38 vs D-028

**PRD §2 事实 #38**:
> VehicleHealth/ResourceHealth 是跨模块的外部可替换契约：电池、autopilot failsafe、执行器、计算、存储和链路资源分别报告状态，允许 `Unknown` 但 Unknown 不等于 Healthy。首版用标准 BatteryState/诊断 adapter 和固定故障 fixture 驱动 role eligibility、Draining/handoff、任务撤销、服务降级与本机 Hold，不实现能耗预测、充电规划或能源最优调度。

**Design D-028**:
> `Accepted(D-028)`：VehicleHealth/ResourceHealth 统一表达 battery、failsafe、actuator、compute、storage、link 状态；Unknown 不等于 Healthy，adapter/fixture 驱动角色、服务、任务和本机执行门控，首版不做能源规划。

**重复度**: 85%  
**建议**: 删除事实 #38，D-028 已涵盖

---

### 🔴 事实 #39 vs D-029

**PRD §2 事实 #39**:
> 跨进程领域消息必须有可验证的 producer/coordinator identity、session、authority epoch、sequence、credential state 和 replay window。未知、未认证、已撤销或过期凭据的消息只能进入诊断，不能改变 membership、map、role、task 或 MotionIntent authority；C1-C8 只验证逻辑 trust/auth adapter 和故障注入，不选择或实现完整 PKI。

**Design D-029**:
> `Accepted(D-029)`：领域 envelope 验证 identity、session、authority epoch、sequence、credential state 和 replay window；未知/未认证/撤销/过期/replay 消息只能诊断，不能改变控制状态；真实 PKI 下放部署子任务。

**重复度**: 85%  
**建议**: 删除事实 #39，D-029 已涵盖

---

## 🟡 高度相关可合并的事实组

### 组 A: Phase 3 基线（事实 #2, #3, #4, #5）

**事实 #2**:
> Phase 3 只认定 3-1 至 3-8 完成。3-9 的基础实现和诊断被冻结，但多 Region、非零 eligible edge/matching、唯一 `Assigned` owner 和真实任务生命周期仍是强制延期项；3-10 等待新架构后重定。

**事实 #3**:
> Phase 3 已冻结 revision、原子提交、Detector oracle、task epoch/revision/lease、allocator 后台管线、KnownFreePathChecker 安全契约、Demo 和 replay 资产，这些是重构比较基线，不是必须保留的部署结构。

**事实 #4**:
> 当前 `/drone_i/octomap` 是 `fullMapToMsg()` 完整快照。无损 source-level replay 可离线推导相邻 snapshot 的变化，但不能还原逐 scan 更新、source session epoch、乱序、丢包或 Relay 路由。

**事实 #5**:
> 固定 Release 三机证据显示 merger 的完整 source decode + normalize 约占 cycle p50/p90 的 60%；仅改变 peer 拓扑或增加 Relay 不会自动消除中央完整地图处理成本。

**合并建议**:
> **Phase 3 冻结基线与重构比较要求**：3-1~3-8 完成，3-9 核心延期（多 Region、唯一 owner、真实生命周期）。已冻结 revision、oracle、replay、allocator pipeline、KnownFreePathChecker 作为重构比较基准。当前 fullMapToMsg 快照模式与 merger 60% 处理成本是性能基线；无损 replay 可推导 delta 但不还原 session epoch、路由或网络行为。

**效果**: 4 条 → 1 条，减少 75%

---

### 组 B: 感知输入（事实 #6, #7, #8）

**事实 #6**:
> 当前 FakeLidar 发布命中点 `PointCloud2` 和带自定义字段的 `scan_returns` PointCloud2；OctoMap Builder 直接依赖后者。倾斜扫描几何还通过 `ring_pitch_rad` 进入 FakeLidar 与高度适配器。

**事实 #7**:
> 已形成感知候选设计：二维输入使用标准 `LaserScan`，三维输入使用标准 `PointCloud2`；每个传感器保留独立 `sensor_id`、frame、采样时间和 origin；不把 `/scan_returns` 作为长期公共接口。

**事实 #8**:
> 当前不接入 Gazebo，也不选择真实雷达品牌。接口应允许 FakeLidar、未来 Gazebo bridge 和真实驱动进入同一标准 ROS 边界。

**合并建议**:
> **标准传感器接口与扩展性**：当前 FakeLidar 使用自定义 `scan_returns`，重构后采用标准 LaserScan/PointCloud2，保留独立 sensor ID/frame/origin。接口设计允许 FakeLidar、Gazebo bridge 和真实驱动无差别接入，不依赖具体品牌。

**效果**: 3 条 → 1 条，减少 67%

---

### 组 C: 健康与降级（事实 #31, #32）

**事实 #31**:
> Active mapper 在启动时声明 minimum viable input contract 和允许的 degraded 输入组合。运行期 sensor 健康变化但不切换 mapper：健康输入仍满足最低契约时继续推进 authoritative map revision，并发布 `Degraded` 与当前有效能力；不再满足时进入 `Unavailable`、停止提交新 revision，地图 freshness 到期，shared contribution 和任务按 Frozen/Hold/撤销规则处理。

**事实 #32**:
> `Degraded` 采用按消费者分层门控，而不是一个全局布尔值：mapper health 决定能否提交地图，shared-view health 决定贡献能否进入协调视图，role/task eligibility 按当前有效 capability 判断，本机执行独立执行 freshness 与 body/segment/path known-free 检查；中央协调器不得覆盖本机安全判断。

**合并建议**:
> **分层健康门控策略**：mapper 声明 minimum viable input，满足时 Degraded 运行，低于时 Unavailable 停止 revision。健康采用分层门控（mapper/shared-view/role-task/local-execution），非全局布尔值，中央不得覆盖本机安全。

**效果**: 2 条 → 1 条，减少 50%

---

## 📊 合并效果预测

| 类别 | 当前事实数 | 删除完全重复 | 合并相关组 | 最终数量 | 减少比例 |
|------|-----------|-------------|-----------|---------|---------|
| Phase 3 基线 | 4 | - | 3 | 1 | -75% |
| 感知输入 | 3 | - | 2 | 1 | -67% |
| 架构决策 | 9 | -4 | - | 5 | -44% |
| 地图与通信 | 9 | - | 3 | 6 | -33% |
| 安全与控制 | 8 | -2 | 2 | 4 | -50% |
| 身份与时间 | 5 | - | 1 | 4 | -20% |
| 仿真 | 1 | - | - | 1 | 0% |
| **总计** | **40** | **-4** | **-11** | **22** | **-45%** |

**实际建议**: 考虑到可读性，最终精简到 **19 条**更合适（额外删除 3 条次要事实）。

---

## 💡 精简原则总结

1. **完全重复** → 删除（引用决策点）
2. **高度相关** → 合并（一个主题一条）
3. **实施细节** → 移至功能需求或设计文档
4. **Phase 3 遗产** → 压缩为背景说明
5. **保留核心** → 现状、方向、约束

---

## 下一步

请审阅以上对比，确认：
1. 是否同意删除 4 条完全重复的事实（#10, #11, #38, #39）？
2. 是否同意合并 Phase 3 基线（4→1）、感知输入（3→1）、健康门控（2→1）？
3. 是否需要看更多合并组的详细对比？

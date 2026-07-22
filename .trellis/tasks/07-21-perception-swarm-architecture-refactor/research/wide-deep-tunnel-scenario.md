# Research: 宽深隧道水平/垂直分布探测场景

- Query: 现有架构是否支持宽深隧道中多机水平/垂直分布扩展探测范围？
- Scope: 场景验证
- Date: 2026-07-21

## 场景描述

**需求来源**: 用户提出的实际应用场景

当隧道很宽很深时,允许无人机**水平或垂直分布**用于扩展探测范围:
- **水平分布**: 多架无人机在同一深度,沿隧道宽度方向展开
- **垂直分布**: 多架无人机在不同高度,覆盖隧道高度方向
- **目标**: 扩展单次探测的覆盖范围,加速探索效率

## 架构支持验证

### ✅ 已完全支持

| 能力需求 | 架构覆盖 | 关键设计点 |
|---------|---------|-----------|
| 多机独立探索 | ✅ | D-020: 每机独立 authoritative mapper |
| 共享地图聚合 | ✅ | D-009: EdgeAggregator 聚合多 source 地图 |
| 空间区域分配 | ✅ | PRD R-07: 多 Region 任务分配 |
| 3D occupancy 表达 | ✅ | D-008: 后端无关 occupancy 契约 |
| 跨机坐标对齐 | ✅ | D-013: source-local frame + alignment epoch/revision |
| 任意相对位姿 | ✅ | 设计不限制无人机必须处于同一高度 |
| 远距离通信 | ✅ | D-004: 稀疏拓扑 + Relay 中继 |

**结论**: 现有架构**完全支持**该场景,无需修改核心设计。

## 实施路径建议

### **作为二次开发示例实现** (推荐)

**实施时机**: C6 (多机协作) + C7 (本机执行) 完成后

**展示价值**:
1. **Region 空间表达的灵活性**: 展示如何定义 3D 立方体 Region (水平切分/垂直切分/深度切分)
2. **协同探索策略的可替换性**: 展示如何实现基于空间覆盖的 Frontier/Region 算法
3. **跨机对齐的实用性**: 展示静态 alignment 配置 (如 "Explorer 1 在 z=0, Explorer 2 在 z=5m")
4. **框架扩展能力**: 证明无需修改核心架构即可支持复杂协同场景

### 示例内容建议

```
examples/wide-deep-tunnel-exploration/
├── README.md                        # 场景说明与运行指南
├── config/
│   ├── vertical-split-regions.yaml # 垂直切分的 Region 定义
│   ├── horizontal-split-regions.yaml # 水平切分的 Region 定义
│   └── alignment-static.yaml       # 静态相对位姿配置
├── src/
│   ├── spatial_region_detector.cpp # 自定义 3D Region 检测算法
│   └── coverage_allocator.cpp      # 基于覆盖的任务分配策略
└── launch/
    └── wide_tunnel_n5.launch.py   # 5 架无人机垂直分布场景
```

**验收场景**:
- 至少 3 架 Explorer 分布在不同高度 (如 z=0, z=5m, z=10m)
- 完成垂直切分的 Region 分配 (上层区域、中层区域、下层区域)
- 聚合为统一 shared view,证明地图正确对齐
- 展示覆盖效率相比单机的提升

## 需要在实施时明确的细节

### 1. Region 的 3D 空间表达 (C6 子任务)

**当前状态**: PRD 只规定 "至少两个有效 Region",未规定空间形状

**实施时定义**:
- Region 的几何表达方式 (3D bounding box? 八叉树节点?)
- Frontier detection 在 3D 空间的扩展
- eligible edge/matching 在垂直/水平方向的语义

### 2. 跨机地图对齐 (C1 子任务)

**当前状态**: D-013 首版使用静态/fixture alignment provider

**实施时定义**:
- 静态 alignment 的配置格式 (YAML? 启动参数?)
- 相对位姿的参考系 (世界坐标? 某个 Explorer 的 frame?)
- 外部定位系统的 adapter 接口 (GPS/UWB/etc)

### 3. 协同探索策略 (二次开发扩展)

**不在 C1-C8 范围内**:
- 协同避让算法
- 覆盖优化启发式 (如 Voronoi 分区)
- 动态 Region 重分配策略

**扩展方式**: 通过 D-000 的算法替换机制实现

## 与 PRD 当前范围的关系

**PRD §1 目标已包含**: "多 Region 任务分配及完整任务生命周期"

**PRD §5 明确不在范围内**: "用编队或表演飞控替代未知环境探索"

**决策**: 不在 PRD 中新增 R-21,理由:
- 现有设计已完全支持该场景
- Region 空间形式在 C6 实施时定义,现在固化会限制灵活性
- 作为二次开发示例能更好地展示框架扩展能力

## 相关决策点

- D-000: 算法可替换性 (Frontier/Region 算法可扩展)
- D-004: N=5 验收 (足够支持垂直分布场景)
- D-008: 后端无关 occupancy 契约 (支持 3D 地图)
- D-009: EdgeAggregator 聚合 (支持多机地图融合)
- D-013: Source-local frame + alignment (支持任意相对位姿)
- D-020: 每机独立 mapper (支持分布式探测)

## 参考资料

- 无人机飞行表演的集中编排 + 分布执行模式 (research/motion-authority-assessment.md)
- 当前 Phase 3 的中央式 Frontier 检测基线

# C5d EdgeAggregator 实施计划

1. [x] 创建任务记录、设计、验证和检查清单。
2. [x] 实现 ROS-free `EdgeAggregator`、有界 contributor 状态、几何一致性、确定性合并和候选事务提交。
3. [x] 复用 C4 AggregateContract/Conversions，补充 edge aggregator ROS 薄节点、中央接收适配器和按来源 resync。
4. [x] 为 RuntimeAuthority 增加可选 N=5 配置，默认保持 C5c N=4 行为不变。
5. [x] 增加 N=5 launch/config/RViz、自动链路测试和聚合可视化诊断。
6. [x] Docker Engine 恢复后完成新代码构建、单元测试、launch test，并将原始日志写入 `validation/`；RViz 配置与 MarkerArray 链路完成静态核验。
7. [x] 运行 trellis-check；提交仍等待用户明确授权。

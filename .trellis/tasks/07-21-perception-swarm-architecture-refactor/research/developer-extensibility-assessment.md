# Research: 当前文档的开发者可扩展性评估

- Query: 现有架构草案是否足以让二次开发者在固定接口下替换各类算法？
- Scope: internal
- Date: 2026-07-21

## Findings

### 已覆盖部分

- 总览已要求开发者可选择二维、三维和多雷达算法，且替换扫描器、拓扑或任务分配算法时不重定义所有上下游接口：`docs/decisions/perception-and-swarm-architecture-refactor.md:16`、`docs/decisions/perception-and-swarm-architecture-refactor.md:30`。
- 总览区分逻辑接口与传输绑定，并要求内部算法、传输和部署变化不能隐式改变数据语义：`docs/decisions/perception-and-swarm-architecture-refactor.md:238`、`docs/decisions/perception-and-swarm-architecture-refactor.md:258`。
- 感知详解已经给出原生二维/三维视图、通用 ray 视图、多种同步策略和自定义融合算法边界：`docs/decisions/perception-observation-interface.md:202`、`docs/decisions/perception-observation-interface.md:220`、`docs/decisions/perception-observation-interface.md:232`。
- 感知详解已有开发者契约测试方向：`docs/decisions/perception-observation-interface.md:263`。

### 尚缺部分

- 没有跨八个功能模块的统一“哪些算法是正式扩展点”清单。
- 没有定义实现注册/选择级别：链接时、启动时配置或运行中热切换。
- 没有统一能力声明、错误/降级、取消、线程安全、资源上限和确定性契约。
- 没有要求 reference implementation、最小二开示例和可复用 conformance suite。
- 没有区分源码 API、C++ ABI 与跨进程 ROS 契约的稳定性承诺。

### 与仓库接口约定的关系

- 仓库要求只有存在第二实现、昂贵依赖 mock 或运行时切换时才建立接口：`AGENTS.md:52`。
- 不得为了测试给被测单元自身套接口，接口正当性来自第二实现：`AGENTS.md:62`、`AGENTS.md:64`。
- 同进程 C++ 替换使用 C++ 抽象接口；跨进程/跨语言使用 ROS 接口：`AGENTS.md:71`。

因此“所有算法可替换”应落地为主要算法族的稳定扩展端口，而不是所有类虚接口化。当前确有多实现需求的算法族包括观测融合、地图更新/聚合、拓扑路由、Frontier/Region、任务分配和本机探索/恢复。

## Recommendation

1. 在父设计中加入横向“开发者扩展模型”。
2. 评审稳定后单独创建 `docs/decisions/algorithm-extension-contract.md`，避免在每个模块重复规则。
3. `Accepted(D-000)`：首版主要算法在进程启动时按配置选择实现；不支持运行中有状态热切换。
4. 每个扩展端口同时交付 reference implementation、示例实现和 conformance suite。
5. 开发期承诺语义与源码级契约，不提前承诺长期 C++ 二进制 ABI。

## Caveats / Not Found

- 当前未选定 `pluginlib`、显式 registry 或静态 factory；应由第一个出现第二实现的子任务根据部署和 ABI 需求评审决定。

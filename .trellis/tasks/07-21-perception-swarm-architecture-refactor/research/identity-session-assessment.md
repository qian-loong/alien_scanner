# Research: 节点身份、source 与启动 session

- Query: 当前系统如何标识无人机、地图 source 和重启实例？
- Scope: internal
- Date: 2026-07-21

## Findings

- 多机 launch 按索引生成 `drone_i` namespace、`/drone_i/octomap` source topic 和 allocator 的 `drone_namespaces` 列表。
- `GlobalMapMergerNode` 以解析后的 source topic 字符串建立 source track；topic remap 同时改变寻址和逻辑身份。
- `ExplorationTask` 含 allocator epoch、revision 和 task ID，但不含目标 vehicle session；无人机节点重启没有统一 session identity。
- 新父设计已经要求 source session epoch、role epoch、route epoch、clock session 和 contributor manifest。如果没有共同的稳定 identity 根，这些 epoch 会分别绑定到 namespace、topic 或进程，无法可靠判断重启与重复 ID。
- 角色能力声明已经属于 C5 输入，但当前没有注册、接受、拒绝或 capability revision 契约。

## Implication

ROS namespace 应只负责寻址，不应充当稳定业务身份。父级需要固定 stable vehicle identity、启动 session、逻辑 source identity、能力登记和重复 ID 隔离的关系；具体 UUID 编码与配置文件格式可由 C5 子任务确定。

## Evidence

- `ws/src/swarm_controller/launch/multi_drone_exploration.launch.py:213`
- `ws/src/swarm_controller/launch/multi_drone_exploration.launch.py:244`
- `ws/src/swarm_controller/src/GlobalMapMergerNode.cpp:315`
- `ws/src/swarm_controller_interfaces/msg/ExplorationTask.msg:6`

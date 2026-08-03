# C2 当前建图基线调查

## 结论

C2 应新增独立的 `perception_local_map` ament 包，而不是继续扩展
`swarm_controller::OctoMapBuilder`。新包复用 C1 的 ROS-free observation、pose、
health 与 ray-evidence 契约；旧 builder 只保留为隔离的行为对照和迁移 oracle。

## 已有输入边界

- `perception_core` 已提供 ROS-free `LidarObservation`、`PoseEstimate`、
  `RayEvidenceCapability`、identity/timestamp 和 `MapperInputContract`：
  `ws/src/alien_perception/perception_core/include/perception_core/`。
- `perception_input_node` 发布 `perception/observations`、`perception/pose`、
  `perception/health`，见
  `ws/src/alien_perception/perception_input_node/src/PerceptionInputNode.cpp:109`。
- `LidarObservation` 保留 sensor/session/frame/stamp、2D 原生 ranges 或 3D XYZ，
  并逐批携带 ray-evidence。C1 明确 C2 不需要第二个 descriptor topic，见
  `.trellis/tasks/archive/2026-07/07-22-c1-perception-input/design.md:778`。
- `PointCloud2` 的当前公共 schema 只能证明 `HitOnly`；C2 不得从点序或 TF
  推断 free ray。完整约束见
  `.trellis/spec/backend/perception-ray-evidence-contract.md`。

## 旧地图路径

- 旧节点订阅 `sensor_msgs/PointCloud2` 的 `scan_returns`，要求私有
  `x/y/z/range/hit/intensity` 字段，并直接查询 TF：
  `ws/src/swarm_controller/src/OctoMapBuilderNode.cpp:52`。
- 旧算法公开 `octomap::OcTree`，没有 source/session/map epoch/revision/freshness
  契约：`ws/src/swarm_controller/include/swarm_controller/OctoMapBuilder.hpp:19`。
- 旧算法把所有合法 return 的 origin-to-endpoint 写为 free，并仅用 `hit` 决定
  endpoint occupied；它不知道 `HitOnly/HitRay/FullRay`：
  `ws/src/swarm_controller/src/OctoMapBuilder.cpp:31`。
- 旧节点由 wall timer 发布完整 OctoMap；map stamp 取最后 observation stamp，
  但没有 authoritative revision：
  `ws/src/swarm_controller/src/OctoMapBuilderNode.cpp:259`。
- 旧测试已锁定 no-return endpoint 保持 unknown、同批 occupied 胜过 free 等行为，
  可作为迁移 oracle：`ws/src/swarm_controller/test/TestOctoMapBuilder.cpp:9`。

## 必须补齐的能力

1. 后端无关 occupied/free/unknown 点查询和确定性 bounded-region 查询。
2. 单一 active mapper、map epoch、epoch 内单调 revision 和事件驱动 freshness。
3. `HitOnly/HitRay/FullRay` 的严格证据消费，不从 payload 类型提升权限。
4. 多 sensor 各自 origin、stamp 和 session 的独立顺序检查。
5. pose history 与 observation acquisition stamp 的确定性配对；pose reset 关闭
   旧链并清空后端。
6. alignment 引用与 map epoch 绑定，pose reset 立即撤销；本机建图不等待
   alignment。
7. 标准 OctoMap 仅作为默认后端的兼容/可视化输出；领域算法不得接触
   `octomap::OcTree`。

## 已知契约缺口与规划处理

- C1 ROS observation 不重复发布 descriptor/mounting geometry。C2 的 ROS 节点按
  observation `frame_id` 采样 `body <- sensor` TF 并转换为普通值；ROS-free
  `SensorExtrinsicRegistry` 负责校验、首次冻结、容差与漂移拒绝。外参不在 C2
  运行期热更新。
- `PoseEstimate.msg` 没有 child/body frame；C2 使用冻结的 `body_frame` 参数解释
  pose，并要求 pose reference `frame_id == source_local_map_frame`。
- `HealthState.msg` 没有 producer identity/session 或 contract identity，不能安全区分
  C1 重启前后的 Healthy，也不能证明 C1/C2 使用同一 frozen inventory/contract。C2
  需为该消息补充 source/session 与 canonical contract fingerprint；旧 session、空值
  或 fingerprint 不匹配只能导致拒绝，C1 aggregate health 只能降低而不能授权 C2
  地图状态。
- 整机 `vehicle_session_id` 尚无公共 C1 类型。C2 为 local map source 配置稳定
  `vehicle_id`，启动时生成 mapper session；sensor/pose producer session 继续使用
  C1 `SessionID`。C5 后续固定 fleet registration 编码时不得改变本机 epoch 语义。
- 标准 `octomap_msgs/Octomap` 无法携带 map epoch/revision/freshness/capability。
  C2 需要一个小型 `LocalMapState.msg` 发布元数据；地图内容仍使用标准 OctoMap，
  C3 再定义 LocalMapUpdate/keyframe/delta。

## 迁移约束

- C2 launch 不启动 FakeLidar、旧 OctoMapBuilder 或 `scan_returns` 链路。
- 新旧 mapper 使用不同 launch 和输出命名；测试不得让两者同时作为同一
  vehicle session 的 authoritative publisher。
- 现有 single/multi-drone exploration launch 仍显式依赖旧 builder、`octomap` 与
  `scan_returns`。C2 不顺手迁移这些入口；等 C3 map-update 和后续总集成具备后再
  做单一 selector 切换，避免局部替换造成新旧契约混用。
- 行为对照只比较固定查询点/区域的 occupied/free/unknown 结果，不比较
  OctoMap 二进制序列化字节。
- C1 隧道环切面 fixture 继续作为输入资产；C2 增加真实 occupancy 输出和 RViz
  display，不复制一套射线生成常量。

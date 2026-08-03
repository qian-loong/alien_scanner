# 修正感知测试雷达几何 - 实施计划

## 1. 实施顺序

### 阶段 0：受保护路径基线

- [x] 在任何实现修改前，记录 `ws/src/drone_scanner` 与 `ws/src/swarm_controller` 的 `git status --short`。
- [x] 记录两个目录的相对文件清单和逐文件 SHA-256，并保存到本任务 `research/` 下。
- [x] 明确基线已包含用户现有的 `ws/src/swarm_controller/CMakeLists.txt` 修改，结束时按内容哈希比较，不对它做回退。

验证门：基线文件可重复读取，且清楚区分既有工作区改动与本任务允许路径。

### 阶段 1：纯 C++ fixture 契约

- [x] 将 `FixtureSceneConfig` 的单环字段替换为方位采样数、径向距离和仰角数组。
- [x] 实现厂商无关的默认 16 线仰角 profile。
- [x] 按标准方位角/仰角公式重写 `cloud_points()`，保持固定输出顺序和 intensity 确定性。
- [x] 增加空数组、非有限值和越界仰角校验。
- [x] 重写 `TestFixtureScene.cpp`，逐项覆盖默认 16 线角表、默认点数、确定性、径向公式、intensity、XY 零仰角、正负仰角、方位覆盖和非法配置。

验证门：只构建并运行 `test_fixture_scene`，确认 ROS-free 几何契约通过。

### 阶段 2：薄 Publisher 与 ROS 接线

- [x] 将 Publisher 参数迁移为 `cloud_azimuth_sample_count`、`cloud_range_m`、`elevation_angles_rad`。
- [x] 删除 3D/mixed launch 与 mixed launch test 中的旧单环俯仰参数。
- [x] 将 mixed 集成测试配置为 3 通道 × 4 方位样本，并断言 observation 中 12 个 XYZ/intensity 的实际值和顺序。
- [x] 在 mixed 集成测试中断言 2D `angle_min/max/increment`、181 点和零角中间样本。
- [x] 保持 `PointCloud2` 字段、topic、frame 和 SensorDataQoS 不变。
- [x] 将 RViz display 名称从特殊单环语义调整为通用多线语义。

验证门：分别运行以下 public launch smoke，并确认节点无未知参数或启动异常：

```bash
timeout 12s ros2 launch perception_fixtures fixture_2d.launch.py
timeout 12s ros2 launch perception_fixtures fixture_3d.launch.py
timeout 12s ros2 launch perception_fixtures fixture_mixed.launch.py
```

三个命令预期由 `timeout` 以 124 结束；在超时前 fixture publisher 与 `perception_input_node` 必须保持运行且日志无 ERROR/FATAL。

### 阶段 3：文档与旧规划纠偏

- [x] 更新 `docs/perception-input-testing.md` 的 fixture 测试说明、默认 profile 与人工 TF 可视化边界。
- [x] 更新 `docs/perception-class-relations.md` 的配置字段和 3D 几何说明。
- [x] 定向修正 C1 任务文档中将通用 fixture 绑定旧 Phase 3 pitch 公式的条目；保留旧 FakeLidar 专属迁移验证边界。
- [x] 确认旧 Phase 2/3 用户文档和代码不因本任务发生变化。

验证门：`rg` 确认 `perception_fixtures` 不含 `ring_pitch_rad`，并人工核对新旧边界描述一致。

### 阶段 4：全量质量门

- [x] 在 ROS 2 Jazzy 容器中执行干净的相关包 Release 构建。
- [x] 运行 `perception_core`、`perception_interfaces`、`perception_adapters`、`perception_input_node`、`perception_fixtures` 全部测试。
- [x] 检查 `colcon test-result --all --verbose` 为零失败、零错误（58 tests）。
- [x] 将受保护目录当前状态、文件清单和 SHA-256 与阶段 0 基线逐项比较，结果完全一致。
- [x] 检查本任务 diff 只包含允许路径，并保留用户已有未提交改动。
- [x] 使用约定代码审核模型进行只读审核，修复高置信问题后复核。

### 阶段 5：统一 RViz 可视化

- [x] 新增 `fixture_visualization.launch.py`，组合水平 2D、倾斜 2D、标准多线 3D 三个独立 fixture publisher。
- [x] 在 `base_link` 下发布三个独立静态 TF；倾斜 2D 的 pitch 仅由 TF 表达并可通过 launch 参数调整。
- [x] 将唯一 RViz 配置改为 `base_link` Fixed Frame，提供两组 LaserScan、MultiLineCloud、TF 和参考网格 display。
- [x] 为运行期补充 `rviz2` 与 `tf2_ros` 依赖，并保持 CMake 现有 launch/config 目录安装规则不变。
- [x] 同步 PRD、技术设计和用户测试文档，不新增专门 mixed RViz 或 examples/demo 包。
- [x] 使用 `show_rviz:=false` 运行可视化 launch smoke，确认三个 topic 与三条 TF 接线正常。
- [x] 增加可视化 launch 回归测试，断言默认倾斜 TF 将传感器本地 +X 映射到 `base_link` 的正 Z 分量。
- [x] 运行相关构建、测试和静态检查，确认可视化增量没有破坏既有质量门或受保护目录。
- [x] 用户在 RViz 中人工目检三类显示及独立勾选行为。

## 2. 验证命令

容器内为每次验证选择从未使用过的 `<run-id>`，并先确认对应目录不存在；不得复用固定 `/tmp` 构建目录：

```bash
cd /workspaces/alien-scanner/ws

colcon --log-base /tmp/alien-fixture-geometry-<run-id>/log build \
  --build-base /tmp/alien-fixture-geometry-<run-id>/build \
  --install-base /tmp/alien-fixture-geometry-<run-id>/install \
  --symlink-install \
  --packages-up-to perception_fixtures \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source /tmp/alien-fixture-geometry-<run-id>/install/setup.bash

ROS_DOMAIN_ID=86 colcon --log-base /tmp/alien-fixture-geometry-<run-id>/test-log test \
  --build-base /tmp/alien-fixture-geometry-<run-id>/build \
  --install-base /tmp/alien-fixture-geometry-<run-id>/install \
  --packages-select perception_core perception_interfaces \
                    perception_adapters perception_input_node \
                    perception_fixtures \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base /tmp/alien-fixture-geometry-<run-id>/build \
  --all --verbose

timeout 12s ros2 launch perception_fixtures fixture_visualization.launch.py \
  show_rviz:=false
```

人工目检：

```bash
DISPLAY=host.docker.internal:0.0 \
  ros2 launch perception_fixtures fixture_visualization.launch.py
```

定向静态检查：

```bash
rg -n "ring_pitch_rad|PitchedCloud|cloud_point_count|ring_radius_m" \
  ws/src/alien_perception/perception_fixtures
rg -n "倾斜 profile|同一 pitch|ring_pitch_rad" \
  .trellis/tasks/07-22-c1-perception-input
```

## 3. 审核门

- 方案审核经用户明确授权改用 `gpt-5.6-sol`、reasoning effort `xhigh`，只读检查 PRD、设计、实施顺序和旧系统边界；原 `gpt-5.6-terra` 因多次服务超时及鉴权失败未产出审核结论。
- 实现审核沿用最近明确约定的 `gpt-5.6-sol`，重点检查坐标公式、ROS 参数类型、测试有效性和未授权范围扩张。
- 审核 finding 必须回到实际代码/需求验证，不能直接照单修改。

## 4. 回滚点

- 几何单测未通过：只回退阶段 1 的新配置与公式，不通过保留 `ring_pitch_rad` 规避失败。
- ROS 参数或 launch 失败：保持新 ROS-free 契约，单独修复 Publisher 参数转换和 launch 接线。
- 5760 点导致测试不可接受的资源回归：记录证据后调整测试输入 profile；默认 16 线产品决定不静默降级。
- 发现下游依赖旧 fixture pitch：停止扩展修改，明确迁移边界后回到规划，不修改旧 `FakeLidar`。

## 5. 完成条件

- [x] PRD AC1-AC10 全部满足。
- [x] 方案和代码审核无未解决的高置信问题。
- [x] 干净构建与全量相关测试通过。
- [x] 用户验收通过，并已明确授权准备提交；除非用户另行明确授权，不执行 `git push`。

## 6. 方案审核记录

### 首轮审核

- 模型：`gpt-5.6-sol`，reasoning effort `xhigh`。
- 结论：暂不通过。
- 已修订：
  - 增加 ROS 层 `LaserScan` 角度元数据与 PointCloud2 XYZ/intensity round-trip 断言；
  - 增加默认 16 线角表、径向公式和 intensity 的逐项 gtest；
  - 用实施前后文件状态、文件清单和 SHA-256 保护旧 `drone_scanner`/`swarm_controller`；
  - 改用唯一临时构建目录，补充三个 public launch smoke 命令并扩展残留语义搜索。
- 复核模型：`gpt-5.6-sol`，reasoning effort `xhigh`。
- 复核结论：无阻塞问题；首轮四项问题均已完整修复，未发现新引入的高置信阻塞问题。

### 实现审核

- 模型：`gpt-5.6-sol`，reasoning effort `xhigh`；复杂跨层审核采用 10 分钟超时。
- 首轮结论：实现代码无问题；发现 C1 `OPTIMIZATION-CHECKLIST.md` 仍可能要求通用 fixture 直接比较旧 FakeLidar 基线。
- 修复：明确通用 fixture 使用解析公式与 ROS round-trip 验证，旧几何等价性只属于 C2 旧路径专属 replay/adapter。
- 复核结论：原技术边界问题已解决；同步修复 checklist 与 `implement.md` 的完成状态后，无未解决的高置信问题。

### 可视化增量复核

- 模型：`gpt-5.6-sol`，reasoning effort `xhigh`，只读模式，10 分钟超时上限。
- 结论：无待修问题。
- 核对范围：Jazzy 静态 TF 参数与方向、topic/frame、RViz QoS 与布局、运行依赖、任务/用户文档和 code-spec 一致性。
- 当时剩余风险为用户在 RViz 中实际切换各 display 并目检倾斜方向与组合效果；该项随后完成并通过。

### 用户目检后的方向修正

- 用户目检确认倾斜 2D 应朝 `base_link` 的 +Z 方向抬起，而不是朝 -Z 方向下倾。
- 默认安装 pitch 从 `+30°` 调整为 `-30°`；`LaserScan` 消息仍保持传感器 frame 的 XY 平面几何。
- 同步更新用户测试文档、类关系说明、PRD、技术设计和 fixture 质量规范；不修改 3D 点云几何或旧 `FakeLidar`。
- 使用新安装树启动静态 TF 后，旋转矩阵第一列为 `(0.866, 0, 0.500)`，证明传感器本地 +X 在 `base_link` 中具有正 Z 分量。
- 增量审核发现初版回归测试没有锁定父 frame，也没有覆盖三个 publisher 节点及全部三条静态 TF；测试已补齐这些断言。
- 审核修复后重新运行定向用例和五包完整测试，结果仍为 `60 tests, 0 errors, 0 failures, 0 skipped`。
- `gpt-5.6-sol / xhigh` 只读复核确认两项 finding 均已闭环，无待修问题；剩余风险仅为方向反转后的人工 RViz 外观目检。

## 7. 实施结果

- 干净构建目录：`/tmp/alien-fixture-geometry-final-20260724-a`。
- 五个相关包 Release 构建成功；`58 tests, 0 errors, 0 failures, 0 skipped`。
- `fixture_2d`、`fixture_3d`、`fixture_mixed` 均通过 12 秒 smoke，按预期由 timeout 以 124 结束且无 ERROR/FATAL。
- 受保护目录保持 136 个文件，聚合 SHA-256 为 `167e8050d0d289d3b88b7d76115e958e2ed3690e1ab48b6e2cc46a2db016f239`，与实施前一致。
- 最终可视化增量构建目录：`/tmp/alien-fixture-visualization-20260725-c`；五包 Release 构建成功，`58 tests, 0 errors, 0 failures, 0 skipped`。
- 初始无界面 smoke 验证三个 fixture publisher、三条静态 TF、三个消息 topic 和正确 frame；用户目检后默认倾斜方向按新要求反转。
- 真实 RViz 启动后 Global Status 为 Ok、画布非空，五个 display 默认全部可见且可独立勾选；最终 DDS 订阅 QoS 为 Best Effort/Volatile。
- RViz 布局调整为 `1400x900`，Displays 帮助区隐藏，水平 2D、倾斜 2D、标准多线 3D 和 SensorTF 名称无需滚动即可看到。
- 受保护目录按基线口径仍为 136 个源码文件、零哈希差异；测试生成的 `__pycache__/*.pyc` 被 Git 忽略且不计入基线。
- 向 +Z 方向修正使用干净目录 `/tmp/alien-fixture-tilt-up-20260725-a`：五包 Release 构建成功，新增方向回归用例后为 `60 tests, 0 errors, 0 failures, 0 skipped`；无 GUI launch 启动三个 publisher 和三条静态 TF，日志无 ERROR/FATAL。
- 修正后的倾斜 TF 为 RPY `(0, -30°, 0)`、四元数约 `(0, -0.258819, 0, 0.965926)`；旋转矩阵将传感器本地 +X 映射为 `(0.866, 0, 0.500)`。
- 用户完成最终 RViz 人工检视，确认倾斜方向、三类显示和组合勾选行为通过验收。

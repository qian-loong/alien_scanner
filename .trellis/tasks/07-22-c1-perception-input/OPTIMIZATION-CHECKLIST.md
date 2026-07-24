# C1 优化检查清单

> 来源：Opus 4.8 评审（2026-07-22）
> 详细内容：见 `REVIEW-RESPONSE.md`

---

## 阶段 3：健康门控

- [x] **优化 1：健康状态立即检测 + 周期发布**
  - **建议**：在传感器回调中立即检测健康状态变化，1 Hz timer 只负责周期发布
  - **原因**：避免 1 秒延迟，提高健康状态变化的响应速度
  - **实施方式**：
    - 传感器回调中：检测 → 更新内部状态 → 立即发布（如果状态变化）
    - 1 Hz timer：定期重发当前状态（用于状态订阅者心跳）

- [x] **优化 2：健康状态抖动抑制**
  - **实现**：恢复路径使用连续 3 次一致评估；故障降级立即生效，避免继续产生伪 freshness
  - **原因**：传感器可能短暂掉线（网络抖动），频繁切换状态会导致下游组件频繁重置
  - **实施方式**：
    - 维护一个长度为 3 的状态历史队列
    - 只有当队列中所有 3 个状态都一致时，才切换到新状态
    - `Unavailable → Healthy` 必须先进入 `Degraded` 过渡态，再通过稳定窗口恢复

---

## 阶段 4：ROS 节点

- [x] **优化 4：Descriptor 冻结拒绝时的详细诊断**
  - **建议**：拒绝时提供详细诊断消息，包括拒绝原因、旧 descriptor、新 descriptor
  - **原因**：帮助开发者快速定位配置错误
  - **实施方式**：
    - 错误消息格式：`"Descriptor frozen: cannot register sensor 'lidar_front'. Reason: [新增传感器]. Current descriptors: [lidar_left, lidar_right]. Attempted descriptor: {sensor_id: lidar_front, type: 2D, ...}"`
    - 区分拒绝原因：新增传感器 vs 修改已有传感器 vs 重连不合法

- [x] **优化 5：不采纳 Humble 最低 patch 版本**
  - **原因**：本仓库明确使用 ROS 2 Jazzy 容器镜像，钉死 Humble 的 `rclcpp` 版本会与实际环境冲突
  - **替代措施**：通过 `alien-scanner-jazzy:latest` 开发镜像统一依赖环境

---

## 阶段 5：Fixture 与回归测试

- [x] **优化 3：详细回归测试标准**
  - **建议**：补充标准 2D / 多线 3D fixture 的测试脚本、输出比对、数值误差范围和判断标准
  - **原因**：确保回归测试可重复、客观，避免主观判断
  - **实施方式**：
    - `FixtureScene` gtest 逐点验证默认角表、点数、距离公式、方位/仰角方向、intensity 和确定性
    - mixed launch test 使用 3 通道 × 4 方位样本，按 `1e-5` 坐标容差验证 ROS round-trip 后的 XYZ、intensity 和顺序
    - 2D round-trip 验证 `[-pi/2,+pi/2]`、`pi/180`、181 点和中间零角样本
    - Phase 3 `FakeLidar` 基线不与通用 `FixtureScene` 直接比较；旧几何等价性由 C2 的旧路径专属 replay/adapter 验证

---

## 完成标准

所有 5 个优化项都勾选完成后，本检查清单任务完成。

**注意**：
- 这些优化项是**建议项**，不是**必须项**
- 如果在实施过程中发现某个优化不合适，可以记录原因后跳过
- 如果发现更好的实施方式，可以调整后勾选完成

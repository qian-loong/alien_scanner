# C1：感知输入与观测数据模型 - 技术设计

> 子任务：07-22-c1-perception-input
> 父任务：07-21-perception-swarm-architecture-refactor

---

## 1. 架构概览

### 1.1 模块分层

```
┌──────────────────────────────────────────────────────┐
│  ROS 节点层 (perception_input_node)                  │
│  - 订阅标准 ROS 消息                                  │
│  - 管理传感器 session 和 descriptor inventory        │
│  - 发布 ROS-free 观测数据                            │
└──────────────────────────────────────────────────────┘
                       ↓
┌──────────────────────────────────────────────────────┐
│  Adapter 层                                          │
│  - LaserScanAdapter                                  │
│  - PointCloud2Adapter                                │
│  - OdometryAdapter                                   │
└──────────────────────────────────────────────────────┘
                       ↓
┌──────────────────────────────────────────────────────┐
│  ROS-free 核心库 (perception_core)                   │
│  - LidarObservation                                  │
│  - PoseEstimate                                      │
│  - SensorDescriptor / SensorHealth                   │
│  - MapperInputContract / MapperHealthGate            │
└──────────────────────────────────────────────────────┘
                       ↓
┌──────────────────────────────────────────────────────┐
│  Mapper (C2 子任务实现)                              │
└──────────────────────────────────────────────────────┘
```

### 1.2 包结构

```
alien_perception/
├── perception_core/              # ROS-free 核心库
│   ├── include/perception_core/
│   │   ├── observation/
│   │   │   ├── lidar_observation.hpp
│   │   │   ├── pose_estimate.hpp
│   │   │   └── sensor_descriptor.hpp
│   │   ├── health/
│   │   │   ├── mapper_health_gate.hpp
│   │   │   ├── mapper_input_contract.hpp
│   │   │   └── health_state.hpp
│   │   └── types/
│   │       ├── identity.hpp         # SensorID, SessionID, SourceID
│   │       └── timestamp.hpp
│   └── src/
│       ├── observation/
│       └── health/
│
├── perception_adapters/          # ROS adapter 库
│   ├── include/perception_adapters/
│   │   ├── laser_scan_adapter.hpp
│   │   ├── point_cloud2_adapter.hpp
│   │   └── odometry_adapter.hpp
│   └── src/
│
├── perception_input_node/        # ROS 节点
│   ├── src/
│   │   ├── perception_input_node.cpp
│   │   └── sensor_session_manager.cpp
│   └── launch/
│       ├── perception_input.launch.py
│       ├── fixture_2d.launch.py
│       └── fixture_3d.launch.py
│
└── perception_fixtures/          # 测试 fixture
    ├── data/
    │   ├── scan_2d_corridor.bag
    │   ├── cloud_3d_room.bag
    │   └── mixed_2d3d.bag
    └── config/
        ├── sensor_descriptors_2d.yaml
        └── sensor_descriptors_3d.yaml
```

---

## 2. 核心类型设计

### 2.1 身份与 Session

```cpp
// types/identity.hpp
namespace perception {

// 稳定的传感器标识符（配置文件指定，不随重启改变）
struct SensorID {
    std::string value;  // 例如: "lidar_front", "lidar_rear"

    bool operator==(const SensorID&) const = default;
    auto operator<=>(const SensorID&) const = default;
};

// 每次启动生成的 session 标识符
struct SessionID {
    uint64_t boot_time_ns;  // 启动时间戳（纳秒）
    uint32_t random_suffix; // 随机后缀（避免时钟回退冲突）

    bool operator==(const SessionID&) const = default;
    auto operator<=>(const SessionID&) const = default;
};

// 位姿来源标识符
using SourceID = SensorID;

}  // namespace perception
```

### 2.2 传感器 Descriptor

```cpp
// observation/sensor_descriptor.hpp
namespace perception {

enum class SensorType {
    LIDAR_2D,
    LIDAR_3D
};

struct FieldOfView {
    double horizontal_min_rad;  // 水平视场角最小值
    double horizontal_max_rad;  // 水平视场角最大值
    double vertical_min_rad;    // 垂直视场角最小值（2D 时为 0）
    double vertical_max_rad;    // 垂直视场角最大值（2D 时为 0）
};

struct SensorDescriptor {
    SensorID sensor_id;
    SensorType type;
    std::string frame_id;              // TF 坐标系名称

    // 安装位姿（相对 base_link）
    Eigen::Vector3d mounting_position;
    Eigen::Quaterniond mounting_orientation;

    // 性能参数
    FieldOfView fov;
    double angular_resolution_rad;     // 角分辨率
    double range_min_m;                // 最小有效距离
    double range_max_m;                // 最大有效距离

    bool operator==(const SensorDescriptor&) const = default;
};

}  // namespace perception
```

### 2.3 观测数据

```cpp
// observation/lidar_observation.hpp
namespace perception {

// 2D 激光扫描
struct Scan2D {
    double angle_min_rad;
    double angle_max_rad;
    double angle_increment_rad;
    double range_min_m;
    double range_max_m;
    std::vector<float> ranges;         // 距离数组
    std::vector<float> intensities;    // 强度数组（可选）
};

// 3D 点云
struct Cloud3D {
    struct Point {
        float x, y, z;
        float intensity;  // 可选
    };
    std::vector<Point> points;
};

// 统一观测数据接口
struct LidarObservation {
    SensorID sensor_id;
    SessionID session_id;
    std::string frame_id;
    std::string clock_domain;          // 时钟域标识（如 "vehicle_steady_clock", "gps_time"）
    Timestamp origin_stamp;            // 观测产生时间

    // 实际数据（根据 descriptor.type 解释）
    std::variant<Scan2D, Cloud3D> data;

    // 获取点数
    size_t point_count() const;

    // 是否为 2D/3D
    bool is_2d() const { return std::holds_alternative<Scan2D>(data); }
    bool is_3d() const { return std::holds_alternative<Cloud3D>(data); }
};

}  // namespace perception
```

### 2.4 位姿估计

```cpp
// observation/pose_estimate.hpp
namespace perception {

struct PoseEstimate {
    SourceID source_id;
    SessionID session_id;
    std::string frame_id;              // 参考坐标系
    std::string clock_domain;          // 时钟域标识（如 "vehicle_steady_clock", "gps_time"）
    Timestamp stamp;

    // 位姿
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;

    // 质量指标
    std::optional<Eigen::Matrix6d> covariance;  // 6x6 协方差矩阵
    double quality;                    // 质量评分 [0, 1]

    // 时效性
    Duration freshness;                // 最后更新距现在的时间

    // Reset epoch（重定位/坐标系变化计数器）
    uint64_t reset_epoch;

    // 是否新鲜（freshness < threshold）
    bool is_fresh(Duration threshold) const {
        return freshness < threshold;
    }
};

}  // namespace perception
```

### 2.5 健康状态

```cpp
// health/health_state.hpp
namespace perception {

enum class HealthState {
    Healthy,      // 满足 minimum viable input
    Degraded,     // 满足 degraded 组合但不满足 minimum
    Unavailable   // 不满足任何组合
};

enum class SensorHealthStatus {
    Active,       // 正常接收数据
    Stale,        // 数据超时
    Lost          // 连接丢失
};

struct SensorHealth {
    SensorID sensor_id;
    SensorHealthStatus status;
    Timestamp last_update;
};

}  // namespace perception
```

### 2.6 Mapper 输入契约

```cpp
// health/mapper_input_contract.hpp
namespace perception {

// 传感器要求
struct SensorRequirement {
    SensorType type;
    size_t min_count;                  // 最少数量

    // 可选：特定传感器 ID
    std::optional<std::vector<SensorID>> specific_sensors;
};

// 降级配置
struct DegradedCombination {
    std::vector<SensorRequirement> requirements;
    std::string description;           // 降级描述（用于日志）
};

// Mapper 输入契约
struct MapperInputContract {
    // 最低要求
    std::vector<SensorRequirement> minimum_viable;

    // 降级配置（按优先级排序）
    std::vector<DegradedCombination> degraded_combinations;

    // 位姿要求
    bool requires_pose = true;
    Duration pose_freshness_threshold = Duration::from_seconds(1.0);
    std::optional<std::string> expected_pose_frame;
    double minimum_pose_quality = 0.0;

    // 恢复状态切换所需的连续健康样本数
    size_t recovery_stability_samples = 3;
};

}  // namespace perception
```

健康门控采用非对称切换策略：传感器或位姿失效导致的降级立即生效，恢复则需要连续 `recovery_stability_samples` 次评估一致。`Unavailable` 恢复到 `Healthy` 时必须先进入 `Degraded` 过渡态，避免单个正常样本产生伪 freshness。

### 2.7 健康门控

```cpp
// health/mapper_health_gate.hpp
namespace perception {

class MapperHealthGate {
public:
    explicit MapperHealthGate(MapperInputContract contract);

    // 评估当前健康状态
    HealthState evaluate(
        const std::vector<SensorDescriptor>& descriptors,
        const std::vector<SensorHealth>& sensor_health,
        const std::optional<PoseEstimate>& pose
    );

    // 获取当前有效的能力集
    struct CapabilitySet {
        bool has_2d_lidar;
        bool has_3d_lidar;
        bool has_pose;
        bool has_fresh_pose;
        bool has_expected_pose_frame;
        bool has_sufficient_pose_quality;
        bool has_usable_pose;
        size_t active_sensor_count;
    };
    CapabilitySet current_capability() const;

    // 获取降级原因（用于诊断）
    std::string degradation_reason() const;

private:
    MapperInputContract contract_;
    HealthState current_state_ = HealthState::Unavailable;
    std::string degradation_reason_;
    CapabilitySet current_capability_;
};

}  // namespace perception
```

---

## 3. ROS Adapter 设计

### 3.1 LaserScanAdapter

```cpp
// perception_adapters/laser_scan_adapter.hpp
namespace perception::adapters {

class LaserScanAdapter {
public:
    // 转换单个扫描
    LidarObservation convert(
        const sensor_msgs::msg::LaserScan& msg,
        const SensorDescriptor& descriptor,
        const SessionID& session_id
    ) const;

    // 验证消息与 descriptor 的兼容性
    struct ValidationResult {
        bool valid;
        std::string error_message;
    };
    ValidationResult validate(
        const sensor_msgs::msg::LaserScan& msg,
        const SensorDescriptor& descriptor
    ) const;
};

}  // namespace perception::adapters
```

### 3.2 PointCloud2Adapter

```cpp
// perception_adapters/point_cloud2_adapter.hpp
namespace perception::adapters {

class PointCloud2Adapter {
public:
    // 转换点云
    LidarObservation convert(
        const sensor_msgs::msg::PointCloud2& msg,
        const SensorDescriptor& descriptor,
        const SessionID& session_id
    ) const;

    // 验证点云格式
    ValidationResult validate(
        const sensor_msgs::msg::PointCloud2& msg,
        const SensorDescriptor& descriptor
    ) const;

private:
    // 从 PointCloud2 提取 XYZ 和强度
    Cloud3D extract_cloud(const sensor_msgs::msg::PointCloud2& msg) const;
};

}  // namespace perception::adapters
```

### 3.3 OdometryAdapter

```cpp
// perception_adapters/odometry_adapter.hpp
namespace perception::adapters {

class OdometryAdapter {
public:
    // 转换里程计
    PoseEstimate convert(
        const nav_msgs::msg::Odometry& msg,
        const SourceID& source_id,
        const SessionID& session_id,
        uint64_t reset_epoch
    ) const;

    // 从 TF 构建位姿估计
    PoseEstimate from_tf(
        const geometry_msgs::msg::TransformStamped& transform,
        const SourceID& source_id,
        const SessionID& session_id,
        uint64_t reset_epoch
    ) const;

private:
    // 计算质量评分（基于协方差）
    double compute_quality(
        const std::array<double, 36>& covariance
    ) const;
};

}  // namespace perception::adapters
```

---

## 4. ROS 节点设计

### 4.1 PerceptionInputNode

```cpp
class PerceptionInputNode : public rclcpp::Node {
public:
    PerceptionInputNode();

private:
    // 传感器 session 管理
    class SensorSessionManager {
    public:
        // 注册传感器（启动时）
        void register_sensor(const SensorDescriptor& descriptor);

        // 获取 session ID
        SessionID get_session_id(const SensorID& sensor_id) const;

        // 检查 descriptor 是否冻结
        bool is_frozen() const { return frozen_; }
        void freeze();

        // 验证传感器是否允许重连
        bool can_reconnect(const SensorDescriptor& descriptor) const;

    private:
        bool frozen_ = false;
        std::unordered_map<SensorID, SensorDescriptor> descriptors_;
        std::unordered_map<SensorID, SessionID> sessions_;
    };

    SensorSessionManager session_manager_;

    // ROS 订阅
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subs_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // ROS 发布
    rclcpp::Publisher<perception_interfaces::msg::LidarObservation>::SharedPtr obs_pub_;
    rclcpp::Publisher<perception_interfaces::msg::PoseEstimate>::SharedPtr pose_pub_;
    rclcpp::Publisher<perception_interfaces::msg::HealthState>::SharedPtr health_pub_;

    // Adapter
    LaserScanAdapter scan_adapter_;
    PointCloud2Adapter cloud_adapter_;
    OdometryAdapter odom_adapter_;

    // 健康门控
    MapperHealthGate health_gate_;

    // 回调
    void on_laser_scan(
        const sensor_msgs::msg::LaserScan::SharedPtr msg,
        const SensorID& sensor_id
    );

    void on_point_cloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg,
        const SensorID& sensor_id
    );

    void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 健康状态更新
    void update_health();
    rclcpp::TimerInterface::SharedPtr health_timer_;
};
```

---

## 5. 数据流

### 5.1 传感器数据流

```
LaserScan/PointCloud2 (ROS)
    ↓
PerceptionInputNode::on_laser_scan/on_point_cloud
    ↓
LaserScanAdapter::convert / PointCloud2Adapter::convert
    ↓
LidarObservation (ROS-free)
    ↓
发布到 perception_interfaces::msg::LidarObservation (ROS)
    ↓
Mapper (C2 子任务)
```

### 5.2 位姿数据流

```
Odometry (ROS)
    ↓
PerceptionInputNode::on_odometry
    ↓
OdometryAdapter::convert
    ↓
PoseEstimate (ROS-free)
    ↓
发布到 perception_interfaces::msg::PoseEstimate (ROS)
    ↓
Mapper (C2 子任务)
```

### 5.3 健康状态流

```
传感器健康 + 位姿健康
    ↓
MapperHealthGate::evaluate
    ↓
HealthState
    ↓
发布到 perception_interfaces::msg::HealthState (ROS)
    ↓
上游任务分配 (C6 子任务)
```

---

## 6. 关键设计决策

### D-C1-001：Descriptor 冻结时机
**决策**：在第一个传感器数据到达后冻结 descriptor inventory。

**理由**：
- 太早冻结（启动时）：可能阻止正常的传感器初始化
- 太晚冻结（从不）：无法保证一致性
- 首次数据到达：平衡点，此时传感器已就绪

**实现**：`SensorSessionManager::freeze()` 在第一个观测到达时调用。

---

### D-C1-002：Session ID 生成
**决策**：使用 `boot_time_ns + random_suffix`。

**理由**：
- `boot_time_ns`：提供时间顺序
- `random_suffix`：避免时钟回退时的 ID 冲突
- 不使用 UUID：太长，不便于日志追踪

**实现**：
```cpp
SessionID generate_session_id() {
    return SessionID{
        .boot_time_ns = std::chrono::steady_clock::now().time_since_epoch().count(),
        .random_suffix = std::random_device{}()
    };
}
```

---

### D-C1-003：健康状态更新频率
**决策**：1 Hz 定时器检查健康状态。

**理由**：
- 太快（> 10 Hz）：不必要的 CPU 开销
- 太慢（< 0.5 Hz）：健康变化响应慢
- 1 Hz：平衡响应速度和开销

**实现**：`rclcpp::create_timer(1s, update_health)`

---

### D-C1-004：Pose reset epoch 检测
**决策**：Pose reset epoch 检测包括 frame 变化、时间回退和位置跳变。

**理由**：
- Frame 变化：明确的坐标系重置
- 时间回退：重定位或重启
- 位置跳变：静默重定位（定位算法未改变 frame_id）

**实现**：
```cpp
// OdometryAdapter 内部维护上一个位姿
std::optional<PoseEstimate> last_pose_;
uint64_t current_reset_epoch_ = 0;

// 配置参数（可从配置文件读取）
double position_jump_threshold_m_ = 5.0;     // 位置跳变阈值（米）
double position_jump_time_window_s_ = 1.0;   // 时间窗口（秒）

PoseEstimate convert(...) {
    uint64_t reset_epoch = current_reset_epoch_;

    if (last_pose_) {
        bool frame_changed = (last_pose_->frame_id != msg.header.frame_id);
        bool time_went_backward = (msg.header.stamp < last_pose_->stamp);

        // 位置跳变检测
        double distance = (position - last_pose_->position).norm();
        double time_delta = (msg.header.stamp - last_pose_->stamp).seconds();
        bool position_jumped = (distance > position_jump_threshold_m_ &&
                               time_delta < position_jump_time_window_s_);

        if (frame_changed || time_went_backward || position_jumped) {
            ++current_reset_epoch_;
            reset_epoch = current_reset_epoch_;

            RCLCPP_WARN(logger_,
                "Pose reset detected: frame_changed=%d, time_backward=%d, "
                "position_jumped=%d (dist=%.2fm, dt=%.2fs)",
                frame_changed, time_went_backward, position_jumped,
                distance, time_delta);
        }
    }

    // ... 构建 PoseEstimate
    last_pose_ = result;
    return result;
}
```

**配置示例**：
```yaml
perception_input_node:
  pose_reset_detection:
    position_jump_threshold: 5.0      # 米，可根据应用场景调整
    position_jump_time_window: 1.0    # 秒
```

### D-C1-005：Python launch 编排
**决策**：C1 范围内的正式 launch 入口统一使用 `.launch.py`，由 `LaunchDescription` 和
`launch_ros.actions.Node` 编排节点；`launch` 与 `launch_ros` 作为运行时依赖声明。

**理由**：用户明确要求 launch 使用 Python，且本阶段的 ROS 接线测试本身已经使用 Python
`launch_testing`。当前入口仍是静态节点组合，不在 launch 中承载业务逻辑；后续只需保持参数和
topic 契约一致即可。

### D-C1-006：可配置的位姿输入与完整门控

**决策**：`PerceptionInputNode` 通过 `pose_input_type` 在 `odometry` 与 `tf`
之间选择单一位姿输入源。TF 模式订阅 `tf2_msgs/TFMessage`，按
`tf_child_frame` 选择 transform，再调用 `OdometryAdapter::from_tf()`。

健康门将位姿可用性定义为三个条件同时成立：

1. `freshness < pose_timeout_s`；
2. frame 与非空 `expected_pose_frame` 一致；
3. `quality >= minimum_pose_quality` 且为有限数值。

故障切换立即生效；恢复继续遵守 `recovery_stability_samples`。现有配置默认
`pose_input_type=odometry`、空 expected frame、最低 quality 为 0，因此兼容旧行为。

### D-C1-007：Producer 重启与 SessionID

**决策**：SessionID 属于 producer 进程生命周期。进程由 launch respawn 后必须
生成新 SessionID，后续观测不得复用旧值。C1 负责产生和传播新 SessionID；
旧 session 的拒绝策略由 C2 observation/map 消费者实现。

---

## 7. 错误处理

### 7.1 Descriptor 不匹配
**场景**：运行中收到与冻结 descriptor 不同的传感器数据。

**处理**：
1. 记录错误日志（包含新旧 descriptor 对比）
2. 拒绝该观测数据
3. 发布诊断消息（建议用户重启 vehicle session）

---

### 7.2 位姿超时
**场景**：长时间未收到位姿更新。

**处理**：
1. `PoseEstimate::freshness` 超过阈值
2. `MapperHealthGate` 标记为 Unavailable
3. 停止地图 revision（C2 子任务负责）

---

### 7.3 传感器掉线后重连
**场景**：传感器短暂断线后恢复。

**处理**：
1. 验证 descriptor 与冻结版本一致
2. 如果一致：允许重连，session ID 不变
3. 如果不一致：拒绝，要求新 vehicle session

---

## 8. 性能考虑

### 8.1 零拷贝优化
- 使用 ROS 2 shared memory transport（DDS-XTypes）
- Adapter 内部避免不必要的数据拷贝
- `Cloud3D::points` 使用 `std::vector` 预分配容量

### 8.2 内存占用
- 单帧 2D 扫描：~1 KB（360 点 × 4 字节）
- 单帧 3D 点云：~100 KB（10K 点 × 16 字节）
- 预算：< 1 MB 总缓冲（10 帧 3D 点云）

### 8.3 C1 收口性能基线（2026-07-24）

固定环境：ROS 2 Jazzy 容器，干净 Release colcon build/install。

- LaserScan Adapter：1000 次，平均 0.107 µs/帧；
- MapperHealthGate：5000 次，平均 0.056 µs/次；
- ROS publish-to-observation：120 个唯一帧、0 丢帧，平均 1.092 ms，
  P95 0.477 ms，最大 106.014 ms；最大值为首帧 DDS 发现离群值。

---

## 9. 测试策略

### 9.1 单元测试
- `LidarObservation` 构造和访问
- `SensorDescriptor` 相等性比较
- `MapperHealthGate` 状态转换
- Adapter 转换正确性

### 9.2 集成测试
- 多传感器并行输入
- 传感器掉线后重连
- Descriptor 冻结后拒绝新传感器
- 位姿 reset epoch 检测

### 9.3 回归测试
- 标准 2D XY / 多线 3D 仰角 fixture
- 旧 FakeLidar 与 Phase 3 baseline 的地图一致性（旧路径专属 replay，由 C2 子任务验证）

---

## 10. 与其他子任务的接口

### 10.1 C2 本机地图（下游）
**提供**：
- `LidarObservation`
- `PoseEstimate`
- `HealthState`

**约定**：
- C2 不直接订阅 ROS 传感器消息
- C2 只消费 ROS-free 观测数据

---

### 10.2 C3 地图更新（下游）
**提供**：
- Sensor ID/session 用于溯源
- Reset epoch 用于地图 epoch 管理

**期望使用方式**：
1. C3 应监听 `PoseEstimate.reset_epoch` 的变化
2. 当 `reset_epoch` 递增时，表示位姿源发生重置（重定位/坐标系变化/位置跳变）
3. C3 应通知 C2 开始新的 map epoch：
   - 冻结当前 map epoch（标记为 "closed"）
   - 创建新的 map epoch（递增 epoch number）
   - 将旧 map epoch 的数据标记为 "stale"（但保留用于诊断）
4. 旧 map epoch 的处理策略：
   - 短期：保留在内存中（用于回放和诊断）
   - 长期：归档到磁盘或丢弃（由后续存储管理决定）

**约定**：
- C3 不直接修改 C1 的 reset_epoch 值
- Reset epoch 的递增是单向的（只增不减）
- 即使位姿源恢复，reset_epoch 也不会回退到旧值

---

### 10.3 父任务架构决策
**遵守**：
- D-011：独立 interfaces 包
- D-015：Stable ID + session
- D-019：Descriptor 冻结
- D-021：Mapper 输入契约
- D-023：位姿外部化
- D-024：Pose reset 处理
- D-025：Vehicle-local 部署

---

## 11. 未来扩展点

### 11.1 多位姿源支持
当前：单一 Odometry 源
未来：支持多个位姿源（VIO + 轮速计融合）

### 11.2 在线传感器标定
当前：Descriptor 静态配置
未来：支持在线标定，更新 descriptor（触发新 vehicle session）

### 11.3 传感器故障诊断
当前：简单的超时检测
未来：详细的故障分类（硬件故障 vs 遮挡 vs 环境干扰）

# Research: 位姿估计与定位边界

- Query: 下一阶段重构是否包含定位/状态估计算法，还是只定义可替换的标准输入契约？
- Scope: internal
- Date: 2026-07-21

## Findings

- 当前位姿由 `FakeOdomNode` 根据确定性轨迹直接发布 `nav_msgs/msg/Odometry` 和 `odom -> base_link` TF；`map -> odom` 在现有仿真中通常是静态零变换。
- 当前 workspace 没有 localization、VIO、LIO、EKF/UKF 或 SLAM estimator 实现；Phase 2/3 的 OctoMap 是用 fake odom/TF 累积观测，不是同时估计位姿的完整 SLAM。
- 总路线图曾描述“单机 SLAM”，但当前已落地基线明确为三维扫描累积 + OctoMap；当前父设计只把 odom/TF 写成本机地图和执行输入，没有定义质量、协方差、重定位或位姿跳变语义。
- 地图 revision、sensor deskew、source-to-shared alignment、本机 known-free 和任务执行都依赖位姿输入；继续隐式信任 TF 会妨碍 Gazebo/真实 estimator 迁移。
- 将真实 VIO/LIO/SLAM 算法纳入本轮会额外引入 IMU/相机输入、初始化、漂移、回环、重定位和地图重投影，显著扩大 C1-C8 的范围。

## Implication

推荐本轮把定位视为外部可替换数据源：定义后端无关的 vehicle pose estimate 契约，并以标准 `nav_msgs/msg/Odometry` + TF 作为 ROS binding；契约保留 source/session、frame、stamp、pose/twist、covariance/quality、freshness 和 discontinuity/reset epoch。FakeOdom、未来 Gazebo odometry、真实 VIO/LIO/SLAM 通过 adapter 接入。首版不实现定位算法，但必须测试 stale、frame 错误、质量降级和 pose reset 对地图、alignment、任务和本机安全的门控。

## Evidence

- `ws/src/drone_scanner/src/FakeOdomNode.cpp:12`
- `ws/src/drone_scanner/src/FakeOdomNode.cpp:424`
- `docs/xenomorph-scanner-plan.md:45`
- `docs/xenomorph-scanner-plan.md:164`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:80`

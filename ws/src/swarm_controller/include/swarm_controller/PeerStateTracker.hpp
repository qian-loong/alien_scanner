#ifndef SWARM_CONTROLLER_PEERSTATETRACKER_HPP
#define SWARM_CONTROLLER_PEERSTATETRACKER_HPP

#include "swarm_controller/Point3f.hpp"

#include <cstddef>
#include <vector>

namespace SwarmController {

    /// PeerStateTracker 的时效与 Hold 语义配置。
    struct PeerStateConfig {
        /// 位置视为新鲜的最长间隔（秒）；超过则该 peer 的软惩罚与硬目标约束同时失效。
        double position_timeout_seconds {2.0};
        /// 目标视为新鲜的最长间隔（秒）；仅目标过期时只移除硬约束、保留位置软惩罚。
        /// 应 >= 执行端 motion.timeout（由节点跨参校验），避免误删仍在追的稀疏目标。
        double goal_timeout_seconds {25.0};
        /// 目标与位置 XY 距离 <= 此值视为 Hold，不计入 active goal（默认取 position_tolerance）。
        float hold_distance {0.20F};
    };

    /// snapshot() 输出：分散所需的新鲜 peer 位置/目标，以及 freshness 计数。
    struct PeerDispersionSnapshot {
        std::vector<Point3f> peer_positions;    ///< 位置新鲜的 peer（软惩罚）
        std::vector<Point3f> active_peer_goals; ///< 位置+目标新鲜且非 Hold（硬分离）
        std::size_t          configured_peers {0U};
        std::size_t          missing_positions {0U};
        std::size_t          fresh_positions {0U};
        std::size_t          stale_positions {0U};
        std::size_t          missing_goals {0U};
        std::size_t          fresh_goals {0U};
        std::size_t          active_goals {0U};
        std::size_t          stale_goals {0U};
    };

    /// 统一管理各 peer 生命周期的 ROS-free 具体类（单一实现，不引入接口）。
    /// 时钟统一使用调用方传入的 steady 接收时间（秒），不使用可能跳变的 ROS 消息时间。
    class PeerStateTracker
    {
    public:
        explicit PeerStateTracker(std::size_t peer_count, PeerStateConfig config = {});

        std::size_t size() const;

        /// index 越界或坐标/时间非有限则忽略，不改变既有状态。
        void updatePosition(
                std::size_t index, const Point3f & map_position, double recv_time_seconds);
        void updateGoal(std::size_t index, const Point3f & map_goal, double recv_time_seconds);

        /// 依据 now_seconds 计算时效并生成分散快照；now 非有限时抛异常。
        PeerDispersionSnapshot snapshot(double now_seconds) const;

    private:
        struct Peer {
            Point3f position {};
            double  position_time {0.0};
            bool    has_position {false};
            Point3f goal {};
            double  goal_time {0.0};
            bool    has_goal {false};
        };

        PeerStateConfig   config_;
        std::vector<Peer> peers_;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_PEERSTATETRACKER_HPP

#include "swarm_controller/PeerStateTracker.hpp"

#include <cmath>
#include <stdexcept>

namespace SwarmController {

    namespace {

        bool isFinite(const Point3f & point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

    }// namespace

    PeerStateTracker::PeerStateTracker(std::size_t peer_count, PeerStateConfig config)
        : config_(config)
        , peers_(peer_count)
    {
        const bool finite = std::isfinite(config_.position_timeout_seconds)
                            && std::isfinite(config_.goal_timeout_seconds)
                            && std::isfinite(config_.hold_distance);
        if(!finite || config_.position_timeout_seconds <= 0.0
           || config_.goal_timeout_seconds <= 0.0 || config_.hold_distance < 0.0F)
        {
            throw std::invalid_argument("invalid peer state tracker config");
        }
    }

    std::size_t PeerStateTracker::size() const
    {
        return peers_.size();
    }

    void PeerStateTracker::updatePosition(
            const std::size_t index, const Point3f & map_position, const double recv_time_seconds)
    {
        if(index >= peers_.size() || !isFinite(map_position)
           || !std::isfinite(recv_time_seconds) || recv_time_seconds < 0.0)
        {
            return;
        }
        if(peers_[index].has_position
           && recv_time_seconds < peers_[index].position_time)
        {
            return;
        }
        peers_[index].position      = map_position;
        peers_[index].position_time = recv_time_seconds;
        peers_[index].has_position  = true;
    }

    void PeerStateTracker::updateGoal(
            const std::size_t index, const Point3f & map_goal, const double recv_time_seconds)
    {
        if(index >= peers_.size() || !isFinite(map_goal) || !std::isfinite(recv_time_seconds)
           || recv_time_seconds < 0.0)
        {
            return;
        }
        if(peers_[index].has_goal && recv_time_seconds < peers_[index].goal_time) {
            return;
        }
        peers_[index].goal      = map_goal;
        peers_[index].goal_time = recv_time_seconds;
        peers_[index].has_goal  = true;
    }

    PeerDispersionSnapshot PeerStateTracker::snapshot(const double now_seconds) const
    {
        if(!std::isfinite(now_seconds) || now_seconds < 0.0) {
            throw std::invalid_argument("peer snapshot time must be finite and non-negative");
        }
        PeerDispersionSnapshot result;
        result.configured_peers = peers_.size();
        for(const Peer & peer : peers_) {
            if(!peer.has_position) {
                ++result.missing_positions;
            } else if(now_seconds - peer.position_time > config_.position_timeout_seconds) {
                ++result.stale_positions;
            } else {
                ++result.fresh_positions;
                result.peer_positions.push_back(peer.position);
            }

            if(!peer.has_goal) {
                ++result.missing_goals;
            } else if(now_seconds - peer.goal_time > config_.goal_timeout_seconds) {
                ++result.stale_goals;
            } else {
                ++result.fresh_goals;
            }

            const bool position_fresh = peer.has_position
                                        && now_seconds - peer.position_time
                                                   <= config_.position_timeout_seconds;
            const bool goal_fresh = peer.has_goal
                                    && now_seconds - peer.goal_time
                                               <= config_.goal_timeout_seconds;
            if(!position_fresh || !goal_fresh) {
                continue;
            }
            const float dx = peer.goal.x - peer.position.x;
            const float dy = peer.goal.y - peer.position.y;
            if(std::sqrt(dx * dx + dy * dy) <= config_.hold_distance) {
                continue;// Hold：目标新鲜但等同当前位姿，不作为 active goal
            }
            ++result.active_goals;
            result.active_peer_goals.push_back(peer.goal);
        }
        return result;
    }

}// namespace SwarmController

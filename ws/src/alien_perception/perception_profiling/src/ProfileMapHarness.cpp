#include "perception_profiling/ProfileMapHarness.hpp"

#include "perception_local_map/OctoMapBackend.hpp"

#include <stdexcept>
#include <utility>

namespace PerceptionProfiling {

    ProfileMapHarness::ProfileMapHarness(
            ProfileScenario scenario,
            Perception::SessionID mapper_session)
        : scenario_(std::move(scenario))
        , mapper_(std::make_unique<PerceptionLocalMap::LocalObservationMapper>(
                  scenario_.mapper_config(mapper_session),
                  [](const PerceptionLocalMap::MapGeometry & geometry) {
                      return std::make_unique<PerceptionLocalMap::OctoMapBackend>(geometry);
                  }))
    {
    }

    const ProfileScenario & ProfileMapHarness::scenario() const noexcept
    {
        return scenario_;
    }

    PerceptionLocalMap::LocalObservationMapper & ProfileMapHarness::mapper() noexcept
    {
        return *mapper_;
    }

    PerceptionLocalMap::InputResult ProfileMapHarness::submit(
            const ScenarioSample & sample)
    {
        const auto make_health = [this](std::int64_t receive_ns) {
            return PerceptionLocalMap::UpstreamHealth {
                    scenario_.config().producer_source_id,
                    scenario_.config().producer_session,
                    scenario_.contract_fingerprint(),
                    Perception::HealthState::Healthy,
                    receive_ns,
                    2'000'000'000LL};
        };

        const auto observation_receive_ns = sample.observation_pose.stamp.nanoseconds;
        const auto lead_receive_ns = sample.lead_pose.stamp.nanoseconds;
        if(!mapper_->submit_health(make_health(observation_receive_ns))) {
            throw std::runtime_error("profile health sample was rejected");
        }
        const auto observation_pose = mapper_->submit_pose(
                sample.observation_pose, observation_receive_ns + 1);
        if(observation_pose.status != PerceptionLocalMap::InputStatus::Accepted) {
            throw std::runtime_error(
                    "profile observation pose was rejected: "
                    + observation_pose.diagnostic);
        }
        if(!mapper_->submit_health(make_health(lead_receive_ns))) {
            throw std::runtime_error("profile lead health sample was rejected");
        }
        const auto lead_pose = mapper_->submit_pose(
                sample.lead_pose, lead_receive_ns + 1);
        if(lead_pose.status != PerceptionLocalMap::InputStatus::Accepted) {
            throw std::runtime_error(
                    "profile lead pose was rejected: " + lead_pose.diagnostic);
        }
        return mapper_->submit_observation(
                sample.observation, scenario_.extrinsic(), lead_receive_ns + 2);
    }

}// namespace PerceptionProfiling

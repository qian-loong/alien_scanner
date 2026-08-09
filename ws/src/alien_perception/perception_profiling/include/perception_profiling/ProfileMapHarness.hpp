#ifndef PERCEPTION_PROFILING_PROFILE_MAP_HARNESS_HPP
#define PERCEPTION_PROFILING_PROFILE_MAP_HARNESS_HPP

#include "perception_profiling/ProfileScenario.hpp"

#include <memory>

namespace PerceptionProfiling {

    class ProfileMapHarness
    {
    public:
        explicit ProfileMapHarness(
                ProfileScenario scenario,
                Perception::SessionID mapper_session = {20'000'000'000ULL, 0x0C2U});

        const ProfileScenario & scenario() const noexcept;
        PerceptionLocalMap::LocalObservationMapper & mapper() noexcept;
        PerceptionLocalMap::InputResult submit(const ScenarioSample & sample);

    private:
        ProfileScenario scenario_;
        std::unique_ptr<PerceptionLocalMap::LocalObservationMapper> mapper_;
    };

}// namespace PerceptionProfiling

#endif// PERCEPTION_PROFILING_PROFILE_MAP_HARNESS_HPP

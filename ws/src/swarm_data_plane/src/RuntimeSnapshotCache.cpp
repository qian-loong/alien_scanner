#include "swarm_data_plane/RuntimeSnapshotCache.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace SwarmDataPlane {

    namespace {

        bool valid_identity(
                const VehicleIdentity & identity,
                const RoleLimits & limits)
        {
            return identity.session.boot_time_ns != 0U
                   && static_cast<bool>(
                           PerceptionMapUpdate::CanonicalCodec::validate_string(
                                   identity.fleet_id, limits.max_identity_bytes,
                                   "runtime fleet identity", false))
                   && static_cast<bool>(
                           PerceptionMapUpdate::CanonicalCodec::validate_string(
                                   identity.vehicle_id, limits.max_identity_bytes,
                                   "runtime vehicle identity", false));
        }

        bool contains_assignment(
                const RoleSnapshot & snapshot,
                const VehicleIdentity & identity) noexcept
        {
            return std::any_of(
                    snapshot.assignments.begin(), snapshot.assignments.end(),
                    [&](const RoleAssignment & assignment) {
                        return assignment.identity == identity;
                    });
        }

        bool same_transition_contract(
                const RoleTransition & lhs,
                const RoleTransition & rhs) noexcept
        {
            return lhs.transition_id == rhs.transition_id
                   && lhs.base_role_epoch == rhs.base_role_epoch
                   && lhs.topology_epoch == rhs.topology_epoch
                   && lhs.target_assignments == rhs.target_assignments
                   && lhs.changed_members == rhs.changed_members;
        }

        bool acknowledgement_superset(
                const RoleTransition & candidate,
                const RoleTransition & current) noexcept
        {
            return std::includes(
                    candidate.acknowledgements.begin(),
                    candidate.acknowledgements.end(),
                    current.acknowledgements.begin(),
                    current.acknowledgements.end());
        }

        bool terminal(RoleTransitionState state) noexcept
        {
            return state == RoleTransitionState::Committed
                   || state == RoleTransitionState::RolledBack;
        }

    }// namespace

    RuntimeSnapshotCache::RuntimeSnapshotCache(
            VehicleIdentity identity,
            TopologyLimits topology_limits,
            RoleLimits role_limits)
            : identity_(std::move(identity))
            , topology_limits_(std::move(topology_limits))
            , role_limits_(std::move(role_limits))
    {
        if(!valid_identity(identity_, role_limits_)) {
            throw std::invalid_argument("runtime snapshot identity is invalid");
        }
    }

    RuntimeSnapshotResult RuntimeSnapshotCache::apply_topology(
            TopologySnapshot snapshot)
    {
        const auto valid = validate_topology_snapshot(snapshot, topology_limits_);
        if(!valid) {
            return {RuntimeSnapshotStatus::RejectedInvalid, false,
                    valid.diagnostic};
        }
        if(snapshot.fleet_id != identity_.fleet_id) {
            return {RuntimeSnapshotStatus::RejectedIdentity, false,
                    "topology fleet does not match the runtime identity"};
        }
        if(topology_.has_value()) {
            if(snapshot.topology_epoch < topology_->topology_epoch) {
                return {RuntimeSnapshotStatus::RejectedStale, false,
                        "topology epoch is stale"};
            }
            if(snapshot.topology_epoch == topology_->topology_epoch) {
                if(snapshot == *topology_) {
                    return {RuntimeSnapshotStatus::IgnoredDuplicate, false, {}};
                }
                return {RuntimeSnapshotStatus::RejectedConflict, false,
                        "topology epoch was reused for different content"};
            }
        }
        topology_ = std::move(snapshot);
        return {RuntimeSnapshotStatus::Applied, true, {}};
    }

    RuntimeSnapshotResult RuntimeSnapshotCache::apply_role(RoleSnapshot snapshot)
    {
        const auto valid = validate_role_snapshot(snapshot, role_limits_);
        if(!valid) {
            return {RuntimeSnapshotStatus::RejectedInvalid, false,
                    valid.diagnostic};
        }
        if(snapshot.fleet_id != identity_.fleet_id) {
            return {RuntimeSnapshotStatus::RejectedIdentity, false,
                    "role fleet does not match the runtime identity"};
        }
        if(topology_.has_value()
           && snapshot.topology_epoch < topology_->topology_epoch) {
            return {RuntimeSnapshotStatus::RejectedStale, false,
                    "role snapshot references a stale topology epoch"};
        }
        if(role_.has_value()) {
            if(snapshot.role_epoch < role_->role_epoch) {
                return {RuntimeSnapshotStatus::RejectedStale, false,
                        "role epoch is stale"};
            }
            if(snapshot.role_epoch == role_->role_epoch) {
                if(snapshot == *role_) {
                    return {RuntimeSnapshotStatus::IgnoredDuplicate, false, {}};
                }
                return {RuntimeSnapshotStatus::RejectedConflict, false,
                        "role epoch was reused for different content"};
            }
        }
        role_ = std::move(snapshot);
        return {RuntimeSnapshotStatus::Applied, true, {}};
    }

    RuntimeSnapshotResult RuntimeSnapshotCache::apply_evidence(
            CapabilityEvidence evidence)
    {
        const auto valid = validate_capability_evidence(evidence, role_limits_);
        if(!valid) {
            return {RuntimeSnapshotStatus::RejectedInvalid, false,
                    valid.diagnostic};
        }
        if(evidence.identity != identity_) {
            return {RuntimeSnapshotStatus::RejectedIdentity, false,
                    "capability evidence does not match the runtime identity"};
        }
        if(evidence_.has_value()) {
            if(evidence.evidence_revision < evidence_->evidence_revision) {
                return {RuntimeSnapshotStatus::RejectedStale, false,
                        "capability evidence revision is stale"};
            }
            if(evidence.evidence_revision == evidence_->evidence_revision) {
                if(evidence == *evidence_) {
                    return {RuntimeSnapshotStatus::IgnoredDuplicate, false, {}};
                }
                return {RuntimeSnapshotStatus::RejectedConflict, false,
                        "capability evidence revision was reused for different content"};
            }
        }
        evidence_ = std::move(evidence);
        return {RuntimeSnapshotStatus::Applied, true, {}};
    }

    RuntimeSnapshotResult RuntimeSnapshotCache::apply_transition(
            RoleTransition transition)
    {
        const auto valid = validate_role_transition(
                transition, identity_.fleet_id, role_limits_);
        if(!valid) {
            return {RuntimeSnapshotStatus::RejectedInvalid, false,
                    valid.diagnostic};
        }
        if(role_.has_value()
           && transition.base_role_epoch < role_->role_epoch
           && !terminal(transition.state)) {
            return {RuntimeSnapshotStatus::RejectedStale, false,
                    "active transition is based on a stale role epoch"};
        }
        if(transition_.has_value()) {
            if(transition.transition_id != transition_->transition_id) {
                if(!terminal(transition_->state)) {
                    return {RuntimeSnapshotStatus::RejectedConflict, false,
                            "another role transition is still active"};
                }
            } else {
                if(!same_transition_contract(transition, *transition_)) {
                    return {RuntimeSnapshotStatus::RejectedConflict, false,
                            "transition identity was reused for different content"};
                }
                if(static_cast<std::uint8_t>(transition.state)
                           < static_cast<std::uint8_t>(transition_->state)
                   || !acknowledgement_superset(transition, *transition_)) {
                    return {RuntimeSnapshotStatus::RejectedStale, false,
                            "transition state or acknowledgements regressed"};
                }
                if(transition == *transition_) {
                    return {RuntimeSnapshotStatus::IgnoredDuplicate, false, {}};
                }
            }
        }
        transition_ = std::move(transition);
        return {RuntimeSnapshotStatus::Applied, true, {}};
    }

    bool RuntimeSnapshotCache::aligned() const noexcept
    {
        return topology_.has_value() && role_.has_value() && evidence_.has_value()
               && role_->topology_epoch == topology_->topology_epoch
               && contains_assignment(*role_, identity_);
    }

    const VehicleIdentity & RuntimeSnapshotCache::identity() const noexcept
    {
        return identity_;
    }

    const TopologySnapshot * RuntimeSnapshotCache::topology() const noexcept
    {
        return topology_.has_value() ? &*topology_ : nullptr;
    }

    const RoleSnapshot * RuntimeSnapshotCache::role() const noexcept
    {
        return role_.has_value() ? &*role_ : nullptr;
    }

    const CapabilityEvidence * RuntimeSnapshotCache::evidence() const noexcept
    {
        return evidence_.has_value() ? &*evidence_ : nullptr;
    }

    const RoleTransition * RuntimeSnapshotCache::transition() const noexcept
    {
        return transition_.has_value() ? &*transition_ : nullptr;
    }

    WorkAdmissionResult RuntimeSnapshotCache::role_admission(
            std::optional<PrimaryRole> required_role,
            TransitionAdmissionMode transition_mode) const noexcept
    {
        if(!role_.has_value()) {
            return {WorkAdmissionStatus::NoAssignment};
        }
        return evaluate_role_work_admission(
                *role_, evidence(), transition(), identity_, required_role,
                transition_mode);
    }

    WorkAdmissionResult RuntimeSnapshotCache::service_admission(
            ServiceKind service,
            TransitionAdmissionMode transition_mode) const noexcept
    {
        if(!role_.has_value()) {
            return {WorkAdmissionStatus::NoAssignment};
        }
        return evaluate_service_work_admission(
                *role_, evidence(), transition(), identity_, service,
                transition_mode);
    }

}// namespace SwarmDataPlane

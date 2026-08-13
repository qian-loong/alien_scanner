#include "swarm_data_plane/TrustValidator.hpp"

#include <stdexcept>
#include <utility>

namespace SwarmDataPlane {

    TrustDecision PermissiveTrustValidator::validate(
            const RoutedMapUpdate &,
            const TrustEvidence &)
    {
        return {true, TrustRejectionReason::None, {}};
    }

    StaticTrustValidator::StaticTrustValidator(
            std::vector<TrustedProducer> trusted_producers)
    {
        for(auto & trusted : trusted_producers) {
            if(trusted.producer.producer_id.empty()
               || trusted.producer.session.boot_time_ns == 0U
               || trusted.authority_epoch == 0U) {
                throw std::invalid_argument("trusted producer configuration is invalid");
            }
            const auto inserted = producers_.emplace(
                    std::move(trusted.producer), State {trusted.authority_epoch, 0U});
            if(!inserted.second) {
                throw std::invalid_argument("trusted producer configuration is duplicated");
            }
        }
    }

    TrustDecision StaticTrustValidator::validate(
            const RoutedMapUpdate & message,
            const TrustEvidence & evidence)
    {
        if(evidence.credential_state == CredentialState::Expired) {
            return {false, TrustRejectionReason::ExpiredCredential,
                    "producer credential is expired"};
        }
        if(evidence.credential_state == CredentialState::Revoked) {
            return {false, TrustRejectionReason::RevokedCredential,
                    "producer credential is revoked"};
        }
        if(evidence.authenticated_producer != message.producer) {
            return {false, TrustRejectionReason::SpoofedProducer,
                    "authenticated producer differs from the claimed envelope producer"};
        }
        const auto found = producers_.find(message.producer);
        if(found == producers_.end()) {
            return {false, TrustRejectionReason::UnknownProducer,
                    "producer session is not in the static trust set"};
        }
        if(evidence.authority_epoch < found->second.authority_epoch) {
            return {false, TrustRejectionReason::OldAuthorityEpoch,
                    "trust evidence belongs to an old authority epoch"};
        }
        if(evidence.authority_epoch > found->second.authority_epoch) {
            return {false, TrustRejectionReason::UnknownAuthorityEpoch,
                    "trust evidence belongs to an unknown future authority epoch"};
        }
        if(message.sequence <= found->second.last_sequence) {
            return {false, TrustRejectionReason::Replay,
                    "producer sequence was already accepted"};
        }
        found->second.last_sequence = message.sequence;
        return {true, TrustRejectionReason::None, {}};
    }

}// namespace SwarmDataPlane

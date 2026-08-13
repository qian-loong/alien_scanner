#ifndef SWARM_DATA_PLANE_TRUST_VALIDATOR_HPP
#define SWARM_DATA_PLANE_TRUST_VALIDATOR_HPP

#include "swarm_data_plane/DataPlaneTypes.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace SwarmDataPlane {

    enum class CredentialState : std::uint8_t
    {
        Valid,
        Expired,
        Revoked
    };

    enum class TrustRejectionReason : std::uint8_t
    {
        None,
        SpoofedProducer,
        Replay,
        OldAuthorityEpoch,
        UnknownAuthorityEpoch,
        ExpiredCredential,
        RevokedCredential,
        UnknownProducer
    };

    struct TrustEvidence {
        ProducerIdentity authenticated_producer;
        CredentialState credential_state = CredentialState::Valid;
        std::uint64_t authority_epoch = 0U;
    };

    struct TrustDecision {
        bool accepted = false;
        TrustRejectionReason reason = TrustRejectionReason::UnknownProducer;
        std::string diagnostic;
    };

    class ITrustValidator
    {
    public:
        virtual ~ITrustValidator() = default;
        virtual TrustDecision validate(
                const RoutedMapUpdate & message,
                const TrustEvidence & evidence) = 0;
    };

    class PermissiveTrustValidator final : public ITrustValidator
    {
    public:
        TrustDecision validate(
                const RoutedMapUpdate & message,
                const TrustEvidence & evidence) override;
    };

    struct TrustedProducer {
        ProducerIdentity producer;
        std::uint64_t authority_epoch = 0U;
    };

    class StaticTrustValidator final : public ITrustValidator
    {
    public:
        explicit StaticTrustValidator(std::vector<TrustedProducer> trusted_producers);

        TrustDecision validate(
                const RoutedMapUpdate & message,
                const TrustEvidence & evidence) override;

    private:
        struct State {
            std::uint64_t authority_epoch = 0U;
            std::uint64_t last_sequence = 0U;
        };

        std::map<ProducerIdentity, State> producers_;
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_TRUST_VALIDATOR_HPP

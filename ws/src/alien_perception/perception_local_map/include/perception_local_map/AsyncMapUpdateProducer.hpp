#ifndef PERCEPTION_LOCAL_MAP_ASYNC_MAP_UPDATE_PRODUCER_HPP
#define PERCEPTION_LOCAL_MAP_ASYNC_MAP_UPDATE_PRODUCER_HPP

#include "perception_local_map/CanonicalSnapshotAdapter.hpp"
#include "perception_local_map/LocalObservationMapper.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace PerceptionLocalMap {

    struct AsyncMapUpdateDiagnostics {
        bool running = false;
        bool pending = false;
        bool in_flight = false;
        bool resync_pending = false;
        std::uint64_t pending_revision = 0U;
        std::uint64_t in_flight_revision = 0U;
        std::uint64_t published_revision = 0U;
        std::uint64_t enqueued_receipts = 0U;
        std::uint64_t coalesced_receipts = 0U;
        std::uint64_t superseded_receipts = 0U;
        std::uint64_t stale_enqueue_rejections = 0U;
        std::uint64_t acquire_failures = 0U;
        std::uint64_t materialize_failures = 0U;
        std::uint64_t resource_rejections = 0U;
        std::uint64_t publish_failures = 0U;
        std::uint64_t published_keyframes = 0U;
        std::uint64_t published_deltas = 0U;
        std::uint64_t revision_only_deltas = 0U;
        std::uint64_t no_new_revision = 0U;
        std::uint64_t last_snapshot_cells = 0U;
        std::uint64_t last_operation_count = 0U;
        std::uint64_t last_payload_bytes = 0U;
        std::int64_t last_acquire_duration_ns = 0;
        std::int64_t last_materialize_duration_ns = 0;
        std::int64_t last_traversal_duration_ns = 0;
        std::int64_t last_canonicalize_duration_ns = 0;
        std::int64_t last_content_hash_duration_ns = 0;
        std::int64_t last_prepare_duration_ns = 0;
        std::int64_t last_validation_duration_ns = 0;
        std::int64_t last_diff_duration_ns = 0;
        std::int64_t last_encode_duration_ns = 0;
        std::int64_t last_update_hash_duration_ns = 0;
        std::int64_t last_publish_duration_ns = 0;
        std::string last_diagnostic;
    };

    class AsyncMapUpdateProducer
    {
    public:
        using PublishCallback = std::function<bool(const PerceptionMapUpdate::MapUpdate &)>;

        AsyncMapUpdateProducer(
                LocalObservationMapper & mapper,
                PublishCallback publish_callback,
                PerceptionMapUpdate::MapUpdateLimits limits = {});
        ~AsyncMapUpdateProducer();

        AsyncMapUpdateProducer(const AsyncMapUpdateProducer &) = delete;
        AsyncMapUpdateProducer & operator=(const AsyncMapUpdateProducer &) = delete;

        bool enqueue(const CommitReceipt & receipt);
        bool request_keyframe(std::string correlation_id);
        AsyncMapUpdateDiagnostics diagnostics() const;
        void shutdown() noexcept;

    private:
        struct PendingReceipt {
            CommitReceipt receipt;
            std::uint64_t coalesced_receipt_count = 0U;
        };

        struct WorkItem {
            std::optional<PendingReceipt> pending;
            std::optional<std::string> correlation_id;
        };

        void worker_loop() noexcept;
        void process(WorkItem item) noexcept;
        void record_failure(std::string diagnostic, bool resource_limit = false);
        void release_failed_resync(
                const std::optional<std::string> & correlation_id) noexcept;

        LocalObservationMapper & mapper_;
        PublishCallback publish_callback_;
        PerceptionMapUpdate::MapUpdateLimits limits_;
        CanonicalSnapshotAdapter snapshot_adapter_;
        PerceptionMapUpdate::MapUpdateProducer producer_;

        mutable std::mutex mutex_;
        std::condition_variable wake_;
        std::optional<PendingReceipt> pending_;
        std::optional<CommitReceipt> in_flight_;
        std::optional<std::string> resync_correlation_id_;
        bool resync_work_scheduled_ = false;
        bool stopping_ = false;
        AsyncMapUpdateDiagnostics diagnostics_;
        std::thread worker_;
    };

}// namespace PerceptionLocalMap

#endif// PERCEPTION_LOCAL_MAP_ASYNC_MAP_UPDATE_PRODUCER_HPP

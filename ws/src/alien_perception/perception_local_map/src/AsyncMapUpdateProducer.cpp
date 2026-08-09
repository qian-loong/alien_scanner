#include "perception_local_map/AsyncMapUpdateProducer.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace PerceptionLocalMap {

    namespace {

        constexpr std::size_t kMaxDiagnosticBytes = 512U;

        bool same_chain(const CommitReceipt & left, const CommitReceipt & right) noexcept
        {
            return left.mapper_session == right.mapper_session
                   && left.map_epoch == right.map_epoch;
        }

        std::int64_t elapsed_ns(std::chrono::steady_clock::time_point start) noexcept
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now() - start)
                    .count();
        }

        std::string bounded_diagnostic(std::string value)
        {
            if(value.size() > kMaxDiagnosticBytes) {
                value.resize(kMaxDiagnosticBytes);
            }
            return value;
        }

    }// namespace

    AsyncMapUpdateProducer::AsyncMapUpdateProducer(
            LocalObservationMapper & mapper,
            PublishCallback publish_callback,
            PerceptionMapUpdate::MapUpdateLimits limits)
            : mapper_(mapper)
            , publish_callback_(std::move(publish_callback))
            , limits_(std::move(limits))
            , snapshot_adapter_(limits_)
            , producer_(limits_)
    {
        if(!publish_callback_) {
            throw std::invalid_argument("async map update producer requires a publish callback");
        }
        diagnostics_.running = true;
        worker_ = std::thread(&AsyncMapUpdateProducer::worker_loop, this);
    }

    AsyncMapUpdateProducer::~AsyncMapUpdateProducer()
    {
        shutdown();
    }

    bool AsyncMapUpdateProducer::enqueue(const CommitReceipt & receipt)
    {
        if(receipt.mapper_session.boot_time_ns == 0U || receipt.map_epoch == 0U
           || receipt.revision == 0U) {
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(stopping_) {
                return false;
            }
            if(pending_.has_value()) {
                if(same_chain(pending_->receipt, receipt)
                   && receipt.revision <= pending_->receipt.revision) {
                    ++diagnostics_.stale_enqueue_rejections;
                    return false;
                }
                const bool coalesced = same_chain(pending_->receipt, receipt);
                if(coalesced
                   && pending_->coalesced_receipt_count
                              == std::numeric_limits<std::uint64_t>::max()) {
                    ++diagnostics_.stale_enqueue_rejections;
                    return false;
                }
                const std::uint64_t next_coalesced = coalesced
                                                               ? pending_->coalesced_receipt_count
                                                                         + 1U
                                                               : 0U;
                pending_ = PendingReceipt {receipt, next_coalesced};
                ++diagnostics_.superseded_receipts;
                if(coalesced) {
                    ++diagnostics_.coalesced_receipts;
                }
            } else {
                if(in_flight_.has_value() && same_chain(*in_flight_, receipt)
                   && receipt.revision <= in_flight_->revision) {
                    ++diagnostics_.stale_enqueue_rejections;
                    return false;
                }
                pending_ = PendingReceipt {receipt, 0U};
            }
            ++diagnostics_.enqueued_receipts;
            diagnostics_.pending = true;
            diagnostics_.pending_revision = receipt.revision;
        }
        wake_.notify_one();
        return true;
    }

    bool AsyncMapUpdateProducer::request_keyframe(std::string correlation_id)
    {
        const auto validation = PerceptionMapUpdate::CanonicalCodec::validate_string(
                correlation_id,
                limits_.max_correlation_id_bytes,
                "correlation_id",
                false);
        if(!validation) {
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(stopping_) {
                return false;
            }
            if(resync_correlation_id_.has_value()
               && *resync_correlation_id_ != correlation_id) {
                return false;
            }
            resync_correlation_id_ = std::move(correlation_id);
            resync_work_scheduled_ = true;
            diagnostics_.resync_pending = true;
        }
        wake_.notify_one();
        return true;
    }

    AsyncMapUpdateDiagnostics AsyncMapUpdateProducer::diagnostics() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return diagnostics_;
    }

    void AsyncMapUpdateProducer::record_failure(
            std::string diagnostic,
            bool resource_limit)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(resource_limit) {
            ++diagnostics_.resource_rejections;
        }
        diagnostics_.last_diagnostic = bounded_diagnostic(std::move(diagnostic));
    }

    void AsyncMapUpdateProducer::release_failed_resync(
            const std::optional<std::string> & correlation_id) noexcept
    {
        if(!correlation_id.has_value()) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if(resync_correlation_id_ == correlation_id) {
            producer_.cancel_keyframe_request(*correlation_id);
            resync_correlation_id_.reset();
            diagnostics_.resync_pending = false;
        }
    }

    void AsyncMapUpdateProducer::worker_loop() noexcept
    {
        for(;;) {
            WorkItem item;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait(lock, [&]() {
                    return stopping_ || pending_.has_value() || resync_work_scheduled_;
                });
                if(stopping_) {
                    break;
                }
                if(pending_.has_value()) {
                    item.pending = std::move(pending_);
                    pending_.reset();
                    in_flight_ = item.pending->receipt;
                    diagnostics_.pending = false;
                    diagnostics_.pending_revision = 0U;
                    diagnostics_.in_flight = true;
                    diagnostics_.in_flight_revision = item.pending->receipt.revision;
                }
                if(resync_work_scheduled_
                   || (item.pending.has_value() && resync_correlation_id_.has_value())) {
                    item.correlation_id = resync_correlation_id_;
                    resync_work_scheduled_ = false;
                }
            }

            process(std::move(item));

            {
                std::lock_guard<std::mutex> lock(mutex_);
                in_flight_.reset();
                diagnostics_.in_flight = false;
                diagnostics_.in_flight_revision = 0U;
            }
        }
    }

    void AsyncMapUpdateProducer::process(WorkItem item) noexcept
    {
        try {
            const auto acquire_start = std::chrono::steady_clock::now();
            AcquireResult acquired;
            std::uint64_t coalesced_receipt_count = 0U;
            if(item.pending.has_value()) {
                coalesced_receipt_count = item.pending->coalesced_receipt_count;
                acquired = mapper_.acquire_read_transaction(item.pending->receipt);
                if(acquired.status == AcquireStatus::Superseded) {
                    bool newer_pending = false;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        ++diagnostics_.superseded_receipts;
                        if(pending_.has_value()) {
                            diagnostics_.last_acquire_duration_ns = elapsed_ns(acquire_start);
                            newer_pending = true;
                        }
                    }
                    if(newer_pending) {
                        release_failed_resync(item.correlation_id);
                        return;
                    }
                    acquired = mapper_.acquire_read_transaction();
                    if(coalesced_receipt_count != std::numeric_limits<std::uint64_t>::max()) {
                        ++coalesced_receipt_count;
                    }
                }
            } else {
                acquired = mapper_.acquire_read_transaction();
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                diagnostics_.last_acquire_duration_ns = elapsed_ns(acquire_start);
            }
            if(acquired.status != AcquireStatus::Ready || !acquired.transaction.has_value()) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    ++diagnostics_.acquire_failures;
                }
                release_failed_resync(item.correlation_id);
                record_failure("failed to acquire exact map read transaction");
                return;
            }

            const auto materialize_start = std::chrono::steady_clock::now();
            auto snapshot = snapshot_adapter_.materialize(*acquired.transaction);
            acquired.transaction->close();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                diagnostics_.last_materialize_duration_ns = elapsed_ns(materialize_start);
                diagnostics_.last_traversal_duration_ns =
                        snapshot.timing.traversal_duration_ns;
                diagnostics_.last_canonicalize_duration_ns =
                        snapshot.timing.canonicalize_duration_ns;
                diagnostics_.last_content_hash_duration_ns =
                        snapshot.timing.content_hash_duration_ns;
            }
            if(snapshot.status != CanonicalSnapshotStatus::Ready
               || !snapshot.snapshot.has_value()) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    ++diagnostics_.materialize_failures;
                }
                release_failed_resync(item.correlation_id);
                record_failure(
                        snapshot.diagnostic,
                        snapshot.status == CanonicalSnapshotStatus::ResourceLimit);
                return;
            }

            if(item.correlation_id.has_value()
               && !producer_.request_keyframe(*item.correlation_id)) {
                release_failed_resync(item.correlation_id);
                record_failure("map update producer rejected resync correlation");
                return;
            }
            auto target_snapshot =
                    std::make_shared<const PerceptionMapUpdate::CanonicalSnapshot>(
                            std::move(*snapshot.snapshot));
            const auto prepare_start = std::chrono::steady_clock::now();
            auto prepared = producer_.prepare(
                    target_snapshot, coalesced_receipt_count);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                diagnostics_.last_prepare_duration_ns = elapsed_ns(prepare_start);
                diagnostics_.last_validation_duration_ns =
                        prepared.timing.validation_duration_ns;
                diagnostics_.last_diff_duration_ns = prepared.timing.diff_duration_ns;
                diagnostics_.last_encode_duration_ns = prepared.timing.encode_duration_ns;
                diagnostics_.last_update_hash_duration_ns =
                        prepared.timing.update_hash_duration_ns;
                diagnostics_.last_snapshot_cells = target_snapshot->cells.size();
            }
            if(prepared.status == PerceptionMapUpdate::ProduceStatus::NoNewRevision) {
                std::lock_guard<std::mutex> lock(mutex_);
                ++diagnostics_.no_new_revision;
                diagnostics_.last_diagnostic.clear();
                return;
            }
            if(!prepared.update.has_value()) {
                release_failed_resync(item.correlation_id);
                record_failure(
                        prepared.diagnostic,
                        prepared.status
                                == PerceptionMapUpdate::ProduceStatus::RejectedResourceLimit);
                return;
            }

            const auto publish_start = std::chrono::steady_clock::now();
            bool published = false;
            try {
                published = publish_callback_(*prepared.update);
            }
            catch(const std::exception & error) {
                record_failure(error.what());
            }
            catch(...) {
                record_failure("unknown map update publish failure");
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                diagnostics_.last_publish_duration_ns = elapsed_ns(publish_start);
            }
            if(!published || !producer_.commit_published(prepared)) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    ++diagnostics_.publish_failures;
                }
                release_failed_resync(item.correlation_id);
                if(!published) {
                    record_failure("map update publisher rejected update");
                } else {
                    record_failure("map update producer rejected published commit");
                }
                return;
            }

            const auto & update = *prepared.update;
            std::lock_guard<std::mutex> lock(mutex_);
            diagnostics_.published_revision = update.new_revision;
            diagnostics_.last_operation_count = update.operation_count;
            diagnostics_.last_payload_bytes = update.payload.size();
            diagnostics_.last_diagnostic.clear();
            if(update.kind == PerceptionMapUpdate::UpdateKind::Keyframe) {
                ++diagnostics_.published_keyframes;
            } else if(update.kind == PerceptionMapUpdate::UpdateKind::Delta) {
                ++diagnostics_.published_deltas;
                if(update.operation_count == 0U) {
                    ++diagnostics_.revision_only_deltas;
                }
            }
            if(resync_correlation_id_.has_value()
               && update.correlation_id == *resync_correlation_id_) {
                resync_correlation_id_.reset();
                diagnostics_.resync_pending = false;
            }
        }
        catch(const std::exception & error) {
            release_failed_resync(item.correlation_id);
            record_failure(error.what());
        }
        catch(...) {
            release_failed_resync(item.correlation_id);
            record_failure("unknown async map update worker failure");
        }
    }

    void AsyncMapUpdateProducer::shutdown() noexcept
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(stopping_) {
                return;
            }
            stopping_ = true;
            pending_.reset();
            resync_work_scheduled_ = false;
            diagnostics_.pending = false;
            diagnostics_.pending_revision = 0U;
            diagnostics_.resync_pending = false;
        }
        wake_.notify_one();
        if(worker_.joinable()) {
            worker_.join();
        }
        std::lock_guard<std::mutex> lock(mutex_);
        diagnostics_.running = false;
        diagnostics_.in_flight = false;
        diagnostics_.in_flight_revision = 0U;
    }

}// namespace PerceptionLocalMap

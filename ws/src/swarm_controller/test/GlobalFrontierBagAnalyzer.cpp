#include "GlobalFrontierBagAnalyzer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <system_error>
#include <utility>
#include <vector>

#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace SwarmController::Test {

    namespace {

        constexpr const char * OCTOMAP_TYPE = "octomap_msgs/msg/Octomap";
        constexpr std::array<std::pair<std::int64_t, std::int64_t>, 8U>
                COMPONENT_NEIGHBORS {
                        std::pair<std::int64_t, std::int64_t> {1, 0},
                        {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1},
                        {1, -1},
                };

        std::string statusName(const FrontierDetectionStatus status)
        {
            switch(status) {
                case FrontierDetectionStatus::Accepted:
                    return "Accepted";
                case FrontierDetectionStatus::Empty:
                    return "Empty";
                case FrontierDetectionStatus::Invalid:
                    return "Invalid";
                case FrontierDetectionStatus::ResourceLimit:
                    return "ResourceLimit";
            }
            return "Invalid";
        }

        std::string componentRejectionName(
                const FrontierComponentRejection rejection)
        {
            switch(rejection) {
                case FrontierComponentRejection::None:
                    return "None";
                case FrontierComponentRejection::Columns:
                    return "Columns";
                case FrontierComponentRejection::Area:
                    return "Area";
                case FrontierComponentRejection::Span:
                    return "Span";
                case FrontierComponentRejection::Direction:
                    return "Direction";
            }
            return "Unknown";
        }

        std::string csvField(const std::string & value)
        {
            if(value.find_first_of(",\"\r\n") == std::string::npos) {
                return value;
            }
            std::string escaped;
            escaped.reserve(value.size() + 2U);
            escaped.push_back('"');
            for(const char character : value) {
                if(character == '"') {
                    escaped.push_back('"');
                }
                escaped.push_back(character);
            }
            escaped.push_back('"');
            return escaped;
        }

        std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
        {
            return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL
                   + static_cast<std::int64_t>(stamp.nanosec);
        }

        void writeStatisticsHeader(std::ostream & output)
        {
            output << "frame_index,bag_timestamp_ns,map_stamp_ns,status,complete,"
                      "scanned_free_voxels,sampled_free_columns,"
                      "unknown_neighbor_candidate_columns,vertical_passed_columns,"
                      "vertical_rejected_columns,support_passed_columns,"
                      "support_rejected_unknown,support_rejected_occupied,"
                      "support_rejected_out_of_bounds,support_samples_attempted,"
                      "support_failure_position_unavailable";
            for(std::size_t index = 0U;
                index < FrontierDetectionDiagnostics::SUPPORT_DEPTH_BUCKETS;
                ++index)
            {
                output << ",support_failure_depth_octile_" << index;
            }
            output << ",components_built";
            for(std::size_t index = 0U;
                index < FrontierDetectionDiagnostics::COMPONENT_SIZE_BUCKETS;
                ++index)
            {
                output << ",component_size_bucket_" << index;
            }
            output << ",component_primary_rejected_columns,"
                      "component_primary_rejected_area,"
                      "component_primary_rejected_span,"
                      "component_primary_rejected_direction,components_accepted,"
                      "detected_regions,reason\n";
        }

        template<std::size_t Size>
        void writeArray(std::ostream & output, const std::array<std::uint64_t, Size> & values)
        {
            for(const auto value : values) {
                output << ',' << value;
            }
        }

        void writeStatisticsRow(
                std::ostream & output, const std::size_t frame_index,
                const std::int64_t bag_timestamp,
                const octomap_msgs::msg::Octomap & message,
                const FrontierDetectionResult & detection)
        {
            const auto & diagnostics = detection.diagnostics;
            output << frame_index << ',' << bag_timestamp << ','
                   << stampNanoseconds(message.header.stamp) << ','
                   << statusName(detection.status) << ','
                   << (diagnostics.complete ? 1 : 0) << ','
                   << diagnostics.scanned_free_voxels << ','
                   << diagnostics.sampled_free_columns << ','
                   << diagnostics.unknown_neighbor_candidate_columns << ','
                   << diagnostics.vertical_passed_columns << ','
                   << diagnostics.vertical_rejected_columns << ','
                   << diagnostics.support_passed_columns << ','
                   << diagnostics.support_rejected_unknown << ','
                   << diagnostics.support_rejected_occupied << ','
                   << diagnostics.support_rejected_out_of_bounds << ','
                   << diagnostics.support_samples_attempted << ','
                   << diagnostics.support_failure_position_unavailable;
            writeArray(output, diagnostics.support_failure_depth_octiles);
            output << ',' << diagnostics.components_built;
            writeArray(output, diagnostics.component_size_buckets);
            output << ',' << diagnostics.component_primary_rejected_columns
                   << ',' << diagnostics.component_primary_rejected_area
                   << ',' << diagnostics.component_primary_rejected_span
                   << ',' << diagnostics.component_primary_rejected_direction
                   << ',' << diagnostics.components_accepted
                   << ',' << detection.regions.size()
                   << ',' << csvField(detection.reason) << '\n';
        }

        void writeTimingHeader(std::ostream & output)
        {
            output << "frame_index,bag_timestamp_ns,map_stamp_ns,leaf_scan_seconds,"
                      "vertical_seconds,support_seconds,component_seconds,total_seconds\n";
        }

        void writeTimingRow(
                std::ostream & output, const std::size_t frame_index,
                const std::int64_t bag_timestamp,
                const octomap_msgs::msg::Octomap & message,
                const FrontierDetectionTimings & timings)
        {
            output << frame_index << ',' << bag_timestamp << ','
                   << stampNanoseconds(message.header.stamp) << ','
                   << timings.leaf_scan_seconds << ',' << timings.vertical_seconds << ','
                   << timings.support_seconds << ',' << timings.component_seconds << ','
                   << timings.total_seconds << '\n';
        }

        void writeComponentHeader(std::ostream & output)
        {
            output << "frame_index,bag_timestamp_ns,map_stamp_ns,component_index,"
                      "stable_key_x,stable_key_y,exact_columns,area,horizontal_span,"
                      "representative_x,representative_y,representative_z,"
                      "unknown_direction_x,unknown_direction_y,unknown_direction_z,"
                      "information_gain,direction_consistency,direction_votes_pos_x,"
                      "direction_votes_pos_y,direction_votes_neg_x,"
                      "direction_votes_neg_y,xy_min_x,xy_min_y,xy_max_x,xy_max_y,"
                      "rejection,columns_complete,edges_complete\n";
        }

        void writeComponentRow(
                std::ostream & output, const std::size_t frame_index,
                const std::int64_t bag_timestamp,
                const octomap_msgs::msg::Octomap & message,
                const FrontierComponentTrace & component)
        {
            output << frame_index << ',' << bag_timestamp << ','
                   << stampNanoseconds(message.header.stamp) << ','
                   << component.component_index << ',' << component.stable_key.x << ','
                   << component.stable_key.y << ',' << component.exact_column_count << ','
                   << component.area << ',' << component.horizontal_span << ','
                   << component.representative.x << ',' << component.representative.y << ','
                   << component.representative.z << ','
                   << component.unknown_direction.x << ','
                   << component.unknown_direction.y << ','
                   << component.unknown_direction.z << ','
                   << component.information_gain << ','
                   << component.direction_consistency;
            writeArray(output, component.direction_votes);
            output << ',' << component.xy_minimum.x << ',' << component.xy_minimum.y
                   << ',' << component.xy_maximum.x << ',' << component.xy_maximum.y
                   << ',' << componentRejectionName(component.rejection)
                   << ',' << (component.columns_complete ? 1 : 0)
                   << ',' << (component.edges_complete ? 1 : 0) << '\n';
        }

        void writeMembershipHeader(std::ostream & output)
        {
            output << "frame_index,component_index,stable_key_x,stable_key_y,"
                      "column_x,column_y\n";
        }

        void writeMembershipRows(
                std::ostream & output, const std::size_t frame_index,
                const FrontierComponentTrace & component)
        {
            for(const FrontierColumnKey & column : component.columns) {
                output << frame_index << ',' << component.component_index << ','
                       << component.stable_key.x << ',' << component.stable_key.y << ','
                       << column.x << ',' << column.y << '\n';
            }
        }

        std::optional<std::string> componentTraceError(
                const FrontierDetectionResult & detection,
                const FrontierDetectionTrace & trace)
        {
            if(!detection.diagnostics.complete) {
                return "detector diagnostics are incomplete";
            }
            if(trace.truncated) {
                return "component trace is truncated";
            }
            if(trace.components.size() != detection.diagnostics.components_built) {
                return "component row count does not match diagnostics";
            }

            std::uint64_t exact_columns = 0U;
            std::array<std::uint64_t, 5U> rejection_counts {};
            for(std::size_t index = 0U; index < trace.components.size(); ++index) {
                const FrontierComponentTrace & component = trace.components[index];
                if(component.component_index != index) {
                    return "component indices are not contiguous";
                }
                if(component.exact_column_count == 0U) {
                    return "component has zero exact columns";
                }
                if(!component.columns_complete || !component.edges_complete) {
                    return "component membership or edges are incomplete";
                }
                if(component.columns.size() != component.exact_column_count) {
                    return "component membership count is incomplete";
                }
                if(!std::is_sorted(component.columns.begin(), component.columns.end())
                   || std::adjacent_find(
                              component.columns.begin(), component.columns.end())
                              != component.columns.end())
                {
                    return "component membership is not sorted and unique";
                }
                if(!(component.stable_key == component.columns.front())) {
                    return "component stable key does not match first column";
                }
                const std::set<FrontierColumnKey> membership(
                        component.columns.begin(), component.columns.end());
                using EdgeKey =
                        std::pair<FrontierColumnKey, FrontierColumnKey>;
                std::set<EdgeKey> expected_edges;
                for(const FrontierColumnKey & column : component.columns) {
                    for(const auto & [dx, dy] : COMPONENT_NEIGHBORS) {
                        const FrontierColumnKey neighbor {
                                column.x + dx, column.y + dy};
                        if(column < neighbor
                           && membership.find(neighbor) != membership.end())
                        {
                            expected_edges.emplace(column, neighbor);
                        }
                    }
                }
                std::set<EdgeKey> actual_edges;
                for(const FrontierComponentEdge & edge : component.edges) {
                    if(!(edge.first < edge.second)
                       || membership.find(edge.first) == membership.end()
                       || membership.find(edge.second) == membership.end())
                    {
                        return "component edge has invalid or unordered endpoint";
                    }
                    if(!actual_edges.emplace(edge.first, edge.second).second) {
                        return "component edge is duplicated";
                    }
                }
                if(actual_edges != expected_edges) {
                    return "component edges do not match membership adjacency";
                }
                const float span = std::hypot(
                        component.xy_maximum.x - component.xy_minimum.x,
                        component.xy_maximum.y - component.xy_minimum.y);
                if(std::fabs(span - component.horizontal_span) > 1.0e-4F) {
                    return "component XY bounds do not reproduce horizontal span";
                }
                const std::uint64_t vote_total = std::accumulate(
                        component.direction_votes.begin(),
                        component.direction_votes.end(), std::uint64_t {0U});
                if(std::fabs(
                           static_cast<double>(vote_total)
                           - static_cast<double>(component.information_gain))
                   > 0.5)
                {
                    return "component votes do not reproduce information gain";
                }
                exact_columns += component.exact_column_count;
                switch(component.rejection) {
                    case FrontierComponentRejection::Columns:
                        ++rejection_counts[0U];
                        break;
                    case FrontierComponentRejection::Area:
                        ++rejection_counts[1U];
                        break;
                    case FrontierComponentRejection::Span:
                        ++rejection_counts[2U];
                        break;
                    case FrontierComponentRejection::Direction:
                        ++rejection_counts[3U];
                        break;
                    case FrontierComponentRejection::None:
                        ++rejection_counts[4U];
                        break;
                }
            }
            const auto & diagnostics = detection.diagnostics;
            if(exact_columns != diagnostics.support_passed_columns) {
                return "component column mass does not match supported columns";
            }
            if(rejection_counts[0U] != diagnostics.component_primary_rejected_columns
               || rejection_counts[1U]
                          != diagnostics.component_primary_rejected_area
               || rejection_counts[2U]
                          != diagnostics.component_primary_rejected_span
               || rejection_counts[3U]
                          != diagnostics.component_primary_rejected_direction
               || rejection_counts[4U] != diagnostics.components_accepted
               || rejection_counts[4U] != detection.regions.size())
            {
                return "component rejection counts do not match diagnostics";
            }
            return std::nullopt;
        }

        struct OutputCommitEntry {
            std::filesystem::path temporary;
            std::filesystem::path destination;
            std::filesystem::path backup;
            bool had_destination {};
            bool backed_up {};
            bool published {};
        };

        bool normalizeOutputPath(
                const std::filesystem::path & input,
                std::filesystem::path & output, std::string & reason)
        {
            if(input.empty()) {
                output.clear();
                return true;
            }
            std::error_code error;
            const std::filesystem::path absolute =
                    std::filesystem::absolute(input, error);
            if(error) {
                reason = "failed to make output path absolute '"
                         + input.string() + "': " + error.message();
                return false;
            }
            const std::filesystem::path canonical_parent =
                    std::filesystem::weakly_canonical(
                            absolute.parent_path(), error);
            if(error) {
                reason = "failed to canonicalize output parent '"
                         + absolute.parent_path().string() + "': "
                         + error.message();
                return false;
            }
            output = (canonical_parent / absolute.filename()).lexically_normal();
            return true;
        }

        std::filesystem::file_status directoryEntryStatus(
                const std::filesystem::path & path,
                std::error_code & error)
        {
            std::filesystem::file_status status =
                    std::filesystem::symlink_status(path, error);
            if(error == std::errc::no_such_file_or_directory) {
                error.clear();
                status = std::filesystem::file_status {
                        std::filesystem::file_type::not_found};
            }
            return status;
        }

        std::optional<std::string> validateOutputEntries(
                const std::vector<OutputCommitEntry> & entries)
        {
            std::vector<std::filesystem::path> all_paths;
            all_paths.reserve(entries.size() * 3U);
            for(const OutputCommitEntry & entry : entries) {
                all_paths.push_back(entry.destination.lexically_normal());
                all_paths.push_back(entry.temporary.lexically_normal());
                all_paths.push_back(entry.backup.lexically_normal());
            }
            std::sort(all_paths.begin(), all_paths.end());
            if(std::adjacent_find(all_paths.begin(), all_paths.end())
               != all_paths.end())
            {
                return "analyzer output and staging paths must be distinct";
            }

            for(std::size_t lhs = 0U; lhs < entries.size(); ++lhs) {
                for(std::size_t rhs = lhs + 1U; rhs < entries.size(); ++rhs) {
                    std::error_code error;
                    const bool lhs_exists = std::filesystem::exists(
                            entries[lhs].destination, error);
                    if(error) {
                        return "failed to inspect output '"
                               + entries[lhs].destination.string() + "': "
                               + error.message();
                    }
                    const bool rhs_exists = std::filesystem::exists(
                            entries[rhs].destination, error);
                    if(error) {
                        return "failed to inspect output '"
                               + entries[rhs].destination.string() + "': "
                               + error.message();
                    }
                    if(lhs_exists && rhs_exists
                       && std::filesystem::equivalent(
                               entries[lhs].destination,
                               entries[rhs].destination, error))
                    {
                        return "analyzer output paths resolve to the same file";
                    }
                    if(error) {
                        return "failed to compare output paths: "
                               + error.message();
                    }
                }
            }

            for(const OutputCommitEntry & entry : entries) {
                for(const auto & staging : {entry.temporary, entry.backup}) {
                    std::error_code error;
                    const std::filesystem::file_status status =
                            directoryEntryStatus(staging, error);
                    if(error) {
                        return "failed to inspect staging path '"
                               + staging.string() + "': " + error.message();
                    }
                    if(status.type() != std::filesystem::file_type::not_found) {
                        return "analyzer staging path already exists: "
                               + staging.string();
                    }
                }
            }
            return std::nullopt;
        }

        std::string rollbackOutputs(std::vector<OutputCommitEntry> & entries)
        {
            std::string rollback_error;
            auto appendError = [&](const std::string & message) {
                if(!rollback_error.empty()) {
                    rollback_error += "; ";
                }
                rollback_error += message;
            };
            for(auto entry = entries.rbegin(); entry != entries.rend(); ++entry) {
                if(!entry->published) {
                    continue;
                }
                std::error_code error;
                std::filesystem::remove(entry->destination, error);
                if(error) {
                    appendError(
                            "failed to remove published output '"
                            + entry->destination.string() + "': "
                            + error.message());
                } else {
                    entry->published = false;
                }
            }
            for(auto entry = entries.rbegin(); entry != entries.rend(); ++entry) {
                if(!entry->backed_up) {
                    continue;
                }
                std::error_code error;
                std::filesystem::rename(
                        entry->backup, entry->destination, error);
                if(error) {
                    appendError(
                            "failed to restore output '"
                            + entry->destination.string() + "': "
                            + error.message());
                } else {
                    entry->backed_up = false;
                }
            }
            return rollback_error;
        }

        bool commitOutputs(
                std::vector<OutputCommitEntry> & entries, std::string & reason)
        {
            for(OutputCommitEntry & entry : entries) {
                std::error_code error;
                const std::filesystem::file_status temporary_status =
                        directoryEntryStatus(entry.temporary, error);
                if(error
                   || temporary_status.type()
                              != std::filesystem::file_type::regular)
                {
                    reason = error
                                     ? "failed to inspect temporary output '"
                                               + entry.temporary.string() + "': "
                                               + error.message()
                                     : "temporary output is not a regular file: "
                                               + entry.temporary.string();
                    return false;
                }
                const std::filesystem::file_status destination_status =
                        directoryEntryStatus(entry.destination, error);
                if(error) {
                    reason = "failed to inspect output '"
                             + entry.destination.string() + "': "
                             + error.message();
                    return false;
                }
                entry.had_destination =
                        destination_status.type()
                        != std::filesystem::file_type::not_found;
                if(entry.had_destination
                   && !std::filesystem::is_regular_file(
                           entry.destination, error))
                {
                    reason = error
                                     ? "failed to inspect output '"
                                               + entry.destination.string() + "': "
                                               + error.message()
                                     : "output destination is not a regular file: "
                                               + entry.destination.string();
                    return false;
                }
                const std::filesystem::file_status backup_status =
                        directoryEntryStatus(entry.backup, error);
                if(error
                   || backup_status.type()
                              != std::filesystem::file_type::not_found)
                {
                    reason = error
                                     ? "failed to inspect backup path '"
                                               + entry.backup.string() + "': "
                                               + error.message()
                                     : "output backup path already exists: "
                                               + entry.backup.string();
                    return false;
                }
            }

            for(OutputCommitEntry & entry : entries) {
                if(!entry.had_destination) {
                    continue;
                }
                std::error_code error;
                std::filesystem::rename(
                        entry.destination, entry.backup, error);
                if(error) {
                    reason = "failed to back up output '"
                             + entry.destination.string() + "': "
                             + error.message();
                    const std::string rollback_error = rollbackOutputs(entries);
                    if(!rollback_error.empty()) {
                        reason += "; rollback failed: " + rollback_error;
                    }
                    return false;
                }
                entry.backed_up = true;
            }

            for(OutputCommitEntry & entry : entries) {
                std::error_code error;
                std::filesystem::rename(
                        entry.temporary, entry.destination, error);
                if(error) {
                    reason = "failed to publish output '"
                             + entry.destination.string() + "': "
                             + error.message();
                    const std::string rollback_error = rollbackOutputs(entries);
                    if(!rollback_error.empty()) {
                        reason += "; rollback failed: " + rollback_error;
                    }
                    return false;
                }
                entry.published = true;
            }

            for(OutputCommitEntry & entry : entries) {
                if(!entry.backed_up) {
                    continue;
                }
                std::error_code ignored;
                std::filesystem::remove(entry.backup, ignored);
                if(!ignored) {
                    entry.backed_up = false;
                }
            }
            return true;
        }

        void removeTemporary(const std::filesystem::path & path)
        {
            if(path.empty()) {
                return;
            }
            std::error_code ignored;
            std::filesystem::remove(path, ignored);
        }

        class TemporaryOutputGuard
        {
        public:
            explicit TemporaryOutputGuard(std::vector<std::filesystem::path> paths)
                : paths_(std::move(paths))
            {
            }

            ~TemporaryOutputGuard()
            {
                if(released_) {
                    return;
                }
                for(const auto & path : paths_) {
                    removeTemporary(path);
                }
            }

            void release()
            {
                released_ = true;
            }

        private:
            std::vector<std::filesystem::path> paths_;
            bool released_ {};
        };

        BagAnalysisResult failure(
                const BagAnalysisStatus status, const std::size_t frames,
                std::string reason, const std::filesystem::path & statistics_temporary,
                const std::filesystem::path & timing_temporary)
        {
            removeTemporary(statistics_temporary);
            removeTemporary(timing_temporary);
            return BagAnalysisResult {status, frames, std::move(reason)};
        }

    }// namespace

    BagAnalysisResult analyzeGlobalFrontierBag(const BagAnalyzerOptions & options)
    {
        if(options.bag_uri.empty() || options.statistics_csv.empty()
           || options.topic_name.empty() || options.expected_frame.empty())
        {
            return BagAnalysisResult {
                    BagAnalysisStatus::OutputError, 0U,
                    "bag URI, statistics output, topic, and frame must be non-empty"};
        }
        const bool component_detail_requested =
                !options.component_csv.empty()
                || !options.component_membership_csv.empty();
        if(options.component_csv.empty()
           != options.component_membership_csv.empty())
        {
            return BagAnalysisResult {
                    BagAnalysisStatus::OutputError, 0U,
                    "component and membership CSV paths must be provided together"};
        }
        std::filesystem::path statistics_destination;
        std::filesystem::path timing_destination;
        std::filesystem::path component_destination;
        std::filesystem::path membership_destination;
        std::string path_reason;
        if(!normalizeOutputPath(
                   options.statistics_csv, statistics_destination,
                   path_reason)
           || !normalizeOutputPath(
                   options.timing_csv, timing_destination, path_reason)
           || !normalizeOutputPath(
                   options.component_csv, component_destination,
                   path_reason)
           || !normalizeOutputPath(
                   options.component_membership_csv,
                   membership_destination, path_reason))
        {
            return BagAnalysisResult {
                    BagAnalysisStatus::OutputError, 0U,
                    std::move(path_reason)};
        }
        const std::filesystem::path statistics_temporary =
                statistics_destination.string() + ".tmp";
        const std::filesystem::path timing_temporary =
                timing_destination.empty()
                        ? std::filesystem::path {}
                        : std::filesystem::path {timing_destination.string() + ".tmp"};
        const std::filesystem::path component_temporary =
                component_destination.empty()
                        ? std::filesystem::path {}
                        : std::filesystem::path {component_destination.string() + ".tmp"};
        const std::filesystem::path membership_temporary =
                membership_destination.empty()
                        ? std::filesystem::path {}
                        : std::filesystem::path {membership_destination.string() + ".tmp"};
        std::vector<OutputCommitEntry> output_entries;
        auto appendOutput = [&](const std::filesystem::path & destination,
                                const std::filesystem::path & temporary) {
            if(destination.empty()) {
                return;
            }
            output_entries.push_back(OutputCommitEntry {
                    temporary, destination,
                    std::filesystem::path {
                            destination.string() + ".backup.tmp"}});
        };
        appendOutput(statistics_destination, statistics_temporary);
        appendOutput(timing_destination, timing_temporary);
        appendOutput(component_destination, component_temporary);
        appendOutput(membership_destination, membership_temporary);
        if(const auto path_error = validateOutputEntries(output_entries);
           path_error.has_value())
        {
            return BagAnalysisResult {
                    BagAnalysisStatus::OutputError, 0U,
                    *path_error};
        }

        rosbag2_cpp::Reader reader;
        try {
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = options.bag_uri.string();
            storage_options.storage_id = "mcap";
            reader.open(storage_options);
        } catch(const std::exception & error) {
            return BagAnalysisResult {
                    BagAnalysisStatus::StorageUnavailable, 0U,
                    std::string("failed to open MCAP bag: ") + error.what()};
        }

        std::vector<rosbag2_storage::TopicMetadata> topics;
        try {
            topics = reader.get_all_topics_and_types();
        } catch(const std::exception & error) {
            return BagAnalysisResult {
                    BagAnalysisStatus::InvalidSchema, 0U,
                    std::string("failed to read bag metadata: ") + error.what()};
        }
        const auto topic = std::find_if(
                topics.begin(), topics.end(), [&](const auto & candidate) {
                    return candidate.name == options.topic_name;
                });
        if(topic == topics.end()) {
            return BagAnalysisResult {
                    BagAnalysisStatus::InvalidSchema, 0U,
                    "required topic is missing: " + options.topic_name};
        }
        if(topic->type != OCTOMAP_TYPE || topic->serialization_format != "cdr") {
            return BagAnalysisResult {
                    BagAnalysisStatus::InvalidSchema, 0U,
                    "topic must use octomap_msgs/msg/Octomap with cdr serialization"};
        }
        rosbag2_storage::StorageFilter filter;
        filter.topics = {options.topic_name};
        try {
            reader.set_filter(filter);
        } catch(const std::exception & error) {
            return BagAnalysisResult {
                    BagAnalysisStatus::InvalidSchema, 0U,
                    std::string("failed to filter bag topic: ") + error.what()};
        }

        std::error_code directory_error;
        for(const OutputCommitEntry & output : output_entries) {
            const auto & output_path = output.destination;
            if(output_path.parent_path().empty()) {
                continue;
            }
            directory_error.clear();
            std::filesystem::create_directories(
                    output_path.parent_path(), directory_error);
            if(directory_error) {
                return failure(
                        BagAnalysisStatus::OutputError, 0U,
                        "failed to create output directory: "
                                + directory_error.message(),
                        statistics_temporary, timing_temporary);
            }
        }
        if(const auto path_error = validateOutputEntries(output_entries);
           path_error.has_value())
        {
            return failure(
                    BagAnalysisStatus::OutputError, 0U, *path_error,
                    statistics_temporary, timing_temporary);
        }
        TemporaryOutputGuard output_guard(
                {statistics_temporary, timing_temporary, component_temporary,
                 membership_temporary});

        std::ofstream statistics(statistics_temporary, std::ios::binary | std::ios::trunc);
        if(!statistics) {
            return failure(
                    BagAnalysisStatus::OutputError, 0U,
                    "failed to open statistics output", statistics_temporary,
                    timing_temporary);
        }
        statistics.imbue(std::locale::classic());
        writeStatisticsHeader(statistics);

        std::ofstream timing;
        if(!timing_temporary.empty()) {
            timing.open(timing_temporary, std::ios::binary | std::ios::trunc);
            if(!timing) {
                statistics.close();
                return failure(
                        BagAnalysisStatus::OutputError, 0U,
                        "failed to open timing output", statistics_temporary,
                        timing_temporary);
            }
            timing.imbue(std::locale::classic());
            timing << std::fixed << std::setprecision(9);
            writeTimingHeader(timing);
        }

        std::ofstream components;
        std::ofstream membership;
        if(component_detail_requested) {
            components.open(
                    component_temporary, std::ios::binary | std::ios::trunc);
            membership.open(
                    membership_temporary, std::ios::binary | std::ios::trunc);
            if(!components || !membership) {
                statistics.close();
                timing.close();
                return failure(
                        BagAnalysisStatus::OutputError, 0U,
                        "failed to open component audit output",
                        statistics_temporary, timing_temporary);
            }
            components.imbue(std::locale::classic());
            components << std::fixed << std::setprecision(9);
            membership.imbue(std::locale::classic());
            writeComponentHeader(components);
            writeMembershipHeader(membership);
        }

        auto detector_config = options.detector_config;
        detector_config.collect_stage_timings = !timing_temporary.empty();
        std::unique_ptr<GlobalFrontierDetector> detector;
        try {
            detector = std::make_unique<GlobalFrontierDetector>(detector_config);
        } catch(const std::exception & error) {
            statistics.close();
            timing.close();
            return failure(
                    BagAnalysisStatus::InvalidMessage, 0U,
                    std::string("invalid detector configuration: ") + error.what(),
                    statistics_temporary, timing_temporary);
        }
        std::size_t frame_index = 0U;
        while(true) {
            bool has_next = false;
            try {
                has_next = reader.has_next();
            } catch(const std::exception & error) {
                statistics.close();
                timing.close();
                return failure(
                        BagAnalysisStatus::InvalidMessage, frame_index,
                        std::string("failed to inspect frame ")
                                + std::to_string(frame_index) + ": " + error.what(),
                        statistics_temporary, timing_temporary);
            }
            if(!has_next) {
                break;
            }
            std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message;
            try {
                bag_message = reader.read_next();
            } catch(const std::exception & error) {
                statistics.close();
                timing.close();
                return failure(
                        BagAnalysisStatus::InvalidMessage, frame_index,
                        std::string("failed to read frame ")
                                + std::to_string(frame_index) + ": " + error.what(),
                        statistics_temporary, timing_temporary);
            }

            octomap_msgs::msg::Octomap message;
            try {
                rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
                rclcpp::Serialization<octomap_msgs::msg::Octomap> serialization;
                serialization.deserialize_message(&serialized, &message);
            } catch(const std::exception & error) {
                statistics.close();
                timing.close();
                return failure(
                        BagAnalysisStatus::InvalidMessage, frame_index,
                        std::string("failed to deserialize frame ")
                                + std::to_string(frame_index) + ": " + error.what(),
                        statistics_temporary, timing_temporary);
            }
            if(message.header.frame_id != options.expected_frame) {
                statistics.close();
                timing.close();
                return failure(
                        BagAnalysisStatus::InvalidMessage, frame_index,
                        "frame_id mismatch at frame " + std::to_string(frame_index),
                        statistics_temporary, timing_temporary);
            }
            if(message.binary) {
                statistics.close();
                timing.close();
                return failure(
                        BagAnalysisStatus::InvalidMessage, frame_index,
                        "binary Octomap is not supported at frame "
                                + std::to_string(frame_index),
                        statistics_temporary, timing_temporary);
            }
            if(message.id != "OcTree") {
                statistics.close();
                timing.close();
                return failure(
                        BagAnalysisStatus::InvalidMessage, frame_index,
                        "Octomap id must be OcTree at frame "
                                + std::to_string(frame_index),
                        statistics_temporary, timing_temporary);
            }
            if(std::fabs(message.resolution - detector_config.resolution) > 1.0e-5) {
                statistics.close();
                timing.close();
                return failure(
                        BagAnalysisStatus::InvalidMessage, frame_index,
                        "Octomap resolution mismatch at frame "
                                + std::to_string(frame_index),
                        statistics_temporary, timing_temporary);
            }

            std::unique_ptr<octomap::AbstractOcTree> abstract;
            try {
                abstract.reset(octomap_msgs::fullMsgToMap(message));
            } catch(const std::exception & error) {
                statistics.close();
                timing.close();
                return failure(
                        BagAnalysisStatus::InvalidMessage, frame_index,
                        std::string("failed to decode OcTree at frame ")
                                + std::to_string(frame_index) + ": " + error.what(),
                        statistics_temporary, timing_temporary);
            }
            auto * raw_tree = dynamic_cast<octomap::OcTree *>(abstract.get());
            if(raw_tree == nullptr) {
                statistics.close();
                timing.close();
                return failure(
                        BagAnalysisStatus::InvalidMessage, frame_index,
                        "message does not contain an OcTree at frame "
                                + std::to_string(frame_index),
                        statistics_temporary, timing_temporary);
            }
            FrontierDetectionResult detection;
            std::optional<FrontierDetectionTrace> component_trace;
            if(component_detail_requested) {
                TracedFrontierDetectionResult traced =
                        detector->detectWithTrace(*raw_tree);
                detection = std::move(traced.result);
                component_trace = std::move(traced.trace);
                const auto trace_error =
                        componentTraceError(detection, *component_trace);
                if(trace_error.has_value()) {
                    statistics.close();
                    timing.close();
                    components.close();
                    membership.close();
                    return failure(
                            BagAnalysisStatus::IncompleteTrace, frame_index,
                            "incomplete component trace at frame "
                                    + std::to_string(frame_index) + ": "
                                    + *trace_error,
                            statistics_temporary, timing_temporary);
                }
                for(const FrontierComponentTrace & component : component_trace->components) {
                    writeComponentRow(
                            components, frame_index, bag_message->recv_timestamp, message,
                            component);
                    writeMembershipRows(membership, frame_index, component);
                }
            } else {
                detection = detector->detect(*raw_tree);
            }
            writeStatisticsRow(
                    statistics, frame_index, bag_message->recv_timestamp, message,
                    detection);
            if(timing) {
                writeTimingRow(
                        timing, frame_index, bag_message->recv_timestamp, message,
                        detection.diagnostics.timings);
            }
            ++frame_index;
        }

        statistics.close();
        timing.close();
        components.close();
        membership.close();
        if(!statistics) {
            return failure(
                    BagAnalysisStatus::OutputError, frame_index,
                    "failed while writing statistics output", statistics_temporary,
                    timing_temporary);
        }
        if(!timing_temporary.empty() && !timing) {
            return failure(
                    BagAnalysisStatus::OutputError, frame_index,
                    "failed while writing timing output", statistics_temporary,
                    timing_temporary);
        }
        if(component_detail_requested && (!components || !membership)) {
            return failure(
                    BagAnalysisStatus::OutputError, frame_index,
                    "failed while writing component audit output",
                    statistics_temporary, timing_temporary);
        }
        std::string commit_reason;
        if(!commitOutputs(output_entries, commit_reason)) {
            return failure(
                    BagAnalysisStatus::OutputError, frame_index,
                    std::move(commit_reason), statistics_temporary, timing_temporary);
        }
        output_guard.release();
        return BagAnalysisResult {BagAnalysisStatus::Success, frame_index, {}};
    }

}// namespace SwarmController::Test

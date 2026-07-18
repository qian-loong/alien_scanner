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

        bool commitOutput(
                const std::filesystem::path & temporary,
                const std::filesystem::path & destination,
                std::string & reason)
        {
            std::error_code error;
            std::filesystem::rename(temporary, destination, error);
            if(!error) {
                return true;
            }
            reason = "failed to commit output '" + destination.string()
                     + "': " + error.message();
            return false;
        }

        void removeTemporary(const std::filesystem::path & path)
        {
            if(path.empty()) {
                return;
            }
            std::error_code ignored;
            std::filesystem::remove(path, ignored);
        }

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

        const std::filesystem::path statistics_temporary =
                options.statistics_csv.string() + ".tmp";
        const std::filesystem::path timing_temporary = options.timing_csv.empty()
                                                                 ? std::filesystem::path {}
                                                                 : std::filesystem::path {
                                                                           options.timing_csv.string()
                                                                           + ".tmp"};
        removeTemporary(statistics_temporary);
        removeTemporary(timing_temporary);
        std::error_code directory_error;
        if(!options.statistics_csv.parent_path().empty()) {
            std::filesystem::create_directories(
                    options.statistics_csv.parent_path(), directory_error);
        }
        if(directory_error) {
            return failure(
                    BagAnalysisStatus::OutputError, 0U,
                    "failed to create statistics output directory: "
                            + directory_error.message(),
                    statistics_temporary, timing_temporary);
        }
        if(!options.timing_csv.empty() && !options.timing_csv.parent_path().empty()) {
            std::filesystem::create_directories(
                    options.timing_csv.parent_path(), directory_error);
        }
        if(directory_error) {
            return failure(
                    BagAnalysisStatus::OutputError, 0U,
                    "failed to create timing output directory: "
                            + directory_error.message(),
                    statistics_temporary, timing_temporary);
        }

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
            const FrontierDetectionResult detection = detector->detect(*raw_tree);
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
        std::string commit_reason;
        if(!commitOutput(
                   statistics_temporary, options.statistics_csv, commit_reason))
        {
            return failure(
                    BagAnalysisStatus::OutputError, frame_index,
                    std::move(commit_reason), statistics_temporary, timing_temporary);
        }
        if(!timing_temporary.empty()
           && !commitOutput(timing_temporary, options.timing_csv, commit_reason))
        {
            return failure(
                    BagAnalysisStatus::OutputError, frame_index,
                    std::move(commit_reason), statistics_temporary, timing_temporary);
        }
        return BagAnalysisResult {BagAnalysisStatus::Success, frame_index, {}};
    }

}// namespace SwarmController::Test

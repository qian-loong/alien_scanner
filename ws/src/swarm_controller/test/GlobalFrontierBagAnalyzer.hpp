#ifndef SWARM_CONTROLLER_TEST_GLOBALFRONTIERBAGANALYZER_HPP
#define SWARM_CONTROLLER_TEST_GLOBALFRONTIERBAGANALYZER_HPP

#include "swarm_controller/GlobalFrontierDetector.hpp"

#include <cstddef>
#include <filesystem>
#include <string>

namespace SwarmController::Test {

    enum class BagAnalysisStatus {
        Success,
        StorageUnavailable,
        InvalidSchema,
        InvalidMessage,
        IncompleteTrace,
        OutputError,
    };

    struct BagAnalyzerOptions {
        std::filesystem::path bag_uri;
        std::filesystem::path statistics_csv;
        std::filesystem::path timing_csv;
        std::filesystem::path component_csv;
        std::filesystem::path component_membership_csv;
        std::string topic_name {"/global_map"};
        std::string expected_frame {"map"};
        GlobalFrontierDetectorConfig detector_config;
    };

    struct BagAnalysisResult {
        BagAnalysisStatus status {BagAnalysisStatus::InvalidMessage};
        std::size_t frames_analyzed {};
        std::string reason;

        bool succeeded() const
        {
            return status == BagAnalysisStatus::Success;
        }
    };

    BagAnalysisResult analyzeGlobalFrontierBag(const BagAnalyzerOptions & options);

}// namespace SwarmController::Test

#endif// SWARM_CONTROLLER_TEST_GLOBALFRONTIERBAGANALYZER_HPP

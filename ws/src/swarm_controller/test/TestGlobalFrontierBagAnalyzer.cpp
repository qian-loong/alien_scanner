#include "GlobalFrontierBagAnalyzer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/string.hpp>

namespace SwarmController::Test {

    namespace {

        class TemporaryDirectory
        {
        public:
            TemporaryDirectory()
            {
                const auto suffix = std::chrono::steady_clock::now()
                                            .time_since_epoch()
                                            .count();
                path_ = std::filesystem::temp_directory_path()
                        / ("global_frontier_bag_analyzer_" + std::to_string(suffix));
                std::filesystem::create_directories(path_);
            }

            ~TemporaryDirectory()
            {
                std::error_code ignored;
                std::filesystem::remove_all(path_, ignored);
            }

            const std::filesystem::path & path() const
            {
                return path_;
            }

        private:
            std::filesystem::path path_;
        };

        octomap_msgs::msg::Octomap validMapMessage(const std::int32_t stamp_seconds = 1)
        {
            octomap::OcTree tree(0.1);
            for(int x = 0; x <= 4; ++x) {
                for(int y = -3; y <= 3; ++y) {
                    for(int z = 12; z <= 18; ++z) {
                        tree.updateNode(
                                octomap::point3d(
                                        0.1F * static_cast<float>(x),
                                        0.1F * static_cast<float>(y),
                                        0.1F * static_cast<float>(z)),
                                false);
                    }
                }
            }
            tree.updateInnerOccupancy();
            octomap_msgs::msg::Octomap message;
            if(!octomap_msgs::fullMapToMsg(tree, message)) {
                throw std::runtime_error("failed to serialize test OcTree");
            }
            message.header.frame_id = "map";
            message.header.stamp.sec = stamp_seconds;
            return message;
        }

        std::unique_ptr<rosbag2_cpp::Writer> openWriter(
                const std::filesystem::path & uri)
        {
            rosbag2_storage::StorageOptions options;
            options.uri = uri.string();
            options.storage_id = "mcap";
            auto writer = std::make_unique<rosbag2_cpp::Writer>();
            writer->open(options);
            return writer;
        }

        void writeMaps(
                const std::filesystem::path & uri,
                const std::vector<octomap_msgs::msg::Octomap> & messages,
                const std::string & topic = "/global_map")
        {
            auto writer = openWriter(uri);
            std::int64_t timestamp = 1'000'000'000LL;
            for(const auto & message : messages) {
                writer->write(message, topic, rclcpp::Time(timestamp));
                timestamp += 1'000'000'000LL;
            }
            writer->close();
        }

        std::string readFile(const std::filesystem::path & path)
        {
            std::ifstream input(path, std::ios::binary);
            std::ostringstream output;
            output << input.rdbuf();
            return output.str();
        }

        void writeFile(
                const std::filesystem::path & path,
                const std::string & content)
        {
            std::ofstream output(path, std::ios::binary | std::ios::trunc);
            output << content;
            if(!output) {
                throw std::runtime_error("failed to write test file");
            }
        }

        void expectNoStagingFile(const std::filesystem::path & output)
        {
            EXPECT_FALSE(std::filesystem::exists(output.string() + ".tmp"));
            EXPECT_FALSE(
                    std::filesystem::exists(output.string() + ".backup.tmp"));
        }

        std::vector<std::string> nonEmptyLines(const std::string & content)
        {
            std::vector<std::string> lines;
            std::istringstream input(content);
            for(std::string line; std::getline(input, line);) {
                if(!line.empty()) {
                    lines.push_back(std::move(line));
                }
            }
            return lines;
        }

        BagAnalyzerOptions optionsFor(
                const std::filesystem::path & bag,
                const std::filesystem::path & statistics)
        {
            BagAnalyzerOptions options;
            options.bag_uri = bag;
            options.statistics_csv = statistics;
            return options;
        }

        GlobalFrontierDetectorConfig auditDetectorConfig()
        {
            GlobalFrontierDetectorConfig config;
            config.column_stride_voxels = 1U;
            config.min_columns = 4U;
            config.min_area = 0.05F;
            config.min_span = 0.3F;
            config.min_direction_consistency = 0.3F;
            config.support_depth = 0.2;
            config.max_frontier_columns = 10'000U;
            return config;
        }

    }// namespace

    TEST(GlobalFrontierBagAnalyzerTest, StatisticsOutputIsByteDeterministic)
    {
        TemporaryDirectory temporary;
        const auto bag = temporary.path() / "valid_bag";
        try {
            writeMaps(bag, {validMapMessage(1), validMapMessage(2)});
        } catch(const std::exception & error) {
            GTEST_SKIP() << "MCAP storage plugin unavailable: " << error.what();
        }

        auto first = optionsFor(bag, temporary.path() / "first.csv");
        first.detector_config = auditDetectorConfig();
        first.timing_csv = temporary.path() / "timing.csv";
        first.component_csv = temporary.path() / "components_first.csv";
        first.component_membership_csv =
                temporary.path() / "membership_first.csv";
        auto second = optionsFor(bag, temporary.path() / "second.csv");
        second.detector_config = auditDetectorConfig();
        second.component_csv = temporary.path() / "components_second.csv";
        second.component_membership_csv =
                temporary.path() / "membership_second.csv";
        auto aggregate_only = optionsFor(
                bag, temporary.path() / "aggregate_only.csv");
        aggregate_only.detector_config = auditDetectorConfig();

        const auto first_result = analyzeGlobalFrontierBag(first);
        const auto second_result = analyzeGlobalFrontierBag(second);
        const auto aggregate_only_result = analyzeGlobalFrontierBag(aggregate_only);

        ASSERT_TRUE(first_result.succeeded()) << first_result.reason;
        ASSERT_TRUE(second_result.succeeded()) << second_result.reason;
        ASSERT_TRUE(aggregate_only_result.succeeded())
                << aggregate_only_result.reason;
        EXPECT_EQ(first_result.frames_analyzed, 2U);
        EXPECT_EQ(second_result.frames_analyzed, 2U);
        const std::string first_statistics = readFile(first.statistics_csv);
        EXPECT_EQ(first_statistics, readFile(second.statistics_csv));
        EXPECT_EQ(first_statistics, readFile(aggregate_only.statistics_csv));
        EXPECT_EQ(readFile(first.component_csv), readFile(second.component_csv));
        EXPECT_EQ(
                readFile(first.component_membership_csv),
                readFile(second.component_membership_csv));
        const auto lines = nonEmptyLines(first_statistics);
        ASSERT_EQ(lines.size(), 3U);
        const std::string expected_header =
                "frame_index,bag_timestamp_ns,map_stamp_ns,status,complete,"
                "scanned_free_voxels,sampled_free_columns,"
                "unknown_neighbor_candidate_columns,vertical_passed_columns,"
                "vertical_rejected_columns,support_passed_columns,"
                "support_rejected_unknown,support_rejected_occupied,"
                "support_rejected_out_of_bounds,support_samples_attempted,"
                "support_failure_position_unavailable,"
                "support_failure_depth_octile_0,support_failure_depth_octile_1,"
                "support_failure_depth_octile_2,support_failure_depth_octile_3,"
                "support_failure_depth_octile_4,support_failure_depth_octile_5,"
                "support_failure_depth_octile_6,support_failure_depth_octile_7,"
                "components_built,component_size_bucket_0,component_size_bucket_1,"
                "component_size_bucket_2,component_size_bucket_3,"
                "component_size_bucket_4,component_size_bucket_5,"
                "component_primary_rejected_columns,component_primary_rejected_area,"
                "component_primary_rejected_span,component_primary_rejected_direction,"
                "components_accepted,detected_regions,reason";
        EXPECT_EQ(lines.front(), expected_header);
        const auto expected_commas = static_cast<std::size_t>(
                std::count(expected_header.begin(), expected_header.end(), ','));
        for(std::size_t index = 1U; index < lines.size(); ++index) {
            EXPECT_EQ(
                    static_cast<std::size_t>(
                            std::count(lines[index].begin(), lines[index].end(), ',')),
                    expected_commas);
        }
        EXPECT_FALSE(readFile(first.timing_csv).empty());
        const auto component_lines = nonEmptyLines(readFile(first.component_csv));
        const auto membership_lines =
                nonEmptyLines(readFile(first.component_membership_csv));
        ASSERT_GT(component_lines.size(), 1U);
        ASSERT_GT(membership_lines.size(), 1U);
        EXPECT_EQ(
                component_lines.front(),
                "frame_index,bag_timestamp_ns,map_stamp_ns,component_index,"
                "stable_key_x,stable_key_y,exact_columns,area,horizontal_span,"
                "representative_x,representative_y,representative_z,"
                "unknown_direction_x,unknown_direction_y,unknown_direction_z,"
                "information_gain,direction_consistency,direction_votes_pos_x,"
                "direction_votes_pos_y,direction_votes_neg_x,direction_votes_neg_y,"
                "xy_min_x,xy_min_y,xy_max_x,xy_max_y,rejection,columns_complete,"
                "edges_complete");
        EXPECT_EQ(
                membership_lines.front(),
                "frame_index,component_index,stable_key_x,stable_key_y,column_x,column_y");
    }

    TEST(GlobalFrontierBagAnalyzerTest, RejectsIncompleteComponentTraceWithoutOutput)
    {
        TemporaryDirectory temporary;
        const auto bag = temporary.path() / "trace_limited_bag";
        try {
            writeMaps(bag, {validMapMessage()});
        } catch(const std::exception & error) {
            GTEST_SKIP() << "MCAP storage plugin unavailable: " << error.what();
        }
        auto options = optionsFor(bag, temporary.path() / "statistics.csv");
        options.detector_config = auditDetectorConfig();
        options.detector_config.max_trace_geometry_elements = 1U;
        options.component_csv = temporary.path() / "components.csv";
        options.component_membership_csv = temporary.path() / "membership.csv";

        const auto result = analyzeGlobalFrontierBag(options);

        EXPECT_EQ(result.status, BagAnalysisStatus::IncompleteTrace);
        EXPECT_EQ(result.frames_analyzed, 0U);
        for(const auto & output :
            {options.statistics_csv, options.component_csv,
             options.component_membership_csv})
        {
            EXPECT_FALSE(std::filesystem::exists(output));
            expectNoStagingFile(output);
        }
    }

    TEST(GlobalFrontierBagAnalyzerTest, RequiresDistinctComponentOutputPair)
    {
        BagAnalyzerOptions missing_pair;
        missing_pair.bag_uri = "unused";
        missing_pair.statistics_csv = "statistics.csv";
        missing_pair.component_csv = "components.csv";
        const auto missing = analyzeGlobalFrontierBag(missing_pair);
        EXPECT_EQ(missing.status, BagAnalysisStatus::OutputError);

        BagAnalyzerOptions duplicate;
        duplicate.bag_uri = "unused";
        duplicate.statistics_csv = "same.csv";
        duplicate.component_csv = "components.csv";
        duplicate.component_membership_csv = "same.csv";
        const auto repeated = analyzeGlobalFrontierBag(duplicate);
        EXPECT_EQ(repeated.status, BagAnalysisStatus::OutputError);

        TemporaryDirectory temporary;
        BagAnalyzerOptions staging_collision;
        staging_collision.bag_uri = "unused";
        staging_collision.statistics_csv = temporary.path() / "report.tmp";
        staging_collision.component_csv = temporary.path() / "report";
        staging_collision.component_membership_csv =
                temporary.path() / "membership";
        const auto collision = analyzeGlobalFrontierBag(staging_collision);
        EXPECT_EQ(collision.status, BagAnalysisStatus::OutputError);
        EXPECT_NE(collision.reason.find("staging paths"), std::string::npos);
        EXPECT_FALSE(std::filesystem::exists(staging_collision.statistics_csv));
        EXPECT_FALSE(std::filesystem::exists(staging_collision.component_csv));
        EXPECT_FALSE(
                std::filesystem::exists(
                        staging_collision.component_membership_csv));

        BagAnalyzerOptions absolute_collision;
        absolute_collision.bag_uri = "unused";
        absolute_collision.statistics_csv = std::filesystem::relative(
                temporary.path() / "absolute_report.tmp",
                std::filesystem::current_path());
        absolute_collision.component_csv =
                temporary.path() / "absolute_report";
        absolute_collision.component_membership_csv =
                temporary.path() / "absolute_membership";
        const auto mixed_identity =
                analyzeGlobalFrontierBag(absolute_collision);
        EXPECT_EQ(mixed_identity.status, BagAnalysisStatus::OutputError);
        EXPECT_NE(
                mixed_identity.reason.find("staging paths"),
                std::string::npos);

        const auto real_directory = temporary.path() / "real";
        const auto alias_directory = temporary.path() / "alias";
        std::filesystem::create_directory(real_directory);
        std::filesystem::create_directory_symlink(
                real_directory, alias_directory);
        BagAnalyzerOptions symlink_collision;
        symlink_collision.bag_uri = "unused";
        symlink_collision.statistics_csv =
                alias_directory / "symlink_report.tmp";
        symlink_collision.component_csv =
                real_directory / "symlink_report";
        symlink_collision.component_membership_csv =
                temporary.path() / "symlink_membership";
        const auto aliased = analyzeGlobalFrontierBag(symlink_collision);
        EXPECT_EQ(aliased.status, BagAnalysisStatus::OutputError);
        EXPECT_NE(aliased.reason.find("staging paths"), std::string::npos);

        const auto first_link = temporary.path() / "first_link.csv";
        const auto second_link = temporary.path() / "second_link.csv";
        writeFile(first_link, "existing output");
        std::filesystem::create_hard_link(first_link, second_link);
        BagAnalyzerOptions hard_link_collision;
        hard_link_collision.bag_uri = "unused";
        hard_link_collision.statistics_csv = first_link;
        hard_link_collision.component_csv = second_link;
        hard_link_collision.component_membership_csv =
                temporary.path() / "hard_link_membership.csv";
        const auto equivalent =
                analyzeGlobalFrontierBag(hard_link_collision);
        EXPECT_EQ(equivalent.status, BagAnalysisStatus::OutputError);
        EXPECT_NE(equivalent.reason.find("same file"), std::string::npos);
        EXPECT_EQ(readFile(first_link), "existing output");
        EXPECT_EQ(readFile(second_link), "existing output");

        const auto dangling_temporary_output =
                temporary.path() / "dangling_temporary.csv";
        const std::filesystem::path dangling_temporary =
                dangling_temporary_output.string() + ".tmp";
        std::filesystem::create_symlink(
                dangling_temporary_output, dangling_temporary);
        BagAnalyzerOptions dangling_temporary_options;
        dangling_temporary_options.bag_uri = "unused";
        dangling_temporary_options.statistics_csv =
                dangling_temporary_output;
        const auto temporary_rejected =
                analyzeGlobalFrontierBag(dangling_temporary_options);
        EXPECT_EQ(temporary_rejected.status, BagAnalysisStatus::OutputError);
        EXPECT_TRUE(std::filesystem::is_symlink(
                std::filesystem::symlink_status(dangling_temporary)));

        const auto dangling_backup_output =
                temporary.path() / "dangling_backup.csv";
        const std::filesystem::path dangling_backup =
                dangling_backup_output.string() + ".backup.tmp";
        std::filesystem::create_symlink(
                dangling_backup_output, dangling_backup);
        BagAnalyzerOptions dangling_backup_options;
        dangling_backup_options.bag_uri = "unused";
        dangling_backup_options.statistics_csv = dangling_backup_output;
        const auto backup_rejected =
                analyzeGlobalFrontierBag(dangling_backup_options);
        EXPECT_EQ(backup_rejected.status, BagAnalysisStatus::OutputError);
        EXPECT_TRUE(std::filesystem::is_symlink(
                std::filesystem::symlink_status(dangling_backup)));
    }

    TEST(GlobalFrontierBagAnalyzerTest, RejectsNonRegularDestinationBeforePublishing)
    {
        TemporaryDirectory temporary;
        const auto bag = temporary.path() / "valid_bag";
        try {
            writeMaps(bag, {validMapMessage()});
        } catch(const std::exception & error) {
            GTEST_SKIP() << "MCAP storage plugin unavailable: " << error.what();
        }
        auto options = optionsFor(bag, temporary.path() / "statistics.csv");
        options.detector_config = auditDetectorConfig();
        options.component_csv = temporary.path() / "components.csv";
        options.component_membership_csv = temporary.path() / "membership.csv";
        writeFile(options.statistics_csv, "old statistics");
        writeFile(options.component_csv, "old components");
        std::filesystem::create_directory(options.component_membership_csv);

        const auto result = analyzeGlobalFrontierBag(options);

        EXPECT_EQ(result.status, BagAnalysisStatus::OutputError);
        EXPECT_EQ(readFile(options.statistics_csv), "old statistics");
        EXPECT_EQ(readFile(options.component_csv), "old components");
        EXPECT_TRUE(std::filesystem::is_directory(
                options.component_membership_csv));
        for(const auto & output :
            {options.statistics_csv, options.component_csv,
             options.component_membership_csv})
        {
            expectNoStagingFile(output);
        }
    }

    TEST(GlobalFrontierBagAnalyzerTest, ReplacesExistingOutputsAndRemovesBackups)
    {
        TemporaryDirectory temporary;
        const auto bag = temporary.path() / "valid_bag";
        try {
            writeMaps(bag, {validMapMessage()});
        } catch(const std::exception & error) {
            GTEST_SKIP() << "MCAP storage plugin unavailable: " << error.what();
        }
        auto options = optionsFor(bag, temporary.path() / "statistics.csv");
        options.detector_config = auditDetectorConfig();
        options.component_csv = temporary.path() / "components.csv";
        options.component_membership_csv = temporary.path() / "membership.csv";
        for(const auto & output :
            {options.statistics_csv, options.component_csv,
             options.component_membership_csv})
        {
            writeFile(output, "old output");
        }

        const auto result = analyzeGlobalFrontierBag(options);

        ASSERT_TRUE(result.succeeded()) << result.reason;
        EXPECT_NE(readFile(options.statistics_csv), "old output");
        EXPECT_NE(readFile(options.component_csv), "old output");
        EXPECT_NE(readFile(options.component_membership_csv), "old output");
        for(const auto & output :
            {options.statistics_csv, options.component_csv,
             options.component_membership_csv})
        {
            expectNoStagingFile(output);
        }
    }

    TEST(GlobalFrontierBagAnalyzerTest, RejectsMissingTopicAndWrongType)
    {
        TemporaryDirectory temporary;
        const auto missing_bag = temporary.path() / "missing_bag";
        try {
            writeMaps(missing_bag, {validMapMessage()}, "/other_map");
        } catch(const std::exception & error) {
            GTEST_SKIP() << "MCAP storage plugin unavailable: " << error.what();
        }
        const auto missing = analyzeGlobalFrontierBag(
                optionsFor(missing_bag, temporary.path() / "missing.csv"));
        EXPECT_EQ(missing.status, BagAnalysisStatus::InvalidSchema);
        EXPECT_FALSE(std::filesystem::exists(temporary.path() / "missing.csv"));

        const auto wrong_type_bag = temporary.path() / "wrong_type_bag";
        auto writer = openWriter(wrong_type_bag);
        std_msgs::msg::String text;
        text.data = "not an Octomap";
        writer->write(text, "/global_map", rclcpp::Time(1'000'000'000LL));
        writer->close();
        const auto wrong_type = analyzeGlobalFrontierBag(
                optionsFor(wrong_type_bag, temporary.path() / "wrong_type.csv"));
        EXPECT_EQ(wrong_type.status, BagAnalysisStatus::InvalidSchema);
        EXPECT_FALSE(std::filesystem::exists(temporary.path() / "wrong_type.csv"));
    }

    TEST(GlobalFrontierBagAnalyzerTest, RejectsInvalidMapContractsWithoutPartialOutput)
    {
        TemporaryDirectory temporary;
        try {
            const auto probe = temporary.path() / "probe_bag";
            writeMaps(probe, {validMapMessage()});
        } catch(const std::exception & error) {
            GTEST_SKIP() << "MCAP storage plugin unavailable: " << error.what();
        }

        auto wrong_frame = validMapMessage();
        wrong_frame.header.frame_id = "odom";
        auto binary = validMapMessage();
        binary.binary = true;
        auto wrong_resolution = validMapMessage();
        wrong_resolution.resolution = 0.2;
        auto wrong_id = validMapMessage();
        wrong_id.id = "UnsupportedTree";
        const std::vector<std::pair<std::string, octomap_msgs::msg::Octomap>> cases {
                {"wrong_frame", wrong_frame},
                {"binary", binary},
                {"wrong_resolution", wrong_resolution},
                {"wrong_id", wrong_id},
        };
        for(const auto & [name, message] : cases) {
            const auto bag = temporary.path() / (name + "_bag");
            const auto output = temporary.path() / (name + ".csv");
            writeMaps(bag, {message});
            const auto result = analyzeGlobalFrontierBag(optionsFor(bag, output));
            EXPECT_EQ(result.status, BagAnalysisStatus::InvalidMessage) << name;
            EXPECT_EQ(result.frames_analyzed, 0U) << name;
            EXPECT_FALSE(std::filesystem::exists(output)) << name;
            EXPECT_FALSE(std::filesystem::exists(output.string() + ".tmp")) << name;
        }
    }

    TEST(GlobalFrontierBagAnalyzerTest, RejectsCorruptedSerialization)
    {
        TemporaryDirectory temporary;
        const auto bag = temporary.path() / "corrupt_bag";
        try {
            auto writer = openWriter(bag);
            auto serialized = std::make_shared<rclcpp::SerializedMessage>(8U);
            auto & raw = serialized->get_rcl_serialized_message();
            raw.buffer[0] = 0x01U;
            raw.buffer[1] = 0x02U;
            raw.buffer[2] = 0x03U;
            raw.buffer[3] = 0x04U;
            raw.buffer_length = 4U;
            writer->write(
                    serialized, "/global_map", "octomap_msgs/msg/Octomap",
                    rclcpp::Time(1'000'000'000LL));
            writer->close();
        } catch(const std::exception & error) {
            GTEST_SKIP() << "MCAP storage plugin unavailable: " << error.what();
        }
        const auto output = temporary.path() / "corrupt.csv";

        const auto result = analyzeGlobalFrontierBag(optionsFor(bag, output));

        EXPECT_EQ(result.status, BagAnalysisStatus::InvalidMessage);
        EXPECT_EQ(result.frames_analyzed, 0U);
        EXPECT_FALSE(std::filesystem::exists(output));
    }

}// namespace SwarmController::Test

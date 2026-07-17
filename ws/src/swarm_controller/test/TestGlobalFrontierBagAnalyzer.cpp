#include "GlobalFrontierBagAnalyzer.hpp"

#include <gtest/gtest.h>

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

        BagAnalyzerOptions optionsFor(
                const std::filesystem::path & bag,
                const std::filesystem::path & statistics)
        {
            BagAnalyzerOptions options;
            options.bag_uri = bag;
            options.statistics_csv = statistics;
            return options;
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
        first.timing_csv = temporary.path() / "timing.csv";
        auto second = optionsFor(bag, temporary.path() / "second.csv");

        const auto first_result = analyzeGlobalFrontierBag(first);
        const auto second_result = analyzeGlobalFrontierBag(second);

        ASSERT_TRUE(first_result.succeeded()) << first_result.reason;
        ASSERT_TRUE(second_result.succeeded()) << second_result.reason;
        EXPECT_EQ(first_result.frames_analyzed, 2U);
        EXPECT_EQ(second_result.frames_analyzed, 2U);
        EXPECT_EQ(readFile(first.statistics_csv), readFile(second.statistics_csv));
        EXPECT_FALSE(readFile(first.timing_csv).empty());
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

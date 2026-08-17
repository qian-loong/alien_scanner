#include "perception_map_update/ros/MapUpdateParameters.hpp"

#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

namespace PerceptionMapUpdate::Test {

    class RclcppScope
    {
    public:
        RclcppScope()
        {
            if(!rclcpp::ok()) {
                int argc = 0;
                rclcpp::init(argc, nullptr);
            }
        }

        ~RclcppScope()
        {
            if(rclcpp::ok()) {
                rclcpp::shutdown();
            }
        }
    };

    TEST(MapUpdateParametersTest, AppliesValidOverridesAndRejectsZeroLimits)
    {
        RclcppScope scope;
        rclcpp::NodeOptions valid_options;
        valid_options.append_parameter_override("map_update.max_known_cells", 42);
        valid_options.append_parameter_override("map_update.max_live_chunks", 21);
        valid_options.append_parameter_override("map_update.max_merkle_nodes", 41);
        valid_options.append_parameter_override("map_update.max_delta_chain_length", 0);
        auto valid_node = std::make_shared<rclcpp::Node>(
                "valid_map_update_parameters", valid_options);
        const auto limits = Ros::declare_map_update_limits(*valid_node);
        EXPECT_EQ(limits.max_known_cells, 42U);
        EXPECT_EQ(limits.max_live_chunks, 21U);
        EXPECT_EQ(limits.max_merkle_nodes, 41U);
        EXPECT_EQ(limits.max_delta_chain_length, 0U);

        rclcpp::NodeOptions zero_cells_options;
        zero_cells_options.append_parameter_override("map_update.max_known_cells", 0);
        auto zero_cells_node = std::make_shared<rclcpp::Node>(
                "zero_map_update_cells", zero_cells_options);
        EXPECT_THROW(
                Ros::declare_map_update_limits(*zero_cells_node),
                std::invalid_argument);

        rclcpp::NodeOptions zero_span_options;
        zero_span_options.append_parameter_override("map_update.max_revision_span", 0);
        auto zero_span_node = std::make_shared<rclcpp::Node>(
                "zero_map_update_revision_span", zero_span_options);
        EXPECT_THROW(
                Ros::declare_map_update_limits(*zero_span_node),
                std::invalid_argument);
    }

}// namespace PerceptionMapUpdate::Test

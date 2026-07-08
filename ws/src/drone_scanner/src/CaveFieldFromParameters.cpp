#include "drone_scanner/CaveFieldFromParameters.hpp"

#include "cave_world/CaveFieldFactory.hpp"

#include <rclcpp/rclcpp.hpp>

namespace DroneScanner {

    void declareCaveFieldParameters(rclcpp::Node & node)
    {
        node.declare_parameter("cave_mode", "tree");

        node.declare_parameter("length", 40.0);
        node.declare_parameter("branch_length", 20.0);
        node.declare_parameter("base_radius", 2.5);
        node.declare_parameter("n_segments", 200);
        node.declare_parameter("branch", true);
        node.declare_parameter("branch_angle", 0.55);
        node.declare_parameter("chamber_at", 0.5);
        node.declare_parameter("chamber_scale", 3.0);
        node.declare_parameter("noise_scale", 0.4);
        node.declare_parameter("density", 400);
        node.declare_parameter("seed", 42);

        node.declare_parameter("tree.approach_length", 12.0);
        node.declare_parameter("tree.loop_yaw", 0.50);
        node.declare_parameter("tree.loop_direct_length", 16.0);
        node.declare_parameter("tree.loop_bulge", 12.0);
        node.declare_parameter("tree.exit1_length", 14.0);
        node.declare_parameter("tree.right_yaw", -0.12);
        node.declare_parameter("tree.right_corridor_length", 10.0);
        node.declare_parameter("tree.exit_yaw_spread", 0.35);
        node.declare_parameter("tree.exit_arm_length", 14.0);
        node.declare_parameter("tree.vertical_step", -0.10);
        node.declare_parameter("tree.asymmetry", 0.22);
        node.declare_parameter("tree.chamber_on_approach", false);
        node.declare_parameter("tree.chamber_at", 0.55);
        node.declare_parameter("tree.chamber_scale", 2.2);
    }

    namespace {

        CaveWorld::ProceduralCaveFieldConfig loadYConfigFromParameters(const rclcpp::Node & node)
        {
            CaveWorld::ProceduralCaveFieldConfig config;
            config.length          = static_cast<float>(node.get_parameter("length").as_double());
            config.branch_length   = static_cast<float>(node.get_parameter("branch_length").as_double());
            config.base_radius     = static_cast<float>(node.get_parameter("base_radius").as_double());
            config.n_segments      = node.get_parameter("n_segments").as_int();
            config.branch          = node.get_parameter("branch").as_bool();
            config.branch_angle    = static_cast<float>(node.get_parameter("branch_angle").as_double());
            config.chamber_at      = static_cast<float>(node.get_parameter("chamber_at").as_double());
            config.chamber_scale   = static_cast<float>(node.get_parameter("chamber_scale").as_double());
            config.noise_scale     = static_cast<float>(node.get_parameter("noise_scale").as_double());
            config.density         = node.get_parameter("density").as_int();
            config.seed            = static_cast<std::uint32_t>(node.get_parameter("seed").as_int());
            return config;
        }

        CaveWorld::TreeCaveFieldConfig loadTreeConfigFromParameters(const rclcpp::Node & node)
        {
            CaveWorld::TreeCaveFieldConfig config;
            config.approach_length =
                    static_cast<float>(node.get_parameter("tree.approach_length").as_double());
            config.base_radius = static_cast<float>(node.get_parameter("base_radius").as_double());
            config.n_segments  = node.get_parameter("n_segments").as_int();
            config.density     = node.get_parameter("density").as_int();
            config.noise_scale = static_cast<float>(node.get_parameter("noise_scale").as_double());
            config.seed        = static_cast<std::uint32_t>(node.get_parameter("seed").as_int());
            config.loop_yaw =
                    static_cast<float>(node.get_parameter("tree.loop_yaw").as_double());
            config.loop_direct_length =
                    static_cast<float>(node.get_parameter("tree.loop_direct_length").as_double());
            config.loop_bulge =
                    static_cast<float>(node.get_parameter("tree.loop_bulge").as_double());
            config.exit1_length =
                    static_cast<float>(node.get_parameter("tree.exit1_length").as_double());
            config.right_yaw =
                    static_cast<float>(node.get_parameter("tree.right_yaw").as_double());
            config.right_corridor_length =
                    static_cast<float>(node.get_parameter("tree.right_corridor_length").as_double());
            config.exit_yaw_spread =
                    static_cast<float>(node.get_parameter("tree.exit_yaw_spread").as_double());
            config.exit_arm_length =
                    static_cast<float>(node.get_parameter("tree.exit_arm_length").as_double());
            config.vertical_step =
                    static_cast<float>(node.get_parameter("tree.vertical_step").as_double());
            config.asymmetry =
                    static_cast<float>(node.get_parameter("tree.asymmetry").as_double());
            config.chamber_on_approach = node.get_parameter("tree.chamber_on_approach").as_bool();
            config.chamber_at =
                    static_cast<float>(node.get_parameter("tree.chamber_at").as_double());
            config.chamber_scale =
                    static_cast<float>(node.get_parameter("tree.chamber_scale").as_double());
            return config;
        }

    }// namespace

    std::shared_ptr<CaveWorld::ICaveField> createCaveFieldFromParameters(const rclcpp::Node & node)
    {
        const std::string cave_mode = node.get_parameter("cave_mode").as_string();
        const auto        y_config  = loadYConfigFromParameters(node);
        const auto        tree_config = loadTreeConfigFromParameters(node);
        return CaveWorld::createCaveField(cave_mode, y_config, tree_config);
    }

}// namespace DroneScanner

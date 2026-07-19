#ifndef SWARM_CONTROLLER_FRONTIERCOMPONENTAUDITREPLAY_HPP
#define SWARM_CONTROLLER_FRONTIERCOMPONENTAUDITREPLAY_HPP

#include "swarm_controller/Point3f.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace SwarmController {

    struct ComponentAuditColumnKey {
        std::int64_t x {};
        std::int64_t y {};

        bool operator==(const ComponentAuditColumnKey & other) const
        {
            return x == other.x && y == other.y;
        }

        bool operator<(const ComponentAuditColumnKey & other) const
        {
            return x < other.x || (x == other.x && y < other.y);
        }
    };

    enum class ComponentAuditRejection {
        None,
        Columns,
        Area,
        Span,
        Direction,
    };

    struct ComponentAuditThresholds {
        double      resolution {0.1};
        std::size_t column_stride_voxels {2U};
        std::size_t min_columns {12U};
        float       min_area {0.48F};
        float       min_span {0.6F};
        float       min_direction_consistency {0.65F};
    };

    struct ComponentAuditComponent {
        std::size_t component_index {};
        ComponentAuditColumnKey stable_key {};
        std::size_t exact_column_count {};
        float area {};
        float horizontal_span {};
        Point3f representative {};
        Point3f unknown_direction {};
        float information_gain {};
        float direction_consistency {};
        std::array<std::size_t, 4U> direction_votes {};
        Point3f xy_minimum {};
        Point3f xy_maximum {};
        ComponentAuditRejection rejection {ComponentAuditRejection::None};
        std::vector<ComponentAuditColumnKey> columns;
    };

    struct ComponentAuditSnapshot {
        std::size_t frame_index {};
        std::uint64_t bag_timestamp_ns {};
        std::uint64_t map_stamp_ns {};
        std::vector<ComponentAuditComponent> components;
    };

    struct ComponentAuditDecision {
        std::size_t exact_column_count {};
        float area {};
        float horizontal_span {};
        float direction_consistency {};
        std::array<std::size_t, 4U> direction_votes {};
        Point3f xy_minimum {};
        Point3f xy_maximum {};
        ComponentAuditRejection rejection {ComponentAuditRejection::None};
    };

    struct ComponentAuditGapPair {
        std::size_t first_component_index {};
        std::size_t second_component_index {};
        ComponentAuditColumnKey first_column {};
        ComponentAuditColumnKey second_column {};
        std::size_t discrete_gap_columns {};
        ComponentAuditDecision merged_decision;
    };

    struct ComponentAuditRadiusGroup {
        std::vector<std::size_t> component_indices;
        ComponentAuditDecision merged_decision;
    };

    struct ComponentAuditAnalysis {
        static constexpr std::size_t REJECTION_CATEGORIES = 5U;

        std::size_t total_components {};
        std::size_t total_columns {};
        std::array<std::size_t, REJECTION_CATEGORIES> component_counts {};
        std::array<std::size_t, REJECTION_CATEGORIES> column_counts {};
        std::vector<ComponentAuditGapPair> one_column_gap_pairs;
        std::vector<ComponentAuditGapPair> beneficial_gap_pairs;
        std::vector<ComponentAuditRadiusGroup> radius_two_groups;
        std::size_t baseline_accepted_components {};
        std::size_t radius_two_accepted_groups {};
    };

    struct ComponentAuditColor {
        float r {};
        float g {};
        float b {};
        float a {1.0F};
    };

    struct ComponentAuditBox {
        std::string ns;
        std::int32_t id {};
        Point3f center {};
        Point3f scale {};
        ComponentAuditColor color {};
    };

    enum class ComponentAuditPointShape {
        Cube,
        Sphere,
    };

    struct ComponentAuditPointLayer {
        std::string ns;
        std::int32_t id {};
        ComponentAuditPointShape shape {ComponentAuditPointShape::Cube};
        Point3f scale {};
        ComponentAuditColor color {};
        std::vector<Point3f> points;
        std::vector<ComponentAuditColor> colors;
    };

    struct ComponentAuditLineLayer {
        std::string ns;
        std::int32_t id {};
        float width {0.02F};
        ComponentAuditColor color {};
        std::vector<Point3f> points;
    };

    struct ComponentAuditArrow {
        std::string ns;
        std::int32_t id {};
        Point3f start {};
        Point3f end {};
        float shaft_diameter {0.04F};
        float head_diameter {0.10F};
        float head_length {0.16F};
        ComponentAuditColor color {};
    };

    struct ComponentAuditText {
        std::string ns;
        std::int32_t id {};
        Point3f position {};
        std::string text;
        float height {0.24F};
        ComponentAuditColor color {};
    };

    struct ComponentAuditScene {
        std::vector<ComponentAuditBox> boxes;
        std::vector<ComponentAuditPointLayer> point_layers;
        std::vector<ComponentAuditLineLayer> line_layers;
        std::vector<ComponentAuditArrow> arrows;
        std::vector<ComponentAuditText> texts;
    };

    struct ComponentAuditStageScene {
        std::string stage;
        ComponentAuditScene scene;
    };

    struct FrontierComponentAuditReplayConfig {
        ComponentAuditThresholds thresholds;
        std::size_t expected_frame_index {3U};
        Point3f local_translation {};
        float column_footprint_height {0.08F};
        bool show_labels {true};
    };

    class FrontierComponentAuditReplay
    {
    public:
        explicit FrontierComponentAuditReplay(
                FrontierComponentAuditReplayConfig config = {});

        ComponentAuditSnapshot loadSnapshot(
                const std::filesystem::path & component_csv,
                const std::filesystem::path & membership_csv) const;
        ComponentAuditAnalysis analyze(
                const ComponentAuditSnapshot & snapshot) const;
        std::vector<ComponentAuditStageScene> buildStageScenes(
                const ComponentAuditSnapshot & snapshot) const;
        const FrontierComponentAuditReplayConfig & config() const;

    private:
        ComponentAuditDecision evaluate(
                const ComponentAuditSnapshot & snapshot,
                const std::vector<std::size_t> & component_indices) const;

        FrontierComponentAuditReplayConfig config_;
    };

    const char * componentAuditRejectionName(ComponentAuditRejection rejection);

}// namespace SwarmController

#endif// SWARM_CONTROLLER_FRONTIERCOMPONENTAUDITREPLAY_HPP

#include "swarm_controller/FrontierComponentAuditReplay.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <utility>

namespace SwarmController {

    namespace {

        constexpr std::string_view COMPONENT_HEADER =
                "frame_index,bag_timestamp_ns,map_stamp_ns,component_index,"
                "stable_key_x,stable_key_y,exact_columns,area,horizontal_span,"
                "representative_x,representative_y,representative_z,"
                "unknown_direction_x,unknown_direction_y,unknown_direction_z,"
                "information_gain,direction_consistency,direction_votes_pos_x,"
                "direction_votes_pos_y,direction_votes_neg_x,direction_votes_neg_y,"
                "xy_min_x,xy_min_y,xy_max_x,xy_max_y,rejection,columns_complete,"
                "edges_complete";
        constexpr std::string_view MEMBERSHIP_HEADER =
                "frame_index,component_index,stable_key_x,stable_key_y,column_x,column_y";
        constexpr std::size_t COMPONENT_FIELDS = 28U;
        constexpr std::size_t MEMBERSHIP_FIELDS = 6U;
        constexpr std::int64_t OCTOMAP_KEY_ORIGIN = 32'768;

        constexpr ComponentAuditColor COLOR_COLUMNS_REJECT {
                0.98F, 0.55F, 0.12F, 0.92F};
        constexpr ComponentAuditColor COLOR_DIRECTION_REJECT {
                0.91F, 0.20F, 0.31F, 0.92F};
        constexpr ComponentAuditColor COLOR_ACCEPTED {
                0.18F, 0.82F, 0.45F, 0.94F};
        constexpr ComponentAuditColor COLOR_CYAN {
                0.16F, 0.76F, 0.95F, 0.95F};
        constexpr ComponentAuditColor COLOR_BLUE {
                0.20F, 0.50F, 0.96F, 0.95F};
        constexpr ComponentAuditColor COLOR_PURPLE {
                0.67F, 0.38F, 0.90F, 0.95F};
        constexpr ComponentAuditColor COLOR_YELLOW {
                0.98F, 0.82F, 0.20F, 0.95F};
        constexpr ComponentAuditColor COLOR_WHITE {
                0.92F, 0.94F, 0.96F, 0.96F};
        constexpr ComponentAuditColor COLOR_MUTED {
                0.47F, 0.53F, 0.58F, 0.78F};
        constexpr ComponentAuditColor COLOR_DARK {
                0.12F, 0.15F, 0.18F, 0.86F};

        std::runtime_error csvError(
                const std::filesystem::path & path, const std::size_t line,
                const std::string & reason)
        {
            return std::runtime_error(
                    path.string() + ":" + std::to_string(line) + ": " + reason);
        }

        void stripCarriageReturn(std::string & line)
        {
            if(!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
        }

        std::vector<std::string> splitCsv(
                const std::filesystem::path & path, const std::size_t line_number,
                const std::string & line, const std::size_t expected_fields)
        {
            if(line.find('"') != std::string::npos) {
                throw csvError(
                        path, line_number,
                        "normalized analyzer CSV must not contain quoted fields");
            }
            std::vector<std::string> fields;
            std::size_t start = 0U;
            while(true) {
                const std::size_t comma = line.find(',', start);
                fields.push_back(line.substr(start, comma - start));
                if(comma == std::string::npos) {
                    break;
                }
                start = comma + 1U;
            }
            if(fields.size() != expected_fields) {
                throw csvError(
                        path, line_number,
                        "expected " + std::to_string(expected_fields) + " fields, got "
                                + std::to_string(fields.size()));
            }
            return fields;
        }

        template<typename Integer>
        Integer parseInteger(
                const std::filesystem::path & path, const std::size_t line,
                const std::string & field, const char * name)
        {
            Integer value {};
            const char * begin = field.data();
            const char * end = begin + field.size();
            const auto parsed = std::from_chars(begin, end, value);
            if(parsed.ec != std::errc {} || parsed.ptr != end) {
                throw csvError(path, line, std::string("invalid ") + name);
            }
            return value;
        }

        std::size_t parseSize(
                const std::filesystem::path & path, const std::size_t line,
                const std::string & field, const char * name)
        {
            const std::uint64_t value =
                    parseInteger<std::uint64_t>(path, line, field, name);
            if(value > std::numeric_limits<std::size_t>::max()) {
                throw csvError(path, line, std::string(name) + " exceeds size_t");
            }
            return static_cast<std::size_t>(value);
        }

        float parseFloat(
                const std::filesystem::path & path, const std::size_t line,
                const std::string & field, const char * name)
        {
            std::istringstream input(field);
            input.imbue(std::locale::classic());
            double value {};
            input >> value;
            if(!input || input.peek() != std::char_traits<char>::eof()
               || !std::isfinite(value)
               || value < -std::numeric_limits<float>::max()
               || value > std::numeric_limits<float>::max())
            {
                throw csvError(path, line, std::string("invalid ") + name);
            }
            return static_cast<float>(value);
        }

        ComponentAuditRejection parseRejection(
                const std::filesystem::path & path, const std::size_t line,
                const std::string & value)
        {
            if(value == "None") {
                return ComponentAuditRejection::None;
            }
            if(value == "Columns") {
                return ComponentAuditRejection::Columns;
            }
            if(value == "Area") {
                return ComponentAuditRejection::Area;
            }
            if(value == "Span") {
                return ComponentAuditRejection::Span;
            }
            if(value == "Direction") {
                return ComponentAuditRejection::Direction;
            }
            throw csvError(path, line, "invalid component rejection");
        }

        std::size_t rejectionIndex(const ComponentAuditRejection rejection)
        {
            return static_cast<std::size_t>(rejection);
        }

        ComponentAuditRejection classify(
                const std::size_t columns, const float area, const float span,
                const float direction_consistency,
                const ComponentAuditThresholds & thresholds)
        {
            if(columns < thresholds.min_columns) {
                return ComponentAuditRejection::Columns;
            }
            if(area < thresholds.min_area) {
                return ComponentAuditRejection::Area;
            }
            if(span < thresholds.min_span) {
                return ComponentAuditRejection::Span;
            }
            if(direction_consistency < thresholds.min_direction_consistency) {
                return ComponentAuditRejection::Direction;
            }
            return ComponentAuditRejection::None;
        }

        bool near(const float first, const float second, const float tolerance)
        {
            return std::fabs(first - second) <= tolerance;
        }

        bool finitePoint(const Point3f & point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y)
                   && std::isfinite(point.z);
        }

        void checkedAdd(std::size_t & target, const std::size_t value)
        {
            if(value > std::numeric_limits<std::size_t>::max() - target) {
                throw std::overflow_error("component audit count overflow");
            }
            target += value;
        }

        ComponentAuditColor rejectionColor(const ComponentAuditRejection rejection)
        {
            switch(rejection) {
                case ComponentAuditRejection::None:
                    return COLOR_ACCEPTED;
                case ComponentAuditRejection::Columns:
                    return COLOR_COLUMNS_REJECT;
                case ComponentAuditRejection::Direction:
                    return COLOR_DIRECTION_REJECT;
                case ComponentAuditRejection::Area:
                    return COLOR_PURPLE;
                case ComponentAuditRejection::Span:
                    return COLOR_BLUE;
            }
            return COLOR_WHITE;
        }

        std::string decimal(const float value, const int precision = 3)
        {
            std::ostringstream output;
            output.imbue(std::locale::classic());
            output << std::fixed << std::setprecision(precision) << value;
            return output.str();
        }

        std::string percent(const std::size_t part, const std::size_t total)
        {
            const float value = total == 0U
                                        ? 0.0F
                                        : 100.0F * static_cast<float>(part)
                                                  / static_cast<float>(total);
            return decimal(value, 1) + "%";
        }

        Point3f translated(
                const Point3f & point,
                const FrontierComponentAuditReplayConfig & config,
                const float z = 0.0F)
        {
            return Point3f {
                    point.x + config.local_translation.x,
                    point.y + config.local_translation.y,
                    z + config.local_translation.z};
        }

        Point3f columnPoint(
                const ComponentAuditColumnKey & key,
                const FrontierComponentAuditReplayConfig & config,
                const float z = 0.0F)
        {
            const double stride =
                    static_cast<double>(config.thresholds.column_stride_voxels);
            const double x =
                    (static_cast<double>(key.x) * stride + stride * 0.5
                     - static_cast<double>(OCTOMAP_KEY_ORIGIN))
                    * config.thresholds.resolution;
            const double y =
                    (static_cast<double>(key.y) * stride + stride * 0.5
                     - static_cast<double>(OCTOMAP_KEY_ORIGIN))
                    * config.thresholds.resolution;
            return translated(
                    Point3f {static_cast<float>(x), static_cast<float>(y), z},
                    config, z);
        }

        const ComponentAuditComponent & componentAt(
                const ComponentAuditSnapshot & snapshot, const std::size_t index)
        {
            if(index >= snapshot.components.size()
               || snapshot.components[index].component_index != index)
            {
                throw std::logic_error("component audit index is not contiguous");
            }
            return snapshot.components[index];
        }

        ComponentAuditPointLayer componentLayer(
                const std::string & ns, const std::int32_t id,
                const ComponentAuditComponent & component,
                const ComponentAuditColor color,
                const FrontierComponentAuditReplayConfig & config)
        {
            ComponentAuditPointLayer layer;
            layer.ns = ns;
            layer.id = id;
            layer.shape = ComponentAuditPointShape::Cube;
            const float footprint = static_cast<float>(
                    config.thresholds.resolution
                    * static_cast<double>(config.thresholds.column_stride_voxels));
            layer.scale = Point3f {
                    footprint * 0.82F, footprint * 0.82F,
                    config.column_footprint_height};
            layer.color = color;
            layer.points.reserve(component.columns.size());
            for(const ComponentAuditColumnKey & column : component.columns) {
                layer.points.push_back(columnPoint(column, config));
            }
            return layer;
        }

        ComponentAuditPointLayer representativeLayer(
                const std::string & ns, const std::int32_t id,
                const ComponentAuditComponent & component,
                const ComponentAuditColor color,
                const FrontierComponentAuditReplayConfig & config)
        {
            ComponentAuditPointLayer layer;
            layer.ns = ns;
            layer.id = id;
            layer.shape = ComponentAuditPointShape::Sphere;
            layer.scale = Point3f {0.28F, 0.28F, 0.28F};
            layer.color = color;
            layer.points.push_back(translated(component.representative, config, 0.24F));
            return layer;
        }

        ComponentAuditLineLayer boundsLayer(
                const std::string & ns, const std::int32_t id,
                const Point3f & minimum, const Point3f & maximum,
                const ComponentAuditColor color,
                const FrontierComponentAuditReplayConfig & config,
                const float z = 0.12F)
        {
            const Point3f a = translated(
                    Point3f {minimum.x, minimum.y, 0.0F}, config, z);
            const Point3f b = translated(
                    Point3f {maximum.x, minimum.y, 0.0F}, config, z);
            const Point3f c = translated(
                    Point3f {maximum.x, maximum.y, 0.0F}, config, z);
            const Point3f d = translated(
                    Point3f {minimum.x, maximum.y, 0.0F}, config, z);
            ComponentAuditLineLayer layer;
            layer.ns = ns;
            layer.id = id;
            layer.width = 0.035F;
            layer.color = color;
            layer.points = {a, b, b, c, c, d, d, a};
            return layer;
        }

        ComponentAuditLineLayer dashedLine(
                const std::string & ns, const std::int32_t id,
                const Point3f & start, const Point3f & end,
                const ComponentAuditColor color, const std::size_t dashes = 6U)
        {
            ComponentAuditLineLayer layer;
            layer.ns = ns;
            layer.id = id;
            layer.width = 0.045F;
            layer.color = color;
            layer.points.reserve(dashes * 2U);
            for(std::size_t index = 0U; index < dashes; ++index) {
                const float first = static_cast<float>(index) /
                                    static_cast<float>(dashes);
                const float second = (static_cast<float>(index) + 0.58F) /
                                     static_cast<float>(dashes);
                auto interpolate = [&](const float ratio) {
                    return Point3f {
                            start.x + (end.x - start.x) * ratio,
                            start.y + (end.y - start.y) * ratio,
                            start.z + (end.z - start.z) * ratio};
                };
                layer.points.push_back(interpolate(first));
                layer.points.push_back(interpolate(second));
            }
            return layer;
        }

        void appendText(
                ComponentAuditScene & scene, const std::string & ns,
                const std::int32_t id, const Point3f & position,
                std::string text, const float height,
                const ComponentAuditColor color = COLOR_WHITE)
        {
            scene.texts.push_back(ComponentAuditText {
                    ns, id, position, std::move(text), height, color});
        }

        class DisjointSet
        {
        public:
            explicit DisjointSet(const std::size_t size)
                : parent_(size), rank_(size, 0U)
            {
                std::iota(parent_.begin(), parent_.end(), 0U);
            }

            std::size_t find(const std::size_t value)
            {
                if(parent_[value] != value) {
                    parent_[value] = find(parent_[value]);
                }
                return parent_[value];
            }

            void join(const std::size_t first, const std::size_t second)
            {
                std::size_t first_root = find(first);
                std::size_t second_root = find(second);
                if(first_root == second_root) {
                    return;
                }
                if(rank_[first_root] < rank_[second_root]) {
                    std::swap(first_root, second_root);
                }
                parent_[second_root] = first_root;
                if(rank_[first_root] == rank_[second_root]) {
                    ++rank_[first_root];
                }
            }

        private:
            std::vector<std::size_t> parent_;
            std::vector<std::size_t> rank_;
        };

    }// namespace

    const char * componentAuditRejectionName(
            const ComponentAuditRejection rejection)
    {
        switch(rejection) {
            case ComponentAuditRejection::None:
                return "None";
            case ComponentAuditRejection::Columns:
                return "Columns";
            case ComponentAuditRejection::Area:
                return "Area";
            case ComponentAuditRejection::Span:
                return "Span";
            case ComponentAuditRejection::Direction:
                return "Direction";
        }
        return "Unknown";
    }

    FrontierComponentAuditReplay::FrontierComponentAuditReplay(
            FrontierComponentAuditReplayConfig config)
        : config_(std::move(config))
    {
        const auto & thresholds = config_.thresholds;
        if(!std::isfinite(thresholds.resolution) || thresholds.resolution <= 0.0
           || thresholds.column_stride_voxels == 0U
           || thresholds.min_columns == 0U || !std::isfinite(thresholds.min_area)
           || thresholds.min_area <= 0.0F || !std::isfinite(thresholds.min_span)
           || thresholds.min_span <= 0.0F
           || !std::isfinite(thresholds.min_direction_consistency)
           || thresholds.min_direction_consistency <= 0.0F
           || thresholds.min_direction_consistency > 1.0F
           || !finitePoint(config_.local_translation)
           || !std::isfinite(config_.column_footprint_height)
           || config_.column_footprint_height <= 0.0F)
        {
            throw std::invalid_argument(
                    "invalid frontier component audit replay configuration");
        }
    }

    ComponentAuditSnapshot FrontierComponentAuditReplay::loadSnapshot(
            const std::filesystem::path & component_csv,
            const std::filesystem::path & membership_csv) const
    {
        ComponentAuditSnapshot snapshot;
        snapshot.frame_index = config_.expected_frame_index;

        std::ifstream components(component_csv, std::ios::binary);
        if(!components) {
            throw std::runtime_error(
                    "failed to open component audit CSV: " + component_csv.string());
        }
        std::string line;
        if(!std::getline(components, line)) {
            throw csvError(component_csv, 1U, "missing header");
        }
        stripCarriageReturn(line);
        if(line != COMPONENT_HEADER) {
            throw csvError(component_csv, 1U, "component schema mismatch");
        }

        std::size_t line_number = 1U;
        bool metadata_initialized = false;
        while(std::getline(components, line)) {
            ++line_number;
            stripCarriageReturn(line);
            if(line.empty()) {
                throw csvError(component_csv, line_number, "empty row");
            }
            const auto fields = splitCsv(
                    component_csv, line_number, line, COMPONENT_FIELDS);
            const std::size_t frame = parseSize(
                    component_csv, line_number, fields[0], "frame_index");
            if(frame != config_.expected_frame_index) {
                throw csvError(
                        component_csv, line_number,
                        "row does not match expected frame_index");
            }
            const std::uint64_t bag_timestamp = parseInteger<std::uint64_t>(
                    component_csv, line_number, fields[1], "bag_timestamp_ns");
            const std::uint64_t map_stamp = parseInteger<std::uint64_t>(
                    component_csv, line_number, fields[2], "map_stamp_ns");
            if(!metadata_initialized) {
                snapshot.bag_timestamp_ns = bag_timestamp;
                snapshot.map_stamp_ns = map_stamp;
                metadata_initialized = true;
            } else if(snapshot.bag_timestamp_ns != bag_timestamp
                      || snapshot.map_stamp_ns != map_stamp)
            {
                throw csvError(
                        component_csv, line_number,
                        "timestamps differ within one audit frame");
            }

            ComponentAuditComponent component;
            component.component_index = parseSize(
                    component_csv, line_number, fields[3], "component_index");
            if(component.component_index != snapshot.components.size()) {
                throw csvError(
                        component_csv, line_number,
                        "component_index must be contiguous and sorted");
            }
            component.stable_key = ComponentAuditColumnKey {
                    parseInteger<std::int64_t>(
                            component_csv, line_number, fields[4], "stable_key_x"),
                    parseInteger<std::int64_t>(
                            component_csv, line_number, fields[5], "stable_key_y")};
            component.exact_column_count = parseSize(
                    component_csv, line_number, fields[6], "exact_columns");
            component.area = parseFloat(
                    component_csv, line_number, fields[7], "area");
            component.horizontal_span = parseFloat(
                    component_csv, line_number, fields[8], "horizontal_span");
            component.representative = Point3f {
                    parseFloat(
                            component_csv, line_number, fields[9],
                            "representative_x"),
                    parseFloat(
                            component_csv, line_number, fields[10],
                            "representative_y"),
                    parseFloat(
                            component_csv, line_number, fields[11],
                            "representative_z")};
            component.unknown_direction = Point3f {
                    parseFloat(
                            component_csv, line_number, fields[12],
                            "unknown_direction_x"),
                    parseFloat(
                            component_csv, line_number, fields[13],
                            "unknown_direction_y"),
                    parseFloat(
                            component_csv, line_number, fields[14],
                            "unknown_direction_z")};
            component.information_gain = parseFloat(
                    component_csv, line_number, fields[15], "information_gain");
            component.direction_consistency = parseFloat(
                    component_csv, line_number, fields[16],
                    "direction_consistency");
            for(std::size_t index = 0U; index < component.direction_votes.size();
                ++index)
            {
                component.direction_votes[index] = parseSize(
                        component_csv, line_number, fields[17U + index],
                        "direction_vote");
            }
            component.xy_minimum = Point3f {
                    parseFloat(component_csv, line_number, fields[21], "xy_min_x"),
                    parseFloat(component_csv, line_number, fields[22], "xy_min_y"),
                    0.0F};
            component.xy_maximum = Point3f {
                    parseFloat(component_csv, line_number, fields[23], "xy_max_x"),
                    parseFloat(component_csv, line_number, fields[24], "xy_max_y"),
                    0.0F};
            component.rejection = parseRejection(
                    component_csv, line_number, fields[25]);
            if(fields[26] != "1" || fields[27] != "1") {
                throw csvError(
                        component_csv, line_number,
                        "fixture requires complete component columns and edges");
            }
            if(component.exact_column_count == 0U || component.area <= 0.0F
               || component.horizontal_span < 0.0F
               || component.direction_consistency < 0.0F
               || component.direction_consistency > 1.0F
               || !finitePoint(component.representative)
               || !finitePoint(component.unknown_direction)
               || component.xy_minimum.x > component.xy_maximum.x
               || component.xy_minimum.y > component.xy_maximum.y)
            {
                throw csvError(component_csv, line_number, "invalid component metrics");
            }

            const float column_size = static_cast<float>(
                    config_.thresholds.resolution
                    * static_cast<double>(
                            config_.thresholds.column_stride_voxels));
            const float expected_area =
                    static_cast<float>(component.exact_column_count)
                    * column_size * column_size;
            const float expected_span = std::hypot(
                    component.xy_maximum.x - component.xy_minimum.x,
                    component.xy_maximum.y - component.xy_minimum.y);
            const std::size_t vote_total = std::accumulate(
                    component.direction_votes.begin(),
                    component.direction_votes.end(), std::size_t {0U});
            const float expected_consistency = vote_total == 0U
                                                       ? 0.0F
                                                       : static_cast<float>(*std::max_element(
                                                                 component.direction_votes.begin(),
                                                                 component.direction_votes.end()))
                                                                 / static_cast<float>(vote_total);
            if(vote_total == 0U
               || !near(component.area, expected_area, 1.0e-4F)
               || !near(component.horizontal_span, expected_span, 1.0e-4F)
               || !near(
                       component.information_gain,
                       static_cast<float>(vote_total), 1.0e-3F)
               || !near(
                       component.direction_consistency,
                       expected_consistency, 1.0e-5F)
               || component.rejection
                          != classify(
                                  component.exact_column_count, component.area,
                                  component.horizontal_span,
                                  component.direction_consistency,
                                  config_.thresholds))
            {
                throw csvError(
                        component_csv, line_number,
                        "component metrics violate production audit invariants");
            }
            snapshot.components.push_back(std::move(component));
        }
        if(!components.eof()) {
            throw std::runtime_error(
                    "failed while reading component audit CSV: "
                    + component_csv.string());
        }
        if(snapshot.components.empty()) {
            throw csvError(component_csv, 2U, "fixture contains no components");
        }

        std::ifstream membership(membership_csv, std::ios::binary);
        if(!membership) {
            throw std::runtime_error(
                    "failed to open component membership CSV: "
                    + membership_csv.string());
        }
        if(!std::getline(membership, line)) {
            throw csvError(membership_csv, 1U, "missing header");
        }
        stripCarriageReturn(line);
        if(line != MEMBERSHIP_HEADER) {
            throw csvError(membership_csv, 1U, "membership schema mismatch");
        }

        line_number = 1U;
        std::optional<std::tuple<std::size_t, std::int64_t, std::int64_t>>
                previous_row;
        std::set<ComponentAuditColumnKey> global_columns;
        while(std::getline(membership, line)) {
            ++line_number;
            stripCarriageReturn(line);
            if(line.empty()) {
                throw csvError(membership_csv, line_number, "empty row");
            }
            const auto fields = splitCsv(
                    membership_csv, line_number, line, MEMBERSHIP_FIELDS);
            const std::size_t frame = parseSize(
                    membership_csv, line_number, fields[0], "frame_index");
            if(frame != config_.expected_frame_index) {
                throw csvError(
                        membership_csv, line_number,
                        "row does not match expected frame_index");
            }
            const std::size_t component_index = parseSize(
                    membership_csv, line_number, fields[1], "component_index");
            if(component_index >= snapshot.components.size()) {
                throw csvError(
                        membership_csv, line_number, "unknown component_index");
            }
            const ComponentAuditColumnKey stable_key {
                    parseInteger<std::int64_t>(
                            membership_csv, line_number, fields[2], "stable_key_x"),
                    parseInteger<std::int64_t>(
                            membership_csv, line_number, fields[3], "stable_key_y")};
            ComponentAuditComponent & component =
                    snapshot.components[component_index];
            if(!(stable_key == component.stable_key)) {
                throw csvError(
                        membership_csv, line_number, "stable key mismatch");
            }
            const ComponentAuditColumnKey column {
                    parseInteger<std::int64_t>(
                            membership_csv, line_number, fields[4], "column_x"),
                    parseInteger<std::int64_t>(
                            membership_csv, line_number, fields[5], "column_y")};
            const auto row_key =
                    std::make_tuple(component_index, column.x, column.y);
            if(previous_row.has_value() && row_key <= *previous_row) {
                throw csvError(
                        membership_csv, line_number,
                        "membership rows must be strictly sorted");
            }
            previous_row = row_key;
            if(!global_columns.insert(column).second) {
                throw csvError(
                        membership_csv, line_number,
                        "column appears in more than one component");
            }
            component.columns.push_back(column);
        }
        if(!membership.eof()) {
            throw std::runtime_error(
                    "failed while reading component membership CSV: "
                    + membership_csv.string());
        }

        std::size_t membership_count = 0U;
        for(const ComponentAuditComponent & component : snapshot.components) {
            if(component.columns.size() != component.exact_column_count) {
                throw std::runtime_error(
                        "component " + std::to_string(component.component_index)
                        + " membership count differs from exact_columns");
            }
            if(component.columns.empty()
               || !(component.columns.front() == component.stable_key))
            {
                throw std::runtime_error(
                        "component " + std::to_string(component.component_index)
                        + " stable key differs from first sorted member");
            }
            checkedAdd(membership_count, component.columns.size());
        }
        if(membership_count != global_columns.size()) {
            throw std::logic_error("component membership conservation failed");
        }
        return snapshot;
    }

    ComponentAuditDecision FrontierComponentAuditReplay::evaluate(
            const ComponentAuditSnapshot & snapshot,
            const std::vector<std::size_t> & component_indices) const
    {
        if(component_indices.empty()) {
            throw std::invalid_argument("cannot evaluate an empty component group");
        }
        ComponentAuditDecision decision;
        decision.xy_minimum = Point3f {
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(), 0.0F};
        decision.xy_maximum = Point3f {
                std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest(), 0.0F};
        std::set<std::size_t> unique_indices;
        for(const std::size_t index : component_indices) {
            if(!unique_indices.insert(index).second) {
                throw std::invalid_argument(
                        "component group contains a duplicate index");
            }
            const ComponentAuditComponent & component = componentAt(snapshot, index);
            checkedAdd(decision.exact_column_count, component.exact_column_count);
            for(std::size_t direction = 0U;
                direction < decision.direction_votes.size(); ++direction)
            {
                checkedAdd(
                        decision.direction_votes[direction],
                        component.direction_votes[direction]);
            }
            decision.xy_minimum.x =
                    std::min(decision.xy_minimum.x, component.xy_minimum.x);
            decision.xy_minimum.y =
                    std::min(decision.xy_minimum.y, component.xy_minimum.y);
            decision.xy_maximum.x =
                    std::max(decision.xy_maximum.x, component.xy_maximum.x);
            decision.xy_maximum.y =
                    std::max(decision.xy_maximum.y, component.xy_maximum.y);
        }
        const float column_size = static_cast<float>(
                config_.thresholds.resolution
                * static_cast<double>(config_.thresholds.column_stride_voxels));
        decision.area = static_cast<float>(decision.exact_column_count)
                        * column_size * column_size;
        decision.horizontal_span = std::hypot(
                decision.xy_maximum.x - decision.xy_minimum.x,
                decision.xy_maximum.y - decision.xy_minimum.y);
        const std::size_t total_votes = std::accumulate(
                decision.direction_votes.begin(), decision.direction_votes.end(),
                std::size_t {0U});
        decision.direction_consistency = total_votes == 0U
                                                 ? 0.0F
                                                 : static_cast<float>(*std::max_element(
                                                           decision.direction_votes.begin(),
                                                           decision.direction_votes.end()))
                                                           / static_cast<float>(total_votes);
        decision.rejection = classify(
                decision.exact_column_count, decision.area,
                decision.horizontal_span, decision.direction_consistency,
                config_.thresholds);
        return decision;
    }

    ComponentAuditAnalysis FrontierComponentAuditReplay::analyze(
            const ComponentAuditSnapshot & snapshot) const
    {
        if(snapshot.frame_index != config_.expected_frame_index
           || snapshot.components.empty())
        {
            throw std::invalid_argument("audit snapshot does not match replay config");
        }
        ComponentAuditAnalysis analysis;
        analysis.total_components = snapshot.components.size();

        std::map<ComponentAuditColumnKey, std::size_t> owner;
        for(const ComponentAuditComponent & component : snapshot.components) {
            const std::size_t category = rejectionIndex(component.rejection);
            ++analysis.component_counts[category];
            checkedAdd(
                    analysis.column_counts[category],
                    component.exact_column_count);
            checkedAdd(analysis.total_columns, component.exact_column_count);
            if(component.rejection == ComponentAuditRejection::None) {
                ++analysis.baseline_accepted_components;
            }
            for(const ComponentAuditColumnKey & column : component.columns) {
                if(!owner.emplace(column, component.component_index).second) {
                    throw std::invalid_argument(
                            "audit snapshot contains duplicate membership columns");
                }
            }
        }

        using ComponentPair = std::pair<std::size_t, std::size_t>;
        using BoundaryPair =
                std::pair<ComponentAuditColumnKey, ComponentAuditColumnKey>;
        std::map<ComponentPair, BoundaryPair> gap_boundaries;
        for(const auto & [column, component_index] : owner) {
            for(std::int64_t dx = -2; dx <= 2; ++dx) {
                for(std::int64_t dy = -2; dy <= 2; ++dy) {
                    const std::int64_t distance =
                            std::max(std::llabs(dx), std::llabs(dy));
                    if(distance == 0) {
                        continue;
                    }
                    const ComponentAuditColumnKey neighbor {
                            column.x + dx, column.y + dy};
                    const auto found = owner.find(neighbor);
                    if(found == owner.end() || found->second == component_index) {
                        continue;
                    }
                    if(distance == 1) {
                        throw std::invalid_argument(
                                "distinct audit components contain XY-adjacent columns");
                    }
                    if(distance != 2) {
                        continue;
                    }
                    const std::size_t first_index =
                            std::min(component_index, found->second);
                    const std::size_t second_index =
                            std::max(component_index, found->second);
                    BoundaryPair boundary = component_index == first_index
                                                    ? BoundaryPair {column, neighbor}
                                                    : BoundaryPair {neighbor, column};
                    const ComponentPair pair {first_index, second_index};
                    const auto existing = gap_boundaries.find(pair);
                    if(existing == gap_boundaries.end()
                       || std::tie(
                                  boundary.first.x, boundary.first.y,
                                  boundary.second.x, boundary.second.y)
                                  < std::tie(
                                          existing->second.first.x,
                                          existing->second.first.y,
                                          existing->second.second.x,
                                          existing->second.second.y))
                    {
                        gap_boundaries[pair] = boundary;
                    }
                }
            }
        }

        analysis.one_column_gap_pairs.reserve(gap_boundaries.size());
        for(const auto & [pair, boundary] : gap_boundaries) {
            ComponentAuditGapPair gap;
            gap.first_component_index = pair.first;
            gap.second_component_index = pair.second;
            gap.first_column = boundary.first;
            gap.second_column = boundary.second;
            gap.discrete_gap_columns = 1U;
            gap.merged_decision = evaluate(
                    snapshot, {pair.first, pair.second});
            analysis.one_column_gap_pairs.push_back(gap);
            const auto & first = componentAt(snapshot, pair.first);
            const auto & second = componentAt(snapshot, pair.second);
            if(first.rejection != ComponentAuditRejection::None
               && second.rejection != ComponentAuditRejection::None
               && gap.merged_decision.rejection == ComponentAuditRejection::None)
            {
                analysis.beneficial_gap_pairs.push_back(gap);
            }
        }

        DisjointSet groups(snapshot.components.size());
        for(const ComponentAuditGapPair & pair : analysis.one_column_gap_pairs) {
            groups.join(
                    pair.first_component_index, pair.second_component_index);
        }
        std::map<std::size_t, std::vector<std::size_t>> grouped_indices;
        for(std::size_t index = 0U; index < snapshot.components.size(); ++index) {
            grouped_indices[groups.find(index)].push_back(index);
        }
        analysis.radius_two_groups.reserve(grouped_indices.size());
        for(auto & [root, indices] : grouped_indices) {
            (void) root;
            ComponentAuditRadiusGroup group;
            group.component_indices = std::move(indices);
            group.merged_decision = evaluate(snapshot, group.component_indices);
            if(group.merged_decision.rejection == ComponentAuditRejection::None) {
                ++analysis.radius_two_accepted_groups;
            }
            analysis.radius_two_groups.push_back(std::move(group));
        }
        std::sort(
                analysis.radius_two_groups.begin(),
                analysis.radius_two_groups.end(), [](const auto & lhs, const auto & rhs) {
                    return lhs.component_indices.front()
                           < rhs.component_indices.front();
                });
        return analysis;
    }

    std::vector<ComponentAuditStageScene>
    FrontierComponentAuditReplay::buildStageScenes(
            const ComponentAuditSnapshot & snapshot) const
    {
        const ComponentAuditAnalysis analysis = analyze(snapshot);
        std::vector<ComponentAuditStageScene> stages;
        stages.reserve(4U);
        auto centeredConfig = [&](const Point3f & raw_center) {
            FrontierComponentAuditReplayConfig result = config_;
            result.local_translation.x = 3.0F - raw_center.x;
            result.local_translation.y = 1.0F - raw_center.y;
            return result;
        };

        ComponentAuditScene overview;
        if(config_.show_labels) {
            appendText(
                    overview, "audit_stage_title", 0, Point3f {3.0F, 1.0F, 3.55F},
                    "STAGE 5 | AUDIT OVERVIEW", 0.25F);
            appendText(
                    overview, "audit_summary", 0, Point3f {3.0F, 1.0F, 3.10F},
                    "FRAME " + std::to_string(snapshot.frame_index) + " | "
                            + std::to_string(analysis.total_components)
                            + " COMPONENTS | "
                            + std::to_string(analysis.total_columns)
                            + " XY COLUMNS",
                    0.18F, COLOR_CYAN);
        }
        constexpr float bar_start = 0.5F;
        constexpr float bar_width = 5.0F;
        const std::array<ComponentAuditRejection, 3U> chart_categories {
                ComponentAuditRejection::Columns,
                ComponentAuditRejection::Direction,
                ComponentAuditRejection::None};
        float cursor = bar_start;
        for(std::size_t id = 0U; id < chart_categories.size(); ++id) {
            const ComponentAuditRejection category = chart_categories[id];
            const std::size_t count =
                    analysis.component_counts[rejectionIndex(category)];
            const float width = bar_width * static_cast<float>(count)
                                / static_cast<float>(analysis.total_components);
            if(width > 0.0F) {
                overview.boxes.push_back(ComponentAuditBox {
                        "audit_component_count_bar", static_cast<std::int32_t>(id),
                        Point3f {cursor + width * 0.5F, 1.0F, 2.35F},
                        Point3f {width, 0.14F, 0.28F}, rejectionColor(category)});
                cursor += width;
            }
        }
        cursor = bar_start;
        for(std::size_t id = 0U; id < chart_categories.size(); ++id) {
            const ComponentAuditRejection category = chart_categories[id];
            const std::size_t count =
                    analysis.column_counts[rejectionIndex(category)];
            const float width = bar_width * static_cast<float>(count)
                                / static_cast<float>(analysis.total_columns);
            if(width > 0.0F) {
                overview.boxes.push_back(ComponentAuditBox {
                        "audit_column_mass_bar", static_cast<std::int32_t>(id),
                        Point3f {cursor + width * 0.5F, 1.0F, 1.65F},
                        Point3f {width, 0.14F, 0.28F}, rejectionColor(category)});
                cursor += width;
            }
        }
        if(config_.show_labels) {
            appendText(
                    overview, "audit_bar_label", 0, Point3f {-0.45F, 1.0F, 2.35F},
                    "COMPONENTS", 0.17F, COLOR_MUTED);
            appendText(
                    overview, "audit_bar_label", 1, Point3f {-0.45F, 1.0F, 1.65F},
                    "COLUMNS", 0.17F, COLOR_MUTED);
            const std::size_t column_reject_index =
                    rejectionIndex(ComponentAuditRejection::Columns);
            const std::size_t direction_reject_index =
                    rejectionIndex(ComponentAuditRejection::Direction);
            const std::size_t accepted_index =
                    rejectionIndex(ComponentAuditRejection::None);
            appendText(
                    overview, "audit_legend", 0, Point3f {3.0F, 1.0F, 1.05F},
                    "MIN_COLUMNS  "
                            + std::to_string(
                                    analysis.component_counts[column_reject_index])
                            + " C / "
                            + std::to_string(
                                    analysis.column_counts[column_reject_index])
                            + " COL ["
                            + percent(
                                    analysis.column_counts[column_reject_index],
                                    analysis.total_columns)
                            + "]",
                    0.17F, COLOR_COLUMNS_REJECT);
            appendText(
                    overview, "audit_legend", 1, Point3f {3.0F, 1.0F, 0.68F},
                    "DIRECTION  "
                            + std::to_string(
                                    analysis.component_counts[direction_reject_index])
                            + " C / "
                            + std::to_string(
                                    analysis.column_counts[direction_reject_index])
                            + " columns  ["
                            + percent(
                                    analysis.column_counts[direction_reject_index],
                                    analysis.total_columns)
                            + "]",
                    0.17F, COLOR_DIRECTION_REJECT);
            appendText(
                    overview, "audit_legend", 2, Point3f {3.0F, 1.0F, 0.31F},
                    "ACCEPTED  "
                            + std::to_string(
                                    analysis.component_counts[accepted_index])
                            + " C / "
                            + std::to_string(
                                    analysis.column_counts[accepted_index])
                            + " columns",
                    0.17F, COLOR_ACCEPTED);
            appendText(
                    overview, "audit_thresholds", 0, Point3f {3.0F, 1.0F, -0.22F},
                    "THRESHOLDS: 12 C | 0.48 m2 | 0.60 m | 0.65",
                    0.17F, COLOR_WHITE);
            appendText(
                    overview, "audit_source", 0, Point3f {3.0F, 1.0F, -0.62F},
                    "SOURCE: FRAME 3 | OFFLINE SNAPSHOT",
                    0.16F, COLOR_MUTED);
        }
        stages.push_back(ComponentAuditStageScene {
                "audit_overview", std::move(overview)});

        if(analysis.beneficial_gap_pairs.empty()) {
            throw std::logic_error(
                    "audit fixture has no rejected pair that passes pair-only merge");
        }
        const auto accepted_it = std::find_if(
                snapshot.components.begin(), snapshot.components.end(),
                [](const ComponentAuditComponent & component) {
                    return component.rejection == ComponentAuditRejection::None;
                });
        if(accepted_it == snapshot.components.end()) {
            throw std::logic_error("audit fixture has no accepted component");
        }
        const ComponentAuditGapPair * closest_pair = nullptr;
        float closest_distance = std::numeric_limits<float>::max();
        for(const ComponentAuditGapPair & pair : analysis.beneficial_gap_pairs) {
            const auto & first =
                    componentAt(snapshot, pair.first_component_index);
            const auto & second =
                    componentAt(snapshot, pair.second_component_index);
            const float midpoint_x =
                    (first.representative.x + second.representative.x) * 0.5F;
            const float midpoint_y =
                    (first.representative.y + second.representative.y) * 0.5F;
            const float distance = std::hypot(
                    midpoint_x - accepted_it->representative.x,
                    midpoint_y - accepted_it->representative.y);
            if(closest_pair == nullptr || distance < closest_distance) {
                closest_pair = &pair;
                closest_distance = distance;
            }
        }

        ComponentAuditScene rejection;
        const auto & rejected_first =
                componentAt(snapshot, closest_pair->first_component_index);
        const auto & rejected_second =
                componentAt(snapshot, closest_pair->second_component_index);
        const FrontierComponentAuditReplayConfig rejection_config =
                centeredConfig(Point3f {
                        (accepted_it->representative.x
                         + rejected_first.representative.x
                         + rejected_second.representative.x)
                                / 3.0F,
                        (accepted_it->representative.y
                         + rejected_first.representative.y
                         + rejected_second.representative.y)
                                / 3.0F,
                        0.0F});
        rejection.point_layers.push_back(componentLayer(
                "audit_component_accepted", 0, *accepted_it, COLOR_ACCEPTED,
                rejection_config));
        rejection.point_layers.push_back(representativeLayer(
                "audit_component_representative", 0, *accepted_it,
                COLOR_ACCEPTED, rejection_config));
        rejection.line_layers.push_back(boundsLayer(
                "audit_component_bounds", 0, accepted_it->xy_minimum,
                accepted_it->xy_maximum, COLOR_ACCEPTED, rejection_config));
        const std::array<const ComponentAuditComponent *, 2U> rejected_components {
                &rejected_first, &rejected_second};
        for(std::size_t index = 0U; index < rejected_components.size(); ++index) {
            const auto & component = *rejected_components[index];
            rejection.point_layers.push_back(componentLayer(
                    "audit_component_min_columns_rejected",
                    static_cast<std::int32_t>(index), component,
                    COLOR_COLUMNS_REJECT, rejection_config));
            rejection.point_layers.push_back(representativeLayer(
                    "audit_component_representative",
                    static_cast<std::int32_t>(index + 1U), component,
                    COLOR_COLUMNS_REJECT, rejection_config));
            rejection.line_layers.push_back(boundsLayer(
                    "audit_component_bounds",
                    static_cast<std::int32_t>(index + 1U),
                    component.xy_minimum, component.xy_maximum,
                    COLOR_COLUMNS_REJECT, rejection_config));
        }
        ComponentAuditPointLayer threshold_ruler;
        threshold_ruler.ns = "audit_min_columns_threshold";
        threshold_ruler.id = 0;
        threshold_ruler.shape = ComponentAuditPointShape::Cube;
        threshold_ruler.scale = Point3f {0.17F, 0.17F, 0.08F};
        threshold_ruler.color = COLOR_MUTED;
        for(std::size_t index = 0U; index < config_.thresholds.min_columns; ++index) {
            threshold_ruler.points.push_back(Point3f {
                    1.85F + static_cast<float>(index) * 0.21F, 1.0F, 1.05F});
        }
        rejection.point_layers.push_back(std::move(threshold_ruler));
        if(config_.show_labels) {
            appendText(
                    rejection, "audit_stage_title", 0,
                    Point3f {3.0F, 1.0F, 3.25F},
                    "STAGE 6 | COMPONENTS", 0.25F);
            appendText(
                    rejection, "audit_thresholds", 0,
                    Point3f {3.0F, 1.0F, 1.45F},
                    "min_columns = 12 XY columns", 0.17F, COLOR_MUTED);
            appendText(
                    rejection, "audit_component_label", 0,
                    Point3f {3.0F, 1.0F, 2.75F},
                    "C" + std::to_string(accepted_it->component_index) + " | "
                            + std::to_string(accepted_it->exact_column_count)
                            + " columns | ACCEPTED",
                    0.17F, COLOR_ACCEPTED);
            appendText(
                    rejection, "audit_component_label", 1,
                    Point3f {3.0F, 1.0F, 2.35F},
                    "C" + std::to_string(rejected_first.component_index) + " | "
                            + std::to_string(rejected_first.exact_column_count)
                            + " < 12 | REJECT",
                    0.16F, COLOR_COLUMNS_REJECT);
            appendText(
                    rejection, "audit_component_label", 2,
                    Point3f {3.0F, 1.0F, 1.95F},
                    "C" + std::to_string(rejected_second.component_index) + " | "
                            + std::to_string(rejected_second.exact_column_count)
                            + " < 12 | REJECT",
                    0.16F, COLOR_COLUMNS_REJECT);
            appendText(
                    rejection, "audit_plane_note", 0,
                    Point3f {3.0F, 1.0F, -0.85F},
                    "LOCAL XY FOOTPRINTS | Z NOT RECONSTRUCTED",
                    0.16F, COLOR_MUTED);
        }
        stages.push_back(ComponentAuditStageScene {
                "component_rejection", std::move(rejection)});

        const auto direction_it = std::min_element(
                snapshot.components.begin(), snapshot.components.end(),
                [](const ComponentAuditComponent & lhs,
                   const ComponentAuditComponent & rhs) {
                    const bool lhs_direction =
                            lhs.rejection == ComponentAuditRejection::Direction;
                    const bool rhs_direction =
                            rhs.rejection == ComponentAuditRejection::Direction;
                    if(lhs_direction != rhs_direction) {
                        return lhs_direction;
                    }
                    if(!lhs_direction) {
                        return lhs.component_index < rhs.component_index;
                    }
                    return std::tie(
                                   lhs.exact_column_count, lhs.component_index)
                           < std::tie(
                                   rhs.exact_column_count, rhs.component_index);
                });
        if(direction_it == snapshot.components.end()
           || direction_it->rejection != ComponentAuditRejection::Direction)
        {
            throw std::logic_error("audit fixture has no direction rejection");
        }
        const FrontierComponentAuditReplayConfig direction_config =
                centeredConfig(direction_it->representative);

        ComponentAuditScene direction;
        direction.point_layers.push_back(componentLayer(
                "audit_direction_rejected_component", 0, *direction_it,
                COLOR_DIRECTION_REJECT, direction_config));
        direction.point_layers.push_back(representativeLayer(
                "audit_component_representative", 0, *direction_it,
                COLOR_DIRECTION_REJECT, direction_config));
        direction.line_layers.push_back(boundsLayer(
                "audit_component_bounds", 0, direction_it->xy_minimum,
                direction_it->xy_maximum, COLOR_DIRECTION_REJECT,
                direction_config));
        const Point3f vote_origin =
                translated(
                        direction_it->representative, direction_config, 0.28F);
        const std::array<Point3f, 4U> vote_directions {
                Point3f {1.0F, 0.0F, 0.0F}, Point3f {0.0F, 1.0F, 0.0F},
                Point3f {-1.0F, 0.0F, 0.0F}, Point3f {0.0F, -1.0F, 0.0F}};
        const std::array<ComponentAuditColor, 4U> vote_colors {
                COLOR_CYAN, COLOR_PURPLE, COLOR_DIRECTION_REJECT, COLOR_YELLOW};
        const std::array<const char *, 4U> vote_names {
                "+X", "+Y", "-X", "-Y"};
        const std::size_t maximum_vote = *std::max_element(
                direction_it->direction_votes.begin(),
                direction_it->direction_votes.end());
        for(std::size_t index = 0U; index < vote_directions.size(); ++index) {
            const float length = 2.0F
                                 * static_cast<float>(
                                         direction_it->direction_votes[index])
                                 / static_cast<float>(maximum_vote);
            direction.arrows.push_back(ComponentAuditArrow {
                    "audit_direction_votes", static_cast<std::int32_t>(index),
                    vote_origin,
                    Point3f {
                            vote_origin.x + vote_directions[index].x * length,
                            vote_origin.y + vote_directions[index].y * length,
                            vote_origin.z},
                    0.055F, 0.13F, 0.20F, vote_colors[index]});
            if(config_.show_labels) {
                appendText(
                        direction, "audit_direction_vote_label",
                        static_cast<std::int32_t>(index),
                        Point3f {
                                vote_origin.x
                                        + vote_directions[index].x * (length + 0.30F),
                                vote_origin.y
                                        + vote_directions[index].y * (length + 0.30F),
                                0.65F},
                        std::string(vote_names[index]) + "  "
                                + std::to_string(
                                        direction_it->direction_votes[index]),
                        0.15F, vote_colors[index]);
            }
        }
        if(config_.show_labels) {
            appendText(
                    direction, "audit_stage_title", 0,
                    Point3f {3.0F, 1.0F, 3.25F},
                    "STAGE 7 | DIRECTION", 0.25F);
            const std::size_t total_votes = std::accumulate(
                    direction_it->direction_votes.begin(),
                    direction_it->direction_votes.end(), std::size_t {0U});
            appendText(
                    direction, "audit_direction_decision", 0,
                    Point3f {3.0F, 1.0F, 2.75F},
                    "C" + std::to_string(direction_it->component_index) + " | "
                            + std::to_string(direction_it->exact_column_count)
                            + " COL | "
                            + std::to_string(maximum_vote) + "/"
                            + std::to_string(total_votes) + " = "
                            + decimal(direction_it->direction_consistency)
                            + " < 0.650",
                    0.18F, COLOR_DIRECTION_REJECT);
            appendText(
                    direction, "audit_direction_decision", 1,
                    Point3f {3.0F, 1.0F, 2.35F},
                    "REJECT: DIRECTION | DOMINANT "
                            + std::string(vote_names[static_cast<std::size_t>(
                                      std::distance(
                                              direction_it->direction_votes.begin(),
                                              std::max_element(
                                                      direction_it->direction_votes.begin(),
                                                      direction_it->direction_votes.end()))) ]),
                    0.17F, COLOR_WHITE);
        }
        stages.push_back(ComponentAuditStageScene {
                "direction_evidence", std::move(direction)});

        const ComponentAuditGapPair & selected_gap =
                analysis.beneficial_gap_pairs.front();
        const auto & gap_first =
                componentAt(snapshot, selected_gap.first_component_index);
        const auto & gap_second =
                componentAt(snapshot, selected_gap.second_component_index);
        const FrontierComponentAuditReplayConfig counterfactual_config =
                centeredConfig(Point3f {
                        (gap_first.representative.x
                         + gap_second.representative.x)
                                * 0.5F,
                        (gap_first.representative.y
                         + gap_second.representative.y)
                                * 0.5F,
                        0.0F});
        ComponentAuditScene counterfactual;
        counterfactual.point_layers.push_back(componentLayer(
                "audit_gap_component_a", 0, gap_first, COLOR_YELLOW,
                counterfactual_config));
        counterfactual.point_layers.push_back(componentLayer(
                "audit_gap_component_b", 0, gap_second, COLOR_BLUE,
                counterfactual_config));
        counterfactual.point_layers.push_back(representativeLayer(
                "audit_component_representative", 0, gap_first,
                COLOR_YELLOW, counterfactual_config));
        counterfactual.point_layers.push_back(representativeLayer(
                "audit_component_representative", 1, gap_second,
                COLOR_BLUE, counterfactual_config));
        counterfactual.line_layers.push_back(boundsLayer(
                "audit_gap_component_bounds", 0, gap_first.xy_minimum,
                gap_first.xy_maximum, COLOR_YELLOW, counterfactual_config));
        counterfactual.line_layers.push_back(boundsLayer(
                "audit_gap_component_bounds", 1, gap_second.xy_minimum,
                gap_second.xy_maximum, COLOR_BLUE, counterfactual_config));
        counterfactual.line_layers.push_back(boundsLayer(
                "audit_pair_counterfactual_bounds", 0,
                selected_gap.merged_decision.xy_minimum,
                selected_gap.merged_decision.xy_maximum, COLOR_CYAN,
                counterfactual_config, 0.18F));
        const Point3f gap_start = columnPoint(
                selected_gap.first_column, counterfactual_config, 0.20F);
        const Point3f gap_end = columnPoint(
                selected_gap.second_column, counterfactual_config, 0.20F);
        counterfactual.line_layers.push_back(dashedLine(
                "audit_one_column_gap", 0, gap_start, gap_end, COLOR_WHITE));
        const bool integral_midpoint =
                (selected_gap.first_column.x + selected_gap.second_column.x) % 2 == 0
                && (selected_gap.first_column.y + selected_gap.second_column.y) % 2
                           == 0;
        const ComponentAuditColumnKey midpoint {
                (selected_gap.first_column.x + selected_gap.second_column.x) / 2,
                (selected_gap.first_column.y + selected_gap.second_column.y) / 2};
        const bool midpoint_is_supported = std::any_of(
                snapshot.components.begin(), snapshot.components.end(),
                [&](const ComponentAuditComponent & component) {
                    return std::binary_search(
                            component.columns.begin(), component.columns.end(),
                            midpoint);
                });
        if(integral_midpoint && !midpoint_is_supported) {
            const Point3f center =
                    columnPoint(midpoint, counterfactual_config, 0.10F);
            const float half = static_cast<float>(
                                       config_.thresholds.resolution
                                       * static_cast<double>(
                                               config_.thresholds.column_stride_voxels))
                               * 0.42F;
            ComponentAuditLineLayer missing;
            missing.ns = "audit_absent_supported_column";
            missing.id = 0;
            missing.width = 0.025F;
            missing.color = COLOR_WHITE;
            const Point3f a {center.x - half, center.y - half, center.z};
            const Point3f b {center.x + half, center.y - half, center.z};
            const Point3f c {center.x + half, center.y + half, center.z};
            const Point3f d {center.x - half, center.y + half, center.z};
            missing.points = {a, b, b, c, c, d, d, a};
            counterfactual.line_layers.push_back(std::move(missing));
        }
        if(config_.show_labels) {
            appendText(
                    counterfactual, "audit_stage_title", 0,
                    Point3f {3.0F, 1.0F, 3.25F},
                    "STAGE 8 | GAP TEST", 0.25F);
            appendText(
                    counterfactual, "audit_gap_component_label", 0,
                    translated(
                            gap_first.representative, counterfactual_config,
                            0.65F),
                    "C" + std::to_string(gap_first.component_index) + " | "
                            + std::to_string(gap_first.exact_column_count)
                            + " < 12 | REJECT",
                    0.16F, COLOR_YELLOW);
            appendText(
                    counterfactual, "audit_gap_component_label", 1,
                    translated(
                            gap_second.representative, counterfactual_config,
                            0.65F),
                    "C" + std::to_string(gap_second.component_index) + " | "
                            + std::to_string(gap_second.exact_column_count)
                            + " < 12 | REJECT",
                    0.16F, COLOR_BLUE);
            appendText(
                    counterfactual, "audit_gap_pair_result", 0,
                    Point3f {3.0F, 1.0F, 2.75F},
                    "PAIR-ONLY: "
                            + std::to_string(
                                    selected_gap.merged_decision.exact_column_count)
                            + " COL | DIR "
                            + decimal(
                                    selected_gap.merged_decision.direction_consistency)
                            + " | WOULD PASS",
                    0.18F, COLOR_CYAN);
            appendText(
                    counterfactual, "audit_gap_pair_result", 1,
                    Point3f {3.0F, 1.0F, 2.35F},
                    "FRAME " + std::to_string(snapshot.frame_index) + ": "
                            + std::to_string(
                                    analysis.beneficial_gap_pairs.size())
                            + " beneficial / "
                            + std::to_string(
                                    analysis.one_column_gap_pairs.size())
                            + " one-column-gap pairs",
                    0.17F, COLOR_WHITE);
            appendText(
                    counterfactual, "audit_radius_two_result", 0,
                    Point3f {3.0F, 1.0F, 1.95F},
                    "RADIUS-2: ACCEPTED "
                            + std::to_string(
                                    analysis.baseline_accepted_components)
                            + " -> "
                            + std::to_string(
                                    analysis.radius_two_accepted_groups)
                            + " | DIRECTION MIX",
                    0.18F, COLOR_DIRECTION_REJECT);
            appendText(
                    counterfactual, "audit_counterfactual_scope", 0,
                    Point3f {3.0F, 1.0F, 1.55F},
                    "READ-ONLY | NO COLUMN ADDED",
                    0.16F, COLOR_MUTED);
        }
        stages.push_back(ComponentAuditStageScene {
                "gap_counterfactual", std::move(counterfactual)});

        return stages;
    }

    const FrontierComponentAuditReplayConfig &
    FrontierComponentAuditReplay::config() const
    {
        return config_;
    }

}// namespace SwarmController

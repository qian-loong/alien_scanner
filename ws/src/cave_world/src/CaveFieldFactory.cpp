#include "cave_world/CaveFieldFactory.hpp"

namespace CaveWorld {

    std::unique_ptr<ICaveField> createCaveField(
            const std::string &               cave_mode,
            const ProceduralCaveFieldConfig & y_config,
            const TreeCaveFieldConfig &       tree_config)
    {
        if(cave_mode == "y") {
            return std::make_unique<ProceduralCaveField>(y_config);
        }
        return std::make_unique<TreeCaveField>(tree_config);
    }

}// namespace CaveWorld

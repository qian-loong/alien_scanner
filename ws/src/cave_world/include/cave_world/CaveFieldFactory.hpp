#ifndef CAVE_WORLD_CAVEFIELDFACTORY_HPP
#define CAVE_WORLD_CAVEFIELDFACTORY_HPP

#include "cave_world/ICaveField.hpp"
#include "cave_world/ProceduralCaveField.hpp"
#include "cave_world/TreeCaveField.hpp"

#include <memory>
#include <string>

namespace CaveWorld {

    /// 从 mode 字符串构造洞穴场；mode=`y` 用 Y 字，否则用 Tree（默认）。
    std::unique_ptr<ICaveField> createCaveField(
            const std::string &           cave_mode,
            const ProceduralCaveFieldConfig & y_config,
            const TreeCaveFieldConfig &       tree_config);

}// namespace CaveWorld

#endif// CAVE_WORLD_CAVEFIELDFACTORY_HPP

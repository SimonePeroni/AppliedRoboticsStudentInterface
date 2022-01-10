#pragma once

#include "rm/RoadMap.hpp"
#include "utils.hpp"

#include <vector>

namespace rm
{
    void visibility(RoadMap &roadmap, const std::vector<Polygon> &obstacles, const Polygon &borders);
}
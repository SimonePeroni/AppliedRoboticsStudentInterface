#pragma once

#include "rm/RoadMap.hpp"

#include <vector>
#include "utils.hpp"

namespace rm
{
    RoadMap &maxClearance(const std::vector<Polygon> &obstacles, const Polygon &borders);
}

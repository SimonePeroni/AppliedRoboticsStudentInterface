#pragma once

#include "rm/RoadMap.hpp"
#include "utils.hpp"

#include <vector>

namespace rm
{
    void visibility(RoadMap &roadmap, const std::vector<Point> points, const std::vector<Polygon> &obstacles, const Polygon &borders);

    void makeVisibilityNodes(const std::vector<Polygon> &obstacles, const Polygon &borders,
                             float offset, std::vector<Point> &nodes, float threshold = 0.0f);
}
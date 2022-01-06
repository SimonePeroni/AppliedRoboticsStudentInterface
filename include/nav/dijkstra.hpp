#pragma once

#include <vector>
#include <set>

#include "rm/RoadMap.hpp"

namespace nav
{
    typedef std::vector<rm::RoadMap::DubinsConnection *> navList;

    navList dijkstraShortestPath(const rm::RoadMap::Node::Orientation &source,
                                 const rm::RoadMap::Node::Orientation &goal);
}
#pragma once

#include <deque>

#include "rm/RoadMap.hpp"

namespace nav
{
    typedef std::deque<rm::RoadMap::DubinsConnection const *> navList;

    navList dijkstraShortestPath(const rm::RoadMap::Node::Orientation &source,
                                 const rm::RoadMap::Node::Orientation &goal);
}
#pragma once

#include <deque>

#include "rm/RoadMap.hpp"

namespace nav
{
    typedef std::deque<rm::RoadMap::DubinsConnection *> navList;

    navList dijkstraShortestPath(const rm::RoadMap::Node::Orientation &source,
                                 const rm::RoadMap::Node::Orientation &goal,
                                 const rm::RoadMap &roadmap);
}
#pragma once
/**
 * @file dijkstra.hpp
 * @brief dijkstra navigation algorithm implementTION
 * 
 */

#include <deque>

#include "rm/RoadMap.hpp"

namespace nav
{

    typedef std::deque<rm::RoadMap::DubinsConnection const *> navList;

 /**
 * @brief 
 * 
 * @param source  Starting node
 * @param goal    Goal node
 * @return        List of pointers to DubinsConnection, representing the full path from start to goal
 * @deprecated    USE NavMap INSTEAD
 */
    navList dijkstraShortestPath(const rm::RoadMap::Node::Orientation &source,
                                 const rm::RoadMap::Node::Orientation &goal);
}

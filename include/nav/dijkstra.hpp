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
 * @param source staring node
 * @param goal goal node 
 * @param roadmap complete roadmap 
 * @return navList 
 */
    navList dijkstraShortestPath(const rm::RoadMap::Node::Orientation &source,
                                 const rm::RoadMap::Node::Orientation &goal);
}

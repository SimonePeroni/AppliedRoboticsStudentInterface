#pragma once

#include "nav/NavMap.hpp"

#include <vector>
namespace nav
{
    void runGame(const std::vector<nav::NavMap> &nm_e, nav::NavMap &nm_p,
                 const rm::RoadMap::Node::Orientation &source_e, const rm::RoadMap::Node::Orientation &source_p,
                 nav::navList &nav_list_e, nav::navList &nav_list_p);
}
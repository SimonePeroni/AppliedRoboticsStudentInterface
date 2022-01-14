#pragma once

#include "rm/RoadMap.hpp"

#include <deque>

namespace nav
{
    typedef std::deque<rm::RoadMap::DubinsConnection const *> navList;

    void discretizePath(const navList &nav_list, float step, std::vector<Pose> &discr_path);

    void truncatePaths(std::vector<Pose> &discr_path_e, std::vector<Pose> &discr_path_p, float robot_size);
}
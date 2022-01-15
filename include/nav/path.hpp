#pragma once

#include "rm/roadmap.hpp"

#include <deque>

/**
 * @file path.hpp
 * @brief This file holds functions for path manipulation.
 * 
 * @see nav
 * @see nav#navList
 * @see nav#discretizePath()
 * @see nav#truncatePaths()
 */

namespace nav
{
    /**
     * @brief Type describing a navigation path. It contains a list of RoadMap DubinsConnections that are to be followed in order.
     * 
     */
    typedef std::deque<rm::RoadMap::DubinsConnection const *> navList;

    /**
     * @brief Discretize a navigation path. Supports wait connections.
     * 
     * @param[in]  nav_list      Navigation path to be discretized
     * @param[in]  step          Discretization step
     * @param[out] discr_path    Out: Discretized path
     */
    void discretizePath(const navList &nav_list, float step, std::vector<Pose> &discr_path);

    /**
     * @brief Truncate two navigation paths at the collision point.
     * 
     * @param[in,out] discr_path1   First discretized path
     * @param[in,out] discr_path2   Second discretized path
     * @param[in]     robot_size    Size of the robot
     */
    void truncatePaths(std::vector<Pose> &discr_path1, std::vector<Pose> &discr_path2, float robot_size);
}
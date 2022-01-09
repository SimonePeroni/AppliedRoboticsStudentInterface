#pragma once
/**
 * @file maxClearance.hpp 
 * @brief In this file the maximum clearance is presented 
 */


#include "rm/RoadMap.hpp"

#include <vector>
#include "utils.hpp"

namespace rm
{
     /**
     * @brief function for the maximum clearance
     * given the list of obstacles and borders it is possible to create a maximum clearance graph which will be used as roadmap for our purposes. 
     * @param obstacles 
     * @param borders 
     * @return RoadMap 
     */
    RoadMap maxClearance(const std::vector<Polygon> &obstacles, const Polygon &borders);
}

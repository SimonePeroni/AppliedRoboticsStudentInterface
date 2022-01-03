#pragma once

#include "clipper/clipper.hpp"
#include "utils.hpp"

#include <vector>

namespace rm
{
    /**
     * @brief 
     * 
     * @param polygons 
     * @param offset 
     * @return std::vector<Polygon> 
     */
    std::vector<Polygon> inflate(const std::vector<Polygon> &polygons, float offset);
}

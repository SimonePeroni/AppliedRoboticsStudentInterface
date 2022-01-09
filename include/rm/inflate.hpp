#pragma once
/**
 * @file inflate.hpp 
 * @brief Implementation of the obstacle inflation using the clipper library
 * clic on the function inflate to have more info about the input parameters and what is returned 
 * 
 */

#include "clipper/clipper.hpp"
#include "utils.hpp"

#include <vector>

namespace rm
{
    /**
     * @brief Inflate polygons by specified offset value. 
     * 
     * @param[in] polygons   Collection of original polygons to inflate.
     * @param[in] offset     How much the polygon should be inflated.
     * @param[in] clockwise  Whether the results should be given in clockwise order.
     * @return               Collection of inflated polygons.
     */
    std::vector<Polygon> inflate(const std::vector<Polygon> &polygons, float offset, bool clockwise = false);
}

#pragma once

#include "clipper/clipper.hpp"
#include "utils.hpp"

#include <vector>

namespace rm
{
    /**
     * @brief Inflate polygons by specified offset value. 
     * 
     * @param polygons   Collection of original polygons to inflate.
     * @param offset     How much the polygon should be inflated.
     * @param clockwise  Whether the results should be given in clockwise order.
     * @return           Collection of inflated polygons.
     */
    std::vector<Polygon> inflate(const std::vector<Polygon> &polygons, float offset, bool clockwise = false);
}

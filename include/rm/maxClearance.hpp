#pragma once

#include <vector>
#include "utils.hpp"

namespace rm
{
    void maxClearance(const std::vector<Polygon> &obstacles, const Polygon &borders);
}

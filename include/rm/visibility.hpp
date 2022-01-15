#pragma once

#include "rm/roadmap.hpp"
#include "utils.hpp"

#include <vector>

/**
 * @file visibility.hpp
 * @brief This file is dedicated to visibility graph creation. \n 
 * 
 * @see rm#visibility()
 * @see rm#makeVisibilityNodes()
 */

namespace rm
{
    /**
     * @brief Compute a visibility graph from a given set of vertices. \n
     * 
     * Each couple of vertices are connected to eachother if the segment connecting them does not intersect any obstacles. 
     * This is a naive O(E*V^2) implementation, where N is the number of vertices and E the number of edges in the obstacles.
     * 
     * @param[in] roadmap   Out: The result is stored in the base directed graph of the roadmap.
     * @param[in] points    Vertices to be included in the graph
     * @param[in] obstacles Obstacles for collision checking
     * @param[in] borders   Borders of the arena for collision checking
     * 
     * @see rm#RoadMap
     */
    void visibility(RoadMap &roadmap, const std::vector<Point> points, const std::vector<Polygon> &obstacles, const Polygon &borders);

    /**
     * @brief Generate a set of vertices for the visibility graph from the inflation of the obstacles. \n 
     * 
     * To avoid chunks of close vertices, a minimum distance threshold can be specified. 
     * Vertices that are closer to each other than the threshold will be averaged into a single one.
     * 
     * @param[in]  obstacles Source obstacles
     * @param[in]  borders   Borders of the arena, considered as obstacles
     * @param[in]  offset    Inflation value of the obstacles.
     * @param[Out] nodes     Out: vector of selected vertices
     * @param[in]  threshold Optional: minimum distance between consecutive nodes
     */
    void makeVisibilityNodes(const std::vector<Polygon> &obstacles, const Polygon &borders,
                             float offset, std::vector<Point> &nodes, float threshold = 0.0f);
}
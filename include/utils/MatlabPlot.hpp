#pragma once
/**
 * @file MatlabPlot.hpp
 * @brief file used to generate a matlab file to visualize the results. 
 * 
 */
#include <fstream>
#include <string>
#include <vector>

#include "utils.hpp"
#include "rm/geometry.hpp"
#include "rm/RoadMap.hpp"

namespace utils
{
    class MatlabPlot
    {
    private:
        std::ofstream _file;
    public:
        MatlabPlot(const char *path);
        ~MatlabPlot();

        void plotSegment(const rm::Segment &s, std::string style = "");
        void plotPoint(const Point &p, std::string style = "");
        void plotNodes(const rm::RoadMap &rm, std::string style = "bo");
        void plotEdges(const rm::RoadMap &rm, std::string style = "c--");
        /**
         * @brief function used to print the Roadmap
         * 
         * @param[in] rm 
         * @param node_style each node present in the roadmap 
         * @param edge_style each edge present in the roadmap 
         */

        void plotGraph(const rm::RoadMap &rm, std::string node_style = "bo", std::string edge_style = "c--");
        /**
         * @brief function used to print the borders and inflated borders
         * 
         * @param[in] poly the borders of the map as rectangle
         * @param style r- for the inflated borders, k- for the original borders
         */

        void plotPolygon(const Polygon poly, std::string style = "r-");
        /**
         * @brief function used to print the gates and obstacles
         * 
         * @param[in] polys the obstacle and gate lists
         * @param style k- for non inflated obstacles, g- for the gates and default r- for the inflated polygons
         */

        void plotPolygons(const std::vector<Polygon> polys, std::string style = "r-");
        /**
         * @brief function used to plot the discretized Dubins path taken by the evader and pursuer
         * 
         * @param[in] path 
         * @param style default m. used for the evader and k. for the pursuer
         */
        void plotDiscretePath(const std::vector<Pose> path, std::string style = "m.");
    };
}

#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "utils.hpp"
#include "rm/geometry.hpp"
#include "rm/RoadMap.hpp"

/**
 * @file MatlabPlot.hpp
 * @brief File dedicated to the class MatlabPlot.
 * 
 * @see utils#MatlabPlot
 */

/**
 * @namespace utils
 * @brief Namespace wrapping some utility functions.
 * 
 */
namespace utils
{
    /**
     * @brief Class used to generate a matlab file for geometric visualizations. 
     * 
     */
    class MatlabPlot
    {
    private:
        std::ofstream _file;
    public:
        /**
         * @brief Construct a new MatlabPlot object and open the file stream.
         * 
         * @param path Filepath of the .m file to write the functions to.
         */
        MatlabPlot(const char *path);
        /**
         * @brief Destroy the Matlab Plot object, closing the file stream.
         * 
         */
        ~MatlabPlot();

        /**
         * @brief Plot a segment.
         * 
         * @param s     Segment
         * @param style Optional: matlab style
         */
        void plotSegment(const rm::Segment &s, std::string style = "");

        /**
         * @brief Plot a point.
         * 
         * @param p     Point
         * @param style Optional: matlab style
         */
        void plotPoint(const Point &p, std::string style = "");

        /**
         * @brief Plot the nodes of a RoadMap object.
         * 
         * @param rm    Roadmap
         * @param style Optional: matlab style
         */
        void plotNodes(const rm::RoadMap &rm, std::string style = "bo");

        /**
         * @brief Plot the edges of a RoadMap object.
         * 
         * @param rm    Roadmap
         * @param style Optional: matlab style
         */
        void plotEdges(const rm::RoadMap &rm, std::string style = "c--");

        /**
         * @brief Plot the base graph of a RoadMap object.
         * 
         * @param rm         Roadmap
         * @param node_style Optional: matlab style for nodes
         * @param edge_style Optional: matlab style for edges
         */
        void plotGraph(const rm::RoadMap &rm, std::string node_style = "bo", std::string edge_style = "c--");

        /**
         * @brief Plot a polygon.
         * 
         * @param poly  Polygon
         * @param style Optional: matlab style
         */
        void plotPolygon(const Polygon poly, std::string style = "r-");

        /**
         * @brief Plot a set of polygons.
         * 
         * @param polys Vector of polygons
         * @param style Optional: matlab style
         */
        void plotPolygons(const std::vector<Polygon> polys, std::string style = "r-");

        /**
         * @brief Plot a discretized path.
         * 
         * @param path  Discretized path
         * @param style Optional: matlab style
         */
        void plotDiscretePath(const std::vector<Pose> path, std::string style = "m.");
    };
}

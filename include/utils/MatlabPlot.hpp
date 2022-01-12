#pragma once

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
        void plotGraph(const rm::RoadMap &rm, std::string node_style = "bo", std::string edge_style = "c--");
        void plotPolygon(const Polygon poly, std::string style = "r-");
        void plotPolygons(const std::vector<Polygon> polys, std::string style = "r-");
        void plotDiscretePath(const std::vector<Pose> path, std::string style = "m.");
    };
}
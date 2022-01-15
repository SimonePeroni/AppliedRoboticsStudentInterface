#include "utils/matlabplot.hpp"

namespace utils
{
    MatlabPlot::MatlabPlot(const char *path)
    {
        _file.open(path);
        _file.clear();
        _file << "figure, hold on" << std::endl << std::endl;
    }

    MatlabPlot::~MatlabPlot()
    {
        _file.close();
    }

    void MatlabPlot::plotSegment(const rm::Segment &s, std::string style)
    {
        _file << "plot([" << s.p0.x << ", " << s.p1.x << "], [" << s.p0.y << ", " << s.p1.y << "]";
        if (!style.empty())
            _file << ", '" << style << "'";
        _file << ")" << std::endl;
    }

    void MatlabPlot::plotPoint(const Point &p, std::string style)
    {
        _file << "plot(" << p.x << ", " << p.y;
        if (!style.empty())
            _file << ", '" << style << "'";
        _file << ")" << std::endl;
    }

    void MatlabPlot::plotNodes(const rm::RoadMap &rm, std::string style)
    {
        _file << "\% Nodes" << std::endl;
        for (size_t i = 0; i < rm.getNodeCount(); i++)
		{
            auto node = rm.getNode(i);
			plotPoint(Point(node.getX(), node.getY()), style);
		}
        _file << std::endl;
    }

    void MatlabPlot::plotEdges(const rm::RoadMap &rm, std::string style)
    {
        _file << "\% Edges" << std::endl;
        for (size_t i = 0; i < rm.getNodeCount(); i++)
		{
			auto node = rm.getNode(i);
			for (size_t j = 0; j < node.getConnectedCount(); j++)
			{
				auto other = node.getConnected(j);
                plotSegment(rm::Segment(node.getX(), node.getY(), other.getX(), other.getY()), style);
			}
		}
        _file << std::endl;
    }

    void MatlabPlot::plotGraph(const rm::RoadMap &rm, std::string node_style, std::string edge_style)
    {
        plotNodes(rm, node_style);
        plotEdges(rm, edge_style);
    }

    void MatlabPlot::plotPolygon(const Polygon poly, std::string style)
    {
        _file << "\% Polygon" << std::endl;
        auto p = rm::getEdges(poly);
        for (auto &edge : p)
        {
            plotSegment(edge, style);
        }
        _file << std::endl;
    }

    void MatlabPlot::plotPolygons(const std::vector<Polygon> polys, std::string style)
    {
        for (auto &poly : polys)
        {
            plotPolygon(poly, style);
        }
    }

    void MatlabPlot::plotDiscretePath(const std::vector<Pose> path, std::string style)
    {
        _file << "\% Path" << std::endl;
        for (auto &pose : path)
		{
			plotPoint(Point(pose.x, pose.y), style);
		}
        _file << std::endl;
    }
}
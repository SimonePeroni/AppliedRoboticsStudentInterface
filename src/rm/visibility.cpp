#include "rm/visibility.hpp"

#include "rm/geometry.hpp"
#include "rm/inflate.hpp"

namespace rm
{
    void visibility(RoadMap &roadmap, const std::vector<Point> points, const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        for (size_t i = 0; i < points.size() - 1; i++)
        {
            for (size_t j = i + 1; j < points.size(); j++)
            {
                auto &p0 = points[i];
                auto &p1 = points[j];
                bool visible = true;
                if (collisionCheck(Segment(p0, p1), borders))
                    continue;
                Segment s(p0, p1);
                for (const Polygon &obst : obstacles)
                {
                    if (collisionCheck(p0, obst) || collisionCheck(p1, obst) || collisionCheck(s, obst))
                    {
                        visible = false;
                        break;
                    }
                }

                if (visible)
                {
                    auto n0 = roadmap.addNode(p0);
                    auto n1 = roadmap.addNode(p1);
                    roadmap.connect(n0, n1);
                    roadmap.connect(n1, n0);
                }
            }
        }
    }

    void makeVisibilityNodes(const std::vector<Polygon> &obstacles, const Polygon &borders,
                             float offset, std::vector<Point> &nodes, float threshold)
    {
        std::vector<Polygon> clip = rm::inflate(obstacles, offset, true);
        Polygon source = rm::inflate(std::vector<Polygon>{borders}, -offset).back();

        ClipperLib::Paths joinedPaths;
        ClipperLib::Clipper clpr;

        const float scale = 1000;

        for (auto const &c : clip)
        {
            ClipperLib::Path clipPath;

            for (auto const &vertex : c)
                clipPath << ClipperLib::IntPoint(vertex.x * scale, vertex.y * scale);

            clpr.AddPath(clipPath, ClipperLib::ptClip, true);
        }

        ClipperLib::Path srcPath;
        for (auto const &vertex : source)
            srcPath << ClipperLib::IntPoint(vertex.x * scale, vertex.y * scale);
        clpr.AddPath(srcPath, ClipperLib::ptSubject, true);

        clpr.Execute(ClipperLib::ctDifference, joinedPaths);

        for (auto &path : joinedPaths)
        {
            float old_x = INFINITY, old_y = INFINITY;
            int weight = 1;
            for (auto const &vertex : path)
            {
                float x = vertex.X / scale;
                float y = vertex.Y / scale;
                if (std::hypotf(x - old_x, y - old_y) < threshold)
                {
                    nodes.pop_back();
                    old_x = (weight * old_x + x) / (weight + 1);
                    old_y = (weight * old_y + y) / (weight + 1);
                    weight++;
                    nodes.push_back(Point(old_x, old_y));
                }
                else
                {
                    nodes.push_back(Point(x, y));
                    old_x = x;
                    old_y = y;
                    weight = 1;
                }
            }
        }
    }
}
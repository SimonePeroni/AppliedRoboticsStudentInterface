#include "rm/geometry.hpp"

#include <limits>

namespace rm
{
    bool collisionCheck(const Segment &s0, const Segment &s1)
    {
        float det = (s1.p1.x - s1.p0.x) * (s0.p0.y - s0.p1.y) - (s0.p0.x - s0.p1.x) * (s1.p1.y - s1.p0.y);
        if (det == 0)
            return false;
        float t = (s1.p0.y - s1.p1.y) * (s0.p0.x - s1.p0.x) + (s1.p1.x - s1.p0.x) * (s0.p0.y - s1.p0.y);
        float u = (s0.p0.y - s0.p1.y) * (s0.p0.x - s1.p0.x) + (s0.p1.x - s0.p0.x) * (s0.p0.y - s1.p0.y);
        return t >= 0 && u >= 0 && t <= det && u <= det;
    }

    bool collisionCheck(const Polygon &p0, const Polygon &p1)
    {
        // Broad phase
        Box box0 = getBoundingBox(p0);
        Box box1 = getBoundingBox(p1);

        if (!collisionCheck(box0, box1))
            return false;

        // Narrow phase
        std::vector<Segment> s0 = getEdges(p0);
        std::vector<Segment> s1 = getEdges(p1);

        for (auto &edge0 : s0)
        {
            for (auto &edge1 : s1)
            {
                if (collisionCheck(edge0, edge1))
                    return true;
            }
        }

        // TODO: complete narrow phase
    }

    bool collisionCheck(const Point &p, const Polygon &poly)
    {

    }

    bool collisionCheck(const Point &p, const Box &b)
    {
        return p.x >= b.xmin && p.x <= b.xmax && p.y >= b.ymin && p.y <= b.ymax;
    }

    bool collisionCheck(const Box &b0, const Box &b1)
    {
        for (auto &vertex : b1.toPoly())
        {
            if (collisionCheck(vertex, b0))
                return true;
        }

        return collisionCheck(Point(b0.xmin, b0.ymin), b1);
    }

    Box getBoundingBox(const Polygon &p)
    {
        float xmin = std::numeric_limits<float>::max();
        float ymin = std::numeric_limits<float>::max();
        float xmax = std::numeric_limits<float>::lowest();
        float ymax = std::numeric_limits<float>::lowest();

        for (auto point : p)
        {
            if (point.x < xmin)
                xmin = point.x;
            if (point.x > xmax)
                xmax = point.x;
            if (point.y < ymin)
                ymin = point.y;
            if (point.y > ymax)
                ymax = point.y;
        }

        return Box(Point(xmin, ymin), Point(xmax, ymax));
    }

    std::vector<Segment> getEdges(const Polygon &p)
    {
        std::vector<Segment> out;
        for (size_t i = 1; i < p.size(); i++)
        {
            out.push_back(Segment(p[i - 1], p[i]));
        }
        out.push_back(Segment(p[p.size()], p[0]));
        return out;
    }

    bool isRightOfOrOn(const Point &p, const Segment &s)
    {
        // TODO: implement isRightOfOrOn
    }
}

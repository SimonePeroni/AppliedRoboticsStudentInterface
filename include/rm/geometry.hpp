#pragma once

#include "utils.hpp"

namespace rm
{
    struct Segment
    {
        Point p0;
        Point p1;
        Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
        Segment(Point p0, Point p1) : p0(p0), p1(p1) {}
    };

    struct Box
    {
        float xmin, xmax, ymin, ymax;
        Box(Point p0, Point p1)
        {
            if (p0.x < p1.x)
            {
                xmin = p0.x;
                xmax = p1.x;
            }
            else
            {
                xmin = p1.x;
                xmax = p0.x;
            }
            if (p0.y < p1.y)
            {
                ymin = p0.y;
                ymax = p1.y;
            }
            else
            {
                ymin = p1.y;
                ymax = p0.y;
            }
        }
        Polygon toPoly() const { return Polygon({Point(xmin, ymin), Point(xmax, ymin),
                                                 Point(xmax, ymax), Point(xmin, ymax)}); }
    };

    bool collisionCheck(const Segment &s0, const Segment &s1);

    bool collisionCheck(const Point &p, const Box &b);

    bool collisionCheck(const Box &b0, const Box &b1);

    bool collisionCheck(const Point &p, const Polygon &poly);

    bool collisionCheck(const Point &p, const std::vector<Segment> &poly);

    bool collisionCheck(const Polygon &p0, const Polygon &p1);

    Box getBoundingBox(const Polygon &p);

    std::vector<Segment> getEdges(const Polygon &p);

    bool isRightOf(const Point &p, const Segment &s);
}

#pragma once
/**
 * @file geometry.hpp
 * @brief implementation of the collision avoidance 
 * 
 */

#include "utils.hpp"
#include "dubins/dubins.hpp"

namespace rm
{
    struct Segment
    {
        Point p0;
        Point p1;
        inline Segment(float x1, float y1, float x2, float y2) : p0(Point(x1, y1)), p1(Point(x2, y2)) {}
        inline Segment(Point p0, Point p1) : p0(p0), p1(p1) {}
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

    bool collisionCheck(const Segment &s, const Polygon &p);

    bool collisionCheck(const dubins::DubinsArc &arc, const Polygon &p);

    /**
     * @brief           Check collision between a circle-arc and a segment.
     * 
     * @param rho       Radius of arc. If positive, the arc takes a left turn from th0 to th1. If negative, the arc takes a right turn from th0 to th1.
     * @param center    Center of curvature.
     * @param th0       Starting angle of arc, given counter-clockwise with respect to positive x axis direction.
     * @param th1       End angle of arc, given counter-clockwise with respect to positive x axis direction.
     * @param s         Segment to verify collision with.
     * @return          true if a collision between the arc and the segment was detected, false otherwise.
     */
    bool collisionCheck(const float &rho, const Point &c, float th0, float th1, const Segment &s);

    bool collisionCheck(const dubins::DubinsCurve &curve, const Polygon &p);

    Box getBoundingBox(const Polygon &p);

    std::vector<Segment> getEdges(const Polygon &p);

    bool isRightOfOrOn(const Point &p, const Segment &s);

    bool inAngleRange(float theta, float th0, float th1, bool clockwise = false);

    void getGatePose(const Polygon &gate, const Polygon &borders, float &x, float &y, float &theta);
}

#pragma once

#include "utils.hpp"
#include "dubins/dubins.hpp"

/**
 * @file geometry.hpp
 * @brief This file holds geometric functions such as collision checking.
 * 
 * @see Segment
 * @see rm#collisionCheck()
 * @see rm#getGatePose()
 */

namespace rm
{
    /**
     * @brief Struct representing a segment.
     * 
     */
    struct Segment
    {
        /** First point of the segment */
        Point p0;
        /** Second point of the segment */
        Point p1;
        /**
         * @brief Construct a new Segment object.
         * 
         * @param[in] x1 First coordinate of the first point
         * @param[in] y1 Second coordinate of the first point
         * @param[in] x2 First coordinate of the second point
         * @param[in] y2 Second coordinate of the second point
         */
        inline Segment(float x1, float y1, float x2, float y2) : p0(Point(x1, y1)), p1(Point(x2, y2)) {}
        /**
         * @brief Construct a new Segment object.
         * 
         * @param[in] p0 First point
         * @param[in] p1 Second point
         */
        inline Segment(Point p0, Point p1) : p0(p0), p1(p1) {}
    };

    /**
     * @brief Check if two segments are colliding.
     * 
     * @param[in] s0    First segment
     * @param[in] s1    Second segment
     * @return      true if segments are colliding, false otherwise
     */
    bool collisionCheck(const Segment &s0, const Segment &s1);

    /**
     * @brief Check if a point is inside a convex polygon.
     * 
     * @warning Unexpected results for non-convex polygons!
     * 
     * @param[in] p     Point
     * @param[in] poly  Convex polygon given as set of vertices
     * @return      true if the point is enclosed in the convex polygon, false otherwise
     */
    bool collisionCheck(const Point &p, const Polygon &poly);

    /**
     * @brief Check if a point is inside a convex polygon.
     * 
     * @warning Unexpected results for non-convex polygons!
     * 
     * @param[in] p     Point
     * @param[in] poly  Convex polygon given as set of edges
     * @return      true if the point is enclosed in the convex polygon, false otherwise
     */
    bool collisionCheck(const Point &p, const std::vector<Segment> &poly);

    /**
     * @brief Check if a segment and a polygon are colliding.
     * 
     * @param[in] s     Segment
     * @param[in] p     Polygon
     * @return      true if the segment collides with the outer border of the polygon, false otherwise
     */
    bool collisionCheck(const Segment &s, const Polygon &p);

    /**
     * @brief Check if a DubinsArc object and a polygon are colliding.
     * 
     * @param[in] arc   DubinsArc
     * @param[in] p     Polygon
     * @return      true if the arc collides with the outer border of the polygon, false otherwise
     * 
     * @see dubins#DubinsArc
     */
    bool collisionCheck(const dubins::DubinsArc &arc, const Polygon &p);

    /**
     * @brief           Check collision between a circle-arc and a segment.
     * 
     * @param[in] rho       Radius of arc. If positive, the arc takes a left turn from th0 to th1. If negative, the arc takes a right turn from th0 to th1.
     * @param[in] center    Center of curvature.
     * @param[in] th0       Starting angle of arc, given counter-clockwise with respect to positive x axis direction.
     * @param[in] th1       End angle of arc, given counter-clockwise with respect to positive x axis direction.
     * @param[in] s         Segment to verify collision with.
     * @return          true if a collision between the arc and the segment was detected, false otherwise.
     */
    bool collisionCheck(const float &rho, const Point &c, float th0, float th1, const Segment &s);

    /**
     * @brief       Check collision between a Dubins curve and a polygon.
     * 
     * @param[in] curve Dubins curve
     * @param[in] p     Polygon
     * @return      true if a collision between the curve and the outer border of the polygon was detected, false otherwise.
     * 
     * @see dubins#DubinsCurve
     */
    bool collisionCheck(const dubins::DubinsCurve &curve, const Polygon &p);

    /**
     * @brief Convert a vertex-list polygon to an edge-list polygon.
     * 
     * @param[in] p Vertex-list polygon
     * @return  Edge-list polygon
     */
    std::vector<Segment> getEdges(const Polygon &p);

    /**
     * @brief   Check if a point is right of a segment or lies on the same line.
     * 
     * @param[in] p Point
     * @param[in] s Segment
     * @return  true if the point is right of the segment or lies on the same line, false otherwise
     */
    bool isRightOfOrOn(const Point &p, const Segment &s);

    /**
     * @brief Check if an angle is inside a range.
     * 
     * @param[in] theta     Angle
     * @param[in] th0       Begninning of range
     * @param[in] th1       End of range
     * @param[in] clockwise Whether the range is given in a clockwise fashion
     * @return          true if the unnormalized angle theta is included in the range [th0,th1] running in the given direction, false otherwise
     */
    bool inAngleRange(float theta, float th0, float th1, bool clockwise = false);

    /**
     * @brief Compute the goal pose associated to a given gate. \n 
     * 
     * The goal is positioned in the midpoint of the polygon and is oriented to lead out of the arena perpendicularly to the closest wall.
     * 
     * @param[in]  gate      Gate
     * @param[in]  borders   Borders of the arena, for gate localization
     * @param[out] x         Out: x-coordinate of the pose
     * @param[out] y         Out: y-coordinate of the pose
     * @param[out] theta     Out: theta-coordinate of the pose
     */
    void getGatePose(const Polygon &gate, const Polygon &borders, float &x, float &y, float &theta);
}

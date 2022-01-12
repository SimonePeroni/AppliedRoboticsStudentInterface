#include <iostream>
#include <vector>
#include <chrono>

#include "rm/RoadMap.hpp"
#include "rm/maxClearance.hpp"
#include "rm/inflate.hpp"
#include "rm/geometry.hpp"

void pretty_print(Point point)
{
    std::cout << "Point(" << point.x << ", " << point.y << ")";
}

int main()
{
    Polygon edge {Point(0.926, 0.407), Point(1.051, 0.402)};
    dubins::DubinsCurve curve;
    curve.arc_1.k = -10;
    curve.arc_1.start.x = 0.918;
    curve.arc_1.start.y = 0.382;
    curve.arc_1.start.theta = 0.785398;
    curve.arc_1.end.x = 0.957628;
    curve.arc_1.end.y = 0.406336;
    curve.arc_1.end.theta = 0.316061;
    curve.arc_2.k = 0;
    curve.arc_2.start.x = 0.957628;
    curve.arc_2.start.y = 0.406336;
    curve.arc_2.start.theta = 0.316061;
    curve.arc_2.end.x = 0.988207;
    curve.arc_2.end.y = 0.416336;
    curve.arc_2.end.theta = 0.316061;
    curve.arc_3.k = -10;
    curve.arc_3.start.x = 0.988207;
    curve.arc_3.start.y = 0.416336;
    curve.arc_3.start.theta = 0.316061;
    curve.arc_3.end.x = 1.09;
    curve.arc_3.end.y = 0.392;
    curve.arc_3.end.theta = 5.49779;
    bool collides = rm::collisionCheck(curve, edge);
    std::cout << collides << std::endl;
}
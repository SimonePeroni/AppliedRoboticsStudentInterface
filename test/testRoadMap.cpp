#include <iostream>
#include <vector>

#include "rm/RoadMap.hpp"
#include "rm/maxClearance.hpp"
#include "rm/inflate.hpp"

int main()
{
    std::vector<Polygon> obstacles;
    obstacles.push_back(Polygon{Point(0.5, 0.5), Point(0.5, 0.6),
                                Point(0.6, 0.6), Point(0.6, 0.5)});
    obstacles.push_back(Polygon{Point(0.20, 0.15), Point(0.10, 0.25),
                                Point(0.20, 0.35), Point(0.30, 0.25)});

    Polygon borders {Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)};

    float offset = 0.05f;
    std::vector<Polygon> infObstacles = rm::inflate(obstacles, offset);
    Polygon infBorders = rm::inflate(std::vector<Polygon>{borders}, -offset).back();

    rm::RoadMap rm = rm::maxClearance(obstacles, borders);

    for (size_t i = 0; i < rm.getNodeCount(); i++)
    {
        auto node = rm.getNodeAt(i);
        std::cout << "Node_" << node.getID() << std::endl
                  << " X: " << node.getX() << std::endl
                  << " Y: " << node.getY() << std::endl;
    }
}
#include <iostream>
#include <vector>

#include "rm/RoadMap.hpp"
#include "rm/maxClearance.hpp"
#include "rm/inflate.hpp"
#include "rm/geometry.hpp"

void pretty_print(Point point)
{
    std::cout << "Point(" << point.x << ", " << point.y << ")";
}

void pretty_print(Polygon poly)
{
    std::cout << "Polygon {";
    for (Point p : poly)
    {
        pretty_print(p);
        std::cout << " ";
    }
    std::cout << "}";
}

int main()
{
    std::vector<Polygon> obstacles;
    obstacles.push_back(Polygon{Point(0.5, 0.5), Point(0.5, 0.6),
                                Point(0.6, 0.6), Point(0.6, 0.5)});
    obstacles.push_back(Polygon{Point(0.20, 0.15), Point(0.10, 0.25),
                                Point(0.20, 0.35), Point(0.30, 0.25)});

    Polygon borders {Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)};

    float offset = 0.05f;
    std::vector<Polygon> infObstacles = rm::inflate(obstacles, offset, true);
    Polygon infBorders = rm::inflate(std::vector<Polygon>{borders}, -offset).back();
    
    std::cout << "number of inflated obstacles: " << infObstacles.size() << std::endl;
    for (Polygon obst : infObstacles){
        pretty_print(obst);
        std::cout << std::endl;
    }
    pretty_print(infBorders);
    std::cout << std::endl;

    rm::RoadMap rm = rm::maxClearance(infObstacles, infBorders);


    std::cout << "number of nodes in rm: " << rm.getNodeCount() << std::endl;
    std::cout << "number of global nodes: " << rm::RoadMap::Node::getTotalNodeCount() << std::endl;
    for (size_t i = 0; i < rm.getNodeCount(); i++)
    {
        auto node = rm.getNodeAt(i);
        std::cout << "Node_" << node.getID() << std::endl
                  << " X: " << node.getX() << std::endl
                  << " Y: " << node.getY() << std::endl;
    }
}
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

    Polygon borders{Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)};

    float offset = 0.05f;
    std::vector<Polygon> infObstacles = rm::inflate(obstacles, offset, true);
    Polygon infBorders = rm::inflate(std::vector<Polygon>{borders}, -offset).back();

    std::cout << "number of inflated obstacles: " << infObstacles.size() << std::endl;
    for (Polygon obst : infObstacles)
    {
        pretty_print(obst);
        std::cout << std::endl;
    }
    pretty_print(infBorders);
    std::cout << std::endl;

    rm::RoadMap rm = rm::maxClearance(infObstacles, infBorders);

    size_t bypass_edges = rm.bypass(0.1f);

    std::cout << "number of nodes in rm: " << rm.getNodeCount() << std::endl;
    std::cout << "number of bypass edges: " << bypass_edges << std::endl;
    for (size_t i = 0; i < rm.getNodeCount(); i++)
    {
        auto node = rm.getNode(i);
        std::cout << "plot(" << node.getX() << ", " << node.getY() << ", 'b*')" << std::endl;
        for (size_t j = 0; j < node.getConnectedCount(); j++)
        {
            auto other = node.getConnected(j);
            std::cout << "plot([" << node.getX() << ", " << other.getX() << "], ["<< node.getY() << ", " << other.getY() << "], 'b-')" << std::endl;
        }
        
    }
    auto tic = std::chrono::high_resolution_clock::now();
    auto n_connections = rm.build(8, 10.0f, infObstacles, infBorders);
    auto toc = std::chrono::high_resolution_clock::now();

    std::chrono::duration<float, std::milli> duration = toc - tic;
    std::cout << "Generated " << n_connections << " dubins connections in " << duration.count() << " milliseconds." << std::endl;
}
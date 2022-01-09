#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>

#include "nav/dijkstra.hpp"
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

    rm::RoadMap rm = rm::maxClearance(infObstacles, infBorders);
    rm.build(8, 100.0f, infObstacles, infBorders);

    auto &source = rm.getNode(0).getPose(0);
    auto &goal = rm.getNode(rm.getNodeCount() - 1).getPose(0);

    auto tic = std::chrono::high_resolution_clock::now();
    auto path = nav::dijkstraShortestPath(source, goal);
    auto toc = std::chrono::high_resolution_clock::now();

    std::chrono::duration<float, std::milli> duration = toc - tic;
    std::cout << "Path found in " << duration.count() << " milliseconds." << std::endl;
    std::cout << "Traversing " << path.size() - 1 << " intermediate nodes." << std::endl;

    std::ofstream matfile;
    matfile.open("plot_dijkstra_path.m");
    matfile.clear();
    float d_offset = 0.0f;
    float step = 0.01f;
    std::vector<Pose> discr_path;
    for (const auto &connection : path)
    {
        /*auto &from = connection->from->getNode();
        auto &to = connection->to->getNode();
        matfile << "plot([" << from.getX() << ", " << to.getX() << "], [" << from.getY() << ", " << to.getY() << "], 'r-')" << std::endl;*/
        dubins::discretizeCurve(connection->path, step, d_offset, discr_path);
    }
    for (const auto &pose : discr_path)
    {
        matfile << "plot(" << pose.x << ", " << pose.y << ", 'r*')" << std::endl;
    }
    matfile.close();
}
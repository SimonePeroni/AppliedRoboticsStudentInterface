#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>

#include "nav/dijkstra.hpp"
#include "rm/RoadMap.hpp"
#include "rm/maxClearance.hpp"
#include "rm/inflate.hpp"
#include "rm/geometry.hpp"
#include "rm/visibility.hpp"

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

    float kmax = 50.0f;
    //rm::RoadMap rm = rm::maxClearance(infObstacles, infBorders);

    std::vector<Point> vertices;
    rm::makeVisibilityNodes(obstacles, borders, offset * 1.5f, vertices);
    rm::RoadMap rm;
    rm::visibility(rm, vertices, infObstacles, infBorders);

    //rm.bypass(0.1, true);
    //rm.bypass(0.1, false);
    rm.build(8, kmax, infObstacles, infBorders);

    //auto &source = rm.getNode(0).getPose(0);
    //auto &goal = rm.getNode(rm.getNodeCount() - 1).getPose(0);

    auto &source = rm.addStartPose(Point(0.5, 0.9), 0.0f, 50, kmax, infObstacles, infBorders);
    auto &goal = rm.addGoalPose(Point(0.1, 0.15), 0.0f, 50, kmax, infObstacles, infBorders);

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
        dubins::discretizeCurve(connection->path, step, d_offset, discr_path);
    }
    for (size_t i = 0; i < rm.getNodeCount(); i++)
    {
        auto node = rm.getNode(i);
        matfile << "plot(" << node.getX() << ", " << node.getY() << ", 'bo')" << std::endl;
        for (size_t j = 0; j < node.getConnectedCount(); j++)
        {
            auto other = node.getConnected(j);
            matfile << "plot([" << node.getX() << ", " << other.getX() << "], ["<< node.getY() << ", " << other.getY() << "], 'b--')" << std::endl;
        }
        
    }
    for (const auto &pose : discr_path)
    {
        matfile << "plot(" << pose.x << ", " << pose.y << ", 'r.')" << std::endl;
    }
    matfile.close();
}
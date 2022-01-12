#include "nav/NavMap.hpp"
#include "utils/timer.hpp"
#include "rm/RoadMap.hpp"
#include "rm/visibility.hpp"
#include "rm/inflate.hpp"

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

    rm.build(8, kmax, infObstacles, infBorders);

    auto &source = rm.addStartPose(Point(0.5, 0.9), 0.0f, 50, kmax, infObstacles, infBorders);
    auto &goal = rm.addGoalPose(Point(0.1, 0.15), 0.0f, 50, kmax, infObstacles, infBorders);

    utils::Timer t;

    nav::NavMap nm(rm);
    nm.compute(source);

    float runningSum = 0;
    for (size_t i = 0; i < 10; i++)
    {
        nav::NavMap nmx(rm);
        t.tic("nm.compute");
        nmx.compute(source);
        t.toc();
        runningSum += t.millis();
    }

    std::cout << "Avg time: " << runningSum / 10 << std::endl;

    t.tic("nm2 = nm + 5");
    nav::NavMap nm2 = nm + 5;
    t.toc();
    t.tic("nm3 = nm + 5");
    nav::NavMap nm3 = nm + 5;
    t.toc();
    t.tic("nm2-=1");
    nm2 -= 1;
    t.toc();

    runningSum = 0;
    for (size_t i = 0; i < 10; i++)
    {
        t.tic("nm + nm2 + 2");
        nm + nm2 + 2;
        t.toc();
        runningSum += t.millis();
    }

    std::cout << "Avg time: " << runningSum / 10 << std::endl;

    t.tic("nm2.rebuild");
    nm2.rebuild();
    t.toc();
}
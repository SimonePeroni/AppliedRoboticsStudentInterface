#include "rm/visibility.hpp"

#include "rm/geometry.hpp"

namespace rm
{
    void visibility(RoadMap &roadmap, const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        for (size_t i = 0; i < obstacles.size() - 1; i++)
        {
            for (size_t j = i + 1; j < obstacles.size(); j++)
            {
                for (const Point &p0 : obstacles[i])
                {
                    for (const Point &p1 : obstacles[j])
                    {
                        bool visible = true;
                        if (collisionCheck(p0, obstacles[j]) || collisionCheck(p1, obstacles[i]) || collisionCheck(Segment(p0, p1), borders))
                            continue;
                        for (const Polygon &obst : obstacles)
                        {
                            if (collisionCheck(Segment(p0, p1), obst))
                            {
                                visible = false;
                                break;
                            }
                        }

                        if (visible)
                        {
                            auto n0 = roadmap.addNode(p0);
                            auto n1 = roadmap.addNode(p1);
                            roadmap.connect(n0, n1);
                            roadmap.connect(n1, n0);
                        }
                    }
                }
            }
        }
    }
}
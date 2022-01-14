#include "nav/path.hpp"

#include "utils.hpp"

#include <vector>

namespace nav
{
    void discretizePath(const navList &nav_list, float step, std::vector<Pose> &discr_path)
    {
        float offset = 0.0f;
        for (const auto &connection : nav_list)
        {
            if (connection->to == connection->from)
            {
                if (connection == nav_list.back())
                    break;
                Pose pose(discr_path.back().s + step - offset,
                          connection->to->getNode().getX(), connection->to->getNode().getY(),
                          connection->to->getTheta(), 0.0f);
                offset = 0.0f;
                float end_goal = pose.s + connection->path.L;
                while (pose.s + step <= end_goal)
                {
                    discr_path.push_back(pose);
                    pose.s += step;
                }
                delete connection;
                continue;
            }

            dubins::discretizeCurve(connection->path, step, offset, discr_path);
        }
    }

    void truncatePaths(std::vector<Pose> &discr_path1, std::vector<Pose> &discr_path2, float robot_size)
    {
        size_t max_count = discr_path1.size() > discr_path2.size() ? discr_path1.size() : discr_path2.size();
        for (size_t i = 0; i < max_count; i++)
        {
            size_t i1 = i >= discr_path1.size() ? discr_path1.size() - 1 : i;
            size_t i2 = i >= discr_path2.size() ? discr_path2.size() - 1 : i;
            if (std::hypotf(discr_path1[i1].x - discr_path2[i2].x, discr_path1[i1].y - discr_path2[i2].y) < robot_size)
            {
                while (discr_path1.size() > i)
                {
                    discr_path1.pop_back();
                }
                while (discr_path2.size() > i)
                {
                    discr_path2.pop_back();
                }
                return;
            }
        }
    }
}
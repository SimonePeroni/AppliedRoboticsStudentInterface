#include "nav/pursuerEvader.hpp"

#include <vector>
#include <random>

namespace nav
{

    rm::RoadMap::DubinsConnection *create_wait_connection(rm::RoadMap::Node::Orientation *pose, float s)
    {
        dubins::DubinsCurve wait_path;
        wait_path.L = s;
        return new rm::RoadMap::DubinsConnection(pose, pose, wait_path);
    }

    void runGame(const std::vector<nav::NavMap> &nm_e, nav::NavMap &nm_p,
                 const rm::RoadMap::Node::Orientation &source_e, const rm::RoadMap::Node::Orientation &source_p,
                 nav::navList &nav_list_e, nav::navList &nav_list_p)
    {
        float evader_s = 0.0f, pursuer_s = 0.0f;
        int goal;

        // Prepare random number generation
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_int_distribution<int> dist(0, nm_e.size() - 1);
        while (true)
        {
            // Move evader
            while (evader_s <= pursuer_s)
            {
                // Pick random exit
                goal = dist(mt);
                // Plan path to that exit
                auto tmp_path = nm_e[goal].planFrom(nav_list_e.empty() ? source_e : *nav_list_e.back()->to);
                // Add segment to output
                nav_list_e.push_back(tmp_path.front());
                // Update evader_s
                evader_s += nav_list_e.back()->path.L;
                // Check if caught
                if (!nav_list_p.empty())
                {
                    auto e_to = nav_list_e.back()->to->getNode().getID();
                    auto e_from = nav_list_e.back()->from->getNode().getID();
                    auto p_to = nav_list_p.back()->to->getNode().getID();
                    auto p_from = nav_list_p.back()->from->getNode().getID();
                    if (e_to == p_to || (e_to == p_from && e_from == p_to))
                        return;
                }
                // Goal reached?
                if (tmp_path.size() == 1)
                {
                    evader_s = INFINITY;
                    break;
                }
            }
            
            // Move pursuer
            nav::navList tmp_path;
            auto e_best_path = nm_e[goal].planFrom(nav_list_e.empty() ? source_e : *nav_list_e.back()->from);
            if (e_best_path.size() == 1)
            {
                tmp_path = nm_e[goal].planFrom(nav_list_p.empty() ? source_p : *nav_list_p.back()->to);
            }
            else
            {
                // Recompute navmap
                nm_p.compute(nav_list_p.empty() ? source_p : *nav_list_p.back()->to);
                // Intercept evader to current goal
                try
                {
                    tmp_path = nm_p.intercept(e_best_path, e_best_path.front()->path.L - evader_s + pursuer_s);
                }
                catch(const std::exception& e)
                {
                    pursuer_s = INFINITY;
                    continue;
                }
                
            }
            while (pursuer_s < evader_s)
            {
                if (tmp_path.empty())
                {
                    if (nav_list_e.back()->to->getNode().getID() == nav_list_p.back()->to->getNode().getID())
                        return;
                    pursuer_s = evader_s;
                    nav_list_p.push_back(create_wait_connection(nav_list_p.back()->to, evader_s - pursuer_s));
                    continue;
                }
                // Add segment to output
                nav_list_p.push_back(tmp_path.front());
                tmp_path.pop_front();
                // Update pursuer_s
                pursuer_s += nav_list_p.back()->path.L;
            }
        }
    }
}
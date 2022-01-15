#include "nav/pursuerevader.hpp"

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

    bool movePursuer(const float &evader_s, float &pursuer_s, const int &goal, const std::vector<nav::NavMap> &nm_e, nav::NavMap &nm_p,
                     const rm::RoadMap::Node::Orientation &source_e, const rm::RoadMap::Node::Orientation &source_p,
                     const nav::navList &nav_list_e, nav::navList &nav_list_p)
    {
        // Predict evader's path
        nav::navList tmp_path;
        auto e_best_path = nm_e[goal].planFrom(nav_list_e.empty() ? source_e : *nav_list_e.back()->from);

        if (e_best_path.size() == 1)
        {
            // Evader is already heading to a gate, so the pursuer will try to go there too
            tmp_path = nm_e[goal].planFrom(nav_list_p.empty() ? source_p : *nav_list_p.back()->to);
        }
        else
        {
            // Recompute navmap
            nm_p.compute(nav_list_p.empty() ? source_p : *nav_list_p.back()->to);
            try
            {
                // Intercept evader in its path to current goal
                tmp_path = nm_p.intercept(e_best_path, e_best_path.front()->path.L - evader_s + pursuer_s);
            }
            catch (const std::logic_error &e)
            {
                // Pursuer got stuck, let evader complete planning
                pursuer_s = INFINITY;
                return false;
            }
        }
        // Plan moves while evader completes movement to next node
        while (pursuer_s < evader_s)
        {
            if (tmp_path.empty())
            {
                // Planned movement was completed
                // Check if meeting evader next
                if (nav_list_e.back()->to->getNode().getID() == nav_list_p.back()->to->getNode().getID())
                    return true;
                // Otherwise create a waiting task until the evader takes a new move
                pursuer_s = evader_s;
                nav_list_p.push_back(create_wait_connection(nav_list_p.back()->to, evader_s - pursuer_s));
                return false;
            }
            // Add path segment to output
            nav_list_p.push_back(tmp_path.front());
            tmp_path.pop_front();
            // Update pursuer_s
            pursuer_s += nav_list_p.back()->path.L;
        }
        return false;
    }

    bool moveEvader(float &evader_s, const float &pursuer_s, int &goal, const std::vector<nav::NavMap> &nm_e,
                    const rm::RoadMap::Node::Orientation &source_e, nav::navList &nav_list_e, const nav::navList &nav_list_p)
    {

        // Prepare random number generation
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_int_distribution<int> dist(0, nm_e.size() - 1);
        while (evader_s <= pursuer_s)
        {
            // Pick random exit
            goal = dist(mt);
            // Plan path to that exit
            auto tmp_path = nm_e[goal].planFrom(nav_list_e.empty() ? source_e : *nav_list_e.back()->to);
            // Add path segment to output
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
                    return true;
            }
            // Goal reached?
            if (tmp_path.size() == 1)
            {
                evader_s = INFINITY;
                break;
            }
        }
        return false;
    }

    void runGame(const std::vector<nav::NavMap> &nm_e, nav::NavMap &nm_p,
                 const rm::RoadMap::Node::Orientation &source_e, const rm::RoadMap::Node::Orientation &source_p,
                 nav::navList &nav_list_e, nav::navList &nav_list_p)
    {
        float evader_s = 0.0f, pursuer_s = 0.0f;
        int goal;

        // Main loop
        while (true)
        {
            // Move evader
            if (moveEvader(evader_s, pursuer_s, goal, nm_e, source_e, nav_list_e, nav_list_p))
                return;

            /* Goal prediction would go here */

            // Move pursuer
            if (movePursuer(evader_s, pursuer_s, goal, nm_e, nm_p, source_e, source_p, nav_list_e, nav_list_p))
                return;
        }
    }
}
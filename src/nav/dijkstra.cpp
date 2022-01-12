#include <cmath>
#include <set>
#include <utility>
#include <vector>
#include <stdexcept>

#include "nav/dijkstra.hpp"

namespace nav
{
    navList dijkstraShortestPath(const rm::RoadMap::Node::Orientation &source,
                                 const rm::RoadMap::Node::Orientation &goal)
    {
        typedef std::pair<float, const rm::RoadMap::Node::Orientation &> dist_pose;

        rm::RoadMap &roadmap = source.getNode().getRoadMap();

        // source and goal nodes will most often have a different number of poses (most likely 1)
        // so we get the number of poses of an intermediate node
        size_t max_poses_count = source.getConnection(0).to->getNode().getPosesCount();

        std::vector<std::vector<float>> dist(
            roadmap.getNodeCount(),
            std::vector<float>(
                max_poses_count,
                INFINITY));
        std::vector<std::vector<rm::RoadMap::DubinsConnection const *>> from(
            roadmap.getNodeCount(),
            std::vector<rm::RoadMap::DubinsConnection const *>(
                max_poses_count,
                nullptr));

        auto cmp = [](dist_pose a, dist_pose b)
        { return a.first == b.first ? &a.second < &b.second : a < b; };
        std::set<dist_pose, decltype(cmp)> set_dist_pose(cmp);

        dist[source.getNode()][source] = 0.0f;
        set_dist_pose.insert(dist_pose(0.0f, source));

        while (!set_dist_pose.empty())
        {
            dist_pose top = *set_dist_pose.begin();
            set_dist_pose.erase(set_dist_pose.begin());

            auto &current_source_pose = top.second;

            for (size_t i = 0; i < current_source_pose.getConnectionCount(); i++)
            {
                auto &connection = current_source_pose.getConnection(i);
                auto &adj_pose = *connection.to;
                float dist_from_adjpose = connection.path.L;

                // Edge relaxation
                if (dist[adj_pose.getNode()][adj_pose] > dist_from_adjpose + dist[current_source_pose.getNode()][current_source_pose])
                {
                    // Remove from set to avoid duplicates
                    if (dist[adj_pose.getNode()][adj_pose] != INFINITY)
                        set_dist_pose.erase(set_dist_pose.find(dist_pose(dist[adj_pose.getNode()][adj_pose], adj_pose)));
                    // Update distance and shortest connection
                    dist[adj_pose.getNode()][adj_pose] = dist_from_adjpose + dist[current_source_pose.getNode()][current_source_pose];
                    from[adj_pose.getNode()][adj_pose] = &connection;
                    // Add node to set
                    set_dist_pose.insert(dist_pose(dist[adj_pose.getNode()][adj_pose], adj_pose));
                }
            }

            // Stop if goal was reached
            if (&top.second == &goal)
                break;
        }

        // Reconstruct path to goal backwards
        navList path;
        path.push_front(from[goal.getNode()][goal]);
        if (path.front() == nullptr)
            throw std::logic_error("DIJKSTRA - NO EXISTING PATH CONNECTING SOURCE AND GOAL");
        while (path.front()->from != &source)
        {
            path.push_front(from[path.front()->from->getNode().getID()][path.front()->from->getID()]);
        }
        return path;
    }
}
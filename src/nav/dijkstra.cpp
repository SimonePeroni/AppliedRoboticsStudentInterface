#include <limits>

#include "nav/dijkstra.hpp"

namespace nav
{
    navList dijkstraShortestPath(const rm::RoadMap::Node::Orientation &source,
                                 const rm::RoadMap::Node::Orientation &goal)
    {
        typedef std::pair<float, const rm::RoadMap::Node::Orientation &> dist_pose;

        rm::RoadMap &rm = source.getNode().getRoadMap();
        float INF = std::numeric_limits<float>::infinity();
        std::vector<std::vector<float>> dist(
            rm.getNodeCount(),
            std::vector<float>(
                source.getNode().getPosesCount(),
                INF));
        std::set<dist_pose> set_dist_pose;

        dist[source.getNode()][source] = 0.0f;
        set_dist_pose.insert(dist_pose(0.0f, source));

        while (!set_dist_pose.empty())
        {
            dist_pose top = *set_dist_pose.begin();
            set_dist_pose.erase(set_dist_pose.begin());

            auto current_source_pose = top.second;

            for (size_t i = 0; i < current_source_pose.getConnectionCount(); i++)
            {
                auto connection = current_source_pose.getConnection(i);
                auto &adj_pose = *connection.to;
                float dist_from_adjpose = connection.path.L;

                // Edge relaxation
                if (dist[adj_pose.getNode()][adj_pose] > dist_from_adjpose + dist[current_source_pose.getNode()][current_source_pose])
                {
                    if (dist[adj_pose.getNode()][adj_pose] != INF)
                        set_dist_pose.erase(set_dist_pose.find(dist_pose(dist[adj_pose.getNode()][adj_pose], adj_pose)));
                    dist[adj_pose.getNode()][adj_pose] = dist_from_adjpose + dist[current_source_pose.getNode()][current_source_pose];
                    set_dist_pose.insert(dist_pose(dist[adj_pose.getNode()][adj_pose], adj_pose));
                }
            }

            if (top.second == goal)
                break;
        }
    }
}
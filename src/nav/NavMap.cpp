#include "nav/NavMap.hpp"

#include <cmath>
#include <set>
#include <utility>
#include <vector>
#include <stdexcept>

namespace nav
{
    NavMap::NavMap(const rm::RoadMap &roadmap) : _rm(roadmap)
    {
        reset();
    }
    NavMap::NavMap(const rm::RoadMap &roadmap, const std::vector<std::vector<float>> &dist)
        : _rm(roadmap), _dist(dist), _need_computing(false), _need_rebuild(true)
    {
        _from = std::vector<std::vector<rm::RoadMap::DubinsConnection const *>>(
            roadmap.getNodeCount(),
            std::vector<rm::RoadMap::DubinsConnection const *>(
                _dist[0].size(),
                nullptr));
    }

    void NavMap::compute(const rm::RoadMap::Node::Orientation &source)
    {
        typedef std::pair<float, const rm::RoadMap::Node::Orientation &> dist_pose;

        auto cmp = [](dist_pose a, dist_pose b)
        { return a.first == b.first ? &a.second < &b.second : a < b; };
        std::set<dist_pose, decltype(cmp)> set_dist_pose(cmp);

        _dist[source.getNode()][source] = 0.0f;
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
                if (_dist[adj_pose.getNode()][adj_pose] > dist_from_adjpose + _dist[current_source_pose.getNode()][current_source_pose])
                {
                    // Remove from set to avoid duplicates
                    if (_dist[adj_pose.getNode()][adj_pose] != INFINITY)
                        set_dist_pose.erase(set_dist_pose.find(dist_pose(_dist[adj_pose.getNode()][adj_pose], adj_pose)));
                    // Update distance and shortest connection
                    _dist[adj_pose.getNode()][adj_pose] = dist_from_adjpose + _dist[current_source_pose.getNode()][current_source_pose];
                    _from[adj_pose.getNode()][adj_pose] = &connection;
                    // Add node to set
                    set_dist_pose.insert(dist_pose(_dist[adj_pose.getNode()][adj_pose], adj_pose));
                }
            }
        }

        _need_computing = false;
        _need_rebuild = false;
    }

    void NavMap::rebuild()
    {
        if (!_need_rebuild)
            return;

        for (size_t n_id = 0; n_id < _rm.getNodeCount(); n_id++)
        {
            auto &node = _rm.getNode(n_id);
            for (size_t p_id = 0; p_id < node.getPosesCount(); p_id++)
            {
                auto &pose = node.getPose(p_id);
                for (size_t c_id = 0; c_id < pose.getConnectionCount(); c_id++)
                {
                    const auto &connection = pose.getConnection(c_id);
                    const auto &connected = *connection.to;
                    auto current_from = _from[connected.getNode()][connected];
                    if (current_from == nullptr || _dist[current_from->from->getNode()][*current_from->from] + current_from->path.L > _dist[node][pose] + connection.path.L)
                    {
                        _from[connected.getNode()][connected] = &connection;
                    }
                }
            }
        }
        _need_rebuild = false;
    }

    void NavMap::reset()
    {
        size_t pose_count = _rm.getNode(0).getPosesCount();
        _dist = std::vector<std::vector<float>>(
            _rm.getNodeCount(),
            std::vector<float>(
                pose_count,
                INFINITY));
        _from = std::vector<std::vector<rm::RoadMap::DubinsConnection const *>>(
            _rm.getNodeCount(),
            std::vector<rm::RoadMap::DubinsConnection const *>(
                pose_count,
                nullptr));
        _need_computing = true;
        _need_rebuild = false;
    }

    navList NavMap::planTo(const rm::RoadMap::Node &goal) const
    {
        size_t best_id = _dist[goal][0];
        for (size_t p_id = 1; p_id < goal.getPosesCount(); p_id++)
        {
            if (_dist[goal][p_id] < _dist[goal][best_id])
                best_id = p_id;
        }
        return planTo(goal.getPose(best_id));
    }
    navList NavMap::planTo(const rm::RoadMap::Node::Orientation &goal) const
    {
        navList path;
        path.push_front(_from[goal.getNode()][goal]);
        if (path.front() == nullptr)
            throw std::logic_error("DIJKSTRA - NO EXISTING PATH CONNECTING SOURCE AND GOAL");
        while (_from[path.front()->from->getNode()][*path.front()->from] != nullptr)
        {
            path.push_front(_from[path.front()->from->getNode()][*path.front()->from]);
        }
        return path;
    }

    NavMap NavMap::operator+(const NavMap &other) const
    {
        std::vector<std::vector<float>> out_dist;
        for (size_t n_id = 0; n_id < _dist.size(); n_id++)
        {
            out_dist.push_back(std::vector<float>());
            for (size_t p_id = 0; p_id < _dist[n_id].size(); p_id++)
            {
                out_dist[n_id].push_back(_dist[n_id][p_id] + other._dist[n_id][p_id]);
            }
        }
        return NavMap(_rm, out_dist);
    }
    NavMap &NavMap::operator+=(const NavMap &other)
    {
        for (size_t n_id = 0; n_id < _dist.size(); n_id++)
        {
            for (size_t p_id = 0; p_id < _dist[n_id].size(); p_id++)
            {
                _dist[n_id][p_id] += other._dist[n_id][p_id];
            }
        }
        return *this;
    }
    NavMap NavMap::operator-(const NavMap &other) const
    {
        std::vector<std::vector<float>> out_dist;
        for (size_t n_id = 0; n_id < _dist.size(); n_id++)
        {
            out_dist.push_back(std::vector<float>());
            for (size_t p_id = 0; p_id < _dist[n_id].size(); p_id++)
            {
                out_dist[n_id].push_back(_dist[n_id][p_id] - other._dist[n_id][p_id]);
            }
        }
        return NavMap(_rm, out_dist);
    }
    NavMap &NavMap::operator-=(const NavMap &other)
    {
        for (size_t n_id = 0; n_id < _dist.size(); n_id++)
        {
            for (size_t p_id = 0; p_id < _dist[n_id].size(); p_id++)
            {
                _dist[n_id][p_id] -= other._dist[n_id][p_id];
            }
        }
        return *this;
    }
    NavMap NavMap::operator-() const
    {
        std::vector<std::vector<float>> out_dist;
        for (size_t n_id = 0; n_id < _dist.size(); n_id++)
        {
            out_dist.push_back(std::vector<float>());
            for (size_t p_id = 0; p_id < _dist[n_id].size(); p_id++)
            {
                out_dist[n_id].push_back(-_dist[n_id][p_id]);
            }
        }
        return NavMap(_rm, out_dist);
    }
    NavMap NavMap::operator+(const float &other) const
    {
        std::vector<std::vector<float>> out_dist;
        for (size_t n_id = 0; n_id < _dist.size(); n_id++)
        {
            out_dist.push_back(std::vector<float>());
            for (size_t p_id = 0; p_id < _dist[n_id].size(); p_id++)
            {
                out_dist[n_id].push_back(_dist[n_id][p_id] + other);
            }
        }
        return NavMap(_rm, out_dist);
    }
    NavMap &NavMap::operator+=(const float &other)
    {
        for (size_t n_id = 0; n_id < _dist.size(); n_id++)
        {
            for (size_t p_id = 0; p_id < _dist[n_id].size(); p_id++)
            {
                _dist[n_id][p_id] += other;
            }
        }
        return *this;
    }
    NavMap NavMap::operator-(const float &other) const
    {
        std::vector<std::vector<float>> out_dist;
        for (size_t n_id = 0; n_id < _dist.size(); n_id++)
        {
            out_dist.push_back(std::vector<float>());
            for (size_t p_id = 0; p_id < _dist[n_id].size(); p_id++)
            {
                out_dist[n_id].push_back(_dist[n_id][p_id] - other);
            }
        }
        return NavMap(_rm, out_dist);
    }
    NavMap &NavMap::operator-=(const float &other)
    {
        for (size_t n_id = 0; n_id < _dist.size(); n_id++)
        {
            for (size_t p_id = 0; p_id < _dist[n_id].size(); p_id++)
            {
                _dist[n_id][p_id] -= other;
            }
        }
        return *this;
    }
}

nav::NavMap operator+(float a, const nav::NavMap &b)
{
    return b + a;
}
nav::NavMap operator-(float a, const nav::NavMap &b)
{
    return b - a;
}
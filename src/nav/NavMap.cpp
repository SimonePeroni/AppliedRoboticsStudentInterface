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
        _need_computing = true;
    }

    void NavMap::compute(const rm::RoadMap::Node::Orientation &source)
    {
        if (_reverse)
            setReverse(false);
        reset();

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
                    _connection[adj_pose.getNode()][adj_pose] = &connection;
                    // Add node to set
                    set_dist_pose.insert(dist_pose(_dist[adj_pose.getNode()][adj_pose], adj_pose));
                }
            }
        }

        _need_computing = false;
        _reverse = false;
    }

    void NavMap::computeReverse(const rm::RoadMap::Node::Orientation &goal)
    {
        if (!_reverse)
            setReverse(true);
        reset();

        typedef std::pair<float, const rm::RoadMap::Node::Orientation &> dist_pose;

        auto cmp = [](dist_pose a, dist_pose b)
        { return a.first == b.first ? &a.second < &b.second : a < b; };
        std::set<dist_pose, decltype(cmp)> set_dist_pose(cmp);

        _dist[goal.getNode()][goal] = 0.0f;
        set_dist_pose.insert(dist_pose(0.0f, goal));

        while (!set_dist_pose.empty())
        {
            dist_pose top = *set_dist_pose.begin();
            set_dist_pose.erase(set_dist_pose.begin());

            auto &current_source_pose = top.second;

            for (size_t i = 0; i < current_source_pose.getFromConnectionCount(); i++)
            {
                auto &connection = current_source_pose.getFromConnection(i);
                auto &adj_pose = *connection.from;
                float dist_from_adjpose = connection.path.L;

                // Edge relaxation
                if (_dist[adj_pose.getNode()][adj_pose] < _dist[current_source_pose.getNode()][current_source_pose] - dist_from_adjpose)
                {
                    // Remove from set to avoid duplicates
                    if (_dist[adj_pose.getNode()][adj_pose] != -INFINITY)
                        set_dist_pose.erase(set_dist_pose.find(dist_pose(-_dist[adj_pose.getNode()][adj_pose], adj_pose)));
                    // Update distance and shortest connection
                    _dist[adj_pose.getNode()][adj_pose] = _dist[current_source_pose.getNode()][current_source_pose] - dist_from_adjpose;
                    _connection[adj_pose.getNode()][adj_pose] = &connection;
                    // Add node to set
                    set_dist_pose.insert(dist_pose(-_dist[adj_pose.getNode()][adj_pose], adj_pose));
                }
            }
        }

        _need_computing = false;
        _reverse = true;
    }

    void NavMap::reset()
    {
        size_t pose_count = _rm.getNode(0).getPosesCount();
        _dist = std::vector<std::vector<float>>(
            _rm.getNodeCount(),
            std::vector<float>(
                pose_count,
                _reverse ? -INFINITY : INFINITY));
        _connection = std::vector<std::vector<rm::RoadMap::DubinsConnection const *>>(
            _rm.getNodeCount(),
            std::vector<rm::RoadMap::DubinsConnection const *>(
                pose_count,
                nullptr));
        _need_computing = true;
    }

    bool NavMap::isReverse() const
    {
        return _reverse;
    }

    void NavMap::setReverse(bool reverse)
    {
        if (reverse == _reverse)
            return;
        _reverse = !_reverse;
    }

    float NavMap::getValue(const rm::RoadMap::Node::Orientation &pose) const
    {
        if (_need_computing)
            return _reverse ? -INFINITY : INFINITY;
        return _dist[pose.getNode()][pose];
    }

    float NavMap::getValue(const rm::RoadMap::Node &node) const
    {
        if (_need_computing)
            return _reverse ? -INFINITY : INFINITY;
        float best = INFINITY;
        for (size_t i = 0; i < node.getPosesCount(); i++)
        {
            if (_dist[node][i] < best)
                best = _dist[node][i];
        }
        return best;
    }

    navList NavMap::intercept(const navList &path, float offset) const
    {
        if (_reverse)
            throw std::logic_error("NAVMAP - INTERCEPT ONLY AVAILABLE FOR DIRECT MAPS");
        float running_length = -offset;
        for (const auto &connection : path)
        {
            running_length += connection->path.L;
            float advantage = running_length - getValue(connection->to->getNode());
            if (advantage >= 0.0f)
                return planTo(connection->to->getNode());
        }
        // If there was no chance of intercepting, go to last node
        return planTo(path.back()->to->getNode());
    }

    navList NavMap::planTo(const rm::RoadMap::Node &goal) const
    {
        if (_need_computing)
            throw std::logic_error("NAVMAP - COMPUTATION REQUIRED BEFORE PLANNING");
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
        if (_need_computing)
            throw std::logic_error("NAVMAP - COMPUTATION REQUIRED BEFORE PLANNING");
        if (_reverse)
            throw std::logic_error("NAVMAP - WRONG PLANNING DIRECTION");
        navList path;
        path.push_front(_connection[goal.getNode()][goal]);
        if (path.front() == nullptr)
            throw std::logic_error("NAVMAP - NO EXISTING PATH CONNECTING SOURCE AND GOAL");
        while (_connection[path.front()->from->getNode()][*path.front()->from] != nullptr)
        {
            path.push_front(_connection[path.front()->from->getNode()][*path.front()->from]);
        }
        return path;
    }

    navList NavMap::planFrom(const rm::RoadMap::Node::Orientation &source) const
    {
        if (_need_computing)
            throw std::logic_error("NAVMAP - COMPUTATION REQUIRED BEFORE PLANNING");
        if (!_reverse)
            throw std::logic_error("NAVMAP - WRONG PLANNING DIRECTION");
        navList path;
        path.push_back(_connection[source.getNode()][source]);
        if (path.front() == nullptr)
            throw std::logic_error("NAVMAP - NO EXISTING PATH CONNECTING SOURCE AND GOAL");
        while (_connection[path.back()->to->getNode()][*path.back()->to] != nullptr)
        {
            path.push_back(_connection[path.back()->to->getNode()][*path.back()->to]);
        }
        return path;
    }
}
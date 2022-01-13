#pragma once

#include <vector>
#include <deque>

#include "rm/RoadMap.hpp"

namespace nav
{
    typedef std::deque<rm::RoadMap::DubinsConnection const *> navList;

    class NavMap
    {
    private:
        std::vector<std::vector<float>> _dist;
        std::vector<std::vector<rm::RoadMap::DubinsConnection const *>> _connection;
        const rm::RoadMap &_rm;
        bool _need_computing;
        bool _reverse;

        NavMap(const rm::RoadMap &roadmap, const std::vector<std::vector<float>> &dist);

    public:
        NavMap(const rm::RoadMap &roadmap);

        void compute(const rm::RoadMap::Node::Orientation &source);

        void computeReverse(const rm::RoadMap::Node::Orientation &goal);

        void reset();

        navList planTo(const rm::RoadMap::Node &goal) const;
        navList planTo(const rm::RoadMap::Node::Orientation &goal) const;

        navList planFrom(const rm::RoadMap::Node::Orientation &source) const;

        navList intercept(const navList &path, float offset = 0.0f) const;

        bool isReverse() const;
        void setReverse(bool reverse);
        float getValue(const rm::RoadMap::Node::Orientation &pose) const;
        float getValue(const rm::RoadMap::Node &node) const;
    };
}
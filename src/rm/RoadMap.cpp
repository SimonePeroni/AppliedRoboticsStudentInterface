#include "rm/RoadMap.hpp"

namespace rm
{
    // RoadMap
    size_t RoadMap::addNode(Point pos)
    {
        size_t id = Node::create(pos);
        if (id == Node::getTotalNodeCount() - 1)
            _nodes.push_back(&Node::getByID(id));
        return id;
    }

    // Node
    size_t RoadMap::Node::create(Point pos)
    {
        // Check if node exists
        for (const auto &n : _all_nodes)
        {
            if (n.getX() == pos.x && n.getY() == pos.y)
                return n.getID();
        }

        size_t id = _all_nodes.size();
        _all_nodes.push_back(Node(id, pos));
        return id;
    }
    size_t RoadMap::Node::getTotalNodeCount() { return _all_nodes.size(); }

    RoadMap::Node::Node(size_t id, Point pos) : _pos(pos), _id(id) {}

    float RoadMap::Node::getX() const { return _pos.x; }

    float RoadMap::Node::getY() const { return _pos.y; }

    size_t RoadMap::Node::getID() const { return _id; }

    size_t RoadMap::Node::getPosesCount() const { return _poses.size(); }

    RoadMap::Node &RoadMap::Node::getByID(size_t id) { return _all_nodes.at(id); }
}
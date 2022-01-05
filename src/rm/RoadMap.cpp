#include "rm/RoadMap.hpp"

#include <cmath>

namespace rm
{
    // RoadMap
    RoadMap::node_id RoadMap::addNode(Point pos)
    {
        node_id id = Node::create(pos);
        for (node_id existing : _nodes)
        {
            if (existing == id)
                return id;
        }
        _nodes.push_back(id);
        return id;
    }

    void RoadMap::connect(node_id fromID, node_id toID)
    {
        _edges.push_back(Edge(fromID, toID));
    }

    size_t RoadMap::getNodeCount() const { return _nodes.size(); }

    RoadMap::Node &RoadMap::getNodeAt(size_t index) const { return Node::getByID(_nodes[index]); }

    void RoadMap::build(unsigned int orientationsPerNode)
    {
        // Generate poses for each node
        for (RoadMap::node_id id : _nodes)
        {
            Node node = Node::getByID(id);
            node.clearPoses();
            float theta = 2 * M_PI / orientationsPerNode;
            for (unsigned int i = 0; i < orientationsPerNode; i++)
            {
                node.addPose(theta * i);
            }
        }

        // Try to connect each pose of a node to each pose of another connected node
        for (RoadMap::node_id id : _nodes)
        {
            Node node = Node::getByID(id);
            
            
        }
    }

    // Edge
    RoadMap::Edge::Edge(node_id from, node_id to) : _from(from), _to(to)
    {
        Node::getByID(from).connectTo(to);
    }

    RoadMap::node_id RoadMap::Edge::getFromID() const { return _from; }
    RoadMap::node_id RoadMap::Edge::getToID() const { return _to; }
    RoadMap::Node &RoadMap::Edge::getFromNode() const { return Node::getByID(_from); }
    RoadMap::Node &RoadMap::Edge::getToNode() const { return Node::getByID(_to); }

    // Node
    std::vector<RoadMap::Node> RoadMap::Node::_all_nodes;

    RoadMap::node_id RoadMap::Node::create(Point pos)
    {
        // Check if node exists
        for (const auto &n : _all_nodes)
        {
            if (n.getX() == pos.x && n.getY() == pos.y)
                return n.getID();
        }

        node_id id = _all_nodes.size();
        _all_nodes.push_back(Node(id, pos));
        return id;
    }
    size_t RoadMap::Node::getTotalNodeCount() { return _all_nodes.size(); }
    RoadMap::Node::Node(node_id id, Point pos) : _pos(pos), _id(id) {}
    float RoadMap::Node::getX() const { return _pos.x; }
    float RoadMap::Node::getY() const { return _pos.y; }
    RoadMap::node_id RoadMap::Node::getID() const { return _id; }
    size_t RoadMap::Node::getPosesCount() const { return _poses.size(); }
    RoadMap::Node &RoadMap::Node::getByID(RoadMap::node_id id) { return _all_nodes.at(id); }
    void RoadMap::Node::clearPoses() { _poses.clear(); }

    size_t RoadMap::Node::addPose(float theta)
    {
        _poses.push_back(Orientation(this, theta));
    }

    void RoadMap::Node::connectTo(node_id to)
    {
        if (to == _id)
            return;
        for (auto id : _connected)
            if (id == to)
                return;

        _connected.push_back(to);
    }

    RoadMap::Node::operator node_id() const
    {
        return _id;
    }

    // Node::Orientation
    RoadMap::Node::Orientation::Orientation(Node *parent, float theta) : _parent(parent), _theta(theta) {}
}
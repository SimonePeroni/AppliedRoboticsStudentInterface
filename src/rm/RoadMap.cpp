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

    unsigned long RoadMap::build(unsigned int orientationsPerNode, float const &kmax, const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        unsigned long n_connections = 0L;
        // Generate poses for each node
        for (RoadMap::node_id id : _nodes)
        {
            Node &node = Node::getByID(id);
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
            Node &node = Node::getByID(id);

            //iterate over connected nodes
            for (size_t other_idx = 0; other_idx < node.getConnectedCount(); other_idx++)
            {
                Node &other = Node::getByID(node.getConnected(other_idx));

                //iterate over poses
                for (size_t pose_idx = 0; pose_idx < node.getPosesCount(); pose_idx++)
                {
                    Node::Orientation &pose = node.getPose(pose_idx);

                    //iterate over poses of connected node
                    for (size_t pose_other_idx = 0; pose_other_idx < other.getPosesCount(); pose_other_idx++)
                    {
                        Node::Orientation &pose_other = other.getPose(pose_other_idx);

                        if (pose.connect(pose_other, kmax, obstacles, borders))
                            n_connections++;
                    }
                }
            }
        }
        return n_connections;
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
    RoadMap::Node::Orientation &RoadMap::Node::getPose(size_t index) { return _poses[index]; }
    size_t RoadMap::Node::getConnectedCount() const { return _connected.size(); }
    RoadMap::Node &RoadMap::Node::getConnected(size_t index) { return getByID(_connected[index]); }

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

    bool RoadMap::Node::Orientation::connect(Orientation &other, float const &kmax, const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        dubins::DubinsCurve curve;
        dubins::Pose2D start, end;
        start.x = _parent->getX();
        start.y = _parent->getY();
        start.theta = _theta;
        end.x = other._parent->getX();
        end.y = other._parent->getY();
        end.theta = other._theta;
        bool success = dubins::findShortestPath(curve, start, end, kmax, obstacles, borders);
        if (success)
        {
            _connections.push_back(RoadMap::DubinsConnection(this, &other, curve));
            return true;
        }
        return false;
    }

    float RoadMap::Node::Orientation::getTheta() const { return _theta; }

    RoadMap::Node &RoadMap::Node::Orientation::getNode() const { return *_parent; }
}
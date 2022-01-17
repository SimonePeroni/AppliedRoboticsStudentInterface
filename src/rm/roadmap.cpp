#include "rm/roadmap.hpp"

#include "rm/geometry.hpp"

#include <cmath>
#include <utility>
#include <set>
#include <limits>
#include <stdexcept>

namespace rm
{
    // RoadMap
    RoadMap::node_id RoadMap::addNode(Point pos)
    {
        // Check if node exists
        for (const auto &n : _nodes)
        {
            if (n.getX() == pos.x && n.getY() == pos.y)
                return n.getID();
        }

        node_id id = _nodes.size();
        _nodes.push_back(Node(this, id, pos));
        return id;
    }

    bool RoadMap::connect(node_id fromID, node_id toID)
    {
        return _nodes[fromID].connectTo(toID);
    }

    size_t RoadMap::getNodeCount() const { return _nodes.size(); }

    RoadMap::Node &RoadMap::getNode(node_id id) { return _nodes[id]; }

    const RoadMap::Node &RoadMap::getNode(node_id id) const { return _nodes[id]; }

    std::vector<RoadMap::node_id> RoadMap::findKClosest(Point pos, int k, node_id skip)
    {
        typedef std::pair<float, node_id> dist_node;
        std::set<dist_node> dist_nodes;
        for (Node &node : _nodes)
        {
            if (node.getID() == skip)
                continue;
            float dx = pos.x - node.getX();
            float dy = pos.y - node.getY();
            float sqr_dist = dx * dx + dy * dy;
            dist_nodes.insert(dist_node(sqr_dist, node));
        }
        std::vector<node_id> out;
        for (size_t i = 0; i < k && !dist_nodes.empty(); i++)
        {
            node_id top = dist_nodes.begin()->second;
            out.push_back(top);
            dist_nodes.erase(dist_nodes.begin());
        }
        return out;
    }

    RoadMap::Node::Orientation &RoadMap::addStartPose(Point pos, float angle, int k, float kmax, const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        node_id id = _nodes.size();
        // Check if node exists
        for (const auto &n : _nodes)
        {
            if (n.getX() == pos.x && n.getY() == pos.y)
            {
                id = n.getID();
                break;
            }
        }
        if (id == _nodes.size())
            _nodes.push_back(Node(this, id, pos));

        size_t pose_id = _nodes[id].addPose(angle);
        auto &pose = _nodes[id].getPose(pose_id);

        auto kClosest = findKClosest(pos, k, id);

        bool ok = false;
        for (node_id &cl_id : kClosest)
        {
            Node &closest = _nodes[cl_id];
            for (size_t i = 0; i < closest.getPosesCount(); i++)
            {
                ok = pose.connect(closest.getPose(i), kmax, obstacles, borders) || ok;
            }
        }
        if (!ok)
            throw std::logic_error("ADD START POSE - UNABLE TO CONNECT TO K-CLOSEST NODES");

        return pose;
    }

    RoadMap::Node::Orientation &RoadMap::addGoalPose(Point pos, float angle, int k, float kmax, const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        node_id id = _nodes.size();
        // Check if node exists
        for (const auto &n : _nodes)
        {
            if (n.getX() == pos.x && n.getY() == pos.y)
            {
                id = n.getID();
                break;
            }
        }
        if (id == _nodes.size())
            _nodes.push_back(Node(this, id, pos));

        size_t pose_id = _nodes[id].addPose(angle);
        auto &pose = _nodes[id].getPose(pose_id);

        auto kClosest = findKClosest(pos, k, id);

        bool ok = false;
        for (node_id &cl_id : kClosest)
        {
            Node &closest = _nodes[cl_id];
            for (size_t i = 0; i < closest.getPosesCount(); i++)
            {
                ok = closest.getPose(i).connect(pose, kmax, obstacles, borders) || ok;
            }
        }
        if (!ok)
            throw std::logic_error("ADD GOAL POSE - UNABLE TO CONNECT FROM K-CLOSEST NODES");

        return pose;
    }

    unsigned long RoadMap::build(unsigned int orientationsPerNode, float const &kmax, const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        unsigned long n_connections = 0L;
        // Generate poses for each node
        for (RoadMap::node_id id : _nodes)
        {
            Node &node = _nodes[id];
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
            Node &node = _nodes[id];

            //iterate over connected nodes
            for (size_t other_idx = 0; other_idx < node.getConnectedCount(); other_idx++)
            {
                Node &other = node.getConnected(other_idx);

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

    // Node
    RoadMap::Node::Node(RoadMap *parent, node_id id, Point pos) : _pos(pos), _id(id), _parent(parent) {}
    float RoadMap::Node::getX() const { return _pos.x; }
    float RoadMap::Node::getY() const { return _pos.y; }
    RoadMap::node_id RoadMap::Node::getID() const { return _id; }
    size_t RoadMap::Node::getPosesCount() const { return _poses.size(); }
    void RoadMap::Node::clearPoses() { _poses.clear(); }
    RoadMap::Node::Orientation &RoadMap::Node::getPose(size_t index) { return _poses[index]; }
    const RoadMap::Node::Orientation &RoadMap::Node::getPose(size_t index) const { return _poses[index]; }
    size_t RoadMap::Node::getConnectedCount() const { return _connected.size(); }
    RoadMap::Node &RoadMap::Node::getConnected(size_t index) { return _parent->getNode(_connected[index]); }
    RoadMap &RoadMap::Node::getRoadMap() { return *_parent; }

    size_t RoadMap::Node::addPose(float theta)
    {
        size_t id = _poses.size();
        _poses.push_back(Orientation(this, id, theta));
        return id;
    }

    bool RoadMap::Node::connectTo(node_id to)
    {
        if (to == _id)
            return false;
        for (auto id : _connected)
            if (id == to)
                return false;

        _connected.push_back(to);
        return true;
    }

    RoadMap::Node::operator node_id() const
    {
        return _id;
    }

    // Node::Orientation
    RoadMap::Node::Orientation::Orientation(Node *parent, size_t id, float theta) : _parent(parent), _id(id), _theta(theta) {}

    bool RoadMap::Node::Orientation::connect(Orientation &other, float const &kmax, const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        std::set<dubins::DubinsCurve> curves;
        dubins::Pose2D start, end;
        start.x = _parent->getX();
        start.y = _parent->getY();
        start.theta = _theta;
        end.x = other._parent->getX();
        end.y = other._parent->getY();
        end.theta = other._theta;
        dubins::findPaths(curves, start, end, kmax);
        while (!curves.empty())
        {
            bool collision = borders.empty() ? false : collisionCheck(*curves.begin(), borders);
            if (!collision)
            {
                for (const auto &obst : obstacles)
                {
                    if (collisionCheck(*curves.begin(), obst))
                    {
                        collision = true;
                        break;
                    }
                }
            }
            if (collision)
            {
                curves.erase(curves.begin());
                continue;
            }
            
            _connections.push_back(RoadMap::DubinsConnection(this, &other, *curves.begin()));
            other._from.push_back(_connections.back());
            return true;
        }
        return false;
    }

    float RoadMap::Node::Orientation::getTheta() const { return _theta; }
    size_t RoadMap::Node::Orientation::getID() const { return _id; }
    RoadMap::Node &RoadMap::Node::Orientation::getNode() const { return *_parent; }
    size_t RoadMap::Node::Orientation::getConnectionCount() const { return _connections.size(); }
    size_t RoadMap::Node::Orientation::getFromConnectionCount() const { return _from.size(); }
    rm::RoadMap::DubinsConnection &RoadMap::Node::Orientation::getConnection(size_t index) { return _connections[index]; };
    const rm::RoadMap::DubinsConnection &RoadMap::Node::Orientation::getConnection(size_t index) const { return _connections[index]; };
    const rm::RoadMap::DubinsConnection &RoadMap::Node::Orientation::getFromConnection(size_t index) const { return _from[index]; };
    RoadMap::Node::Orientation::operator std::size_t() const { return _id; }
}
#pragma once

#include <vector>

#include "utils.hpp"
#include "dubins/dubins.hpp"

namespace rm
{
    class RoadMap
    {
    public:
        struct DubinsConnection;
        typedef size_t node_id;

        class Node
        {
        public:
            class Orientation
            {
            private:
                float _theta;
                Node *_parent;
                std::vector<rm::RoadMap::DubinsConnection> _connections;

            public:
                Orientation(Node *parent, float theta);

                bool connect(Orientation &other, float const &kmax, const std::vector<Polygon> &obstacles, const Polygon &borders);

                float getTheta() const;
                Node &getNode() const;
                size_t getConnectionCount() const;
                rm::RoadMap::DubinsConnection &getConnection(size_t index);
            };

        private:
            static std::vector<Node> _all_nodes;

            node_id _id;
            Point _pos;
            std::vector<Orientation> _poses;
            std::vector<node_id> _connected;

        public:
            static node_id create(Point pos);
            static Node &getByID(node_id id);
            static size_t getTotalNodeCount();

            float getX() const;
            float getY() const;
            node_id getID() const;
            size_t getPosesCount() const;
            Orientation &getPose(size_t index);
            size_t getConnectedCount() const;
            Node &getConnected(size_t index);
            void clearPoses();
            size_t addPose(float theta);

            void connectTo(node_id to);

            operator node_id() const;

        private:
            Node(node_id id, Point pos);
        }; // Node

        class Edge
        {
        private:
            node_id _from;
            node_id _to;

        public:
            Edge(node_id from, node_id to);

            node_id getFromID() const;
            node_id getToID() const;
            Node &getFromNode() const;
            Node &getToNode() const;
        }; // Edge

        struct DubinsConnection
        {
            Node::Orientation *from;
            Node::Orientation *to;
            dubins::DubinsCurve path;

            inline DubinsConnection(Node::Orientation *from, Node::Orientation *to,
                                    dubins::DubinsCurve path) : from(from), to(to), path(path) {}
        }; // DubinsConnection

    private:
        std::vector<node_id> _nodes;
        std::vector<Edge> _edges;

    public:
        node_id addNode(Point pos);
        void connect(node_id fromID, node_id toID);
        unsigned long build(unsigned int orientationsPerNode, float const &kmax, const std::vector<Polygon> &obstacles, const Polygon &borders);

        size_t getNodeCount() const;
        Node &getNodeAt(size_t index) const;
    }; // RoadMap
}
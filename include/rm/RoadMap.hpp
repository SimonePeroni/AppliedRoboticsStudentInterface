#pragma once

#include <vector>

#include "utils.hpp"

namespace rm
{
    class RoadMap
    {
    public:
        typedef size_t node_id;

        class Node
        {
        public:
            class Orientation
            {
            private:
                float _theta;
                Node *_parent;
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
            Orientation &getPose(size_t id);

            void connectTo(node_id to);

            operator node_id() const { return _id; }

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

        class OrientationEdge
        {
        }; // OrientationEdge

    private:
        std::vector<node_id> _nodes;
        std::vector<Edge> _edges;
        std::vector<OrientationEdge *> _orientEdge;

    public:
        node_id addNode(Point pos);
        void connect(node_id fromID, node_id toID);

        size_t getNodeCount() const;

        Node &getNodeAt(size_t index) const;
    }; // RoadMap
}
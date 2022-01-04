#pragma once

#include <vector>

#include "utils.hpp"

namespace rm
{
    class RoadMap
    {
    public:
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

            size_t _id;
            Point _pos;
            std::vector<Orientation> _poses;
            RoadMap *_parent;

        public:
            static size_t create(Point pos);
            static Node &getByID(size_t id);
            static size_t getTotalNodeCount();

            float getX() const;
            float getY() const;
            size_t getID() const;
            size_t getPosesCount() const;
            Orientation &getPose(size_t id);

        private:
            Node(size_t id, Point pos);
        };

        class Edge
        {
        };

        class OrientationEdge
        {
        };

    private:
        std::vector<Node *> _nodes;
        std::vector<Edge *> _edges;
        std::vector<OrientationEdge *> _orientEdge;

    public:
        size_t addNode(Point pos);
        void connect(size_t fromID, size_t toID);
    };
}
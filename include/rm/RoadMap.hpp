#pragma once

#include <vector>

#include "utils.hpp"
#include "dubins/dubins.hpp"

namespace rm
{
    /**
     * @brief Roadmap for navigation in a 2-dimensional space.
     * 
     */
    class RoadMap
    {
    public:
        struct DubinsConnection;
        typedef size_t node_id;

        /**
         * @brief Positional node of the roadmap. It stores information about poses that can be achieved in a given position
         * 
         */
        class Node
        {
        public:
            /**
             * @brief Specific orientation on a positional node
             * 
             */
            class Orientation
            {
            private:
                float _theta;
                Node *_parent;
                std::vector<rm::RoadMap::DubinsConnection> _connections;
                size_t _id;

            public:
                /**
                 * @brief Construct a new Orientation object
                 * 
                 * @param parent    Pointer to the Node object this pose belongs to
                 * @param id        Index of this Pose object in the pose list stored in the parent Node object
                 * @param theta     Angle of the pose with respect to the x-axis, measured counter-clockwise
                 */
                Orientation(Node *parent, size_t id, float theta);

                /**
                 * @brief Try to build a connection between two poses by evaluating a Dubins path
                 * 
                 * @param other     Pose to connect to
                 * @param kmax      Maximum curvature of Dubins curves
                 * @param obstacles List of obstacles to perform collision check when evaluating the Dubins path
                 * @param borders   Borders of the arena to perform collision check when evaluating the Dubins path
                 * @return          true if a feasible path was found, false otherwise
                 */
                bool connect(Orientation &other, float const &kmax, const std::vector<Polygon> &obstacles, const Polygon &borders);

                /**
                 * @brief   Get the value of the angle
                 * 
                 * @return  Angle of the pose with respect to the x-axis, measured counter-clockwise
                 */
                float getTheta() const;

                /**
                 * @brief   Get id of the pose
                 * 
                 * @return  Index of this Pose object in the pose list stored in the parent Node object
                 */
                size_t getID() const;

                /**
                 * @brief Get the parent Node object
                 * 
                 * @return Reference of the Node object this pose belongs to
                 */
                Node &getNode() const;

                /**
                 * @brief Get the number of other poses this pose is connected to
                 * 
                 * @return Number of connections
                 */
                size_t getConnectionCount() const;

                /**
                 * @brief       Get a connection starting from this pose by its index
                 * 
                 * @param index Index of the connection in the connection list stored in the pose
                 * @return      Reference to the connection at the given index
                 */
                rm::RoadMap::DubinsConnection &getConnection(size_t index);

                /**
                 * @brief       Get a connection starting from this pose by its index
                 * 
                 * @param index Index of the connection in the connection list stored in the pose
                 * @return      Constant reference to the connection at the given index
                 */
                const rm::RoadMap::DubinsConnection &getConnection(size_t index) const;

                /**
                 * @brief Implicit conversion of the pose to its ID
                 * 
                 * @return ID of the pose
                 */
                operator size_t() const;
            };

        private:
            RoadMap *_parent;
            size_t _idx_in_parent;
            node_id _id;
            Point _pos;
            std::vector<Orientation> _poses;
            std::vector<node_id> _connected;

        public:
            /**
             * @brief           Construct a new Node object
             * 
             * @param parent    Pointer to the RoadMap object this pose belongs to
             * @param id        Index of this Node object in the node list stored in the parent RoadMap object
             * @param pos       2-dimensional position of the node
             */
            Node(RoadMap *parent, node_id id, Point pos);

            /**
             * @brief Get x-coordinate of the node
             * 
             * @return x-coordinate
             */
            float getX() const;

            /**
             * @brief Get y-coordinate of the node
             * 
             * @return y-coordinate 
             */
            float getY() const;

            /**
             * @brief   Get ID of the Node object
             * 
             * @return  Index of this Node object in the node list stored in the parent RoadMap object
             */
            node_id getID() const;

            /**
             * @brief   Get the number of poses for this node
             * 
             * @return  Number of poses stored in the Node object
             */
            size_t getPosesCount() const;

            /**
             * @brief       Get a Pose object by its ID
             * 
             * @param index ID of the Pose object
             * @return      Reference to the Pose object at given index
             */
            Orientation &getPose(size_t index);

            /**
             * @brief   Get the number of connected nodes according to the base directed graph of the RoadMap
             * 
             * @return  Number of connected nodes
             */
            size_t getConnectedCount() const;

            /**
             * @brief Get a connected node by its index
             * 
             * @param index Index of the connected node in the connection list
             * @return Reference to the Node object at given index in the connection list
             */
            Node &getConnected(size_t index);

            /**
             * @brief Remove all the stored poses of the Node object
             * 
             */
            void clearPoses();

            /**
             * @brief Add a pose to this Node object
             * 
             * @param theta Angle of the pose with respect to the x-axis, measured counter-clockwise
             * @return      ID of the newly created pose
             */
            size_t addPose(float theta);

            /**
             * @brief Get the parent RoadMap object
             * 
             * @return Reference to the RoadMap object this node belongs to
             */
            RoadMap &getRoadMap();
            
            /**
             * @brief Connect this Node object to another Node object in the base directed graph of the RoadMap
             * 
             * @param to ID of the Node object this node should be connected to
             */
            void connectTo(node_id to);

            operator node_id() const;
        }; // Node

        /**
         * @brief Edge of the base directed graph of the RoadMap
         * 
         */
        class Edge
        {
        private:
            node_id _from;
            node_id _to;
            RoadMap *_parent;

        public:
            /**
             * @brief Construct a new Edge object
             * 
             * @param parent    Pointer to the RoadMap object this edge belongs to
             * @param from      ID of the starting Node object
             * @param to        ID of the destination Node object
             */
            Edge(RoadMap *parent, node_id from, node_id to);

            /**
             * @brief Get the ID of the starting Node object
             * 
             * @return ID of the starting Node object 
             */
            node_id getFromID() const;

            /**
             * @brief Get the ID of the destination Node object
             * 
             * @return ID of the destination Node object 
             */
            node_id getToID() const;

            /**
             * @brief Get the starting Node object
             * 
             * @return Reference of the starting Node object 
             */
            Node &getFromNode() const;

            /**
             * @brief Get the destination Node object
             * 
             * @return Reference of the destination Node object 
             */
            Node &getToNode() const;

            /**
             * @brief Get the parent RoadMap object
             * 
             * @return Reference to the parent RoadMap object
             */
            RoadMap &getRoadMap();
        }; // Edge

        /**
         * @brief Connection between two poses formed by a Dubins path
         * 
         */
        struct DubinsConnection
        {
            /** Pointer to the starting pose */
            Node::Orientation *from;
            /** Pointer to the destination pose */
            Node::Orientation *to;
            /** Dubins path that connects the starting pose to the destination pose */
            dubins::DubinsCurve path;

            /**
             * @brief Construct a new DubinsConnection object
             * 
             * @param from  Pointer to the starting pose
             * @param to    Pointer to the destination pose
             * @param path  Dubins path that connects the starting pose to the destination pose
             */
            inline DubinsConnection(Node::Orientation *from, Node::Orientation *to,
                                    dubins::DubinsCurve path) : from(from), to(to), path(path) {}
        }; // DubinsConnection

    private:
        std::vector<Node> _nodes;
        std::vector<Edge> _edges;

    public:
        /**
         * @brief Add a positional node to the RoadMap
         * 
         * @param pos   Position of the node
         * @return      ID of the newly created Node object
         */
        node_id addNode(Point pos);

        /**
         * @brief Connect two Node objects in the base directed graph of the RoadMap
         * 
         * @param fromID    ID of the starting Node object
         * @param toID      ID of the destination Node object
         */
        void connect(node_id fromID, node_id toID);

        /**
         * @brief Build the roadmap.
         * 
         * This process starts with the creation of a given number of poses for each positional node, with evenly spaced angles.
         * Following the base directed graph of the roadmap, for each couple of connected nodes the algorithm tries to generate
         * Dubins paths that connect each pose of the starting node to each pose of the destination node. In this phase, it is
         * checked whether the path leads to collision with obtacles or with the arena borders. The feasible paths are added to
         * the navigation graph, which can be explored by checking the connections of each pose.
         * 
         * @param orientationsPerNode   Number of poses to be created on each positional node
         * @param kmax                  Maximum curvature of Dubins paths
         * @param obstacles             Obstacles to check collision against when computing Dubins paths
         * @param borders               Borders of the arena to check collision against when computing Dubins paths
         * @return                      Number of Dubins paths that are created in the process
         */
        unsigned long build(unsigned int orientationsPerNode, float const &kmax, const std::vector<Polygon> &obstacles, const Polygon &borders);

        /**
         * @brief Get the number of positional nodes in this RoadMap
         * 
         * @return Number of positional nodes belonging to this RoadMap
         */
        size_t getNodeCount() const;

        /**
         * @brief Get a Node object by its ID
         * 
         * @param id    ID of the Node object
         * @return      Reference to the Node object with given ID
         */
        Node &getNode(node_id id);
    }; // RoadMap
}
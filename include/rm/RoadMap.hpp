#pragma once

#include <vector>

#include "utils.hpp"
#include "dubins/dubins.hpp"

/**
 * @file RoadMap.hpp
 * @brief This file is dedicated to the class RoadMap. \n 
 * 
 * @see rm#RoadMap
 */

/**
 * @namespace rm
 * @brief This namespace collects classes and functions for the creation of a roadmap.
 * 
 */
namespace rm
{
    /**
     * @brief Roadmap for navigation in a 2-dimensional space. \n 
     * 
     * The roadmap stores two directed graphs. The base graph is created by the user by adding positional nodes and connections and defines the skeleton of the roadmap. 
     * The built graph is computed from the base graph when the build() method is called. It creates a fixed number of poses for each positional node 
     * and generates all the feasible Dubins curves connecting two nodes according to the base graph edges.
     * 
     * @see RoadMap#build()
     * @see RoadMap#addNode()
     * @see RoadMap#connect()
     */
    class RoadMap
    {
    public:
        struct DubinsConnection;

        /** Type for the ID of a node */
        typedef size_t node_id;

        /**
         * @brief Positional node of the roadmap. Once the roadmap is built, it stores information about poses that can be achieved in a given position.
         * 
         */
        class Node
        {
        public:
            /**
             * @brief Specific orientation on a positional node.
             * 
             * @see RoadMap#Node
             */
            class Orientation
            {
            private:
                float _theta;
                Node *_parent;
                std::vector<rm::RoadMap::DubinsConnection> _connections;
                std::vector<rm::RoadMap::DubinsConnection> _from;
                size_t _id;

            public:
                /**
                 * @brief Construct a new Orientation object.
                 * 
                 * @param[in] parent    Pointer to the Node object this pose belongs to
                 * @param[in] id        Index of this Pose object in the pose list stored in the parent Node object
                 * @param[in] theta     Angle of the pose with respect to the x-axis, measured counter-clockwise
                 */
                Orientation(Node *parent, size_t id, float theta);

                /**
                 * @brief Try to build a connection between two poses by evaluating a Dubins path.
                 * 
                 * @param[in] other     Pose to connect to
                 * @param[in] kmax      Maximum curvature of Dubins curves
                 * @param[in] obstacles List of obstacles to perform collision check when evaluating the Dubins path
                 * @param[in] borders   Borders of the arena to perform collision check when evaluating the Dubins path
                 * @return          true if a feasible path was found, false otherwise
                 */
                bool connect(Orientation &other, float const &kmax, const std::vector<Polygon> &obstacles, const Polygon &borders);

                /**
                 * @brief   Get the value of the angle.
                 * 
                 * @return  Angle of the pose with respect to the x-axis, measured counter-clockwise
                 */
                float getTheta() const;

                /**
                 * @brief   Get the id of the pose.
                 * 
                 * @return  Index of this Pose object in the pose list stored in the parent Node object
                 */
                size_t getID() const;

                /**
                 * @brief Get the parent Node object.
                 * 
                 * @return Reference of the Node object this pose belongs to
                 */
                Node &getNode() const;

                /**
                 * @brief Get the number of other poses this pose is connected to.
                 * 
                 * @return Number of connections
                 */
                size_t getConnectionCount() const;

                /**
                 * @brief       Get a connection starting from this pose by its index.
                 * 
                 * @param[in] index Index of the connection in the connection list stored in the pose
                 * @return      Reference to the connection at the given index
                 */
                rm::RoadMap::DubinsConnection &getConnection(size_t index);

                /**
                 * @brief       Get a connection starting from this pose by its index.
                 * 
                 * @param[in] index Index of the connection in the connection list stored in the pose
                 * @return      Read-only reference to the connection at the given index
                 */
                const rm::RoadMap::DubinsConnection &getConnection(size_t index) const;

                /**
                 * @brief Get the number of connections leading to this node.
                 * 
                 * @return Number of connections
                 */
                size_t getFromConnectionCount() const;

                /**
                 * @brief       Get a connection leading to this pose by its index.
                 * 
                 * @param[in] index Index of the connection in the connection list stored in the pose
                 * @return      Read-only reference to the connection at the given index
                 */
                const rm::RoadMap::DubinsConnection &getFromConnection(size_t index) const;

                /**
                 * @brief Implicit conversion of the pose to its ID.
                 * 
                 * @return ID of the pose
                 */
                operator size_t() const;
            };

        private:
            RoadMap *_parent;
            node_id _id;
            Point _pos;
            std::vector<Orientation> _poses;
            std::vector<node_id> _connected;

        public:
            /**
             * @brief           Construct a new Node object.
             * 
             * @param[in] parent    Pointer to the RoadMap object this pose belongs to
             * @param[in] id        Index of this Node object in the node list stored in the parent RoadMap object
             * @param[in] pos       2-dimensional position of the node
             */
            Node(RoadMap *parent, node_id id, Point pos);

            /**
             * @brief Get x-coordinate of the node.
             * 
             * @return x-coordinate
             */
            float getX() const;

            /**
             * @brief Get y-coordinate of the node.
             * 
             * @return y-coordinate 
             */
            float getY() const;

            /**
             * @brief   Get ID of the Node object.
             * 
             * @return  Index of this Node object in the node list stored in the parent RoadMap object
             */
            node_id getID() const;

            /**
             * @brief   Get the number of poses for this node.
             * 
             * @return  Number of poses stored in the Node object
             */
            size_t getPosesCount() const;

            /**
             * @brief       Get a Pose object by its ID.
             * 
             * @param[in] index ID of the Pose object
             * @return      Reference to the Pose object at given index
             */
            Orientation &getPose(size_t index);

            /**
             * @brief       Get a Pose object by its ID.
             * 
             * @param[in] index ID of the Pose object
             * @return      Read-only reference to the Pose object at given index
             */
            const Orientation &getPose(size_t index) const;

            /**
             * @brief   Get the number of connected nodes according to the base directed graph of the RoadMap.
             * 
             * @return  Number of connected nodes
             */
            size_t getConnectedCount() const;

            /**
             * @brief Get a connected node by its index.
             * 
             * @param[in] index Index of the connected node in the connection list
             * @return Reference to the Node object at given index in the connection list
             */
            Node &getConnected(size_t index);

            /**
             * @brief Remove all the stored poses of the Node object.
             * 
             */
            void clearPoses();

            /**
             * @brief Add a pose to this Node object.
             * 
             * @param[in] theta Angle of the pose with respect to the x-axis, measured counter-clockwise
             * @return      ID of the newly created pose
             */
            size_t addPose(float theta);

            /**
             * @brief Get the parent RoadMap object.
             * 
             * @return Reference to the RoadMap object this node belongs to
             */
            RoadMap &getRoadMap();

            /**
             * @brief Connect this Node object to another Node object in the base directed graph of the RoadMap.
             * 
             * @param[in] to    ID of the Node object this node should be connected to
             * @return      true if a new connection was created, false otherwise
             */
            bool connectTo(node_id to);

            operator node_id() const;
        }; // Node

        /**
         * @brief Connection between two poses formed by a Dubins path.
         * 
         * @see dubins#DubinsCurve
         * @see Roadmap#Node#Orientation
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
             * @param[in] from  Pointer to the starting pose
             * @param[in] to    Pointer to the destination pose
             * @param[in] path  Dubins path that connects the starting pose to the destination pose
             */
            inline DubinsConnection(Node::Orientation *from, Node::Orientation *to,
                                    dubins::DubinsCurve path) : from(from), to(to), path(path) {}
        }; // DubinsConnection

    private:
        std::vector<Node> _nodes;

    public:
        /**
         * @brief Add a positional node to the RoadMap.
         * 
         * @param[in] pos   Position of the node
         * @return      ID of the newly created Node object
         */
        node_id addNode(Point pos);

        /**
         * @brief Add a positional node and dedicated pose for the starting point of a robot.
         * 
         * @param[in] pos       Position of the start point
         * @param[in] angle     Angle of the start pose with respect to the x-axis, measured counter-clockwise
         * @param[in] k         Number of closest nodes the start pose should be connected to
         * @param[in] kmax      Maximum curvature of dubins paths
         * @param[in] obstacles Obstacles for collision checking
         * @param[in] borders   Borders for collision checking
         * @return          Reference to the created pose
         */
        Node::Orientation &addStartPose(Point pos, float angle, int k, float kmax, const std::vector<Polygon> &obstacles, const Polygon &borders);

        /**
         * @brief Add a positional node and dedicated pose for the goal point of a robot.
         * 
         * @param[in] pos       Position of the goal point
         * @param[in] angle     Angle of the goal pose with respect to the x-axis, measured counter-clockwise
         * @param[in] k         Number of closest nodes the start pose should be connected to
         * @param[in] kmax      Maximum curvature of dubins paths
         * @param[in] obstacles Obstacles for collision checking
         * @param[in] borders   Borders for collision checking
         * @return          Reference to the created pose
         */
        Node::Orientation &addGoalPose(Point pos, float angle, int k, float kmax, const std::vector<Polygon> &obstacles, const Polygon &borders);

        /**
         * @brief Connect two Node objects in the base directed graph of the RoadMap.
         * 
         * @param[in] fromID    ID of the starting Node object
         * @param[in] toID      ID of the destination Node object
         * @return          true if a new edge was created, false otherwise
         */
        bool connect(node_id fromID, node_id toID);

        /**
         * @brief Build the roadmap. \n 
         * 
         * This process starts with the creation of a given number of poses for each positional node, with evenly spaced angles.
         * Following the base directed graph of the roadmap, for each couple of connected nodes the algorithm tries to generate
         * Dubins paths that connect each pose of the starting node to each pose of the destination node. In this phase, it is
         * checked whether the path leads to collision with obtacles or with the arena borders. The feasible paths are added to
         * the navigation graph, which can be explored by checking the connections of each pose.
         * 
         * @param[in] orientationsPerNode   Number of poses to be created on each positional node
         * @param[in] kmax                  Maximum curvature of Dubins paths
         * @param[in] obstacles             Obstacles to check collision against when computing Dubins paths
         * @param[in] borders               Borders of the arena to check collision against when computing Dubins paths
         * @return                      Number of Dubins paths that are created in the process
         */
        unsigned long build(unsigned int orientationsPerNode, float const &kmax, const std::vector<Polygon> &obstacles, const Polygon &borders);

        /**
         * @brief Get the number of positional nodes in this RoadMap.
         * 
         * @return Number of positional nodes belonging to this RoadMap
         */
        size_t getNodeCount() const;

        /**
         * @brief Get a Node object by its ID.
         * 
         * @param[in] id    ID of the Node object
         * @return      Reference to the Node object with given ID
         */
        Node &getNode(node_id id);

        /**
         * @brief Get a Node object by its ID.
         * 
         * @param[in] id    ID of the Node object
         * @return      Read-only reference to the Node object with given ID
         */
        const Node &getNode(node_id id) const;

        /**
         * @brief Find the k-closest nodes to a given position.
         * 
         * @param[in] pos   Position
         * @param[in] k     Number of closest points
         * @param[in] skip  ID of node that should be skipped in the search
         * @return      Vector of closest nodes IDs
         */
        std::vector<node_id> findKClosest(Point pos, int k, node_id skip = -1);
    }; // RoadMap
}

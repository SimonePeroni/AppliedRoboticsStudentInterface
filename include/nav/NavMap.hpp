#pragma once

#include <vector>
#include <deque>

#include "rm/RoadMap.hpp"
#include "nav/path.hpp"

/**
 * @file NavMap.hpp
 * @brief This file is dedicated to the NavMap class.
 * 
 * @see nav
 * @see nav#NavMap
 */

/**
 * @namespace nav
 * @brief This namespace has a set of classes and functions for navigation.
 * @see nav#NavMap
 */
namespace nav
{
    /**
     * @brief Class to create pre-computed navigation maps for a RoadMap.\n
     * 
     * @see RoadMap
     */
    class NavMap
    {
    private:
        std::vector<std::vector<float>> _dist;
        std::vector<std::vector<rm::RoadMap::DubinsConnection const *>> _connection;
        const rm::RoadMap &_rm;
        bool _need_computing;
        bool _reverse;

    public:
        /**
         * @brief NavMap constructor.
         * 
         * @param[in] roadmap Associated RoadMap object.
         */
        NavMap(const rm::RoadMap &roadmap);

        /**
         * @brief Pre-compute the navigation map for direct planning.\n 
         * 
         * The algorithm explores the RoadMap from the given source pose and saves the shortest distances to every other pose in the RoadMap, 
         * as well as the last connection of the best path from source to every other pose. This allows for backward reconstruction of the full path.
         * 
         * @param[in] source Source pose from which exploration is started
         * 
         * @see planTo()
         * @see computeReverse()
         */
        void compute(const rm::RoadMap::Node::Orientation &source);

        /**
         * @brief Pre-compute the navigation map for reverse planning.\n 
         * 
         * The algorithm explores the RoadMap running every connection backwards from the given goal pose and saves the shortest distances from every other pose in the RoadMap, 
         * as well as the first connection of the best path to goal from every other pose. This allows for forward reconstruction of the full path.
         * 
         * @param[in] source Goal pose from which backward exploration is started
         * 
         * @see planFrom()
         * @see compute()
         */
        void computeReverse(const rm::RoadMap::Node::Orientation &goal);

        /**
         * @brief Clear all pre-computed values. Called automatically before every new computation.
         * 
         */
        void reset();

        /**
         * @brief Plan the shortest path to a given position, regardless of the orientation. Requires forward pre-computation.
         * 
         * @param[in] goal  Goal position
         * @return      Planned path
         * 
         * @see compute()
         * @see navList
         */
        navList planTo(const rm::RoadMap::Node &goal) const;

        /**
         * @brief Plan the shortest path to a given pose. Requires forward pre-computation.
         * 
         * @param[in] goal  Goal pose
         * @return      Planned path
         * 
         * @see compute()
         * @see navList
         */
        navList planTo(const rm::RoadMap::Node::Orientation &goal) const;

        /**
         * @brief Plan the shortest path from a given source. Requires reverse pre-computation.
         * 
         * @param[in] source    Source pose
         * @return          Planned path
         * 
         * @see compute()
         * @see navList
         */
        navList planFrom(const rm::RoadMap::Node::Orientation &source) const;

        /**
         * @brief Plan the shortest path that intercepts another path. Requires forward pre-computation. \n 
         * 
         * If interception is not possible, it plans the shortest path to the last position of the given path.
         * 
         * @param[in] path      Path to be intercepted
         * @param[in] offset    From which length of the given path the actual path should start at present.
         * @return navList 
         *
         * @see compute()
         * @see navList
         */
        navList intercept(const navList &path, float offset = 0.0f) const;

        /**
         * @brief Return true if NavMap was pre-computed with computeReverse.
         * 
         * @return true if NavMap was pre-computed with computeReverse
         * 
         * @see computeReverse()
         */
        bool isReverse() const;

        /**
         * @brief Get the pre-computed value stored for a given pose.
         * 
         * @param[in] pose  Pose
         * @return      Pre-computed value
         * 
         * @see compute()
         * @see computeReverse()
         */
        float getValue(const rm::RoadMap::Node::Orientation &pose) const;
        /**
         * @brief Get the smallest pre-computed value stored for a given position.
         * 
         * @param[in] node  Position
         * @return      Best pre-computed value
         * 
         * @see compute()
         * @see computeReverse()
         */
        float getValue(const rm::RoadMap::Node &node) const;
    };
}
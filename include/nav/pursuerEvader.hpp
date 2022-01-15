#pragma once

#include "nav/NavMap.hpp"

#include <vector>

/**
 * @file pursuerEvader.hpp
 * @brief This file is dedicated to the implementation of the pursuer-evader game.
 * 
 * @see nav
 * @see nav#runGame()
 */

namespace nav
{
    /**
     * @brief Implementation of the pursuer-evader game. \n 
     * 
     * The game requires two robots with two different roles: the evader and the pursuer. \n
     * The evader's objective is that of reaching one of the gates to get out of the arena. \n 
     * The pursuer's objective is that of catching the evader before it is able to escape. \n 
     * The evader acts unaware of the pursuer's presence. When reaching a new node of the RoadMap, 
     * it chooses one of the gates randomly and replans its path to that gate. \n 
     * The pursuer is aware of the current goal the evader is heading to at every time. This is a strong assumption,
     * but it would surely be possible to make a prediction of the gate the evader is heading to based on its short-term past motion. 
     * This implementation, however, takes the current goal for known at planning time, making it a simpler deterministic scenario.
     * 
     * @param nm_e          Vector of backward-computed navigation maps to all the gates in the arena.
     * @param nm_p          Navigation map used by the pursuer and re-computed several times throughout the game.
     * @param source_e      Starting pose of the evader.
     * @param source_p      Starting pose of the pursuer.
     * @param nav_list_e    Out: Navigation path of the evader throughout the game.
     * @param nav_list_p    Out: Navigation path of the pursuer throughout the game.
     */
    void runGame(const std::vector<nav::NavMap> &nm_e, nav::NavMap &nm_p,
                 const rm::RoadMap::Node::Orientation &source_e, const rm::RoadMap::Node::Orientation &source_p,
                 nav::navList &nav_list_e, nav::navList &nav_list_p);
}
#pragma once

#include "utils.hpp"

/**
 * @file student_planning_interface.hpp
 * @brief Where the magic happens. See student::planPath() for a detailed description of the process.
 * 
 */

namespace student
{
    /**
    * Plan the motion of pursuer and evader inside the arena. \n 
    * 
    * Robot number 0 is given the role of evader. \n 
    * Robot number 1 is given the role of pursuer. \n 
    * 
    * The algorithm executes the following steps:
    *   -# Obstacles and borders are inflated to account for the robots encumbrance. The robots can then be handled as oriented points. See rm::inflate()
    *   -# A set of vertices are selected for the visibility graph, accounting for a distance from the inflated obstacles that allows for a higher chance of feasible Dubins curves existing. See rm::makeVisibilityNodes()
    *   -# A visibility graph is created as the base graph for the RoadMap. See rm::visibility()
    *   -# The RoadMap is built, pre-computing all feasible Dubins paths connecting nodes according to the base graph. See rm::RoadMap::build() and dubins
    *   -# The initial poses of the robots are added to the graph. See rm::RoadMap::addStartPose()
    *   -# The goal pose of each gate is computed and added to the graph. See rm::getGatePose() and rm::RoadMap::addGoalPose()
    *   -# Navigation maps to the goal poses are pre-computed. See nav::NavMap()
    *   -# The acutal pursuer-evader game takes place. See nav::runGame()
    *   -# The computed paths are discretized. See nav::discretizePath()
    *   -# The discretized paths are truncated at the point where the two robots collide, terminating the game. See nav::truncatePaths()
    *   -# [Optional] A matlab file containing functions to plot the arena configuration and the computed paths is generated. See utils::MatlabPlot
    * 
    * @param[in]  borders        Borders of the arena [m]
    * @param[out] obstacle_list  List of obstacle polygon [m]
    * @param[out] victim_list    List of pair victim_id and polygon [m]
    * @param[out] gate           Polygon representing the gate [m]
    * @param[out] x              x position of the robot in the arena reference system
    * @param[out] y              y position of the robot in the arena reference system
    * @param[out] theta          Yaw of the robot in the arena reference system
    * @param[in]  config_folder  A custom string from config file.
    */
    bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                  const std::vector<Polygon> &gate_list,
                  const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta,
                  std::vector<Path> &path, const std::string &config_folder);

}

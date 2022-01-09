#pragma once
/**
 * @file dubins.hpp
 * @brief Dubins implementation
 * In this file it is explained the implementation of the Dubins curves. All the material related to the equations employed has been taken from the professor's slides 
 */

#include <vector>
#include <iostream>
#include "utils.hpp"

namespace dubins
{
    /**
     * @brief function used to implement the sinc(t)
     * 
     * @param t 
     * @return 1 if t is 1 otherwise  sin(t)/t
     */
    float sinc(float t);
    /**
     * @brief function used to normalize an angle 
     * 
     * @param angle 
     * @return float normalized angle in range [0:2pi]
     */

    float mod2pi(float angle);
     /**
     * @brief function used to normalize angular difference
     * 
     * @param ang 
     * @return float normalized angular difference in range[0:2pi]
     */

    float normAngle(float ang);

    // ---------------- Struct defining the position of the robot ---------------------------------------------------------------------------

/**
 * @brief This structure represents the robot position in 2D with its orientation 
 * 
 */
    struct Pose2D
    {
        float x;
        float y;
        float theta;
    };

    // ------------------ Struct for a Dubins arc -------------------------------------------------------------------------------------------
    /**
     * @brief struct defining a dubins arc: it can be either a straight line or arc
     * 
     */
    struct DubinsArc
    {

        Pose2D start; //--- inital robot pose coordinates
        Pose2D end; //--- final robot pose coordinates
        float k, s;
    };

    // --------------- Struct for a Dubins curve ---------------------------------------------------------------------------------------------
     /**
     * @brief Define the structure of a complete dubins curve
     * @param arc_1
     * @param arc_2
     * @param arc_3
     * @param  L total length of the curve 
     * 
     */
    struct DubinsCurve
    {
        float L;
        DubinsArc arc_1;
        DubinsArc arc_2;
        DubinsArc arc_3;
    };

/**
 * @brief checks the validity of a Dubins problem
 * It checks the validity of problem by evaluating the 3 equations
 * 
 * @param s1 original length of dubins curve
 * @param k0 original angle of the curve
 * @param s2 original length of dubins curve
 * @param k1 original angle of the curve
 * @param s3 original length of dubins curve
 * @param k2 original angle of the curve
 * @param th0 original inital angle 
 * @param thf original final angle 
 * @return true if the validity of all 3 equations is met
 * 
 */
    bool check(float const &s1, float const &k0, float const &s2, float const &k1, float const &s3, float const &k2, float const &th0, float const &thf);
    // -------- Scaling the functions ------------------------------------------
	
/**
   * @brief Function to scale input problem to standard form 
   * 
   * @param start struct with inital robot pose
   * @param end struct with final robot pose
   * @param kmax maximum turning angle of the curve 
   * @param sc_th_init scaled inital robot orientation
   * @param sc_th_fin scaled final robot orientation 
   * @param sc_kmax scaled maximum turning angle
   * @param lambda 
   */
    void scaleToStandard(Pose2D start, Pose2D end, float kmax, float &sc_th_init, float &sc_th_fin, float &sc_kmax, float &lambda);
	
/**
 * @brief function used to scale the solution back to the original problem form 
 * 
 * @param lambda 
 * @param sc_s1 scaled length of dubins curve
 * @param sc_s2 scaled length of dubins curve
 * @param sc_s3 scaled length of dubins curve
 * @param s1 
 * @param s2 
 * @param s3 
 */

    void scaleFromStandard(float lambda, float sc_s1, float sc_s2, float sc_s3, float &s1, float &s2, float &s3);

    // --- Dubins functions ----------------------------------------------------
/**
 * @brief Implementation of one of the six Dubins primitives.
 * The same parameter and procedure followed for this equation has been applied to the remaining 5
 * 
 * @param sc_th_init scaled initial robot orientation
 * @param sc_th_fin scaled final robot orientation
 * @param sc_kmax scaled maximum turning angle 
 * @param ctrl is set to false if all the Dubins curves are 0 
 * @param sc_s1 scaled length of a Dubins curve
 * @param sc_s2 scaled length of a Dubins curve
 * @param sc_s3 scaled length of a Dubins curve 
 */

    void RSR(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void RSL(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void RLR(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void LSR(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void LSL(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void LRL(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

     /**
     * @brief function used to compute a Dubins arc
     * 
     * @param ptr struct of a Dubins arc computed inside this function
     * @param start struct defining the initial pose of the robot 
     * @param k maximum turning angle 
     * @param s total length of arc 
     */


    void setDubinsArc(DubinsArc &ptr, const Pose2D &start, float k, float s);
	/**
     * @brief function used to compute a Dubins curve 
     * 
     * @param curve struct of a Dubins curve computed in this function 
     * @param start initial 2D pose of the robot 
     * @param s1 
     * @param s2 
     * @param s3 
     * @param k0 
     * @param k1 
     * @param k2 
     */

    void setDubinsCurve(DubinsCurve &curve, const Pose2D &start, float s1, float s2, float s3, float k0, float k1, float k2);

    // ----- Find the shortest path ---------------------------------------------
	/**
     * @brief function used to find which is the shortest Dubins curve among the ones computed 
     * 
     * @param curve Dubins curve 
     * @param start initial 2D pose of the robot
     * @param end final 2D pose of the robot 
     * @param kmax maximum turning angle 
     * @param obstacles list of obstalces present in the map
     * @param borders borders defining the map 
     * @return true if the analyzed curve is the shortest one 
     */
    bool findShortestPath(DubinsCurve &curve, Pose2D start, Pose2D end, float const &kmax, const std::vector<Polygon> &obstacles = std::vector<Polygon>(), const Polygon &borders = Polygon());
	
	/**
     * @brief function used to calculate the robot pose 
     * The calculation of the robot pose is obtained using the sin() function for the x and y coordinates and mod2pi 
     * @param s length of the arc
     * @param p0 
     * @param k maximum turning angle of the arc
     * @return Pose2D 
     * @see float sinc(float t)
     * @see float mod2pi(float angle)
     */
    Pose2D poseOnArc(float s, Pose2D p0, float k);
    // ----- Discretization of the arc ------------------------------------------
	/**
     * @brief function used to discretize the arc
     * number of points for the discretization calculated with rounding down (arc.s - offset) / step and adding  1 afterwards
     * @param arc Dubins arc to be discretized
     * @param s_end 
     * @param step parameter used in the discretization 
     * @param offset 
     * @return std::vector<Pose> discretized arc 
     */
    void discretizeArc(const DubinsArc &arc, float step, float &offset, std::vector<Pose> &path);
	
     /**
     * @brief function used to discretize the Dubins curve 
     * The procedure described in the discretization of a Dubins arc is done for each arc of the curve and the results are combined together. 
     * 
     * @param curve 
     * @param s_end 
     * @param step 
     * @param offset 
     * @return std::vector<Pose> 
     */

    void discretizeCurve(const DubinsCurve &curve, float step, float &offset, std::vector<Pose> &path);
} // dubins

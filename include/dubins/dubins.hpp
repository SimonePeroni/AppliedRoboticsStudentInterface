#pragma once

#include <vector>
#include <iostream>
#include "utils.hpp"

namespace dubins
{
    float sinc(float t);

    float mod2pi(float angle);

    float normAngle(float ang);

    // ---------------- Struct defining the position of the robot ---------------------------------------------------------------------------

    struct Pose2D
    {
        float x;
        float y;
        float theta;
    };

    // ------------------ Struct for a Dubins arc -------------------------------------------------------------------------------------------
    struct DubinsArc
    {

        Pose2D start; //--- inital robot pose coordinates
        Pose2D end; //--- final robot pose coordinates
        float k, s;
    };

    // --------------- Struct for a Dubins curve ---------------------------------------------------------------------------------------------
    struct DubinsCurve
    {
        float L;
        DubinsArc arc_1;
        DubinsArc arc_2;
        DubinsArc arc_3;
    };

    bool check(float const &s1, float const &k0, float const &s2, float const &k1, float const &s3, float const &k2, float const &th0, float const &thf);
    // -------- Scaling the functions ------------------------------------------
    void scaleToStandard(Pose2D start, Pose2D end, float kmax, float &sc_th_init, float &sc_th_fin, float &sc_kmax, float &lambda);

    void scaleFromStandard(float lambda, float sc_s1, float sc_s2, float sc_s3, float &s1, float &s2, float &s3);

    // --- Dubins functions ----------------------------------------------------

    void RSR(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void RSL(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void RLR(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void LSR(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void LSL(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void LRL(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    void setDubinsArc(DubinsArc &ptr, const Pose2D &start, float k, float s);

    void setDubinsCurve(DubinsCurve &curve, const Pose2D &start, float s1, float s2, float s3, float k0, float k1, float k2);

    // ----- Find the shortest path ---------------------------------------------
    bool findShortestPath(DubinsCurve &curve, Pose2D start, Pose2D end, float const &kmax, const std::vector<Polygon> &obstacles = std::vector<Polygon>(), const Polygon &borders = Polygon());

    // ----- Discretization of the arc ------------------------------------------
    void discretize_arc(DubinsArc &full_arc, float &s, int &npts, std::vector<Path> &path);
} // dubins

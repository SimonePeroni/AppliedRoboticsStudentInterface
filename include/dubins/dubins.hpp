#pragma once

#include <vector>
#include <iostream>
#include "utils.hpp"

/**
 * @file dubins.hpp
 * @brief Library of functions for Dubins curves computation.\n
 * 
 * The maths used in this library is derived from the course material of "Robot Planning and its applications", University of Trento
 */

/**
 * @brief This namespace holds the functions required for Dubins curves computation.
 * 
 */
namespace dubins
{
    /**
     * @brief Implementation of the sinc(t) function. The result is computed in a numerically stable way.
     * 
     * @param[in] t 
     * @return 1 if t=1, sin(t)/t otherwise
     */
    float sinc(float t);

    /**
     * @brief Angle normalization in range [0:2pi).
     * 
     * @param[in] angle Angle to be normalized
     * @return          Normalized angle in range [0:2pi)
     */
    float mod2pi(float angle);

    /**
     * @brief Angle normalization in range (-pi:pi].
     * 
     * @param[in] angle Angle to be normalized
     * @return          Normalized angle in range (-pi:pi]
     */
    float normAngle(float angle);

    /**
     * @brief Structure representing a pose on the 2D xy-plane.
     * 
     */
    struct Pose2D
    {
        /** First coordinate of the pose. */
        float x;
        /** Second coordinate of the pose. */
        float y;
        /** Angle of the pose, given counter-clockwise with respect to positive x-axis. */
        float theta;
    };

    /**
     * @brief Structure representing an arc of the Dubins curve. The concept of arc here extends to straight lines, with 0 curvature.
     * @see setDubinsArc()
     */
    struct DubinsArc
    {
        /** Initial pose */
        Pose2D start;
        /** Final pose */
        Pose2D end;
        /** Arc curvature */
        float k;
        /** Arc length */
        float s;
    };

    /**
     * @brief Structure representing a Dubins curve connecting two poses in the 2D space.
     * @see setDubinsCurve()
     */
    struct DubinsCurve
    {
        /** Length of the curve */
        float L;
        /** First arc of the curve */
        DubinsArc arc_1;
        /** Second arc of the curve */
        DubinsArc arc_2;
        /** Third arc of the curve */
        DubinsArc arc_3;
    };

    /**
     * @brief Checks the validity of a computed solution for the Dubins problem.
     * 
     * @param[in] s1    Length of first arc
     * @param[in] k0    Curvature of first arc
     * @param[in] s2    Length of second arc
     * @param[in] k1    Curvature of second arc
     * @param[in] s3    Length of third arc
     * @param[in] k2    Curvature of third arc
     * @param[in] th0   Initial angle
     * @param[in] thf   Final angle
     * @return      true if the computed solution is valid
     * 
     */
    bool check(float const &s1, float const &k0, float const &s2, float const &k1, float const &s3, float const &k2, float const &th0, float const &thf);

    /**
     * @brief Scale input parameters to the standard form.
     * 
     * @param[in] start          Initial pose
     * @param[in] end            Final pose
     * @param[in] kmax           Maximum curvature
     * @param[out] sc_th_init    Out: Scaled inital orientation
     * @param[out] sc_th_fin     Out: Scaled final orientation 
     * @param[out] sc_kmax       Out: Scaled maximum curvature
     * @param[out] lambda        Out: Lambda value
     * @see scaleFromStandard()
     */
    void scaleToStandard(Pose2D start, Pose2D end, float kmax, float &sc_th_init, float &sc_th_fin, float &sc_kmax, float &lambda);

    /**
     * @brief Scale a solution in the standard form back to the original space. 
     * 
     * @param[in] lambda    Lambda value
     * @param[in] sc_s1     Length of first arc of the Dubins curve in the standard form
     * @param[in] sc_s2     Length of second arc of the Dubins curve in the standard form
     * @param[in] sc_s3     Length of third arc of the Dubins curve in the standard form
     * @param[out] s1       Out: Length of first arc
     * @param[out] s2       Out: Length of second arc
     * @param[out] s3       Out: Length of third arc
     * @see scaleToStandard()
     */
    void scaleFromStandard(float lambda, float sc_s1, float sc_s2, float sc_s3, float &s1, float &s2, float &s3);

    /**
     * @brief Compute the RSR (right-turn - straight line - right-turn) Dubins primitive.
     * 
     * @param[in] sc_th_init     Inital orientation in standard form
     * @param[in] sc_th_fin      Final orientation in standard form
     * @param[in] sc_kmax        Maximum curvature in standard form
     * @param[out] ctrl          Out: false if no feasible path was found
     * @param[out] sc_s1         Out: Length of first arc in the standard form
     * @param[out] sc_s2         Out: Length of second arc in the standard form
     * @param[out] sc_s3         Out: Length of third arc in the standard form
     */
    void RSR(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    /**
     * @brief Compute the RSL (right-turn - straight line - left-turn) Dubins primitive.
     * 
     * @param[in] sc_th_init     Inital orientation in standard form
     * @param[in] sc_th_fin      Final orientation in standard form
     * @param[in] sc_kmax        Maximum curvature in standard form
     * @param[out] ctrl          Out: false if no feasible path was found
     * @param[out] sc_s1         Out: Length of first arc in the standard form
     * @param[out] sc_s2         Out: Length of second arc in the standard form
     * @param[out] sc_s3         Out: Length of third arc in the standard form
     */
    void RSL(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    /**
     * @brief Compute the RLR (right-turn - left-turn - right-turn) Dubins primitive.
     * 
     * @param[in] sc_th_init     Inital orientation in standard form
     * @param[in] sc_th_fin      Final orientation in standard form
     * @param[in] sc_kmax        Maximum curvature in standard form
     * @param[out] ctrl          Out: false if no feasible path was found
     * @param[out] sc_s1         Out: Length of first arc in the standard form
     * @param[out] sc_s2         Out: Length of second arc in the standard form
     * @param[out] sc_s3         Out: Length of third arc in the standard form
     */
    void RLR(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    /**
     * @brief Compute the LSR (left-turn - straight line - right-turn) Dubins primitive.
     * 
     * @param[in] sc_th_init     Inital orientation in standard form
     * @param[in] sc_th_fin      Final orientation in standard form
     * @param[in] sc_kmax        Maximum curvature in standard form
     * @param[out] ctrl          Out: false if no feasible path was found
     * @param[out] sc_s1         Out: Length of first arc in the standard form
     * @param[out] sc_s2         Out: Length of second arc in the standard form
     * @param[out] sc_s3         Out: Length of third arc in the standard form
     */
    void LSR(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    /**
     * @brief Compute the LSL (left-turn - straight line - left-turn) Dubins primitive.
     * 
     * @param[in] sc_th_init     Inital orientation in standard form
     * @param[in] sc_th_fin      Final orientation in standard form
     * @param[in] sc_kmax        Maximum curvature in standard form
     * @param[out] ctrl          Out: false if no feasible path was found
     * @param[out] sc_s1         Out: Length of first arc in the standard form
     * @param[out] sc_s2         Out: Length of second arc in the standard form
     * @param[out] sc_s3         Out: Length of third arc in the standard form
     */
    void LSL(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    /**
     * @brief Compute the LRL (left-turn - right-turn - left-turn) Dubins primitive.
     * 
     * @param[in] sc_th_init     Inital orientation in standard form
     * @param[in] sc_th_fin      Final orientation in standard form
     * @param[in] sc_kmax        Maximum curvature in standard form
     * @param[out] ctrl          Out: false if no feasible path was found
     * @param[out] sc_s1         Out: Length of first arc in the standard form
     * @param[out] sc_s2         Out: Length of second arc in the standard form
     * @param[out] sc_s3         Out: Length of third arc in the standard form
     */
    void LRL(float sc_th_init, float sc_th_fin, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3);

    /**
     * @brief Create a DubinsArc object from the given parameters.
     * 
     * @param[out] ptr   Out: DubinsArc to be set
     * @param[in]  start Initial pose
     * @param[in]  k     Arc curvature
     * @param[in]  s     Arc length
     * @see DubinsArc
     * @see DubinsCurve
     */
    void setDubinsArc(DubinsArc &ptr, const Pose2D &start, float k, float s);

    /**
     * @brief Create a DubinsCurve object from the given parameters.
     * 
     * @param[out] curve Out: DuvinsCurve to be set
     * @param[in] start Initial pose
     * @param[in] s1    First arc length
     * @param[in] s2    Second arc length
     * @param[in] s3    Third arc length
     * @param[in] k0    First arc curvature
     * @param[in] k1    Second arc curvature
     * @param[in] k2    Third arc curvature
     * @see DubinsCurve
     */
    void setDubinsCurve(DubinsCurve &curve, const Pose2D &start, float s1, float s2, float s3, float k0, float k1, float k2);

    /**
     * @brief Find the shortest Dubins curve connecting two poses in a 2D space.
     * 
     * @param[out] curve     Out: Computed Dubins curve 
     * @param[in]  start     Start pose
     * @param[in]  end       End pose
     * @param[in]  kmax      Maximum curvature
     * @param[in]  obstacles Optional: vector of obstacles for collision check
     * @param[in]  borders   Optional: vector of borders for collision check
     * @return          true if a feasible curve was found, false otherwise
     * @see DubinsCurve
     */
    bool findShortestPath(DubinsCurve &curve, Pose2D start, Pose2D end, float const &kmax, const std::vector<Polygon> &obstacles = std::vector<Polygon>(), const Polygon &borders = Polygon());

    /**
     * @brief Compute the pose of a circular-arc trajectory at a given parameterized length.
     * 
     * @param[in] s     Parameterized length. Must be in range [0, full arc length]
     * @param[in] p0    Initial pose of the arc
     * @param[in] k     Arc curvature
     * @return      Pose of circular-arc trajectory at given s
     */
    Pose2D poseOnArc(float s, Pose2D p0, float k);

    /**
     * @brief Discretize a Dubins arc.
     * 
     * @param[in]     arc       Dubins arc to be discretized
     * @param[in]     step      Discretization step. Distance between two consecutive poses in the discretized path.
     * @param[in,out] offset    At what length of the arc to begin discretization. It is updated according to the remainder of the arc after discretization.
     * @param[out]    path      Out: Vector of poses to which the discretized arc is appended
     * @see discretizeCurve()
     */
    void discretizeArc(const DubinsArc &arc, float step, float &offset, std::vector<Pose> &path);

    /**
     * @brief Discretize a Dubins curve.
     * 
     * @param[in]     curve     Dubins curve to be discretized
     * @param[in]     step      Discretization step. Distance between two consecutive poses in the discretized path.
     * @param[in,out] offset    At what length of the curve to begin discretization. It is updated according to the remainder of the curve after discretization.
     * @param[out]    path      Out: Vector of poses to which the discretized curve is appended
     * @see discretizeArc()
     */
    void discretizeCurve(const DubinsCurve &curve, float step, float &offset, std::vector<Pose> &path);
} // dubins

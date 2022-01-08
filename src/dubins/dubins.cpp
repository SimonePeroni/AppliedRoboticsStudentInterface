#include <iostream>
#include <vector>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include "dubins/dubins.hpp"
#include "utils.hpp"
#include "rm/geometry.hpp"

using namespace std;

namespace dubins
{
	float check_threshold = 1e-4;

	// ------- Sinc function -----------------------------------------------------
	float sinc(float t)
	{
		if (abs(t) < 0.002)
			return 1 - t * t / 120.f * (20.0f - t * t); // ---- Taylor series approximation
		else
			return sin(t) / t;
	}

	// --------------- We need this function to normalize the angle in the range 0 to 360 deg --------------------------------

	float mod2pi(float angle)
	{
		float p = 2.0f * M_PI;
		while (angle < 0)
		{
			angle = angle + p;
		}
		while (angle >= p)
		{
			angle = angle - p;
		}
		return angle;
	}

	// --------------- We need this function to normalize the angle in the range -180 to 180 deg --------------------------------

	float normAngle(float ang)
	{
		float p = 2.0f * M_PI;
		while (ang <= -M_PI)
			ang = ang + p;
		while (ang > M_PI)
			ang = ang - p;
		return ang;
	}

	// --------------------------------------
	bool check(float const &s1, float const &k0, float const &s2, float const &k1, float const &s3, float const &k2, float const &th0, float const &thf)
	{
		float eq1 = s1 * sinc(0.5f * k0 * s1) * cos(th0 + 0.5f * k0 * s1) + s2 * sinc(0.5f * k1 * s2) * cos(th0 + k0 * s1 + 0.5f * k1 * s2) + s3 * sinc(0.5f * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + 0.5f * k2 * s3) - 2.f;
		float eq2 = s1 * sinc(0.5f * k0 * s1) * sin(th0 + 0.5f * k0 * s1) + s2 * sinc(0.5f * k1 * s2) * sin(th0 + k0 * s1 + 0.5f * k1 * s2) + s3 * sinc(0.5f * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + 0.5f * k2 * s3);
		float eq3 = normAngle(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

		float err = sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3);

		bool tmp = ((s1 > 0) || (s2 > 0) || (s3 > 0)) && (err < check_threshold);

		return tmp;
	}
	// ------------------------------------------------------------------------------------------------------------------------ //

	void scaleToStandard(Pose2D start, Pose2D end, float kmax, float &sc_th0, float &sc_thf, float &sc_kmax, float &lambda)
	{
		float dx = end.x - start.x;
		float dy = end.y - start.y;
		lambda = hypot(dx, dy) * 0.5f;
		float phi = atan2(dy, dx);
		sc_th0 = mod2pi(start.theta - phi);
		sc_thf = mod2pi(end.theta - phi);
		sc_kmax = kmax * lambda;
	}

	// --- These two functions are used to scale from and to the standard form

	void scaleFromStandard(float lambda, float sc_s1, float sc_s2, float sc_s3, float &s1, float &s2, float &s3)
	{
		s1 = sc_s1 * lambda;
		s2 = sc_s2 * lambda;
		s3 = sc_s3 * lambda;
	}

	// ------------------------------------------------------------------------------------------------------------------------- //

	// ---------- Implementation of the primitives functions following the guidelines given by the professor in class --------------------
	void LSL(float sc_th0, float sc_thf, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3)
	{
		float invK = 1.0 / sc_kmax;
		float C = cos(sc_thf) - cos(sc_th0);
		float S = 2.0 * sc_kmax + sin(sc_th0) - sin(sc_thf);
		float temp1 = atan2(C, S);
		sc_s1 = invK * mod2pi(temp1 - sc_th0);
		float temp2 = 2.0 + 4.0 * pow(sc_kmax, 2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
		if (temp2 < 0)
		{
			sc_s1 = 0;
			sc_s2 = 0;
			sc_s3 = 0;
			ctrl = false;
		}
		else
		{
			sc_s2 = invK * sqrt(temp2);
			sc_s3 = invK * mod2pi(sc_thf - temp1);
			ctrl = true;
		}
	}

	void RSR(float sc_th0, float sc_thf, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3)
	{
		float invK = 1.0 / sc_kmax;
		float C = cos(sc_th0) - cos(sc_thf);
		float S = 2.0 * sc_kmax - sin(sc_th0) + sin(sc_thf);
		float temp1 = atan2(C, S);
		sc_s1 = invK * mod2pi(sc_th0 - temp1);
		float temp2 = 2.0 + 4.0 * pow(sc_kmax, 2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
		if (temp2 < 0)
		{
			sc_s1 = 0;
			sc_s2 = 0;
			sc_s3 = 0;
			ctrl = false;
		}
		else
		{
			sc_s2 = invK * sqrt(temp2);
			sc_s3 = invK * mod2pi(temp1 - sc_thf);
			ctrl = true;
		}
	}

	void LSR(float sc_th0, float sc_thf, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3)
	{
		float invK = 1.0 / sc_kmax;
		float C = cos(sc_th0) + cos(sc_thf);
		float S = 2.0 * sc_kmax + sin(sc_th0) + sin(sc_thf);
		float temp1 = atan2(-C, S);
		float temp3 = 4.0 * pow(sc_kmax, 2) - 2.0 + 2.0 * cos(sc_th0 - sc_thf) + 4.0 * sc_kmax * (sin(sc_th0) + sin(sc_thf));

		if (temp3 < 0)
		{
			sc_s1 = 0;
			sc_s2 = 0;
			sc_s3 = 0;
			ctrl = false;
		}
		else
		{
			sc_s2 = invK * sqrt(temp3);
			float temp2 = -atan2(-2.0, sc_s2 * sc_kmax);
			sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
			sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
			ctrl = true;
		}
	}

	void RSL(float sc_th0, float sc_thf, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3)
	{
		float invK = 1.0 / sc_kmax;
		float C = cos(sc_th0) + cos(sc_thf);
		float S = 2.0 * sc_kmax - sin(sc_th0) - sin(sc_thf);
		float temp1 = atan2(C, S);
		float temp3 = 4.0 * pow(sc_kmax, 2) - 2.0 + 2.0 * cos(sc_th0 - sc_thf) - 4.0 * sc_kmax * (sin(sc_th0) + sin(sc_thf));
		if (temp3 < 0)
		{
			sc_s1 = 0;
			sc_s2 = 0;
			sc_s3 = 0;
			ctrl = false;
		}
		else
		{
			sc_s2 = invK * sqrt(temp3);
			float temp2 = atan2(2.0, sc_s2 * sc_kmax);
			sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
			sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
			ctrl = true;
		}
	}

	void RLR(float sc_th0, float sc_thf, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3)
	{
		float invK = 1.0 / sc_kmax;
		float C = cos(sc_th0) - cos(sc_thf);
		float S = 2.0 * sc_kmax - sin(sc_th0) + sin(sc_thf);
		float temp1 = atan2(C, S);
		float temp2 = 0.125 * (6.0 - 4.0 * pow(sc_kmax, 2) + 2.0 * cos(sc_th0 - sc_thf) + 4.0 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
		if (abs(temp2) > 1.0)
		{
			sc_s1 = 0;
			sc_s2 = 0;
			sc_s3 = 0;
			ctrl = false;
		}
		else
		{
			sc_s2 = invK * mod2pi(2.0 * M_PI - acos(temp2));
			sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_kmax);
			sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_kmax * (sc_s2 - sc_s1));
			ctrl = true;
		}
	}

	void LRL(float sc_th0, float sc_thf, float sc_kmax, bool &ctrl, float &sc_s1, float &sc_s2, float &sc_s3)
	{
		float invK = 1.0 / sc_kmax;
		float C = cos(sc_thf) - cos(sc_th0);
		float S = 2.0 * sc_kmax + sin(sc_th0) - sin(sc_thf);
		float temp1 = atan2(C, S);
		float temp2 = 0.125 * (6.0 - 4.0 * pow(sc_kmax, 2) + 2.0 * cos(sc_th0 - sc_thf) - 4.0 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
		if (abs(temp2) > 1.0)
		{

			sc_s1 = 0;
			sc_s2 = 0;
			sc_s3 = 0;
			ctrl = false;
		}
		else
		{
			sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
			sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_kmax);
			sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_kmax * (sc_s2 - sc_s1));
			ctrl = true;
		}
	}

	// ----- Function to implement a structure representing a Dubins arc for both straight and circular ----------------------------------------------------
	void setDubinsArc(DubinsArc &ptr, const Pose2D &start, float k, float s)
	{

		ptr.start.x = start.x;
		ptr.start.y = start.y;
		ptr.start.theta = start.theta;
		ptr.k = k;
		ptr.s = s;
		ptr.end.x = start.x + s * sinc(k * s / 2.0) * cos(start.theta + k * s / 2.0);
		ptr.end.y = start.y + s * sinc(k * s / 2.0) * sin(start.theta + k * s / 2.0);
		ptr.end.theta = mod2pi(start.theta + k * s);
	}

	// ----- Function to implement a structure representing 3 arcs which make up the Dubins curve ------------------------------------------
	void setDubinsCurve(DubinsCurve &curve, const Pose2D &start, float s1, float s2, float s3, float k0, float k1, float k2)
	{

		setDubinsArc(curve.arc_1, start, k0, s1);
		setDubinsArc(curve.arc_2, curve.arc_1.end, k1, s2);
		setDubinsArc(curve.arc_3, curve.arc_2.end, k2, s3);
		curve.L = curve.arc_1.s + curve.arc_2.s + curve.arc_3.s;
	}

	// ----- Function to find the shortest path ---------------------------------------------------------------------------------------------
	bool findShortestPath(DubinsCurve &curve, Pose2D start, Pose2D end, float const &kmax, const vector<Polygon> &obstacles, const Polygon &borders)
	{
		float sc_th0, sc_thf, sc_kmax;
		float lambda;
		scaleToStandard(start, end, kmax, sc_th0, sc_thf, sc_kmax, lambda);

		typedef void (*maneuver)(float, float, float, bool &, float &, float &, float &);

		maneuver LSL_ptr = &LSL;
		maneuver RSR_ptr = &RSR;
		maneuver LSR_ptr = &LSR;
		maneuver RSL_ptr = &RSL;
		maneuver RLR_ptr = &RLR;
		maneuver LRL_ptr = &LRL;

		maneuver primitives[6] = {LSL_ptr, RSR_ptr, LSR_ptr, RSL_ptr, RLR_ptr, LRL_ptr};
		int ksigns[6][3] = {
			{1, 0, 1},	 //LSL
			{-1, 0, -1}, //RSR
			{1, 0, -1},	 //LSR
			{-1, 0, 1},	 //RSL
			{-1, 1, -1}, //RLR
			{1, -1, 1}}; //LRL

		int pidx = -1;
		double L = numeric_limits<double>::infinity(); // Infinite value
		bool ctrl;
		float s1, s2, s3;
		float sc_s1, sc_s2, sc_s3;
		float sc_s1_c, sc_s2_c, sc_s3_c;
		int Lcur;
		DubinsCurve best;

		for (size_t i = 0; i < 6; i++)
		{
			primitives[i](sc_th0, sc_thf, sc_kmax, ctrl, sc_s1_c, sc_s2_c, sc_s3_c);

			Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
			if (ctrl && Lcur < L)
			{
				DubinsCurve current;
				scaleFromStandard(lambda, sc_s1_c, sc_s2_c, sc_s3_c, s1, s2, s3);
				setDubinsCurve(current, start, s1, s2, s3, ksigns[i][0] * kmax, ksigns[i][1] * kmax, ksigns[i][2] * kmax);
				bool colliding = false;
				if (!obstacles.empty())
				{
					for (auto &obst : obstacles)
					{
						if (rm::collisionCheck(current, obst))
						{
							colliding = true;
							break;
						}
					}
				}
				if (!colliding && !borders.empty())
				{
					if (rm::collisionCheck(current, borders))
						colliding = true;
				}
				if (!colliding)
				{
					L = Lcur;
					sc_s1 = sc_s1_c;
					sc_s2 = sc_s2_c;
					sc_s3 = sc_s3_c;
					best = current;
					pidx = i;
				}
			}
		}

		if (pidx >= 0)
		{
			curve = best;

			// Check correctess of solution
			if (!check(sc_s1, ksigns[pidx][0] * sc_kmax,
					   sc_s2, ksigns[pidx][1] * sc_kmax,
					   sc_s3, ksigns[pidx][2] * sc_kmax,
					   sc_th0, sc_thf))
				throw std::logic_error("DUBINS - INCORRECT COMPUTED SOLUTION");

			return true;
		}
		return false;
	}

	Pose2D poseOnArc(float s, Pose2D p0, float k)
	{
		Pose2D out;
		out.x = p0.x + s * sinc(.5f * k * s) * cos(p0.theta + .5f * k * s);
		out.y = p0.y + s * sinc(.5f * k * s) * sin(p0.theta + .5f * k * s);
		out.theta = mod2pi(p0.theta + k * s);
		return out;
	}

	//----------------- This last discretization step is needed in order to create smaller arcs that the robots will follow ----------------------

	void discretizeArc(const DubinsArc &arc, float step, float &offset, std::vector<Pose> &path)
	{
		// skip degenerate arcs
		if (arc.s > 0.0f)
		{
			float s_end = path.empty() ? 0.0f : path.back().s;
			int n_points = floor((arc.s - offset) / step) + 1;
			for (size_t i = 0; i < n_points; i++)
			{
				float s = offset + step * i;
				Pose2D current = poseOnArc(s, arc.start, arc.k);
				path.push_back(Pose(s_end + step - offset + s, current.x, current.y, current.theta, arc.k));
			}
			offset = step * n_points + offset - arc.s;
		}
	}

	void discretizeCurve(const DubinsCurve &curve, float step, float &offset, std::vector<Pose> &path)
	{
		discretizeArc(curve.arc_1, step, offset, path);
		discretizeArc(curve.arc_2, step, offset, path);
		discretizeArc(curve.arc_3, step, offset, path);
	}
} // dubins

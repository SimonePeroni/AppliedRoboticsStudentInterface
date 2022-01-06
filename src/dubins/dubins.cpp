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

using namespace std;

namespace dubins
{
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

	// --------------------------------------
	bool check(float const &s1, float const &k0, float const &s2, float const &k1, float const &s3, float const &k2, float const &th0, float const &thf)
	{

		float x0 = -1.f;
		float xf = 1.f;
		float y0 = 0.f;
		float yf = 0.f;

		float eq1 = x0 + s1 * sinc((1 / 2.0) * k0 * s1) * cos(th0 + (1 / 2.0) * k0 * s1) + s2 * sinc((1 / 2.0) * k1 * s2) * cos(th0 + k0 * s1 + (1 / 2.0) * k1 * s2) + s3 * sinc((1 / 2.0) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1 / 2.0) * k2 * s3) - xf;
		float eq2 = y0 + s1 * sinc((1 / 2.0) * k0 * s1) * sin(th0 + (1 / 2.0) * k0 * s1) + s2 * sinc((1 / 2.0) * k1 * s2) * sin(th0 + k0 * s1 + (1 / 2.0) * k1 * s2) + s3 * sinc((1 / 2.0) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1 / 2.0) * k2 * s3) - yf;
		float eq3 = mod2pi(x0 * s1 + k1 * s2 + k2 * s3 + th0 + thf);

		float sqt = sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3);
		double thresh = 1e-10;

		bool tmp = ((s1 > 0) || (s2 > 0) || (s3 > 0)) && (sqt < thresh);

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
			sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
			sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_kmax);
			sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_kmax * (sc_s2 - sc_s1));
			ctrl = true;
		}
	}

	// ----- Function to implement a structure representing a Dubins arc for both straight and circular ----------------------------------------------------
	void set_DBNarc(DubinsArc &ptr, const Pose2D &start, float k, float s)
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
	void set_DBNcurve(DubinsCurve &curve, const Pose2D &start, float s1, float s2, float s3, float k0, float k1, float k2)
	{

		set_DBNarc(curve.arc_1, start, k0, s1);
		set_DBNarc(curve.arc_2, curve.arc_1.end, k1, s2);
		set_DBNarc(curve.arc_3, curve.arc_2.end, k2, s3);
		curve.L = curve.arc_1.s + curve.arc_2.s + curve.arc_3.s;
	}

	// ----- Function to find the shortest path ---------------------------------------------------------------------------------------------
	void DBN_shortest(DubinsCurve &curve, Pose2D start, Pose2D end, float const &kmax)
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
				{1, 0, -1},  //LSR
				{-1, 0, 1},  //RSL
				{-1, 1, -1}, //RLR
				{1, -1, 1}}; //LRL

		int pidx = -1;
		double L = numeric_limits<double>::infinity(); // Infinite value
		bool ctrl;
		float s1, s2, s3;
		float sc_s1, sc_s2, sc_s3;
		float sc_s1_c, sc_s2_c, sc_s3_c;
		int Lcur;

		for (size_t i = 0; i < 6; i++)
		{
			primitives[i](sc_th0, sc_thf, sc_kmax, ctrl, sc_s1_c, sc_s2_c, sc_s3_c);

			Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
			if (ctrl and Lcur < L)
			{
				L = Lcur;
				sc_s1 = sc_s1_c;
				sc_s2 = sc_s2_c;
				sc_s3 = sc_s3_c;
				pidx = i;
			}
		}

		if (pidx >= 0)
		{ // ----------------  here we transform problem to standard form & we construct the Dubins curve  ----------------------------------------

			scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);

			// Construct Dubins curve
			set_DBNcurve(curve, start, s1, s2, s3, ksigns[pidx][0] * kmax, ksigns[pidx][1] * kmax, ksigns[pidx][2] * kmax);
			
			// Check correctess of solution
			if (!check(s1, ksigns[pidx][0] * sc_kmax,
						s2, ksigns[pidx][1] * sc_kmax,
						s3, ksigns[pidx][2] * sc_kmax,
						sc_th0, sc_thf))
				throw std::logic_error("DUBINS - INCORRECT COMPUTED SOLUTION");
				
		}
	}

	//----------------- This last discretization step is needed in order to create smaller arcs that the robots will follow ----------------------

	void discretize_arc(DubinsArc &full_arc, float &s, int &npts, std::vector<Path> &path)
	{

		int tmp = 0;
		while (tmp <= npts)
		{
			DubinsArc small_arc;
			float s_local = full_arc.s / npts * tmp;
			tmp++;
			set_DBNarc(small_arc, full_arc.start.x, full_arc.start.y, full_arc.start.theta, full_arc.k, s_local);
			path.points.emplace_back(s_local, small_arc.end.x, small_arc.end.y, small_arc.end.theta, small_arc.k);

			s = s + full_arc.s / npts;
		}
	}
} // dubins

#pragma once

#include <vector>
#include<iostream>
#include "utils.hpp"

using namespace std;

float sinc(float t);

float mod2pi(float angle);

float rangeSymm(float const& angle);

// ---------------- Struct defining the position of the robot ---------------------------------------------------------------------------

struct RBTpos {
    float x; 
    float th; 
    float y; 
};  

// ------------------ Struct for a Dubins arc -------------------------------------------------------------------------------------------
struct DBNarc {
	
    RBTpos init;  //--- inital robot pose coordinates
    float k, s;
    RBTpos fin; //--- final robot pose coordinates
};


// --------------- Struct for a Dubins curve ---------------------------------------------------------------------------------------------
struct DBNcurve {
	float L;
    DBNarc arc_1;
    DBNarc arc_2;
    DBNarc arc_3;
	
};




bool control(float const& s1, float const& k0, float const& s2, float const& k1, float const& s3, float const& k2, RBTpos init, RBTpos fin);
// -------- Scaling the functions ------------------------------------------
void scaleToStandard(RBTpos init, RBTpos fin,  float kmax, float& sc_th_init, float& sc_th_fin, float& sc_kmax, float& lambda);

void scaleFromStandard(float lambda, float sc_s1, float sc_s2, float sc_s3, float& s1, float& s2, float& s3);

// --- Dubins functions ----------------------------------------------------


void RSR(float sc_th_init, float sc_th_fin, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3);

void RSL(float sc_th_init, float sc_th_fin, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3);  

void RLR(float sc_th_init, float sc_th_fin, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3);

void LSR(float sc_th_init, float sc_th_fin, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3);

void LSL(float sc_th_init, float sc_th_fin, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3);

void LRL(float sc_th_init, float sc_th_fin, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3);





void set_DBNarc(DBNarc& ptr, RBTpos init,  float k, float s);

void set_DBNcurve(DBNcurve& curve_ptr, RBTpos init, float s1, float s2, float s3, float k0, float k1, float k2);

// ----- Find the shortest path ---------------------------------------------
void DBN_shortest(DBNcurve& curve, RBTpos init,  RBTpos fin, float const& kmax);

// ----- Discretization of the arc ------------------------------------------
void discretize_arc(DBNarc& full_arc, float& s, int& npts, vector<Path>& path);


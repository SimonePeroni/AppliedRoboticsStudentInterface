#include <iostream>
#include <vector>
#include <cmath>
#include<math.h>
#include<algorithm>
#include <cstdlib>
#include "Dub_curves.h"
#include "utils.hpp"	
#include "limits.h"

using namespace std;

// ------- Sinc function -----------------------------------------------------
float sinc(float t){
    float tsinc =0.0;
    float p = 2.0;
    float j;

	if (abs(t) < 0.002){
        j = pow(t,p)/6.0*(1.0-pow(t,p)/20.0); // ---- Taylor series approximatio0n
        tsinc = 1 - j;
	} else {
		tsinc = sin(t)/t;
	
    } return tsinc;}


// --------------- We need this function to normalize the angle in the range 0 to 360 deg --------------------------------

float mod2pi(float angle){

    float p = 2.0;
	while (angle < 0){
		angle = angle + p*M_PI;
	}
	while (angle >= p*M_PI){
		angle = angle - p*M_PI;

	} return angle;}

// -------------- We need this function to normalize the angular difference -----------------------------
float rangeSymm(float const& angle){
	float oput;
    oput = angle;
    float p = 2.0;
	while (oput < 0){
		oput = oput + p*M_PI;
	}
	while (oput >= p*M_PI){
		oput = oput - p*M_PI;
	} return oput;}

// -------------------------------------- 
bool control(float const& s1, float const& k0, float const& s2, float const& k1, float const& s3, float const& k2, RBTpos init, RBTpos fin){
	
  float x0 = init.x = -1; float xf = fin.x = 1;
  float y0 = init.y = 0; float yf = fin.y = 0;

	float eq1 = x0 + s1*sinc((1/2.0)*k0*s1)*cos(init.th+(1/2.0)*k0*s1) + s2*sinc((1/2.0)*k1*s2)*cos(init.th + k0*s1 + (1/2.0)*k1*s2) + s3*sinc((1/2.0)*k2*s3)*cos(init.th + k0*s1 + k1*s2 + (1/2.0)*k2*s3) - xf;
	float eq2 = y0 + s1*sinc((1/2.0)*k0*s1)*sin(init.th+(1/2.0)*k0*s1) + s2*sinc((1/2.0)*k1*s2)*sin(init.th + k0*s1 + (1/2.0)*k1*s2) + s3*sinc((1/2.0)*k2*s3)*sin(init.th + k0*s1 + k1*s2 + (1/2.0)*k2*s3) - yf;
	float eq3 =rangeSymm(x0*s1 + k1*s2 + k2*s3 + init.th + fin.th);

   float sqt = sqrt(pow(eq1, 2)+pow(eq2, 2)+ pow(eq3, 2));
   double thresh= 1e-10;

	bool tmp = ((s1>0) or (s2>0) or (s3>0)) and (sqt < thresh) ;

	return tmp;
}
// ------------------------------------------------------------------------------------------------------------------------ //

void scaleToStandard(RBTpos init, RBTpos fin, float kmax, float& sc_th0, float& sc_thf, float& sc_kmax, float& lambda){

	lambda = hypot((fin.y -init.y),(fin.x - init.x))/2.0;
    sc_th0 = mod2pi(init.th- atan2((fin.y -init.y),(fin.x - init.x)));
	sc_thf = mod2pi(fin.th-atan2((fin.y -init.y),(fin.x - init.x)));
	sc_kmax = kmax * lambda;
}

// --- These two functions are used to scale from and to the standard form


void scaleFromStandard(float lambda, float sc_s1, float sc_s2, float sc_s3, float& s1, float& s2, float& s3){
	
	s1 = sc_s1*lambda; s2 = sc_s2*lambda; s3 = sc_s3*lambda;	}

// ------------------------------------------------------------------------------------------------------------------------- //

// ---------- Implementation of the primitives functions following the guidelines given by the professor in class --------------------
void LSL(float sc_th0, float sc_thf, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3){
	float invK = 1.0/sc_kmax;
	float C = cos(sc_thf) - cos(sc_th0);
	float S = 2.0*sc_kmax + sin(sc_th0) - sin(sc_thf);
	float temp1 = atan2(C,S);
	sc_s1 = invK*mod2pi(temp1-sc_th0);
	float temp2 = 2.0+4.0*pow(sc_kmax,2)-2*cos(sc_th0-sc_thf)+4*sc_kmax*(sin(sc_th0)-sin(sc_thf));
	if (temp2 < 0){
		sc_s1 = 0;
		sc_s2 = 0;
		sc_s3 = 0;
        ctrl= false;
	}else{
	sc_s2 = invK*sqrt(temp2);
	sc_s3 = invK*mod2pi(sc_thf-temp1);
	ctrl = true;}
}


void RSR(float sc_th0, float sc_thf, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3){
	float invK = 1.0/sc_kmax;
	float C = cos(sc_th0) - cos(sc_thf);
	float S = 2.0*sc_kmax - sin(sc_th0) + sin(sc_thf);
	float temp1 = atan2(C,S);
	sc_s1 = invK*mod2pi(sc_th0-temp1);
	float temp2 = 2.0+4.0*pow(sc_kmax,2)-2*cos(sc_th0-sc_thf)-4*sc_kmax*(sin(sc_th0)-sin(sc_thf));
	if (temp2 < 0){
		sc_s1 = 0;
		sc_s2 = 0;
		sc_s3 = 0;
        ctrl = false;
	}else{
	sc_s2 = invK*sqrt(temp2);
	sc_s3 = invK*mod2pi(temp1-sc_thf);
	ctrl = true;}
}


void LSR(float sc_th0, float sc_thf, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3){
	float invK = 1.0/sc_kmax;
	float C = cos(sc_th0) + cos(sc_thf);
	float S = 2.0*sc_kmax + sin(sc_th0) + sin(sc_thf);
	float temp1 = atan2(-C,S);
	float temp3 = 4.0*pow(sc_kmax,2)-2.0+2.0*cos(sc_th0-sc_thf)+4.0*sc_kmax*(sin(sc_th0)+sin(sc_thf));

	if (temp3 < 0){
		sc_s1 = 0;
		sc_s2 = 0;
		sc_s3 = 0;
        ctrl = false;
	}else{
	sc_s2 = invK*sqrt(temp3);
	float temp2 = -atan2(-2.0,sc_s2*sc_kmax);
	sc_s1 = invK*mod2pi(temp1+temp2-sc_th0);
	sc_s3 = invK*mod2pi(temp1+temp2-sc_thf);
	ctrl = true;}

}


void RSL(float sc_th0, float sc_thf, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3){
	float invK = 1.0/sc_kmax;
	float C = cos(sc_th0) + cos(sc_thf);
	float S = 2.0*sc_kmax - sin(sc_th0) - sin(sc_thf);
	float temp1 = atan2(C,S);
	float temp3 = 4.0*pow(sc_kmax,2)-2.0+2.0*cos(sc_th0-sc_thf)-4.0*sc_kmax*(sin(sc_th0)+sin(sc_thf));
	if (temp3 < 0){
		sc_s1 = 0;
		sc_s2 = 0;
		sc_s3 = 0;
        ctrl = false;
	}else{
	sc_s2 = invK*sqrt(temp3);
	float temp2 = atan2(2.0,sc_s2*sc_kmax);
	sc_s1 = invK*mod2pi(sc_th0-temp1+temp2);
	sc_s3 = invK*mod2pi(sc_thf-temp1+temp2);
	ctrl = true;}
}


void RLR(float sc_th0, float sc_thf, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3){
	float invK = 1.0/sc_kmax;
	float C = cos(sc_th0) - cos(sc_thf);
	float S = 2.0*sc_kmax - sin(sc_th0) + sin(sc_thf);
	float temp1 = atan2(C,S);
	float temp2 = 0.125*(6.0-4.0*pow(sc_kmax,2)+2.0*cos(sc_th0-sc_thf)+4.0*sc_kmax*(sin(sc_th0)-sin(sc_thf)));
	if (abs(temp2) > 1.0){
		sc_s1 = 0;
		sc_s2 = 0;
		sc_s3 = 0;
        ctrl = false;
		}else{
	sc_s2 = invK*mod2pi(2.0*M_PI-acos(temp2));
	sc_s1 = invK*mod2pi(sc_th0-temp1+0.5*sc_s2*sc_kmax);
	sc_s3 = invK*mod2pi(sc_th0-sc_thf+sc_kmax*(sc_s2-sc_s1));
	ctrl = true;}
}


void LRL(float sc_th0, float sc_thf, float sc_kmax, bool& ctrl, float& sc_s1, float& sc_s2, float& sc_s3){
	float invK = 1.0/sc_kmax;
	float C = cos(sc_thf) - cos(sc_th0);
	float S = 2.0*sc_kmax + sin(sc_th0) - sin(sc_thf);
	float temp1 = atan2(C,S);
	float temp2 = 0.125*(6.0-4.0*pow(sc_kmax,2)+2.0*cos(sc_th0-sc_thf)+4.0*sc_kmax*(sin(sc_th0)-sin(sc_thf)));
	if (abs(temp2) > 1.0){
		
		sc_s1 = 0;
		sc_s2 = 0;
		sc_s3 = 0;
		ctrl= false;
	}else{
	sc_s2 = invK*mod2pi(2*M_PI-acos(temp2));
	sc_s1 = invK*mod2pi(temp1-sc_th0+0.5*sc_s2*sc_kmax);
	sc_s3 = invK*mod2pi(sc_thf-sc_th0+sc_kmax*(sc_s2-sc_s1));
	 ctrl= true;}
}

// ----- Function to implement a structure representing a Dubins arc for both straight and circular ----------------------------------------------------
void set_DBNarc(DBNarc& ptr, RBTpos init,  float k, float s){
	
	ptr.init.x = init.x;
	ptr.init.y = init.y;
	ptr.init.th = init.th;
	ptr.k = k;
	ptr.s = s;
	ptr.fin.x = init.x + s * sinc(k*s/2.0) * cos(init.th+k*s/2.0);
	ptr.fin.y = init.y+ s * sinc(k*s/2.0) * sin(init.th+k*s/2.0);
	ptr.fin.th = mod2pi(init.th+k*s);

}

// ----- Function to implement a structure representing 3 arcs which make up the Dubins curve ------------------------------------------
void set_DBNcurve(DBNcurve& curve_ptr, RBTpos init, float s1, float s2, float s3, float k0, float k1, float k2){
    
	set_DBNarc(curve_ptr.arc_1, init, k0, s1);
	set_DBNarc(curve_ptr.arc_2, curve_ptr.arc_1.fin.x,curve_ptr.arc_1.fin.y, curve_ptr.arc_1.fin.th, k1, s2);
	set_DBNarc(curve_ptr.arc_3, curve_ptr.arc_2.fin.x, curve_ptr.arc_2.fin.y, curve_ptr.arc_2.fin.th, k2, s3);
	curve_ptr.L = curve_ptr.arc_1.s + curve_ptr.arc_2.s + curve_ptr.arc_3.s;
}


// ----- Function to find the shortest path ---------------------------------------------------------------------------------------------
void DBN_shortest(DBNcurve& curve, RBTpos init,  RBTpos fin, float const& kmax){
	float sc_th0, sc_thf, sc_kmax;
    float lambda;
	scaleToStandard(init, fin, kmax, sc_th0, sc_thf, sc_kmax, lambda);
	
	typedef void (*type0)(float,float,float,bool&,float&,float&,float&);

	void(*LSL_ptr)(float,float,float,bool&,float&,float&,float&)= &LSL;
	void(*RSR_ptr)(float,float,float,bool&,float&,float&,float&)= &RSR;
	void(*LSR_ptr)(float,float,float,bool&,float&,float&,float&)= &LSR;
	void(*RSL_ptr)(float,float,float,bool&,float&,float&,float&)= &RSL;
	void(*RLR_ptr)(float,float,float,bool&,float&,float&,float&)= &RLR;
	void(*LRL_ptr)(float,float,float,bool&,float&,float&,float&)= &LRL;
	
	type0 primitives[6] = {LSL_ptr,RSR_ptr,LSR_ptr,RSL_ptr,RLR_ptr,LRL_ptr}; 
	int ksigns[6][3] = {{1,0,1},{-1,0,-1},{1,0,-1},{-1,0,1},{-1,1,-1},{1,-1,1}};

	int pidx = -1.0;
	double L = numeric_limits<double>::infinity();	// Infinite value
	bool ctrl;
	float s1, s2, s3;
	float sc_s1, sc_s2, sc_s3;
	float sc_s1_c, sc_s2_c, sc_s3_c;
	int Lcur;
    int tmp = 0;
	while (tmp < 6){
		primitives[tmp](sc_th0, sc_thf, sc_kmax, ctrl, sc_s1_c, sc_s2_c, sc_s3_c);

		Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
		if (ctrl and Lcur < L){
			L = Lcur;
			sc_s1 = sc_s1_c;
			sc_s2 = sc_s2_c;
			sc_s3 = sc_s3_c;
			pidx = tmp;
		}
         tmp++;
	}

	if (pidx >= 0){ // ----------------  here we transform problem to standard form & we construct the Dubins curve  ----------------------------------------
		
		scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);

		// Construct Dubins curve		
		set_DBNcurve(curve, init, s1, s2, s3, ksigns[pidx][0]*kmax, ksigns[pidx][1]*kmax, ksigns[pidx][2]*kmax);
	}

}

//----------------- This last discretization step is needed in order to create smaller arcs that the robots will follow ----------------------  

void discretize_arc(DBNarc& full_arc, float& s, int& npts, std::vector<Path>& path){

   int tmp = 0;
	while(tmp <= npts){
		DBNarc small_arc;
		float s_local = full_arc.s/npts*tmp; 
        tmp++;
		set_DBNarc(small_arc, full_arc.init.x, full_arc.init.y, full_arc.init.th, full_arc.k, s_local);
        path.points.emplace_back(s_local, small_arc.fin.x, small_arc.fin.y, small_arc.fin.th, small_arc.k);

		s = s + full_arc.s/npts;

	} 

}


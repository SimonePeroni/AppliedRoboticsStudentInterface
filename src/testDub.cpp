
#include <cmath>
#include<iostream>
#include "Dub_curves.h"
using namespace std; 

int main() {

    RBTpos posF, posI; // initial & Final pose 

    // initial pose
    posI.x = 15;
    posI.y = 12;
    posI.th = M_PI/4; // 

     // final pose 

    posF.x = 4;
    posF.y = 0;
    posF.th = M_PI/4;


    float Kmax = 16.0;  // max angle 


    // Temporary s values
	float s1 = 2.0;	
	float s2 = 2.0;
	float s3 = 2.0;

	// Temporary k values
	float k0 =	4.0;
	float k1 =	4.0;
	float k2 =	4.0;


// --------------------------------------------------------------------
	DBNcurve curve;
	set_DBNcurve(curve,posI,s1,s2,s3,k0,k1,k2);
	DBN_shortest(curve, posI, posF,Kmax);

bool print = true ;

if (print){

    cout << curve.arc_1.init.x <<endl;
    cout << curve.arc_1.init.y <<endl;
	

	cout << curve.arc_1.fin.x <<endl;
    cout << curve.arc_1.fin.y <<endl;
    cout<<'*********************'<<endl; 
    cout << curve.arc_2.init.x <<endl;
    cout << curve.arc_2.init.y <<endl;
	
    cout << curve.arc_2.fin.x <<endl;
    cout << curve.arc_2.fin.y <<endl;
    cout<<'*********************'<<endl; 
	cout << curve.L <<endl;}


	return 0;}



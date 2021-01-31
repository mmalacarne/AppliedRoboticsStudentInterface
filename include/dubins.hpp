#ifndef __DUBINS_HPP__
#define __DUBINS_HPP__

#include <cmath>
#include <tuple>
#include <vector>
#include <string>
#include <assert.h>
#include <iostream>

#include "matplotlibcpp.h"

//**********************************************************************
// AUXILIARY UTILITY FUNCTION
//**********************************************************************
double sinc(double t);

double mod2pi(double ang);

double rangeSymm(double ang);

bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf);

std::tuple<double, double, double> circline(double s, double x0, double y0, double th0, double k);



//**********************************************************************
// DATA STRUCTURES AND GETTERS
//**********************************************************************
typedef struct{
	double x0;
    double y0;
    double th0;
    double k;
    double L;
    double xf;
    double yf;
    double thf;
}arc;

typedef struct{
	arc arc1;
    arc arc2;
    arc arc3;
    double L;
}curve;

arc getDubinsArc(double x0, double y0, double th0, double k, double L);

curve getDubinsCurve(double x0, double y0, double th0, double s1,double s2, double s3, double k0, double k1, double k2);



//**********************************************************************
// SCALING FUNCTIONS
//**********************************************************************
/*std::tuple<double, double, double, double> scaleOrig2Std(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);

std::tuple<double, double, double> scaleStd2Orig(double lambda, double sc_s1, double sc_s2, double sc_s3);*/



//**********************************************************************
// DUBINS PRIMITIVES
//**********************************************************************
std::tuple<bool, double, double, double> LSL(double sc_th0, double sc_thf, double sc_Kmax);

std::tuple<bool, double, double, double> RSR(double sc_th0, double sc_thf, double sc_Kmax);

std::tuple<bool, double, double, double> LSR(double sc_th0, double sc_thf, double sc_Kmax);

std::tuple<bool, double, double, double> RSL(double sc_th0, double sc_thf, double sc_Kmax);

std::tuple<bool, double, double, double> RLR(double sc_th0, double sc_thf, double sc_Kmax);

std::tuple<bool, double, double, double> LRL(double sc_th0, double sc_thf, double sc_Kmax);



//**********************************************************************
// DUBINS SHORTEST PATH
//**********************************************************************
std::pair<int, curve> dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);



//**********************************************************************
// PLOTTING FUNCTIONS
//**********************************************************************
std::tuple<std::vector<double>, std::vector<double>> getPlottableArc(arc a);

void plotDubins(curve c);



//**********************************************************************
// DUBINS CLASS
//**********************************************************************
class DubinsProblem{
private:
    double x0, y0, th0, xf, yf, thf, kmax, k_thj, m;
    int pts_counter;
    std::vector<std::tuple<double, double, double>> all_pts_thj; // <pt_x, pt_y, th_j>
    std::vector<std::pair<int, curve>> all_best_curves; // <pidx, curve>
public:
    DubinsProblem(double x0, double y0, double th0, double xf, double yf, double thf, double kmax);
    ~DubinsProblem();
    void setK_thj(double k_thj);
    void setM(double m);
    void addMiddlePt(double ptx, double pty);
    void printInfo();
    void findShortestPath();
    void solveDubins();
    void printSolutionPts();
    void plot();
};

#endif

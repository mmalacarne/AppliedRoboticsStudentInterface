#ifndef __COLLISION_DETECTION_MOD_HPP__
#define __COLLISION_DETECTION_MOD_HPP__

#include <cmath>
#include <tuple>
#include <vector>
#include <string>
#include <stdlib.h> 
#include <iostream>
#include <opencv2/opencv.hpp>

#include "dubins.hpp" // auxiliary utility functions required (i.e. circline..)
//#include "utils.hpp"
#include "matplotlibcpp.h"

//**********************************************************************
// PUBLIC FUNCTION FOR COLLISION DETECTION
//**********************************************************************
std::tuple<bool, double, double> intersLineLine(double L1_x0, double L1_y0, double L1_xf, double L1_yf, 
	double L2_x0, double L2_y0, double L2_xf, double L2_yf);

std::tuple<bool, double, double, double, double, double, double> intersCircleLine(double cx, double cy, double r, 
	double L_x0, double L_y0, double L_xf, double L_yf);

std::tuple<bool, double, double, double, double> intersArcLine(double arc_x0, double arc_y0, double arc_th0, 
	double arc_xf, double arc_yf, double arc_L, double arc_k, 
	double L_x0, double L_y0, double L_xf, double L_yf);

//bool intersPtPolygon(double pt_x, double pt_y, Polygon p);

#endif

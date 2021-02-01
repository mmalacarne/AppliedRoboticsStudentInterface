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
// PUBLIC DATA STRUCTURES AND GETTERS
//**********************************************************************
typedef struct{
	double x0;
	double y0;
	double xf;
	double yf;

	/*segment(double x0, double y0, double xf, double yf):
		x0(x), y0(y), xf(xf), yf(yf)
	{}*/
}segment;

typedef struct{
	double x;
	double y;
	double r;
}circle;

segment getSegment(double x0, double y0, double xf, double yf);

circle getCircle(double x, double y, double r);

//**********************************************************************
// PUBLIC FUNCTION FOR COLLISION DETECTION
//**********************************************************************
std::tuple<bool, double, double> intersLineLine(segment L1, segment L2);

std::tuple<bool, double, double, double, double, double, double> intersCircleLine(circle c, segment L);

std::tuple<bool, double, double, double, double> intersArcLine(arc a, segment L);

//bool intersPtPolygon(double pt_x, double pt_y, Polygon p);

#endif

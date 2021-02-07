#ifndef __PRM_HPP__
#define __PRM_HPP__

#include <map>
#include <cmath>
#include <tuple>
#include <vector>
#include <string>
#include <stdlib.h> 
#include <iostream>
#include <random>
#include <set>
#include <assert.h>

//#include "dubins.hpp"
#include "collisionDetectionModule.hpp"
#include "utils.hpp"
#include "matplotlibcpp.h"

//**********************************************************************
// PUBLIC PRM
//**********************************************************************
void getGraph(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
	const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
	const Point& robot_bc, const int n_pts, const int knn, 
	std::map<Point,std::vector<Point>>& graph);

void getDijkstraPath(const Point& q_i, const Point& q_f, 
	const std::map<Point,std::vector<Point>>& graph, 
	const std::vector<Polygon>& obstacle_list, std::vector<Point>& g_path);

void pathSmoother(const std::vector<Point>& g_path, 
	const std::vector<Polygon>& obstacle_list, std::vector<Point>& s_path);

#endif
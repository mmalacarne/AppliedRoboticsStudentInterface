#ifndef __PATH_PLANNING_HPP__
#define __PATH_PLANNING_HPP__

#include "utils.hpp"
#include "dubins.hpp"
#include "collisionDetectionModule.hpp"
#include "probabilisticRoadMap.hpp"
#include "matplotlibcpp.h"

#include <map>
#include <cmath>
#include <vector>
#include <tuple>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

bool my_planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
	const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
	const float x, const float y, const float theta, Path& path, const std::string& config_folder);

#endif

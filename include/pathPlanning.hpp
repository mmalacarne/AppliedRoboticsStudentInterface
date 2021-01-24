#include "utils.hpp"
#include "dubins.hpp"
//#include "dubinsPrimitives.hpp"

#include <cmath>
#include <vector>
#include <tuple>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

bool my_planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
	const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
	const float x, const float y, const float theta, Path& path, const std::string& config_folder);



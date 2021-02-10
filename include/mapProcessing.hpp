#ifndef __MAP_PROCESSING_HPP__
#define __MAP_PROCESSING_HPP__

#include "utils.hpp"
#include "clipper.hpp"

#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <experimental/filesystem>

bool my_processMap(const cv::Mat& img_in, const double scale,  std::vector<Polygon>& obstacles_list, 
	std::vector<std::pair<int,Polygon>>& victims_list, Polygon& gate, const std::string& config_folder);

bool my_findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, 
	double& theta, const std::string& config_folder);

#endif

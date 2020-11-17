#include "utils.hpp"

#include <vector>
#include <opencv2/opencv.hpp>

bool processMap(const cv::Mat& img_in, const double scale,  std::vector<Polygon>& obstacle_list, 
	std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder);

bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, 
	double& theta, const std::string& config_folder);

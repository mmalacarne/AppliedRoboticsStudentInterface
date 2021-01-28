#ifndef __IMAGE_MANAGER_HPP__
#define __IMAGE_MANAGER_HPP__

#include <vector>
#include <dirent.h>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>

void my_loadImage(cv::Mat& img_out, const std::string& config_folder);

void my_genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder);

#endif

//#ifndef __MY_EXTRINSIC_CALIB_H__
//#define __MY_EXTRINSIC_CALIB_H__

#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <vector>
#include <string>
#include <experimental/filesystem>
#include <atomic>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>

bool my_extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, 
    const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder);

//#endif
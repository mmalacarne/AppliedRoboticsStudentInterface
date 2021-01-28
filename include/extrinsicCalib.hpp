#ifndef __EXTRINSIC_CALIB_HPP__
#define __EXTRINSIC_CALIB_HPP__

#include <unistd.h>
#include <vector>
#include <string>
#include <atomic>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>

bool my_extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, 
    const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder);

void my_imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
    const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder);

void my_findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, 
    const std::vector<cv::Point3f>& object_points_plane, const std::vector<cv::Point2f>& dest_image_points_plane, 
    cv::Mat& plane_transf, const std::string& config_folder);

void my_unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
    const std::string& config_folder);

#endif
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "imageManager.hpp"
#include "extrinsicCalib.hpp"
#include "mapProcessing.hpp"
#include "pathPlanning.hpp"

#include <stdexcept>
#include <experimental/filesystem>
#include <opencv2/opencv.hpp>

namespace student {
  void loadImage(cv::Mat& img_out, const std::string& config_folder){
    //log
    //std::cout << "STUDENT FUNCTION - loadImage" << std::endl;

    /*
    //path p = std::experimental::filesystem::current_path();
    //std::cout << std::experimental::filesystem::current_path() << std::endl;
    */

    /*
    //workaround here: ok that i have to load image... but which one???
    static std::string img_name = "00.jpg";
    static std::string path = config_folder + "/camera_image0/" + img_name;

    // as in first lab session
    img_out = cv::imread(path, cv::IMREAD_COLOR);

    if(img_out.empty()){
      std::cout << "Could not open or find the image" << std::endl;
      exit(EXIT_FAILURE);
    }

    cv::namedWindow("Loaded_img", cv::WINDOW_AUTOSIZE);
    cv::imshow("Loaded_img", img_out);
    cv::waitKey(0);
    */

    my_loadImage(img_out, config_folder);
  }

  void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){

    my_genericImageListener(img_in, topic, config_folder);
  }

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, 
    const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){

    return my_extrinsicCalib(img_in, object_points, camera_matrix, rvec, tvec, config_folder);
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
    const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    
    my_imageUndistort(img_in, img_out, cam_matrix, dist_coeffs, config_folder);
  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, 
    const std::vector<cv::Point3f>& object_points_plane, const std::vector<cv::Point2f>& dest_image_points_plane, 
    cv::Mat& plane_transf, const std::string& config_folder){

    my_findPlaneTransform(cam_matrix, rvec, tvec, object_points_plane, 
    dest_image_points_plane, plane_transf, config_folder);
  }

  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
    const std::string& config_folder){

    my_unwarp(img_in, img_out, transf, config_folder);
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacles_list, 
    std::vector<std::pair<int,Polygon>>& victims_list, Polygon& gate, const std::string& config_folder){
    
    return my_processMap(img_in, scale, obstacles_list, victims_list, gate, config_folder);
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, 
    double& x, double& y, double& theta, const std::string& config_folder){
    
    return my_findRobot(img_in, scale, triangle, x, y, theta, config_folder);  
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
    const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
    const float x, const float y, const float theta, Path& path, const std::string& config_folder){

    return my_planPath(borders, obstacle_list, victim_list, gate, x, y, theta, path, config_folder);
  }
}


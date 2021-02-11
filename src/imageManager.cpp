#include "imageManager.hpp"

#define LOG_LI
#define LOG_GIL
#define DEBUG_LI

namespace stdfs = std::experimental::filesystem;

//**********************************************************************
// PUBLIC FUNCTION
//**********************************************************************
/*!
* "Mask" for loadImage. For more info look in student::loadImage docs.
* Replace the use of the simulator -> useful for testing the shape detection 
* on real picture.
*/
void my_loadImage(cv::Mat& img_out, const std::string& config_folder){
  #ifdef LOG_LI
    std::cout << "STUDENT FUNCTION - my_loadImage" << std::endl;
  #endif

  #ifdef DEBUG_LI
    //static std::string dir_path = "/home/mala/Desktop/RP_Lab/Lab5_07-10-20/lect8/demo_shape_detection/test/";
    //static std::string jpg_file = dir_path + "test_arena.jpg";
    static std::string dir_path = "/home/mala/Desktop/Material-20210211/camera_image_raw/";
    static std::string jpg_file = dir_path + "000.jpg";
  #else
    static std::string dir_path = config_folder + "/img_to_load/";
    static std::string jpg_file = dir_path + "*.jpg";
  #endif
  
  img_out = cv::imread(jpg_file);
}

/*!
* "Mask" for genericImageListener. For more info look in student::genericImageListener docs.
* Useful for saving chessboard pictures on which to perform the camera calibration.
*/
void my_genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
	#ifdef LOG_GIL
    std::cout << "STUDENT FUNCTION - my_genericImageListener" << std::endl;
  #endif

  static int dir_id = 0;
  static std::string path = config_folder + "/camera_image" + std::to_string(dir_id) + "/";
  static bool folder_exists = stdfs::exists(path);

  while (folder_exists){
    path = config_folder + "/camera_image" + std::to_string(++dir_id) + "/";
    folder_exists = stdfs::exists(path);
  }

  stdfs::create_directories(path);

  static int img_id = 0;  
  cv::imshow(topic, img_in);
  char key = cv::waitKey(30);
  std::stringstream img_file;

  switch (key){      
  case 's':   
    img_file << path << std::setw(2)  << std::setfill('0') << (img_id++) << ".jpg";
    cv::imwrite(img_file.str(), img_in);
    std::cout << "Saved image " << img_file.str() << std::endl;
    break;
  default:
    break;
  }
}

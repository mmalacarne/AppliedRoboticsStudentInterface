#include "imageManager.hpp"

namespace stdfs = std::experimental::filesystem;

//**********************************************************************
// PRIVATE FUNCTION
//**********************************************************************

/*!
* Support function which retrives all the images path inside a given directory
* @param[in]
* @param[out] 
*/
std::vector<std::string> get_filenames(std::experimental::filesystem::path path){
	//namespace stdfs = std::experimental::filesystem;
  std::vector<std::string> filenames;
  const stdfs::directory_iterator end{};

  for(stdfs::directory_iterator iter{path}; iter != end; ++iter){
  	if(stdfs::is_regular_file(*iter))
  		filenames.push_back(iter->path().string());
  }

  return filenames;
}

//**********************************************************************
// PUBLIC FUNCTION
//**********************************************************************

void my_loadImage(cv::Mat& img_out, const std::string& config_folder){
	//log
  std::cout << "STUDENT FUNCTION - my_loadImage" << std::endl;

	static std::string dir_path = config_folder + "/camera_image";

  for (const auto& img_path: get_filenames(dir_path)){
    img_out = cv::imread(img_path, cv::IMREAD_COLOR);

    if(img_out.empty()){
      std::cout << "Could not open or find the image" << std::endl;
      exit(EXIT_FAILURE);
    }

    cv::namedWindow("Loaded_img", cv::WINDOW_AUTOSIZE);
    cv::imshow("Loaded_img", img_out);
    cv::waitKey(0);
  }
}

void my_genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
	//log
  std::cout << "STUDENT FUNCTION - my_genericImageListener" << std::endl;
  /*
  std::cout << std::experimental::filesystem::current_path() << std::endl;
  std::cout << "config_folder = " << config_folder << std::endl;
  */

  static int dir_id = 0;
  static std::string path = config_folder + "/camera_image" + std::to_string(dir_id) + "/";
  //static bool folder_exist = std::experimental::filesystem::exists(path);
  static bool folder_exist = stdfs::exists(path);

  while (folder_exist){
    path = config_folder + "/camera_image" + std::to_string(++dir_id) + "/";
    //folder_exist = std::experimental::filesystem::exists(path);
    folder_exist = stdfs::exists(path);
  }

  //std::experimental::filesystem::create_directories(path);
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

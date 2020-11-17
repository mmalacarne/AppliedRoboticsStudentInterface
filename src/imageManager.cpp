#include "imageManager.hpp"

#define DEBUG_LI
#define DEBUG_GIL

namespace stdfs = std::experimental::filesystem;

//**********************************************************************
// PRIVATE FUNCTION
//**********************************************************************

/*!
* Support function which retrives all the images path inside a given directory
* @param[in]
* @param[out] 
*/
void get_filenames(std::string path, std::vector<std::string>& img_list){
  /*for (auto& fn: stdfs::directory_iterator(path)){
    if (stdfs::is_regular_file(fn))
  		img_list.push_back(fn.path());
  }*/
  const char* dir_path = path.c_str();
  DIR* dir = opendir(dir_path);
  struct dirent* dir_reader;

  if (dir == nullptr){
    perror("get_filenames - error in opendir");
    throw std::logic_error( "get_filenames - cannot open: " +  path);
  }

  while ((dir_reader = readdir(dir)) != nullptr){
    std::string fn = dir_reader->d_name;    

    if (fn != "." & fn != ".."){
      std::string img_path = path + "/" + fn;
      img_list.push_back(img_path);
    }
  }
  closedir(dir);

  /*for(auto& ip: img_list)
    std::cout << "File name: " << ip << std::endl;*/
}

//**********************************************************************
// PUBLIC FUNCTION
//**********************************************************************

void my_loadImage(cv::Mat& img_out, const std::string& config_folder){
	#ifdef DEBUG_LI
  std::cout << "STUDENT FUNCTION - my_loadImage" << std::endl;
  #endif

  static bool init = false;
  #ifdef DEBUG_LI
    static std::string dir_path = "/home/mala/Desktop/RP_Lab/Lab5_07-10-20/lect8/demo_shape_detection/test";
  #else
	 static std::string dir_path = config_folder + "/img_to_load";
  #endif  
  static size_t img_id = 0;
  static std::vector<std::string> filenames;
  get_filenames(dir_path, filenames);
  
  if (!init){
    if (!filenames.size() > 0) 
      throw std::logic_error("my_loadImage - There is no file in: " +  dir_path);
    /*else
      throw std::logic_error("OK");*/

    init = true;
    img_id = 0;

    #ifdef DEBUG_LI
      std::cout << "init == false" << std::endl;
    #endif
  }else{
    // some img has already been read --> keep it until 'n' is pressed
    // when 'n' is pressed --> update img_id
    #ifdef DEBUG_LI
      std::cout << "init == true" << std::endl;
    #endif

    char key = cv::waitKey(30);
    if (key == 'n'){
      img_id = (img_id + 1) % filenames.size();
      
      #ifdef DEBUG_LI
        std::cout << "key \'n\' arrived" << std::endl;
      #endif
    }      

    #ifdef DEBUG_LI
      std::cout << "img_id: " << img_id << std::endl;
    #endif
  }

  // Finally, read the img with the appropriate img_id
  img_out = cv::imread(filenames[img_id]);
}

void my_genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
	#ifdef DEBUG_GIL
  std::cout << "STUDENT FUNCTION - my_genericImageListener" << std::endl;
  #endif
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

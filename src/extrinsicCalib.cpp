#include "extrinsicCalib.hpp"

#define LOG_EC
#define LOG_IU
#define LOG_FPT
#define LOG_U
#define DIST_COEFFS_DEFAULT

//**********************************************************************
// PRIVATE EXTRINSIC CALIBRATION FUNCTIONS (from professor_interface)
//**********************************************************************
// Defintion of the function pickNPoints and the callback mouseCallback.
// The function pickNPoints is used to display a window with a background
// image, and to prompt the user to select n points on this image.
static cv::Mat bg_img;
static std::vector<cv::Point2f> result;
static std::string name;
static std::atomic<bool> done;
static int n;
static double show_scale = 1.0;

void mouseCallback(int event, int x, int y, int, void* p){
	if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;

	result.emplace_back(x*show_scale, y*show_scale);
	cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
	cv::imshow(name.c_str(), bg_img);

	if (result.size() >= n) {
	  usleep(500*1000);
	  done.store(true);
	}
}

std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img){
	result.clear();
	cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
	cv::resize(img, bg_img, small_size);
	name = "Pick " + std::to_string(n0) + " points";
	cv::imshow(name.c_str(), bg_img);
	cv::namedWindow(name.c_str());
	n = n0;

	done.store(false);

	cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
	while (!done.load()) {
	  cv::waitKey(500);
	}

	cv::destroyWindow(name.c_str());
	return result;
}

//**********************************************************************
// SUPPORT FUNCTIONS - PRIVATE
//**********************************************************************
/*!
* It writes points coordinates line per line in a csv file.
* @param[in]  file_path     Path to the csv file.
* @param[in]  img_points    Vector with the points.
*/
void writePts2CSV(std::string file_path, std::vector<cv::Point2f>& img_points){
	std::ofstream output(file_path);

	if (!output.is_open()) throw std::runtime_error("Cannot write file: " + file_path);

	for (const auto pt: img_points){
		output << pt.x << " " << pt.y << std::endl;
	}

	output.close();
}

/*!
* It retrives points coordinates wrote line per line in a csv file.
* @param[in]  file_path     Path to the csv file.
* @param[out] img_points    Vector with the read points.
*/
void readPtsFromCSV(std::string file_path, std::vector<cv::Point2f>& img_points){
	std::ifstream input(file_path);
	if (!input.is_open()) throw std::runtime_error("Cannot read file: " + file_path);

	while (!input.eof()){
    	double x, y;
    	if (!(input >> x >> y)){
    		if (input.eof()) break;
    		else throw std::runtime_error("Malformed file: " + file_path);
    	}

    	img_points.emplace_back(x, y);
    }

    input.close();
}

/*!
* It reads the distortion coefficients from src/intrinsic_calibration.xml.
* @param[out]  dist_coeffs  The distortion coefficients.
*/
/*void getDistCoeffs(cv::Mat& dist_coeffs){
	std::cout << "getDistCoeffs is reading from intrinsic_calibration.xml" << std::endl;

    std::string this_file_path = __FILE__;
    std::string this_file_name = "extrinsicCalib.cpp";
    int upper_bound = this_file_path.length() - this_file_name.length();
    std::string xml_path = this_file_path.substr(0, upper_bound) + "intrinsic_calibration.xml";

    std::cout << "xml_path = " << xml_path << std::endl;

    // Read xml
    pugi::xml_document doc;
    if (!doc.load_file(xml_path.c_str())) 
        std::cerr << "pugixml - FILE NOT LOADED" << std::endl;

    pugi::xml_node root = doc.document_element();
    pugi::xml_node node = root.child("opencv_storage");
    pugi::xml_node dc_node = node.child("distortion_coefficients");
    pugi::xml_node data_node = dc_node.child("data");

    std::cout << "dist_coeffs_node = " << data_node.text() << std::endl;
}*/

//**********************************************************************
// PUBLIC FUNCTION
//**********************************************************************
bool my_extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, 
    const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    #ifdef LOG_EC
        std::cout << "STUDENT FUNCTION - my_extrinsicCalib" << std::endl;
    #endif

    std::string file_path = config_folder + "/extrinsicCalib.csv";
    std::vector<cv::Point2f> img_points;

    // csv file doesn't exist -> create it
    if (!std::experimental::filesystem::exists(file_path)){
      std::experimental::filesystem::create_directories(config_folder);
      img_points = pickNPoints(4, img_in);
      writePts2CSV(file_path, img_points);
    } else {
    	// csv file must exists now -> load it
    	readPtsFromCSV(file_path, img_points);
    }
    
    // Define the distortion coefficients
    cv::Mat dist_coeffs;

    #ifdef DIST_COEFFS_DEFAULT
        dist_coeffs = cv::Mat1d::zeros(1, 4); // dist_coeffs = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
        // Look at mail reply
        std::cout << "STUDENT FUNCTION - DIST_COEFFS_DEFAULT [0,0,0,0,0]" << std::endl;
        //std::cout << "Shouldn't I get dist_coeffs from intrinsic_calibration.xml???" << std::endl;
        //std::cout << std::experimental::filesystem::current_path() << std::endl;
    #else
        std::cout << "STUDENT FUNCTION - NO DIST_COEFFS_DEFAULT" << std::endl;
        //getDistCoeffs(dist_coeffs);

        // Copied & pasted dist coeffs value from intrinsic_calibration.xml
        double dc_k1 = -3.8003887070277098e-01;
        double dc_k2 = 1.6491942975982371e-01;
        double dc_k3 = -7.2969848408770512e-04;
        double dc_p1 = -8.3850307681785957e-04;
        double dc_p2 = 0.;

        dist_coeffs = (cv::Mat1d(1,4) << dc_k1, dc_k2, dc_k3, dc_p1, dc_p2);
    #endif

    // Get rotational and transposal vectors
    bool all_good = cv::solvePnP(object_points, img_points, camera_matrix, dist_coeffs, rvec, tvec);

    if (!all_good) std::cerr << "STUDENT FUNCTION - my_extrinsicCalib - solvePnP FAILED" << std::endl;

    return all_good;
}

void my_imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
    const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    #ifdef LOG_IU
        std::cout << "STUDENT FUNCTION - imageUndistort" << std::endl;
    #endif

    static bool optimize = true;

    if (!optimize){
      // slow version
      cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
    }else{
      // fast version
      static bool m_init = false;
      static cv::Mat map1, map2;

      if(!m_init){
        cv::Mat R;
        cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, 
                                    img_in.size(), CV_16SC2, map1, map2);

        m_init = true;
      }

      // Initialize output image    
      cv::remap(img_in, img_out, map1, map2, cv::INTER_LINEAR);
    }
}

void my_findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, 
    const std::vector<cv::Point3f>& object_points_plane, const std::vector<cv::Point2f>& dest_image_points_plane, 
    cv::Mat& plane_transf, const std::string& config_folder){
    #ifdef LOG_FPT
        std::cout << "STUDENT FUNCTION - my_findPlaneTransform" << std::endl;
    #endif

    cv::Mat image_points;
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);
    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
}

void my_unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
    const std::string& config_folder){
    #ifdef LOG_U
        std::cout << "STUDENT FUNCTION - unwarp" << std::endl;
    #endif

    cv::warpPerspective(img_in, img_out, transf, img_in.size());
}
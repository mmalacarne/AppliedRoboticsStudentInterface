#include "extrinsicCalib.hpp"

#define LOG_EC
#define LOG_IU
#define LOG_FPT
#define LOG_U

//**********************************************************************
// EXTRINSIC CALIBRATION FUNCTIONS - PRIVATE
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
// PRIVATE FUNCTION
//**********************************************************************
/*!
* Support function which...
* @param[in]
* @param[out] 
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
* Support function which...
* @param[in]
* @param[out] 
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
* Support function which...
* @param[in]
* @param[out] 
*/
void getDistCoeffs(cv::Mat& dist_coeffs){
	std::cout << "getDistCoeffs is reading from intrinsic_calibration.xml" << std::endl;
	std::string xml_path = "/home/workspace/project/intrinsic_calibration.xml";

	//READ XML
}

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
    
    // get rotational and transposal vectors
    bool AS_IN_PROFESSOR_INTERFACE = true;
    cv::Mat dist_coeffs;

    if (AS_IN_PROFESSOR_INTERFACE){
    	dist_coeffs = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
    	std::cout << "dist_coeffs = [0,0,0,0,0]" << std::endl;
    	std::cout << "Shouldn't I get dist_coeffs from intrinsic_calibration.xml???" << std::endl;
    	std::cout << std::experimental::filesystem::current_path() << std::endl;
    }
    else getDistCoeffs(dist_coeffs);

    bool all_good = cv::solvePnP(object_points, img_points, camera_matrix, dist_coeffs, rvec, tvec);

    if (!all_good) std::cerr << "FAILED SOLVE_PNP" << std::endl;

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
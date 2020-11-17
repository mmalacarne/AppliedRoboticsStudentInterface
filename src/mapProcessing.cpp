#include "mapProcessing.hpp"

#define DEBUG_PM
#define DEBUG_FR

//**********************************************************************
// PRIVATE FUNCTION
//**********************************************************************
void removeNoise(cv::Mat& mask){
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

	cv::erode(mask, mask, kernel);
	cv::dilate(mask, mask, kernel);
}

void getMaskContours(cv::Mat& mask, std::vector<std::vector<cv::Point>>& contours_approx, int dist_accuracy){
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> approx_curve;

	for (int i = 0; i<contours.size(); ++i){
		approxPolyDP(contours[i], approx_curve, dist_accuracy, true);
		contours_approx = {approx_curve};
	}
}

//**********************************************************************
// PUBLIC FUNCTION
//**********************************************************************
bool my_processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, 
	std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){    
    #ifdef DEBUG_PM
    std::cout << "STUDENT FUNCTION - my_processMap" << std::endl;
    #endif

    // Convert color space from BGR to HSV
	cv::Mat hsv_img;
	cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

	///////////////////////////////////////////////////////
	// Extract green color region [80°, 150°] -> [40°, 75°]
	cv::Mat green_mask;
	cv::inRange(hsv_img, cv::Scalar(40, 10, 10), cv::Scalar(75, 255, 255), green_mask);

	// Get rid of noise via erode and dilate
	removeNoise(green_mask);

	// Find green mask contours
	std::vector<std::vector<cv::Point>> contours_approx;
	int dist_accuracy = 3;
	
	#ifndef DEBUG_PM
		getMaskContours(green_mask, contours_approx, dist_accuracy);
	#else
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Point> approx_curve;
		cv::Mat contours_img = img_in.clone();

		drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
		std::cout << "N. contours: " << contours.size() << std::endl;
		for (int i=0; i<contours.size(); ++i){
			std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;

			approxPolyDP(contours[i], approx_curve, dist_accuracy, true);
			contours_approx = {approx_curve};

			drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 5, cv::LINE_AA);
			std::cout << "\tApproximated contour size: " << approx_curve.size() << std::endl;
		}
		std::cout << std::endl;

		cv::imshow("Contours", contours_img);
		char key = cv::waitKey(0);
		while (key != 'm'){
			cv::waitKey(0);
		}
	#endif

	///////////////////////////////////////////////////////
	// Extract red color region [0°, 20°] & [340°, 360°] -> [0°, 10°] & [170°, 179°]
	cv::Mat red_mask_low, red_mask_high, red_mask;
	cv::inRange(hsv_img, cv::Scalar(0, 10, 10), cv::Scalar(10, 255, 255), red_mask_low);
	cv::inRange(hsv_img, cv::Scalar(170, 50, 50), cv::Scalar(179, 255, 255), red_mask_high);
	cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask);

	// Find red mask contours
	/*std::vector<std::vector<cv::Point>> contours_approx;
	int dist_accuracy = 3;
	
	#ifndef DEBUG_PM
		getMaskContours(red_mask, contours_approx, dist_accuracy);
	#else
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Point> approx_curve;
		cv::Mat contours_img = img_in.clone();

		drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
		std::cout << "N. contours: " << contours.size() << std::endl;
		for (int i=0; i<contours.size(); ++i){
			std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;

			approxPolyDP(contours[i], approx_curve, dist_accuracy, true);
			contours_approx = {approx_curve};

			drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 5, cv::LINE_AA);
			std::cout << "\tApproximated contour size: " << approx_curve.size() << std::endl;
		}
		std::cout << std::endl;

		cv::imshow("Contours", contours_img);
		char key = cv::waitKey(0);
		while (key != 'm'){
			cv::waitKey(0);
		}
	#endif*/
}

bool my_findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, 
	double& theta, const std::string& config_folder){
    #ifdef DEBUG_FR
    std::cout << "STUDENT FUNCTION - my_findRobot" << std::endl;
    #endif
}
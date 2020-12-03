#include "mapProcessing.hpp"

#define DEBUG_PM
#define DEBUG_OBSTACLES
#define DEBUG_VICTIMS_GATE
#define DEBUG_FR

//**********************************************************************
// PRIVATE FUNCTION
//**********************************************************************
/*!
* Erode and dilate the mask with a 4x4 kernel.
* @param[out] mask 	Bitmap mask matrix for a certain color. 
*/
void removeNoise(cv::Mat& mask){
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));

	cv::erode(mask, mask, kernel);
	cv::dilate(mask, mask, kernel);
}

/*!
* Retrieves the approximated contours of mask.
* @param[in]  mask 				Denoised bitmap mask matrix for a certain color.
* @param[in]  dist_accuracy 	Max distance between the original curve and its approximation.
* @param[in]  scale 			Scale.
* @param[out] obj_contour_list 	List of objs contours.
*/
void getApproxMaskContours(cv::Mat& mask, int dist_accuracy, const double scale, std::vector<Polygon>& obj_contour_list){
	std::vector<cv::Point> approx_curve;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	for (int i = 0; i < contours.size(); ++i){
		cv::approxPolyDP(contours[i], approx_curve, dist_accuracy, true);

		// Create the contour poligon and populate it with verteces
		Polygon obj_contour;
		for (const auto& pt: approx_curve){
			obj_contour.emplace_back(pt.x/scale, pt.y/scale);
		}
		// Populate the obj contour vector
		obj_contour_list.push_back(obj_contour);
	}
}

/*!
* Retrieves the obstacles.
* @param[in]  hsv_img 			Original img in hsv space.
* @param[in]  scale 			Scale.
* @param[out] obstacles_list 	List of obstacles.
*/
void getObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacles_list){
	// Extract red color region [0°, 20°] & [340°, 360°] -> [0°, 10°] & [170°, 179°]
	cv::Mat red_mask_low, red_mask_high, red_mask;
	cv::inRange(hsv_img, cv::Scalar(0, 30, 30), cv::Scalar(10, 255, 255), red_mask_low);
	cv::inRange(hsv_img, cv::Scalar(170, 50, 50), cv::Scalar(179, 255, 255), red_mask_high);
	cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask);

	// Get rid of noise via erode and dilate
	removeNoise(red_mask);

	// Find red mask contours
	int dist_accuracy = 10;
	getApproxMaskContours(red_mask, dist_accuracy, scale, obstacles_list);

	#ifdef DEBUG_OBSTACLES
		cv::Mat img_in;
		cv::cvtColor(hsv_img, img_in, cv::COLOR_HSV2BGR);
		//cv::imshow("Original img", img_in);
		//cv::imshow("HSV", hsv_img);
		
		for (const auto& obstacle: obstacles_list){
			cv::Mat contours_img = img_in.clone();
			std::cout << "Approximated contour size: " << obstacle.size() << std::endl;

			std::vector<cv::Point> contours;
			for (const auto& pt: obstacle){
				contours.emplace_back(pt.x*scale, pt.y*scale);
			}
			std::vector<std::vector<cv::Point>> drawable_contours = {contours};
			// cv::Scalar(40,190,40) = green
			// cv::Scalar(0,170,220) = yellow
			drawContours(contours_img, drawable_contours, -1, cv::Scalar(0,170,220), 2, cv::LINE_AA);

			std::string window_name = "Obstacles contours - size = " + std::to_string(obstacle.size());
			cv::imshow(window_name, contours_img);
			cv::waitKey(0);
			cv::destroyWindow(window_name);
		}

		std::cout << "getObstacles() - END" << std::endl;
	#endif
}

/*!
* Retrieves the victims and the gate. Victims and gate are discriminated by the number of verteces.
* @param[in]  hsv_img 		Original img in hsv space.
* @param[in]  scale 		Scale.
* @param[out] victims_list 	List of obstacles.
* @param[out] gate 			Gate polygon.
*/
void getVictimsAndGate(const cv::Mat& hsv_img, const double scale, 
	std::vector<std::pair<int,Polygon>>& victims_list, Polygon& gate){
	int victim_id = 0;
	// Extract green color region [80°, 150°] -> [40°, 75°]
	cv::Mat green_mask;
	cv::inRange(hsv_img, cv::Scalar(40, 10, 10), cv::Scalar(75, 255, 255), green_mask);

	// Get rid of noise via erode and dilate
	removeNoise(green_mask);

	// Find green mask contours
	int dist_accuracy = 5;
	std::vector<Polygon> obj_list;
	getApproxMaskContours(green_mask, dist_accuracy, scale, obj_list);

	// Since victims are circles and gate is rectangular,
	// then the green obj with less verteces must be the gate
	int min_verteces = obj_list[0].size();
	int best_i = 0;
	for (int i = 1; i < obj_list.size(); i++){
		if (obj_list[i].size() < min_verteces){
			min_verteces = obj_list[i].size();
			best_i = i;
		}
	}

	// Retrieve victims and gate depending on verteces
	for (int i = 0; i < obj_list.size(); i++){
		// Create the green obj contour poligon
		Polygon green_obj_contour;
		for (const auto& pt: obj_list[i]){
			green_obj_contour.emplace_back(pt.x/scale, pt.y/scale);
		}

		if (i != best_i)
			// Populate the victims vector with id and contour
			victims_list.push_back({victim_id++, green_obj_contour});
		else
			// Populate the gate obj
			gate = green_obj_contour;
	}

	#ifdef DEBUG_VICTIMS_GATE
		cv::Mat img_in;
		cv::cvtColor(hsv_img, img_in, cv::COLOR_HSV2BGR);
		//cv::imshow("Original img", img_in);
		//cv::imshow("HSV", hsv_img);
		
		for (int i = 0; i < obj_list.size(); i++){
			cv::Mat contours_img = img_in.clone();
			std::cout << "Approximated contour size: " << obj_list[i].size() << std::endl;

			std::vector<cv::Point> contours;
			for (const auto& pt: obj_list[i]){
				contours.emplace_back(pt.x*scale, pt.y*scale);
			}
			std::vector<std::vector<cv::Point>> drawable_contours = {contours};
			// cv::Scalar(40,190,40) = green
			// cv::Scalar(0,170,220) = yellow
			drawContours(contours_img, drawable_contours, -1, cv::Scalar(0,170,220), 2, cv::LINE_AA);

			std::string type_of_green;
			if (best_i == i)
				type_of_green = "Gate";
			else
				type_of_green = "Victim";
			std::string window_name = type_of_green + " contours - size = " + std::to_string(obj_list[i].size());
			cv::imshow(window_name, contours_img);
			cv::waitKey(0);
			cv::destroyWindow(window_name);
		}

		std::cout << "getObstacles() - END" << std::endl;
	#endif
}

/*!
* Retrieves the robot/triangle.
* @param[in]  hsv_img 	Original img in hsv space.
* @param[in]  scale 	Scale.
* @param[out] triangle 	Robot/triangle polygon.
*/
bool getRobot(const cv::Mat& hsv_img, const double scale, Polygon& triangle){
	// Extract blue color region [190°, 270°] -> [95°, 135°]
	cv::Mat blue_mask;
	cv::inRange(hsv_img, cv::Scalar(95, 30, 30), cv::Scalar(135, 255, 255), blue_mask);

	// Get rid of noise via erode and dilate
	removeNoise(blue_mask);

	// Find blue mask contours
	int dist_accuracy = 10;
	std::vector<cv::Point> approx_curve;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	cv::approxPolyDP(contours[0], approx_curve, dist_accuracy, true);

	#ifdef DEBUG_FR
		cv::Mat img_in;
		cv::cvtColor(hsv_img, img_in, cv::COLOR_HSV2BGR);
		cv::Mat contours_img = img_in.clone();

		// cv::Scalar(40,190,40) = green
		// cv::Scalar(0,170,220) = yellow
		drawContours(contours_img, contours, -1, cv::Scalar(0,170,220), 2, cv::LINE_AA);
	#endif

	if (approx_curve.size() == 3){
		// Create the contour polygon and populate it with verteces
		for (const auto& pt: approx_curve)
			triangle.emplace_back(pt.x/scale, pt.y/scale);

		#ifdef DEBUG_FR
			std::string window_name = "Robot contours - #edges = " + std::to_string(triangle.size());
			cv::imshow(window_name, contours_img);
			cv::waitKey(0);
			cv::destroyWindow(window_name);
		#endif

		return true;
	} else {
		#ifdef DEBUG_FR
			std::string window_name = "Robot contours - #edges = " + std::to_string(approx_curve.size());
			cv::imshow(window_name, contours_img);
			cv::waitKey(0);
			cv::destroyWindow(window_name);
		#endif

		throw std::logic_error("Error! Robot with " + std::to_string(approx_curve.size()) + " edges");
		return false;
	}

	/*// Create the contour poligon and populate it with verteces
	for (const auto& pt: approx_curve) 
		triangle.emplace_back(pt.x/scale, pt.y/scale);

	#ifdef DEBUG_FR
		cv::Mat img_in;
		cv::cvtColor(hsv_img, img_in, cv::COLOR_HSV2BGR);
		cv::Mat contours_img = img_in.clone();

		// cv::Scalar(40,190,40) = green
		// cv::Scalar(0,170,220) = yellow
		drawContours(contours_img, contours, -1, cv::Scalar(0,170,220), 2, cv::LINE_AA);

		std::string window_name = "Robot contours - #edges = " + std::to_string(triangle.size());
		cv::imshow(window_name, contours_img);
		cv::waitKey(0);
		cv::destroyWindow(window_name);
	#endif

	if (triangle.size() != 3)
		throw std::logic_error("Error! Robot with " + std::to_string(triangle.size()) + " edges");*/
}

/*!
* Retrieves the robot/trianlge's baricenter.
* @param[in]  triangle 		Robot/triangle polygon.
* @param[out] baricenter_x 	Baricenter's x coordinate.
* @param[out] baricenter_y	Baricenter's y coordinate.
*/
void getBaricenter(Polygon& triangle, double& baricenter_x, double& baricenter_y){
	baricenter_x = 0;
	baricenter_y = 0;

	for (auto pt: triangle){
		baricenter_x += pt.x;
		baricenter_y += pt.y;
	}

	baricenter_x /= static_cast<double>(triangle.size());
	baricenter_y /= static_cast<double>(triangle.size());
}

/*!
* Retrieves the robot/trianlge's orientation.
* @param[in]  triangle 		Robot/triangle polygon.
* @param[in]  baricenter_x 	Baricenter's x coordinate.
* @param[in]  baricenter_y	Baricenter's y coordinate.
* @param[out] theta			Robot/triangle's orientation.
*/
void getOrientation(Polygon& triangle, double& baricenter_x, double& baricenter_y, double& theta){
	// Find the top vertex (i.e. P3) -> the most distant from the baricenter
	double best_dx;
	double best_dy;
	double best_dist = 0;
	Point top_vertex;
	for (auto& pt: triangle){
		double dx = baricenter_x - pt.x;
		double dy = baricenter_y - pt.y;
		double dist = dx * dx + dy * dy;
		if (dist > best_dist){
			best_dist = dist;
			top_vertex = pt;
			best_dx = dx;
			best_dy = dy;
		}
	}

	// Retrieve theta
	theta = std::atan2(best_dy, best_dx);
}

//**********************************************************************
// PUBLIC FUNCTION
//**********************************************************************
bool my_processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacles_list, 
	std::vector<std::pair<int,Polygon>>& victims_list, Polygon& gate, const std::string& config_folder){    
    #ifdef DEBUG_PM
    	std::cout << "STUDENT FUNCTION - my_processMap" << std::endl;
    #endif

    // Convert color space from BGR to HSV
	cv::Mat hsv_img;
	cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

	// Retrive obstacles
	getObstacles(hsv_img, scale, obstacles_list);
	// Retrive victims and gate
	getVictimsAndGate(hsv_img, scale, victims_list, gate);

	return true;
}

bool my_findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, 
	double& theta, const std::string& config_folder){
    #ifdef DEBUG_FR
    	std::cout << "STUDENT FUNCTION - my_findRobot" << std::endl;
    #endif

    // Convert color space from RGB to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    // Retrieve robot/triangle and its details
    bool robot_ok = getRobot(hsv_img, scale, triangle);
    if (robot_ok){
    	getBaricenter(triangle, x, y);
    	getOrientation(triangle, x, y, theta);
    }

    #ifdef DEBUG_FR
    Point top_vertex;
    double best_dist = 0;
	for (auto& pt: triangle){
		double dx = x - pt.x;
		double dy = y - pt.y;
		double dist = dx * dx + dy * dy;
		if (dist > best_dist){
			best_dist = dist;
			top_vertex = pt;
		}
	}	
	std::vector<cv::Point> orientation_line;
	orientation_line.emplace_back(top_vertex.x*scale, top_vertex.y*scale);
	orientation_line.emplace_back(x*scale, y*scale);
	std::vector<std::vector<cv::Point>> drawable = {orientation_line};

	cv::Mat contours_img = img_in.clone();
	// cv::Scalar(40,190,40) = green
	// cv::Scalar(0,170,220) = yellow
	drawContours(contours_img, drawable, -1, cv::Scalar(0,170,220), 2, cv::LINE_AA);
	std::string window_name = "Robot orientation";
	cv::imshow(window_name, contours_img);
	cv::waitKey(0);
	cv::destroyWindow(window_name);
    #endif

	return robot_ok;
}
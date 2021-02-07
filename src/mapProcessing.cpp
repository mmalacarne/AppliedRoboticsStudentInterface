#include "mapProcessing.hpp"

//#define DEBUG_PM
//#define DEBUG_OBSTACLES
//#define DEBUG_OFFSET_OBSTACLES
//#define DEBUG_TR_R
//#define DEBUG_VICTIMS_GATE
//#define DEBUG_VICTIMS_ID
//#define DEBUG_getTemplateImgs
//#define DEBUG_getTMDigit
//#define DEBUG_FR

namespace clpr = ClipperLib;

//**********************************************************************
// PRIVATE FUNCTIONS
//**********************************************************************
/*!
* Erode and dilate the mask with a 4x4 kernel.
* @param[out] 	mask 	Bitmap mask matrix for a certain color. 
*/
void removeNoise(cv::Mat& mask){
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));

	cv::erode(mask, mask, kernel);
	//cv::GaussianBlur(mask, mask, cv::Size(5, 5), 2, 2);
	cv::dilate(mask, mask, kernel);
}

/*!
* Retrieves the approximated contours of mask.
* @param[in] 	mask 				Denoised bitmap mask matrix for a certain color.
* @param[in] 	dist_accuracy 		Max distance between the original curve and its approximation.
* @param[in] 	scale 				Scale of the arena (1px/scale = X meters).
* @param[out] 	obj_contour_list 	List of objs contours (vertex in meters).
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
			obj_contour.emplace_back(pt.x/scale, pt.y/scale); // pts are in px --> pt/scale = meter
		}
		// Populate the obj contour vector
		obj_contour_list.push_back(obj_contour);
	}
}

/*!
* Debugging function: shows the contour of a list of polygons.
* @param[in]  hsv_img 		Original img in hsv space.
* @param[in]  objs_list 	List of objects.
* @param[in]  obj_name 		Objects type (i.e. "Obstacle", "Victim", "Gate").
*/
void showRetrievedObjs(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& objs_list, 
	std::string obj_name){
	cv::Mat img_in;
	cv::cvtColor(hsv_img, img_in, cv::COLOR_HSV2BGR);
	//cv::imshow("Original img", img_in);
	//cv::imshow("HSV", hsv_img);

	for (const auto& obj: objs_list){
		cv::Mat contours_img = img_in.clone();
		std::cout << obj_name << " - Approximated contour size: " << obj.size() << std::endl;

		std::vector<cv::Point> contours;
		for (const auto& pt: obj){
			contours.emplace_back(pt.x*scale, pt.y*scale); // pts are in meter --> pt*scale = px
		}
		std::vector<std::vector<cv::Point>> drawable_contours = {contours};
		// cv::Scalar(40,190,40) // green
		// cv::Scalar(0,170,220) // yellow
		// cv::Scalar(255, 0, 0) // blue
		// cv::Scalar(160, 158, 95) // cadetblue
		drawContours(contours_img, drawable_contours, -1, cv::Scalar(0,170,220), 1, cv::LINE_AA);

		std::string window_name = obj_name + " contours - size = " + std::to_string(obj.size());
		cv::imshow(window_name, contours_img);
	}

	cv::waitKey(0);
	cv::destroyAllWindows();
}

void showRetrievedObjs(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& objs_list, 
	std::string obj_name){
	cv::Mat img_in;
	cv::cvtColor(hsv_img, img_in, cv::COLOR_HSV2BGR);
	//cv::imshow("Original img", img_in);
	//cv::imshow("HSV", hsv_img);

	for (const auto& obj: objs_list){
		cv::Mat contours_img = img_in.clone();
		std::cout << obj_name << " - Approximated contour size: " << obj.second.size() << std::endl;

		std::vector<cv::Point> contours;
		for (const auto& pt: obj.second){
			contours.emplace_back(pt.x*scale, pt.y*scale); // pts are in meter --> pt*scale = px
		}
		std::vector<std::vector<cv::Point>> drawable_contours = {contours};
		// cv::Scalar(40,190,40) // green
		// cv::Scalar(0,170,220) // yellow
		// cv::Scalar(255, 0, 0) // blue
		// cv::Scalar(160, 158, 95) // cadetblue
		drawContours(contours_img, drawable_contours, -1, cv::Scalar(0,170,220), 1, cv::LINE_AA);

		std::string window_name = obj_name + " contours - id = " + std::to_string(obj.first);
		cv::imshow(window_name, contours_img);
	}

	cv::waitKey(0);
	cv::destroyAllWindows();
}

/*!
* Retrieves the radius of the circle that circumscribes the triangle.
* @param[in]   		hsv_img 	Original img in hsv space.
* @param[in] 		scale 		Scale of the arena (1px/scale = X meters).
* @return[double] 	radius 		Retrieved radius (in meters).
*/
double getTriangleRadius(const cv::Mat& hsv_img, const double scale){
	Polygon triangle;

	// Here I can't use the function getRobot() due to scope problems while compiling.
	// Dirty workaround --> Copy&Paste of getRobot()
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
	if (approx_curve.size() == 3){
		// Create the contour polygon and populate it with verteces
		for (const auto& pt: approx_curve)
			triangle.emplace_back(pt.x/scale, pt.y/scale); // pts are in px --> pt/scale = meter
	} else {
		throw std::logic_error("getTriangleRadius() - Error! Robot with " + std::to_string(approx_curve.size()) + " edges");
	}
	// Dirty workaround end

	// Here I can't use the function getBaricenter() due to scope problems while compiling.
	// Dirty workaround --> Copy&Paste of getBaricenter()
	/*baricenter_x = 0;
	baricenter_y = 0;

	for (auto pt: triangle){
		baricenter_x += pt.x;
		baricenter_y += pt.y;
	}

	baricenter_x /= static_cast<double>(triangle.size());
	baricenter_y /= static_cast<double>(triangle.size());*/
	// Dirty workaround end

	// Get baricenter
	double baricenter_x, baricenter_y;
	getBaricenter(triangle, baricenter_x, baricenter_y);


	// Get the top vertex of the triangle
	Point top_vertex;
	double best_dist = 0;
	for (auto& pt: triangle){
		double dx = baricenter_x - pt.x;
		double dy = baricenter_y - pt.y;
		double dist = dx * dx + dy * dy;
		if (dist > best_dist){
			best_dist = dist;
			top_vertex = pt;
		}
	}

	double dx = top_vertex.x - baricenter_x;
	double dy = top_vertex.y - baricenter_y;

	#ifdef DEBUG_TR_R
		cv::Mat img_in;
		cv::cvtColor(hsv_img, img_in, cv::COLOR_HSV2BGR);
		double radius = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

		cv::line(img_in, cv::Point(baricenter_x*scale, baricenter_y*scale), 
			cv::Point(top_vertex.x*scale, top_vertex.y*scale), cv::Scalar(0, 0, 0), 2);
		cv::circle(img_in, cv::Point(baricenter_x*scale, baricenter_y*scale), radius*scale, cv::Scalar(255, 0, 0), 2);
		std::string window_name = "Robot radius contours - r = " + std::to_string(radius);
		cv::imshow(window_name, img_in);
		//cv::waitKey(0);
		//cv::destroyAllWindows();
	#endif

	return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)); // pts are in meter
	//return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2))*scale; // pts are in meter --> pt*scale = px
}

/*!
* Inflates the obstacles of a list with a given offset by means of ClipperLib.
* @param[in] 	offset 			Offset to apply (in meters).
* @param[in] 	obstacles_list 	List of obstacles (vertex in meters).
* @param[in] 	scale 			Scale of the arena (1px/scale = X meters).
* @param[out] 	obstacles_list 	List of offsetted obstacles (vertex in meters).
*/
void offsetObstacles(double offset, std::vector<Polygon>& obstacles_list, const double scale){
	clpr::Paths all_offset_obstacles;
	clpr::ClipperOffset co;
	//co.MiterLimit = 4;

	// ClipperLib offset is in px --> offset*scale = px
	double px_offset = offset * scale;
	#ifdef DEBUG_OFFSET_OBSTACLES
		std::cout << "px_offset = offset*scale = " << offset << "*"<< scale << " = " << px_offset<< std::endl;
	#endif

	const double INT_ROUND = 1000.;
	for (const auto& obstacle: obstacles_list){
		clpr::Path clpr_obstacle;
		for (const auto& pt: obstacle){
			// Obstacle's pts are in meters --> pt*scale = px
			//int x = pt.x * scale * INT_ROUND;
			//int y = pt.y * scale * INT_ROUND;
			int x = pt.x * INT_ROUND;
			int y = pt.y * INT_ROUND;

			#ifdef DEBUG_OFFSET_OBSTACLES
				std::cout << "pt.x = " << pt.x << " x = " << x << std::endl;
				std::cout << "pt.y = " << pt.y << " y = " << y << std::endl;
			#endif

			clpr_obstacle << clpr::IntPoint(x, y);
		}

		if (obstacle.size() == 3)
			co.AddPath(clpr_obstacle, clpr::jtSquare, clpr::etClosedLine);
		else
			co.AddPath(clpr_obstacle, clpr::jtSquare, clpr::etClosedPolygon);

		//co.AddPath(clpr_obstacle, clpr::jtMiter, clpr::etClosedPolygon);
	}

	co.Execute(all_offset_obstacles, px_offset);

	// Update obstacles_list with the new offsetted obstacles
	obstacles_list.clear();

	for (const clpr::Path& clpr_obstacle: all_offset_obstacles){
		Polygon offsetted_obstacle;

		if (clpr::Orientation(clpr_obstacle)){
			for (const clpr::IntPoint& pt: clpr_obstacle){
				// Clipper obstacle's pts are in px --> pt/scale = meters
				//double x = (pt.X / scale) / INT_ROUND;
				//double y = (pt.Y / scale) / INT_ROUND;
				double x = pt.X / INT_ROUND;
				double y = pt.Y / INT_ROUND;

				/*#ifdef DEBUG_OFFSET_OBSTACLES
					std::cout << "pt.X = " << pt.X << " x = " << x << std::endl;
					std::cout << "pt.Y = " << pt.Y << " y = " << y << std::endl;
				#endif*/

				offsetted_obstacle.emplace_back(x, y);
			}

			obstacles_list.push_back(offsetted_obstacle);
		}
	}
}

/*!
* Retrieves the obstacles.
* @param[in] 	hsv_img 		Original img in hsv space.
* @param[in] 	scale 			Scale of the arena (1px/scale = X meters).
* @param[out] 	obstacles_list 	List of obstacles (vertex in meters).
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
		showRetrievedObjs(hsv_img, scale, obstacles_list, "Obstacle");
		std::cout << "getObstacles()- get the triangle radius" << std::endl;
	#endif

	// Set the obstacle offset a little bit more than the robot_radius
	double extra_margin = 0.05; // additional 5cm margin --> better be safe than sorry;)
	//double offset = getTriangleRadius(hsv_img, scale) + extra_margin;
	double offset = getTriangleRadius(hsv_img, scale);

	// Offsetting the obstacles
	offsetObstacles(offset, obstacles_list, scale);

	#ifdef DEBUG_OFFSET_OBSTACLES
		//std::cout << "Radius = " << offset-extra_margin << " m" << std::endl;
		//std::cout << "Radius = Radius + margin = " << offset << " m" << std::endl;
		//std::cout << "Radius*scale = " << offset*scale << " px" << std::endl;

		std::cout << "Radius = " << offset << " m" << std::endl;
		std::cout << "Radius*scale = " << offset*scale << " px" << std::endl;

		std::cout << "getObstacles()- triangle radius retrieved" << std::endl;
		showRetrievedObjs(hsv_img, scale, obstacles_list, "Offsetted obstacle");
		cv::waitKey(0);
		cv::destroyAllWindows();
		std::cout << "getObstacles() - END" << std::endl;
	#endif
}

/*!
* Support function which retrives all the template images from src/template_images.
* @param[out] 	template_img_list 	Vector of pairs <represented id, template image>.
*/
void getTemplateImgs(std::vector<std::pair<int,cv::Mat>>& template_img_list){
  std::string this_file_path = __FILE__;
  std::string this_file_name = "mapProcessing.cpp";
  int upper_bound = this_file_path.length() - this_file_name.length();
  std::string path = this_file_path.substr(0, upper_bound) + "template_images/";
  std::string img_path;
  cv::Mat template_img;

  for (int id = 0; id < 10; id++){
  	img_path = path + std::to_string(id) + ".png";
  	template_img = cv::imread(img_path);
  	template_img_list.push_back({id, template_img});
  }

  #ifdef DEBUG_getTemplateImgs
  	std::cout << "Path to template images: " << path << std::endl;

  	for (const auto& id_img: template_img_list){
  		std::string ti_window = "Template image ID = " + std::to_string(id_img.first);
  		cv::imshow(ti_window, id_img.second);
  	}
  	cv::waitKey(0);
  	cv::destroyAllWindows();
  #endif
}

/*
// BRUTE FORCE APPROACH
//#define DEBUG_getAllROI
// Retrieves all ROIs: it rotates the original ROI of rotation_angle and then it flips it
void getAllROI(cv::Mat& orig_ROI, const int rotation_angle, std::vector<std::vector<cv::Mat>>& all_ROI){
	// All kind of edited ROI
	std::vector<cv::Mat> all_r_ROI; 	// all rotated ROI
	std::vector<cv::Mat> all_fx_ROI; 	// all rotated and x-axis-flipped ROI
	std::vector<cv::Mat> all_fy_ROI; 	// all rotated and y-axis-flipped ROI
	// Get ROI center
	cv::Point2f ROI_center(orig_ROI.cols/2., orig_ROI.rows/2.);

	// Set rotation
	int max_i = (360 / rotation_angle);

	// Support structures
	double angle;
	double scale = 1.0;
	cv::Mat rotation_matrix, r_ROI, f_x_ROI, f_y_ROI;

	// Rotate ROI of rotation angle and flip
	for (int i = 0; i < max_i; i++){
    	angle = static_cast<double>(i*rotation_angle);
    	rotation_matrix = cv::getRotationMatrix2D(ROI_center, angle, scale);
    	cv::warpAffine(orig_ROI, r_ROI, rotation_matrix, orig_ROI.size(), 
    		cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

    	// flip(src,dst,x) x=0 -> x-axis-flip, x=1 y-axis-flip, x=2 z-axis-flip
    	cv::flip(r_ROI, f_x_ROI, 0);
    	cv::flip(r_ROI, f_y_ROI, 1);

    	// Populate the vector of ROIs
    	all_r_ROI.push_back(r_ROI);
    	all_fx_ROI.push_back(f_x_ROI);
    	all_fy_ROI.push_back(f_y_ROI);

    	#ifdef DEBUG_getAllROI
    		cv::imshow("Rotated " + std::to_string(angle), r_ROI);
    		cv::imshow("Flipped X " + std::to_string(angle), f_x_ROI);
    		cv::imshow("Flipped Y " + std::to_string(angle), f_y_ROI);
    		cv::waitKey(0);
    		cv::destroyAllWindows();
    	#endif
	}

	all_ROI.push_back(all_r_ROI);
	all_ROI.push_back(all_fx_ROI);
	all_ROI.push_back(all_fy_ROI);

	#ifdef DEBUG_getAllROI
		std::cout << "all_ROI.size() = " << all_ROI.size() << std::endl; // all_ROI.size() = 3
		std::cout << "all_ROI[0].size() = " << all_ROI[0].size() << std::endl; // allROI[i].size() = 36
		std::cout << "all_ROI[1].size() = " << all_ROI[1].size() << std::endl;
		std::cout << "all_ROI[2].size() = " << all_ROI[2].size() << std::endl;
		std::cout << "getAllROI() - END" << std::endl;
	#endif
}

// Given allROI it computes the best template matching
int getTMDigit(std::vector<std::vector<cv::Mat>>& all_ROI, 
	std::vector<std::pair<int,cv::Mat>>& template_images, std::pair<int,int>& ROI_idx){
	double best_score = 0;
	cv::Mat match;
	double score;
	int id;
	
	for (int i = 0; i < all_ROI.size(); i++){
		for (int j = 0; j < all_ROI[i].size(); j++){
			for (const auto& id_templ: template_images){
				cv::matchTemplate(all_ROI[i][j], id_templ.second, match, cv::TM_CCOEFF);
				cv::minMaxLoc(match, nullptr, &score);

				if (score > best_score){
					best_score = score;
					id = id_templ.first;
					ROI_idx.first = i;
					ROI_idx.second = j;
				}
			}
		}
	}

	#ifdef DEBUG_getTMDigit
		std::cout << "getTMDigit() - triple for loop" << std::endl;
		std::cout << "Best score = " << best_score << std::endl;
		std::cout << "ID = " << id << std::endl;
		std::cout << "ROI_idx.first = " << ROI_idx.first << " ";
		std::cout << "ROI_idx.second = " << ROI_idx.second << std::endl;
	#endif

	return id;
}

// OCR approach - t computes the best recognized digit
std::string getOCRDigit(std::vector<std::vector<cv::Mat>>& all_ROI){
	// Retrieve the digit with the heighest recognition confidence
    std::string final_id, recognized_id;
	// Retrieve the digit with the highest confidence from all ROIs
	int best_confidence = 0;
	int confidence;
	int cols = all_ROI[0][0].cols;
	int rows = all_ROI[0][0].cols;
	int step = all_ROI[0][0].cols;

	#ifdef DEBUG_VICTIMS_ID
		cv::Mat best_ROI;
	#endif

	for (int i = 0; i < all_ROI.size(); i++){
		for (int j = 0; j < all_ROI[i].size(); j++){
			// Create Tesseract object and retrieve digit
			tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
			ocr->Init(NULL, "eng");
			ocr->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);
			ocr->SetVariable("tessedit_char_whitelist", "0123456789"); //[0, 6] during exam?
		    ocr->SetImage(all_ROI[i][j].data, cols, rows, 3, step);

		    recognized_id = std::string(ocr->GetUTF8Text());
		    confidence = ocr->MeanTextConf();

		    if (confidence > best_confidence){
		    	final_id = recognized_id;
		    	best_confidence = confidence;
		    	#ifdef DEBUG_VICTIMS_ID
		    		best_ROI = all_ROI[i][j];
		    	#endif
		    }

		    ocr->End();
		}
	}

	#ifdef DEBUG_VICTIMS_ID
		std::cout << "Best ROI with confidence = " << best_confidence << std::endl;
		std::string best_w = "OCR - Best ROI - ID = " + final_id;
		cv::imshow(best_w, best_ROI);
		cv::waitKey(0);
		cv::destroyAllWindows();
	#endif

	return final_id;
}
*/

/*!
* Assuming all victims have been rotated and flipped in the same known way, it applies a series 
* of modification in order to make orig_ROI readable/recognizable.
* @param[in] 	orig_ROI 		Original ROI.
* @param[in] 	angle 			Angle of rotation.
* @param[in] 	flag_x_flip 	Flag indicating whether the ROI has to be flipped on the x-axis.
* @param[in] 	flag_y_flip 	Flag indicating whether the ROI has to be flipped on the y-axis.
* @param[out] 	edited_ROI 		Readable/recognizable ROI.
*/
void getEditedROI(cv::Mat& orig_ROI, const double angle, bool flag_x_flip, bool flag_y_flip, cv::Mat& edited_ROI){
	cv::Point2f ROI_center(orig_ROI.cols/2., orig_ROI.rows/2.);
	double r_scale = 1.0;

	cv::Mat rotation_matrix = cv::getRotationMatrix2D(ROI_center, angle, r_scale);
    cv::warpAffine(orig_ROI, edited_ROI, rotation_matrix, orig_ROI.size(),
    	cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

    // Flip x-axis
    if (flag_x_flip) cv::flip(edited_ROI, edited_ROI, 0);
    // Flip y-axis
    if (flag_y_flip) cv::flip(edited_ROI, edited_ROI, 1);
}

/*!
* Retrieves the best matching digit (a.k.a the victim's id).
* @param[in] 	edited_ROI 			Readable/recognizable ROI.
* @param[in] 	template_img_list 	Vector of pairs <represented id, template image>.
* @param[out] 	victim_id 			Matched digit in edited_ROI.
*/
void getTMDigit(cv::Mat& edited_ROI, std::vector<std::pair<int,cv::Mat>>& template_img_list, int& victim_id){
	double best_score = 0;
	double score;
	cv::Mat match;

	for (const auto& id_templ: template_img_list){
		cv::matchTemplate(edited_ROI, id_templ.second, match, cv::TM_CCOEFF);
		cv::minMaxLoc(match, nullptr, &score);

		if (score > best_score){
			best_score = score;
			victim_id = id_templ.first;
		}
	}

	#ifdef DEBUG_getTMDigit
		std::cout << "getTMDigit() - Single for loop" << std::endl;
		std::cout << "Best score = " << best_score << std::endl;
		std::cout << "ID = " << victim_id << std::endl;
	#endif
}

/*!
* Recognize and retrieves the victim's ID by means of tesserract.
* @param[in] 	hsv_img 			Original image in hsv space.
* @param[in] 	mask 				Green mask.
* @param[in] 	scale 				Scale of the arena (1px/scale = X meters).
* @param[in] 	victim_contours 	Victim points which define the contours.
* @param[in] 	template_images 	Vector of template images.
* @return[int] 	victim_id 			The retrieved victim id.
*/
int getVictimID(const cv::Mat& hsv_img, const cv::Mat& mask, const double scale, 
	const Polygon& victim_contour, std::vector<std::pair<int,cv::Mat>>& template_images){
	// Original image
	cv::Mat img_in;
	cv::cvtColor(hsv_img, img_in, cv::COLOR_HSV2BGR);

	// Retrieve the image with only victim's id
	cv::Mat inverted_mask;
	cv::Mat filtered(img_in.rows, img_in.cols, CV_8UC3, cv::Scalar(255,255,255));
	cv::bitwise_not(mask, inverted_mask);
    img_in.copyTo(filtered, inverted_mask); // filtered = img_in - green shapes

	// "Cast" utils::Polygon to std::vector<cv::Point>
	std::vector<cv::Point> approx_curve;
	for (const auto& pt: victim_contour)
		approx_curve.emplace_back(cv::Point(pt.x*scale, pt.y*scale));
		//std::cout << "x -> " << pt.x*scale << " y -> " << pt.y*scale << std::endl;

	// Get victim ROI
	cv::Rect box = cv::boundingRect(cv::Mat(approx_curve)); // Victim's bounding box
	cv::Mat orig_ROI(filtered, box); // ROI with the digit
	cv::resize(orig_ROI, orig_ROI, cv::Size(200, 200)); // Resize orig_ROI
    cv::threshold(orig_ROI, orig_ROI, 100, 255, 0); // Threshold and binarize ROI to suppress noise

    // Apply some additional smoothing and filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2)+1, (2*2)+1));
    cv::erode(orig_ROI, orig_ROI, kernel);
    cv::GaussianBlur(orig_ROI, orig_ROI, cv::Size(5, 5), 2, 2);
    cv::erode(orig_ROI, orig_ROI, kernel);

    /*// BRUTE-FORCE APPROACH NOT WORKING AT ALL
    // Template matching variables declaration
	static std::pair<int,int> ROI_idx;
	cv::Mat editedROI;
	int rotation_angle = 10.0;
	int victim_id;

    // Retrieve all edited ROIs: rotated of rotation_angle and flipped
    std::vector<std::vector<cv::Mat>> all_ROI;
    getAllROI(orig_ROI, rotation_angle, all_ROI);

    victim_id = getTMDigit(all_ROI, template_images, ROI_idx);

	#ifdef DEBUG_VICTIMS_ID
		std::string ew = "Edited ROI - ID = " + std::to_string(victim_id);
		cv::imshow(ew, all_ROI[ROI_idx.first][ROI_idx.second]);
	#endif*/

    // Rotate and flip orig_ROI in order to be recognizable/readable
    // TODO: the param for the edit can be read from a specific file in /tmp
    cv::Mat edited_ROI;
	double angle = 90;
	bool flag_x_flip = false;
	bool flag_y_flip = true;
	int victim_id;

    getEditedROI(orig_ROI, angle, flag_x_flip, flag_y_flip, edited_ROI);
    getTMDigit(edited_ROI, template_images, victim_id);

	#ifdef DEBUG_VICTIMS_ID
    	cv::imshow("img_in", img_in);
		cv::imshow("Inverted mask", inverted_mask);
		cv::imshow("Filtered image", filtered);

		std::string ew = "Edited ROI - ID = " + std::to_string(victim_id);
		cv::imshow(ew, edited_ROI);
		cv::imshow("Original ROI", orig_ROI);
		cv::waitKey(0);
		cv::destroyAllWindows();

		std::cout << "Victim's ID: " << victim_id << std::endl;
	#endif

	return victim_id;
}

/*!
* Retrieves the victims and the gate. Victims and gate are discriminated by the number of verteces.
* @param[in] 	hsv_img 		Original img in hsv space.
* @param[in] 	scale 			Scale of the arena (1px/scale = X meters).
* @param[out] 	victims_list 	List of obstacles.
* @param[out] 	gate 			Gate polygon.
*/
void getVictimsAndGate(const cv::Mat& hsv_img, const double scale, 
	std::vector<std::pair<int,Polygon>>& victims_list, Polygon& gate){
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

	// Retrieve the template images
	std::vector<std::pair<int,cv::Mat>> template_images;
	std::string config_path = "/tmp"; // TODO: delete and pass as argument to function
	getTemplateImgs(template_images);

	// Retrieve victims and gate depending on verteces
	int victim_id;
	for (int i = 0; i < obj_list.size(); i++){
		// Create the green obj contour poligon
		Polygon green_obj_contour;
		for (const auto& pt: obj_list[i]){
			green_obj_contour.emplace_back(pt.x, pt.y);
		}

		if (i != best_i){
			// Populate the victims vector with id and contour
			victim_id = getVictimID(hsv_img, green_mask, scale, green_obj_contour, template_images);
			victims_list.push_back({victim_id, green_obj_contour});
		}
		else
			// Populate the gate obj
			gate = green_obj_contour;
	}

	#ifdef DEBUG_VICTIMS_GATE
		// Draw victims
		showRetrievedObjs(hsv_img, scale, victims_list, "Victims");

		// Draw gate
		std::vector<Polygon> gate_list = {gate};
		showRetrievedObjs(hsv_img, scale, gate_list, "Gate");

		std::cout << "getVictimsAndGate() - END" << std::endl;
	#endif
}

/*!
* Retrieves the robot/triangle.
* @param[in] 	hsv_img 	Original img in hsv space.
* @param[in] 	scale 		Scale of the arena (1px/scale = X meters).
* @param[out] 	triangle 	Robot/triangle polygon.
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
* Retrieves the robot/trianlge's orientation.
* @param[in] 	triangle 		Robot/triangle polygon.
* @param[in] 	baricenter_x 	Baricenter's x coordinate.
* @param[in] 	baricenter_y 	Baricenter's y coordinate.
* @param[out] 	theta 			Robot/triangle's orientation.
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
// PUBLIC FUNCTIONS
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
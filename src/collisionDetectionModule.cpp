#include "collisionDetectionModule.hpp"

//#define DEBUG_ILL
//#define DEBUG_ICL
//#define DEBUG_IAL
//#define DEBUG_GCFA

namespace plt = matplotlibcpp;

//**********************************************************************
// DATA STRUCTURES AND GETTERS
//**********************************************************************
/*!
* Create a structure representing a segment.
* @param[in]  		x0 	Initial x-coord.
* @param[in]  		y0 	Initial y-coord.
* @param[in]  		xf 	Final x-coord.
* @param[in]  		yf 	Final y-coord.
* @return[segment] 	L 	Segment type.
*/
segment getSegment(double x0, double y0, double xf, double yf){
	segment L;
	L.x0 = x0;
	L.y0 = y0;
	L.xf = xf;
	L.yf = yf;

	return L;
}

/*!
* Create a structure representing a circle.
* @param[in]  		x 	Center x-coord.
* @param[in]  		y 	Center y-coord.
* @param[in]  		r 	Radius of the circle.
* @return[circle] 	c 	Circle type.
*/
circle getCircle(double x, double y, double r){
	circle c;

	c.x = x;
	c.y = y;
	c.r = r;

	return c;
}



//**********************************************************************
// PLOTTING SUPPORT PRIVATE FUNCTIONS
//**********************************************************************
/*!
* Retrieves the two vectors: the former is filled with segment x-coords 
* and the latter with segment y-coords.
* @param[in]   L 			Segment.
* @param[out]  L_x_data 	Vector of x-coords.
* @param[out]  L_y_data 	Vector of y-coords.
*/
void getPlottableSegment(segment L, std::vector<double>& L_x_data, std::vector<double>& L_y_data){
	L_x_data.push_back(L.x0);
	L_x_data.push_back(L.xf);
	L_y_data.push_back(L.y0);
	L_y_data.push_back(L.yf);
}

/*!
* Retrieves the two vectors: the former is filled with circle x-coords 
* and the latter with circle y-coords.
* @param[in]   c 			Circle.
* @param[out]  L_x_data 	Vector of x-coords.
* @param[out]  L_y_data 	Vector of y-coords.
*/
void getPlottableCircle(circle c, std::vector<double>& cf_x, std::vector<double>& cf_y){
	for (double angle = 0; angle <= 360; angle++){
		// radians = degree * PI / 180
		cf_x.push_back(c.r * std::cos(angle * M_PI / 180) + c.x);
		cf_y.push_back(c.r * std::sin(angle * M_PI / 180) + c.y);
	}
}

/*!
* Retrieves the two vectors: the former is filled with arc x-coords 
* and the latter with arc y-coords.
* @param[in]   a 			Arc.
* @param[out]  L_x_data 	Vector of x-coords.
* @param[out]  L_y_data 	Vector of y-coords.
*/
void getPlottableArc(arc a, std::vector<double>& arc_x_data, std::vector<double>&  arc_y_data){
	int npts = 1000;
	double s, x, y;

	for (int i = 0; i < npts; i++){
		s = a.L / npts * i;
		std::tie(x, y, std::ignore) = circline(s, a.x0, a.y0, a.th0, a.k);
		arc_x_data.push_back(x);
		arc_y_data.push_back(y);
	}
}

/*!
* It plots two segments and save it as Segmentss_intersection.png. in 
* /home/user_name/workspace/project/src/testing_imgs .
* @param[in]  L1 	Arc.
* @param[in]  L2 	Vector of x-coords.
*/
void plotLL(segment L1, segment L2){
	// Create structures for plot
	// First segment
	std::vector<double> L1_x_data, L1_y_data;
	getPlottableSegment(L1, L1_x_data, L1_y_data);

	// Second segment
	std::vector<double> L2_x_data, L2_y_data;
	getPlottableSegment(L2, L2_x_data, L2_y_data);

	// Plot
    plt::named_plot("Segment 1", L1_x_data, L1_y_data, "r-");
    plt::named_plot("Segment 2", L2_x_data, L2_y_data, "b-");

    // Set x-axis and y-axis to [xmin-1, xmax+1] and [ymin-1, ymax+1] respectively
	double x_min_max[2] = {plt::xlim()[0], plt::xlim()[1]};
	double y_min_max[2] = {plt::ylim()[0], plt::ylim()[1]};
    plt::xlim(x_min_max[0] - 1, x_min_max[1] + 1);
    plt::ylim(y_min_max[0] - 1, y_min_max[1] + 1);

    plt::title("Line intersection"); 	// Add graph title
    plt::legend(); 						// Enable legend.

    // Save png
	std::string this_file_path = __FILE__;
	std::string this_file_name = "collisionDetectionModule.cpp";
	int upper_bound = this_file_path.length() - this_file_name.length();
	std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Segments_intersection.png";
    plt::save(png_name);
}

/*!
* It plots a circle and a segment and save it as Circle_segment_intersection.png in 
* /home/user_name/workspace/project/src/testing_imgs .
* @param[in]  c 	Circle.
* @param[in]  L 	Segment.
*/
void plotCL(circle c, segment L){
	// Create structures for plot
	// Circle
	std::vector<double> cf_x, cf_y;
	getPlottableCircle(c, cf_x, cf_y);

	// Segment
	std::vector<double> L_x_data, L_y_data;
	getPlottableSegment(L, L_x_data, L_y_data);

	// Plot
    plt::named_plot("Circle", cf_x, cf_y, "r-");
    plt::named_plot("Segment", L_x_data, L_y_data, "b-");

    // Set x-axis and y-axis to [-max-1, max+1]
    // Squared picture: otherwise the circle seems an elipses
    //double min = std::fmin(plt::xlim()[0], plt::ylim()[0]);
    double max = std::fmax(plt::xlim()[1], plt::ylim()[1]);
    plt::xlim(- max - 1, max + 1);
    plt::ylim(- max - 1, max + 1);

    plt::title("Circle - line intersection"); 	// Add graph title
    plt::legend(); 								// Enable legend.

    // Save png
	std::string this_file_path = __FILE__;
	std::string this_file_name = "collisionDetectionModule.cpp";
	int upper_bound = this_file_path.length() - this_file_name.length();
	std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Circle_segment_intersection.png";
    plt::save(png_name);
}

/*!
* It plots an arc and a segment and save it as Arc_segment_intersection.png in 
* /home/user_name/workspace/project/src/testing_imgs .
* @param[in]  c 	Circle.
* @param[in]  a 	Arc.
* @param[in]  L 	Segment.
* @param[in]  i_x0 	First intersection point x-coord.
* @param[in]  i_y0 	First intersection point y-coord.
* @param[in]  i_x1 	Second intersection point x-coord.
* @param[in]  i_y1 	Second intersection point y-coord.
*/
void plotAL(circle c, arc a, segment L, double i_x0, double i_y0, double i_x1, double i_y1){
	// Create structures for plot
	// Circle
	std::vector<double> cf_x, cf_y;
	getPlottableCircle(c, cf_x, cf_y);

	// Arc
	std::vector<double> arc_x_data, arc_y_data;
	getPlottableArc(a, arc_x_data, arc_y_data);

	// Segment
	std::vector<double> L_x_data, L_y_data;
	getPlottableSegment(L, L_x_data, L_y_data);

	// Plot 
    plt::named_plot("Circle", cf_x, cf_y, "r-");
    plt::named_plot("Arc", arc_x_data, arc_y_data, "g-");
    plt::named_plot("Segment", L_x_data, L_y_data, "b-");

	// Segment from circle center to i_x0, i_y0
	if (! std::isnan(i_x0)){
		segment ci0 = getSegment(c.x, c.y, i_x0, i_y0);
		std::vector<double> ci0_x_data, ci0_y_data;
		getPlottableSegment(ci0, ci0_x_data, ci0_y_data);
		plt::named_plot("Intersection 1", ci0_x_data, ci0_y_data, "black");

	}
	
	// Segment from circle center to i_x1, i_y1
	if (! std::isnan(i_x1)){
		segment ci1 = getSegment(c.x, c.y, i_x1, i_y1);
		std::vector<double> ci1_x_data, ci1_y_data;
		getPlottableSegment(ci1, ci1_x_data, ci1_y_data);
		plt::named_plot("Intersection 2", ci1_x_data, ci1_y_data, "black");
	}

    // Set x-axis and y-axis
    double min = std::fmin(plt::xlim()[0], plt::ylim()[0]);
    double max = std::fmax(plt::xlim()[1], plt::ylim()[1]);
    plt::xlim(min - 1, max + 1);
    plt::ylim(min - 1, max + 1);

    plt::title("Arc - line intersection"); 		// Add graph title
    plt::legend(); 								// Enable legend.

    // Save png
	std::string this_file_path = __FILE__;
	std::string this_file_name = "collisionDetectionModule.cpp";
	int upper_bound = this_file_path.length() - this_file_name.length();
	std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Arc_segment_intersection.png";
    plt::save(png_name);
}



//**********************************************************************
// SUPPORT PRIVATE FUNCTIONS
//**********************************************************************
/*!
* It returns the angular coefficient of the line passing via the 
* provided segment (i.e. m = delta_y / delta_x).
* @param[in]   L 	Segment.
* @return[double] 	Angular coefficient (i.e. m).
*/
double getAngularCoeff(segment L){

	return (L.yf - L.y0) / (L.xf - L.x0);
}

/*!
* It returns the intercept of the line passing via the provided segment with 
* a given angular coefficient.
* @param[in]   L 	Segment.
* @param[in]   m 	Angular coefficient of the line passing via L.
* @return[double] 	Intercept (i.e. q).
*/
double getIntercept(segment L, double m){

	return (L.yf - m * L.xf);
}

/*!
* Given a segment it returns true if it is verticla (i.e. angular coeff = INF).
* @param[in]   L 	Segment.
* @return[bool] 	True if vertical, false otherwise.
*/
bool isVertical(segment L){

	return (L.x0 == L.xf);
}

/*!
* Given a segment it returns true if it is horizontal (i.e. angular coeff = 0).
* @param[in]   L 	Segment.
* @return[bool] 	True if horizontal, false otherwise.
*/
bool isHorizontal(segment L){

	return (L.y0 == L.yf);
}

/*!
* Given a segment it returns the middle point coordinates.
* @param[in]   L 						Segment.
* @return[std::tuple<double, double>] 	Tuple with x-coord and y-coord.
*/
std::tuple<double, double> getMiddlePtCoords(segment L){

	return std::make_tuple( ( (L.x0 + L.xf) / 2 ), ( (L.y0 + L.yf) / 2 ) );
}

/*!
* It returns the the length of a given segment.
* @param[in]   L 	Segment.
* @return[double] 	Tuple with x-coord and y-coord.
*/
double getSegmentLength(segment L){

	return std::sqrt(std::pow((L.x0 - L.xf), 2) + std::pow((L.y0 - L.yf), 2));
}

/*!
* Retrieves two chords from an arc. None of them is vertical nor horizontal.
* @param[in]  a 						Arc.
* @return[std::tuple<segment, segment>] Tuple with 2 segment chords.
*/
std::tuple<segment, segment> getChords(arc a){
	int npts = 2;
	double _x0, _y0, s;
	segment C0, C1;

	do{
		// Define (_x0, _y0) as the arc point where a.L = arc_L/npts
		s = a.L / npts;
		std::tie(_x0, _y0, std::ignore) = circline(s, a.x0, a.y0, a.th0, a.k);

		C0 = getSegment(a.x0, a.y0, _x0, _y0); // Segment / chord from (a.x0, a.0) to (_x0, _y0)
		C1 = getSegment(a.xf, a.yf, _x0, _y0); // Segment / chord from (a.xf, a.f) to (_x0, _y0)

		npts += 1;
	}
	while (isVertical(C0) || isVertical(C1) || isHorizontal(C0) || isHorizontal(C1));

	return std::make_tuple(C0, C1);
}

/*!
* Given a dubins arc it returns the corresponding circle.
* It finds two chords within the arc. Then i looks for chords bisectors. 
* In the end, from the bisectors intersection, it finds the center.
* @param[in]   a 	Arc.
* @return[circle] 	Circle derived from the provided arc.
*/
circle getCricleFromArc(arc a){
	// Get 2 chords
	segment C0, C1; 					// C0 = (a.x0, a.y0) to (_x0, _y0)
	std::tie(C0, C1) = getChords(a); 	// C1 = (a.xf, a.yf) to (_x0, _y0)

	// Define angular coeff and intercept of C0
	double C0_m = getAngularCoeff(C0);
	double C0_q = getIntercept(C0, C0_m);
	// Find the middle pt coords of C0
	double mid_x0, mid_y0;
	std::tie(mid_x0, mid_y0) = getMiddlePtCoords(C0);
	// Build a perpendicular line PL0 from the middle pt (mid_x0, mid_y0)
	double PL0_m = -(1 / C0_m); // 2 lines are perpendicular if C0_m * PL0_m = -1
	double PL0_q = mid_y0 - PL0_m * mid_x0;

	// Define angular coeff and intercept of C1
	double C1_m = getAngularCoeff(C1);
	double C1_q = getIntercept(C1, C1_m);
	// Find the middle pt coords of C1
	double mid_x1, mid_y1;
	std::tie(mid_x1, mid_y1) = getMiddlePtCoords(C1);
	// Build a perpendicular line PL2 from the middle pt (mid_x1, mid_y1)
	double PL1_m = -(1 / C1_m); // 2 lines are perpendicular if C1_m * PL1_m = -1
	double PL1_q = mid_y1 - PL1_m * mid_x1;

	// Find circle center by resolving PL0 = PL1 -> intersection point (cx, cy) is the circle center
	double cx = (- PL0_q + PL1_q) / (PL0_m - PL1_m); 	// cx that resolve PL0 = PL1
	double cy = PL0_m * cx + PL0_q;

	// Find circle radius
	double r = std::sqrt(std::pow((cx - a.x0), 2) + std::pow((cy - a.y0), 2));

	#ifdef DEBUG_GCFA
		std::cout << "Center in cx = " << cx << ", cy = " << cy << std::endl;
		/*// Check reverse formulation
		double cx1 = (- PL1_q + PL0_q) / (PL1_m - PL0_m);
		double cy1 = PL1_m * cx1 + PL1_q;
		if (cy == cy1) std::cout << "Center found" << std::endl;*/

	    // Check radius
		double r_ = std::sqrt(std::pow((cx - C0.xf), 2) + std::pow((cy - C0.yf), 2)); // C0.xf == _x0, C0.yf == _y0
		double rf = std::sqrt(std::pow((cx - a.xf), 2) + std::pow((cy - a.yf), 2));
		std::cout << "r = " << r << " - rf = " << rf << " - r_ = " << r_ << std::endl;
		
		// Arc
		std::vector<double> arc_x_data, arc_y_data;
		getPlottableArc(a, arc_x_data, arc_y_data);
		// Chord 0
		std::vector<double> C0_x_data, C0_y_data;
		getPlottableSegment(C0, C0_x_data, C0_y_data);
		// PL0
		std::vector<double> PL0_x_data, PL0_y_data;
		segment PL0 = getSegment(mid_x0, mid_y0, cx, cy);
		getPlottableSegment(PL0, PL0_x_data, PL0_y_data);
		// Chord 1
		std::vector<double> C1_x_data, C1_y_data;
		getPlottableSegment(C1, C1_x_data, C1_y_data);
		// PL1
		std::vector<double> PL1_x_data, PL1_y_data;
		segment PL1 = getSegment(mid_x1, mid_y1, cx, cy);
		getPlottableSegment(PL1, PL1_x_data, PL1_y_data);
		// Radius from (arc_x0, arc_y0)
		std::vector<double> R0_x_data, R0_y_data;
		segment r_seg0 = getSegment(a.x0, a.y0, cx, cy);
		getPlottableSegment(r_seg0, R0_x_data, R0_y_data);
		// Radius from (arc_xf, arc_yf)
		std::vector<double> Rf_x_data, Rf_y_data;
		segment r_segf = getSegment(a.xf, a.yf, cx, cy);
		getPlottableSegment(r_segf, Rf_x_data, Rf_y_data);

		// Plot 
    	plt::named_plot("Arc", arc_x_data, arc_y_data, "g-");

	    plt::named_plot("C0", C0_x_data, C0_y_data, "b-");
	    plt::named_plot("PL0", PL0_x_data, PL0_y_data, "b-");
    	
	    plt::named_plot("C1", C1_x_data, C1_y_data, "r-");
	    plt::named_plot("PL1", PL1_x_data, PL1_y_data, "r-");
    	
	    plt::named_plot("R0", R0_x_data, R0_y_data, "black");
	    plt::named_plot("Rf", Rf_x_data, Rf_y_data, "black");

	    // Set x-axis and y-axis to [-max-1, max+1]
	    //double min = std::fmin(plt::xlim()[0], plt::ylim()[0]);
	    double max = std::fmax(plt::xlim()[1], plt::ylim()[1]);
	    plt::xlim(- max - 1, max + 1);
	    plt::ylim(- max - 1, max + 1);

	    plt::title("Retrieve center via cords");
	    //plt::legend();
	    //plt::show();

	    // Save png
		std::string this_file_path = __FILE__;
		std::string this_file_name = "collisionDetectionModule.cpp";
		int upper_bound = this_file_path.length() - this_file_name.length();
		std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Center_via_cords.png";
	    plt::save(png_name);
	#endif

	return getCircle(cx, cy, r);
}



//**********************************************************************
// PUBLIC FUNCTIONS FOR COLLISION DETECTION
//**********************************************************************
/*!
* Determine if there is an intersection between two segments. 
* If there is an intersection it returns also the coords.
* @param[in]  L1 				First segment.
* @param[in]  L2				Second segment.
* @param[out] <flag_i, x, y> 	Tuple with bool true if intersection exists, intersection (x-coord, y-coord).
*/
std::tuple<bool, double, double> intersLineLine(segment L1, segment L2){
	double L1_min_x = std::fmin(L1.x0, L1.xf);
	double L1_min_y = std::fmin(L1.y0, L1.yf);
	double L1_max_x = std::fmax(L1.x0, L1.xf);
	double L1_max_y = std::fmax(L1.y0, L1.yf);

	double L2_min_x = std::fmin(L2.x0, L2.xf);
	double L2_min_y = std::fmin(L2.y0, L2.yf);
	double L2_max_x = std::fmax(L2.x0, L2.xf);
	double L2_max_y = std::fmax(L2.y0, L2.yf);

	// Position check
	if (L2_max_x < L1_min_x || 	// L2 completely left of L1
		L2_min_x > L1_max_x || 	// L2 completely right of L1
		L2_max_y < L1_min_y || 	// L2 completely below L1
		L2_min_y > L1_max_y){ 	// L2 completely above L1
		#ifdef DEBUG_ILL
			plotLL(L1, L2);
		#endif

		return std::make_tuple(false, std::nan("1"), std::nan("1")); // segments do not intersect
	}

	// Special cases check (perpendicular crossed segments)
	if (isVertical(L1) && isHorizontal(L2)){
		#ifdef DEBUG_ILL
			plotLL(L1, L2);
		#endif

		return std::make_tuple(true, L1.x0, L2.y0);
	}
	if (isVertical(L2) && isHorizontal(L1)){
		#ifdef DEBUG_ILL
			plotLL(L1, L2);
		#endif

		return std::make_tuple(true, L2.x0, L1.y0);
	}

	// Special cases check (only one segment in vertical/horizontal position)
	if (isVertical(L1)){
		double L2_m = getAngularCoeff(L2);
		double L2_q = getIntercept(L2, L2_m);

		// Resolve L1 = L2 and find intersection point (x, y)
		double x = L1.x0; 				// x == L1.x0 == L1.xf
		double y = L2_m * x + L2_q;

		// Find out if the intersection point (x,y) belong to L1 and L2
		bool pt_on_L1 = (x >= L1_min_x && x <= L1_max_x && y >= L1_min_y && y <= L1_max_y); // (x,y) is on L1
		bool pt_on_L2 = (x >= L2_min_x && x <= L2_max_x && y >= L2_min_y && y <= L2_max_y); // (x,y) is on L2

		#ifdef DEBUG_ILL
			plotLL(L1, L2);
		#endif

		if (pt_on_L1 && pt_on_L2) 
			return std::make_tuple(true, x, y);
		else 
			return std::make_tuple(false, std::nan("1"), std::nan("1"));
	}
	if (isHorizontal(L1)){
		double L2_m = getAngularCoeff(L2);
		double L2_q = getIntercept(L2, L2_m);

		// Resolve L1 = L2 and find intersection point (x, y)
		double y = L1.y0; 				// y == L1.y0 == L1.yf
		double x = (y - L2_q) / L2_m; 	// y = m * x + q -> (x = y - q) / m

		// Find out if the intersection point (x,y) belong to L1 and L2
		bool pt_on_L1 = (x >= L1_min_x && x <= L1_max_x && y >= L1_min_y && y <= L1_max_y); // (x,y) is on L1
		bool pt_on_L2 = (x >= L2_min_x && x <= L2_max_x && y >= L2_min_y && y <= L2_max_y); // (x,y) is on L2

		#ifdef DEBUG_ILL
			plotLL(L1, L2);
		#endif

		if (pt_on_L1 && pt_on_L2) 
			return std::make_tuple(true, x, y);
		else 
			return std::make_tuple(false, std::nan("1"), std::nan("1"));
	}
	if (isVertical(L2)){
		double L1_m = getAngularCoeff(L1);
		double L1_q = getIntercept(L1, L1_m);

		// Resolve L1 = L2 and find intersection point (x, y)
		double x = L2.x0; 				// x == L2.x0 == L2.xf
		double y = L1_m * x + L1_q;

		// Find out if the intersection point (x,y) belong to L1 and L2
		bool pt_on_L1 = (x >= L1_min_x && x <= L1_max_x && y >= L1_min_y && y <= L1_max_y); // (x,y) is on L1
		bool pt_on_L2 = (x >= L2_min_x && x <= L2_max_x && y >= L2_min_y && y <= L2_max_y); // (x,y) is on L2

		#ifdef DEBUG_ILL
			plotLL(L1, L2);
		#endif

		if (pt_on_L1 && pt_on_L2) 
			return std::make_tuple(true, x, y);
		else 
			return std::make_tuple(false, std::nan("1"), std::nan("1"));
	}
	if (isHorizontal(L2)){
		double L1_m = getAngularCoeff(L1);
		double L1_q = getIntercept(L1, L1_m);

		// Resolve L1 = L2 and find intersection point (x, y)
		double y = L2.y0; 				// y == L2.y0 == L2.yf
		double x = (y - L1_q) / L1_m; 	// y = m * x + q -> (x = y - q) / m

		// Find out if the intersection point (x,y) belong to L1 and L2
		bool pt_on_L1 = (x >= L1_min_x && x <= L1_max_x && y >= L1_min_y && y <= L1_max_y); // (x,y) is on L1
		bool pt_on_L2 = (x >= L2_min_x && x <= L2_max_x && y >= L2_min_y && y <= L2_max_y); // (x,y) is on L2

		#ifdef DEBUG_ILL
			plotLL(L1, L2);
		#endif

		if (pt_on_L1 && pt_on_L2) 
			return std::make_tuple(true, x, y);
		else 
			return std::make_tuple(false, std::nan("1"), std::nan("1"));
	}

	// "Classic" case - Line eq. -> y = m * x + q
	double L1_m = getAngularCoeff(L1);
	double L2_m = getAngularCoeff(L2);
	double L1_q = getIntercept(L1, L1_m);
	double L2_q = getIntercept(L2, L2_m);

	if (L1_m == L2_m){
		double x = std::nan("1");
		double y = std::nan("1");

		if (L1_q == L2_q){
			#ifdef DEBUG_ILL
				plotLL(L1, L2);
			#endif

			return std::make_tuple(true, x, y); 	// Same line with at least 1 pt in common up to infinite
		}
		else{
			#ifdef DEBUG_ILL
				plotLL(L1, L2);
			#endif

			return std::make_tuple(false, x, y); 	// Parallel different lines
		}
	}

	// Resolve L1 = L2 and find intersection point (x, y)
	double x = (L2_q - L1_q) / (L1_m - L2_m); 	// x that resolve L1 = L2
	double y = L1_m * x + L1_q; 				// equivalent to y = L2_m * x + L2_q

	// Find out if the intersection point (x,y) belong to L1 and L2
	bool pt_on_L1 = (x >= L1_min_x && x <= L1_max_x && y >= L1_min_y && y <= L1_max_y); // (x,y) is on L1
	bool pt_on_L2 = (x >= L2_min_x && x <= L2_max_x && y >= L2_min_y && y <= L2_max_y); // (x,y) is on L2

	#ifdef DEBUG_ILL
		plotLL(L1, L2);
	#endif

	if (pt_on_L1 && pt_on_L2) 
		return std::make_tuple(true, x, y);
	else 
		return std::make_tuple(false, std::nan("1"), std::nan("1"));
}

/*!
* Determine if there is an intersection between a circle and a segment. If there is an 
* intersection it returns also the coords.
* @param[in]  c																Circle.
* @param[in]  L																Segment.
* @return[std::tuple<bool, double, double, double, double, double, double>] Tuple with bool true if intersection exists, 1st intersection (x-coord, y-coord), 1st intersection angle w.r.t. center, 2nd intersection (x-coord, y-coord), 2nd intersection angle w.r.t. center.
*/
std::tuple<bool, double, double, double, double, double, double> intersCircleLine(circle c, segment L){	
	// Compute the various polynoms (as in the provided slides / matlab code)
	double p1 = 2 * L.x0 * L.xf;
    double p2 = 2 * L.y0 * L.yf;
    double p3 = 2 * c.x * L.x0;
    double p4 = 2 * c.x * L.xf;
    double p5 = 2 * c.y * L.y0;
    double p6 = 2 * c.y * L.yf;

    double c1 = std::pow(L.x0, 2) + std::pow(L.xf, 2) - p1 + std::pow(L.y0, 2) + std::pow(L.yf, 2) - p2;
    double c2 = - 2 * std::pow(L.xf, 2) + p1 - p3 + p4 - 2 * std::pow(L.yf, 2) + p2 - p5 + p6;
    double c3 = std::pow(L.xf, 2) - p4 + std::pow(c.x, 2) + std::pow(L.yf, 2) - p6 + std::pow(c.y, 2) - std::pow(c.r, 2);

    double delta = std::pow(c2, 2) - 4 * c1 * c3;    

    // No intersections at all
    if (delta < 0){
    	#ifdef DEBUG_ICL
    		plotCL(c, L);
    	#endif

    	return std::make_tuple(false, std::nan("1"), std::nan("1"), std::nan("1"), 
    		std::nan("1"), std::nan("1"), std::nan("1"));
    }

    double t1, t2;
    if (delta > 0){
    	// two intersection points
    	t1 = (- c2 + std::sqrt(delta)) / (2 * c1);
    	t2 = (- c2 - std::sqrt(delta)) / (2 * c1);
    } else {
    	// delta == 0 --> only one intersection point
    	t1 = - c2 / 2 * c1;
    	t2 = t1;
    }

    // Intersection pts coords and intersection angles w.r.t. the circle center
    double i_x0 = std::nan("1"), i_y0 = std::nan("1"), i_th0 = std::nan("1");
    double i_x1 = std::nan("1"), i_y1 = std::nan("1"), i_th1 = std::nan("1");

    if (t1 >= 0 && t1 <= 1){
    	i_x0 = L.x0 * t1 + L.xf * (1 - t1);
    	i_y0 = L.y0 * t1 + L.yf * (1 - t1);
    	i_th0 = std::atan2((i_y0 - c.y), (i_x0 - c.x));
    }

    if (t2 >= 0 && t2 <= 1 && t2 != t1){
    	i_x1 = L.x0 * t2 + L.xf * (1 - t2);
    	i_y1 = L.y0 * t2 + L.yf * (1 - t2);
    	i_th1 = std::atan2((i_y1 - c.y), (i_x1 - c.x));
    }

    #ifdef DEBUG_ICL
    	plotCL(c, L);
    #endif

	// Check whether the segment is internal to the circle
	if (std::isnan(i_x0) && std::isnan(i_x1))
		return std::make_tuple(false, i_x0, i_y0, i_th0, i_x1, i_y1, i_th1);

    return std::make_tuple(true, i_x0, i_y0, i_th0, i_x1, i_y1, i_th1);
}

/*!
* Determine if there is an intersection between an arc of a circle and a segment. 
* If there is an intersection it returns also the coords.
* @param[in]  a 											Arc.
* @param[in]  L 											Segment.
* @return[std::tuple<bool, double, double, double, double>] Tuple with bool true if intersection exists, 1st intersection (x-coord, y-coord), 2nd intersection (x-coord, y-coord).
*/
std::tuple<bool, double, double, double, double> intersArcLine(arc a, segment L){
	// Get the circle from the arc
	circle c = getCricleFromArc(a);

	// Get intersections between circle and line if exist
	bool flag_i;
	double i_x0, i_y0, i_th0;
	double i_x1, i_y1, i_th1;
	std::tie(flag_i, i_x0, i_y0, i_th0, i_x1, i_y1, i_th1) = intersCircleLine(c, L);

	if (flag_i){
		// Get the angle of the extreme pts of the arc w.r.t. the circle center
		double c_arc_th0 = std::atan2((a.y0 - c.y), (a.x0 - c.x));
		double c_arc_thf = std::atan2((a.yf - c.y), (a.xf - c.x));
		// Get the the ordered angles interval
		double c_min_th = std::fmin(c_arc_th0, c_arc_thf);
		double c_max_th = std::fmax(c_arc_th0, c_arc_thf);

		// If c_min_th <= i_th0 <= c_max_th --> circle intersection point lies on the arc
		// Manage the first intersection point
		if (!(std::isnan(i_x0))){ // check if 1st intersection pt exists
			if (!(i_th0 >= c_min_th && i_th0 <= c_max_th)){ // if angle not in interval
				i_x0 = std::nan("1");
				i_y0 = std::nan("1");
			}
		}

		// If c_min_th <= i_th1 <= c_max_th --> circle intersection point lies on the arc
		// Manage the second intersection point
		if (!(std::isnan(i_x1))){ // check if 2nd intersection pt exists
			if (!(i_th1 >= c_min_th && i_th1 <= c_max_th)){ // if angle not in interval
				i_x1 = std::nan("1");
				i_y1 = std::nan("1");
			}
		}

		bool flag_a_i = !(std::isnan(i_x0) && std::isnan(i_x1)); // check if at least one intersection has survived

		#ifdef DEBUG_IAL
			plotAL(c, a, L, i_x0, i_y0, i_x1, i_y1);
		#endif

		return std::make_tuple(flag_a_i, i_x0, i_y0, i_x1, i_y1);
	}

	#ifdef DEBUG_IAL
		plotAL(c, a, L, i_x0, i_y0, i_x1, i_y1);
	#endif

	// All previous checks have failed --> no intersection
	return std::make_tuple(false, std::nan("1"), std::nan("1"), std::nan("1"), std::nan("1"));
}



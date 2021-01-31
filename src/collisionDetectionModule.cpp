#include "collisionDetectionModule.hpp"

//#define DEBUG_IAL
//#define DEBUG_GCFA

namespace plt = matplotlibcpp;

//**********************************************************************
// SUPPORT PRIVATE FUNCTIONS
//**********************************************************************
void getPlottableSegment(double L_x0, double L_y0, double L_xf, double L_yf, 
	std::vector<double>& L_x_data, std::vector<double>& L_y_data){
	L_x_data.push_back(L_x0);
	L_x_data.push_back(L_xf);
	L_y_data.push_back(L_y0);
	L_y_data.push_back(L_yf);
}

void getPlottableCircle(double cx, double cy, double r, 
	std::vector<double>& cf_x, std::vector<double>& cf_y){
	for (double angle = 0; angle <= 360; angle++){
		// radians = degree * PI / 180
		cf_x.push_back(r * std::cos(angle * M_PI / 180) + cx);
		cf_y.push_back(r * std::sin(angle * M_PI / 180) + cy);
	}
}

void getPlottableArc(double arc_x0, double arc_y0, double arc_th0, double arc_L, double arc_k, 
	std::vector<double>& arc_x_data, std::vector<double>&  arc_y_data){
	int npts = 1000;
	double s, x, y;

	for (int i = 0; i < npts; i++){
		s = arc_L / npts * i;
		std::tie(x, y, std::ignore) = circline(s, arc_x0, arc_y0, arc_th0, arc_k);
		arc_x_data.push_back(x);
		arc_y_data.push_back(y);
	}
}

void plotLL(double L1_x0, double L1_y0, double L1_xf, double L1_yf, 
	double L2_x0, double L2_y0, double L2_xf, double L2_yf){
	// Create structures for plot
	// First segment
	std::vector<double> L1_x_data, L1_y_data;
	getPlottableSegment(L1_x0, L1_y0, L1_xf, L1_yf, L1_x_data, L1_y_data);

	// Second segment
	std::vector<double> L2_x_data, L2_y_data;
	getPlottableSegment(L2_x0, L2_y0, L2_xf, L2_yf, L2_x_data, L2_y_data);

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
	std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Line_intersection.png";
    plt::save(png_name);
}

void plotCL(double cx, double cy, double r, double L_x0, double L_y0, double L_xf, double L_yf){
	// Create structures for plot
	// Circle
	std::vector<double> cf_x, cf_y;
	getPlottableCircle(cx, cy, r, cf_x, cf_y);

	// Segment
	std::vector<double> L_x_data, L_y_data;
	getPlottableSegment(L_x0, L_y0, L_xf, L_yf, L_x_data, L_y_data);

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
	std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Circle_line_intersection.png";
    plt::save(png_name);
}

void plotAL(double cx, double cy, double r, 
	double arc_L, double arc_x0, double arc_y0, double arc_th0, double arc_k,
	double L_x0, double L_y0, double L_xf, double L_yf, 
	double i_x0, double i_y0, double i_x1, double i_y1){
	// Create structures for plot
	// Circle
	std::vector<double> cf_x, cf_y;
	getPlottableCircle(cx, cy, r, cf_x, cf_y);

	// Arc
	std::vector<double> arc_x_data, arc_y_data;
	getPlottableArc(arc_x0, arc_y0, arc_th0, arc_L, arc_k, arc_x_data, arc_y_data);

	// Segment
	std::vector<double> L_x_data, L_y_data;
	getPlottableSegment(L_x0, L_y0, L_xf, L_yf, L_x_data, L_y_data);

	// Plot 
    plt::named_plot("Circle", cf_x, cf_y, "r-");
    plt::named_plot("Arc", arc_x_data, arc_y_data, "g-");
    plt::named_plot("Segment", L_x_data, L_y_data, "b-");

	// Segment to i_x0, i_y0
	if (! std::isnan(i_x0)){
		std::vector<double> L_x0_data, L_y0_data;
		getPlottableSegment(cx, cy, i_x0, i_y0, L_x_data, L_y_data);
		plt::named_plot("Intersection 1", L_x0_data, L_y0_data, "black");

	}
	
	// Segment to i_x1, i_y1
	if (! std::isnan(i_x1)){
		std::vector<double> L_x1_data, L_y1_data;
		getPlottableSegment(cx, cy, i_x1, i_y1, L_x1_data, L_y1_data);
		plt::named_plot("Intersection 2", L_x1_data, L_y1_data, "black");
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
	std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Arc_line_intersection.png";
    plt::save(png_name);
}

/*!
* Return true if a point lies/belongs to a given segment, false otherwise.
* @param[in]  pt_x 	Point x-coord.
* @param[in]  pt_y	Point y-coord.
* @param[in]  L_x0 	Segment inital point x-coord.
* @param[in]  L_y0	Segment inital point y-coord.
* @param[in]  L_xf	Segment final point x-coord.
* @param[in]  L_yf	Segment final point y-coord.
* @param[out] flag 	True if point lies on the line.
*/
bool isPtOnSegment(double pt_x, double pt_y, double L_x0, double L_y0, double L_xf, double L_yf){
	// Check if the point is outside the segment range
	double L_min_x = std::fmin(L_x0, L_xf);
	double L_min_y = std::fmin(L_y0, L_yf);
	double L_max_x = std::fmax(L_x0, L_xf);
	double L_max_y = std::fmax(L_y0, L_yf);

	if (!(pt_x >= L_min_x && pt_x <= L_max_x && pt_y >= L_min_y && pt_y <= L_max_y))
		return false;

	std::cout << "Segment min_x = " << L_min_x << " max_x = " << L_max_x << std::endl;
	std::cout << "Segment min_y = " << L_min_y << " max_y = " << L_max_y << std::endl;
	std::cout << "Pt in range (" << pt_x << ", " << pt_y << ")" << std::endl;
	// Point is in range hence...
	// Pt belongs to L iff the line passing via (pt_x, pt_y) and (L_x0, L_y0)
	// has the same angular coeff (i.e. m) and intercept (q) of the line passing via the segment.

	// m and q of line upon which relies the segment
	double L_m = (L_y0 - L_yf) / (L_x0 - L_xf); 	// m = delta_y / delta_x
	double L_q = L_y0 - L_m * L_x0; 				// q = y - m * x

	// m and q of line passing via (pt_x, pt_y) and (L_x0, L_y0)
	double Pt_L_m = (L_y0 - pt_y) / (L_x0 - pt_x); 	// m = delta_y / delta_x
	double Pt_L_q = L_y0 - Pt_L_m * L_x0; 			// q = y - m * x

	if (L_m == Pt_L_m && L_q == Pt_L_q) return true;
	else return false;
}

/*!
* Retrieves the center coords and radius of a circle from a given dubins arc.
* It finds two chords within the arc. Looks for chords bisectors. From the 
* bisectors intersection it finds the center.
* @param[in]  arc_x0 		Arc initial x-coord.
* @param[in]  arc_y0 		Arc initial y-coord.
* @param[in]  arc_th0 		Arc initial theta (w.r.t. robot).
* @param[in]  arc_xf 		Arc final x-coord.
* @param[in]  arc_yf 		Arc final y-coord.
* @param[in]  arc_L 		Arc lenght.
* @param[in]  arc_k 		Arc max curvature.
* @param[out] <cx, cy, r> 	Tuple with center x-coord, center y-coord and radius.
*/
std::tuple<double, double, double> getCricleFromArc(double arc_x0, double arc_y0, double arc_th0, 
	double arc_xf, double arc_yf, double arc_L, double arc_k){
	// Get two points on the arc
	int npts = 2;

	// Define (_x0, _y0) as the arc point where arc_L = arc_L/2
	// Find the segment / chord C0 from (arc_x0, arc_y0) to (_x0, _y0)
	double _x0, _y0;
	double s = arc_L / npts;
	std::tie(_x0, _y0, std::ignore) = circline(s, arc_x0, arc_y0, arc_th0, arc_k);
	// Define angular coeff and intercept of C0
	double C0_m = (arc_y0 - _y0) / (arc_x0 - _x0);
	double C0_q = arc_y0 - C0_m * arc_x0;
	// Find the middle pt coords of C0
	double mid_x0 = (arc_x0 + _x0) / 2;
	double mid_y0 = (arc_y0 + _y0) / 2;
	// Build a perpendicular line PL0 from the middle pt
	double PL0_m = -(1 / C0_m); // 2 lines are perpendicular if C0_m * PL0_m = -1
	double PL0_q = mid_y0 - PL0_m * mid_x0;


	// Find the segment / chord C1 from (arc_xf, arc_yf) to (_x0, _y0)
	// Define angular coeff and intercept of C1
	double C1_m = (arc_yf - _y0) / (arc_xf - _x0);
	double C1_q = arc_yf - C1_m * arc_xf;
	// Find the middle pt coords of C1
	double mid_x1 = (arc_xf + _x0) / 2;
	double mid_y1 = (arc_yf + _y0) / 2;
	// Build a perpendicular line PL2 from the middle pt
	double PL1_m = -(1 / C1_m); // 2 lines are perpendicular if C1_m * PL1_m = -1
	double PL1_q = mid_y1 - PL1_m * mid_x1;

	// Find circle center by resolving PL0 = PL1 -> intersection point (cx, cy) is the circle center
	double cx = (- PL0_q + PL1_q) / (PL0_m - PL1_m); 	// cx that resolve PL0 = PL1
	double cy = PL0_m * cx + PL0_q;

	// Find circle radius
	double r = std::sqrt(std::pow((cx - arc_x0), 2) + std::pow((cy - arc_y0), 2));

	#ifdef DEBUG_GCFA
		std::cout << "Center in cx = " << cx << ", cy = " << cy << std::endl;
		/*// Check reverse formulation
		double cx1 = (- PL1_q + PL0_q) / (PL1_m - PL0_m);
		double cy1 = PL1_m * cx1 + PL1_q;
		if (cy == cy1) std::cout << "Center found" << std::endl;*/

	    // Check radius
		double r_ = std::sqrt(std::pow((cx - _x0), 2) + std::pow((cy - _y0), 2));
		double rf = std::sqrt(std::pow((cx - arc_xf), 2) + std::pow((cy - arc_yf), 2));
		std::cout << "r = " << r << " - rf = " << rf << " - r_ = " << r_ << std::endl;
		
		// Arc
		std::vector<double> arc_x_data, arc_y_data;
		getPlottableArc(arc_x0, arc_y0, arc_th0, arc_L, arc_k, arc_x_data, arc_y_data);
		// Chord 0
		std::vector<double> C0_x_data, C0_y_data;
		getPlottableSegment(arc_x0, arc_y0, _x0, _y0, C0_x_data, C0_y_data);
		// PL0
		std::vector<double> PL0_x_data, PL0_y_data;
		getPlottableSegment(mid_x0, mid_y0, cx, cy, PL0_x_data, PL0_y_data);
		// Chord 1
		std::vector<double> C1_x_data, C1_y_data;
		//getPlottableSegment(arc_xf, arc_yf, _xf, _yf, C1_x_data, C1_y_data);
		getPlottableSegment(arc_xf, arc_yf, _x0, _y0, C1_x_data, C1_y_data);
		// PL1
		std::vector<double> PL1_x_data, PL1_y_data;
		getPlottableSegment(mid_x1, mid_y1, cx, cy, PL1_x_data, PL1_y_data);
		// Radius from (arc_x0, arc_y0)
		std::vector<double> R0_x_data, R0_y_data;
		getPlottableSegment(arc_x0, arc_y0, cx, cy, R0_x_data, R0_y_data);
		// Radius from (arc_xf, arc_yf)
		std::vector<double> Rf_x_data, Rf_y_data;
		getPlottableSegment(arc_xf, arc_yf, cx, cy, Rf_x_data, Rf_y_data);

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
	    plt::show();

	    // Save png
		//std::string this_file_path = __FILE__;
		//std::string this_file_name = "collisionDetectionModule.cpp";
		//int upper_bound = this_file_path.length() - this_file_name.length();
		//std::string png_name = this_file_path.substr(0, upper_bound) + "Center_via_cords.png";
	    //plt::save(png_name);
	#endif

	return std::make_tuple(cx, cy, r);
}



//**********************************************************************
// PUBLIC FUNCTIONS FOR COLLISION DETECTION
//**********************************************************************
/*!
* Determine if there is an intersection between two segments. 
* If there is an intersection it returns also the coords.
* @param[in]  x1 L1_x0 				First segment inital point x-coord.
* @param[in]  y1 L1_y0				First segment inital point y-coord.
* @param[in]  x2 L1_xf				First segment final point x-coord.
* @param[in]  y2 L1_yf				First segment final point y-coord.
* @param[in]  x3 L2_x0				Second segment inital point x-coord.
* @param[in]  y3 L2_y0				Second segment inital point y-coord.
* @param[in]  x4 L2_xf				Second segment final point x-coord.
* @param[in]  y4 L2_yf				Second segment final point y-coord.
* @param[out] <flag_i, x, y> 	Tuple with bool true if intersection exists, intersection (x-coord, y-coord).
*/
std::tuple<bool, double, double> intersLineLine(double x1, double y1, double x2, double y2, 
	double x3, double y3, double x4, double y4){
	// P1 = (x1, y1), P2 = (x2, x3), P3 = (x3, y3), P4 = (x4, y4)
	// L1 pass through P1, P2
	// L2 pass through P3, P4
	double L1_min_x = std::fmin(x1, x2);
	double L1_min_y = std::fmin(y1, y2);
	double L1_max_x = std::fmax(x1, x2);
	double L1_max_y = std::fmax(y1, y2);

	double L2_min_x = std::fmin(x3, x4);
	double L2_min_y = std::fmin(y3, y4);
	double L2_max_x = std::fmax(x3, x4);
	double L2_max_y = std::fmax(y3, y4);

	// Position check
	if (L2_max_x < L2_min_x || 	// L2 completely left of L1
		L2_min_x > L1_max_x || 	// L2 completely right of L1
		L2_max_y < L1_min_y || 	// L2 completely below L1
		L2_min_y > L1_max_y) 	// L2 completely above L1
		return std::make_tuple(false, std::nan("1"), std::nan("1")); // segments do not intersect

	// Line eq --> y = m * x + q
	// Angular coefficients - m = delta_y / delta_x
	double L1_m = (y1 - y2) / (x1 - x2);
	double L2_m = (y3 - y4) / (x3 - x4);

	// Intercepts - q = y - m * x
	double L1_q = y1 - L1_m * x1;
	double L2_q = y3 - L2_m * x3;

	// Intersection pt
	double x, y;

	if (L1_m == L2_m){
		x = std::nan("1");
		y = std::nan("1");

		if (L1_q != L2_q) return std::make_tuple(false, std::nan("1"), std::nan("1")); // parallel different lines
		else { 								// same line
			bool P1_on_L2 = (x1 >= L2_min_x && x1 <= L2_max_x && y1 >= L2_min_y && y1 <= L2_max_y); // P1 on L2
			bool P2_on_L2 = (x2 >= L2_min_x && x2 <= L2_max_x && y2 >= L2_min_y && y2 <= L2_max_y); // P2 on L2
			
			if (P1_on_L2 || P2_on_L2) 
				return std::make_tuple(true, std::nan("1"), std::nan("1")); // infinite intersection points
			else 
				return std::make_tuple(false, std::nan("1"), std::nan("1")); // segments on the same line but no intersection
		}
	}

	// Resolve L1 = L2 and find intersection point (x, y)
	x = (L2_q - L1_q) / (L1_m - L2_m); 	// x that resolve L1 = L2
	y = L1_m * x + L1_q; 				// equivalent to y = L2_m * x + L2_q

	// Find out if the intersection point (x,y) belong to L1 and L2
	bool pt_on_L1 = (x >= L1_min_x && x <= L1_max_x && y >= L1_min_y && y <= L1_max_y); // (x,y) are on L1
	bool pt_on_L2 = (x >= L2_min_x && x <= L2_max_x && y >= L2_min_y && y <= L2_max_y); // (x,y) are on L2

	if (pt_on_L1 && pt_on_L2) 
		return std::make_tuple(true, x, y);
	else 
		return std::make_tuple(false, std::nan("1"), std::nan("1"));
}

/*!
* Determine if there is an intersection between a circle and a segment. If there is an 
* intersection it returns also the coords.
* @param[in]  cx 											Circle center x-coord.
* @param[in]  cy 											Circle center y-coord.
* @param[in]  r 											Circle radius.
* @param[in]  L_x0											Segment inital point x-coord.
* @param[in]  L_y0 											Segment inital point y-coord.
* @param[in]  L_xf 											Segment final point x-coord.
* @param[in]  L_yf 											Segment final point y-coord.
* @param[out] <flag, i_x0, i_y0, i_th0, i_x1, i_y1, i_th1> 	Tuple with bool true if intersection exists, 1st intersection (x-coord, y-coord), 1st intersection angle w.r.t. center, 2nd intersection (x-coord, y-coord), 2nd intersection angle w.r.t. center.
*/
std::tuple<bool, double, double, double, double, double, double> intersCircleLine(double cx, double cy, double r, 
	double L_x0, double L_y0, double L_xf, double L_yf){	
	// Compute the various polynoms (as in the provided slides / matlab code)
	double p1 = 2 * L_x0 * L_xf;
    double p2 = 2 * L_y0 * L_yf;
    double p3 = 2 * cx * L_x0;
    double p4 = 2 * cx * L_xf;
    double p5 = 2 * cy * L_y0;
    double p6 = 2 * cy * L_yf;

    double c1 = std::pow(L_x0, 2) + std::pow(L_xf, 2) - p1 + std::pow(L_y0, 2) + std::pow(L_yf, 2) - p2;
    double c2 = - 2 * std::pow(L_xf, 2) + p1 - p3 + p4 - 2 * std::pow(L_yf, 2) + p2 - p5 + p6;
    double c3 = std::pow(L_xf, 2) - p4 + std::pow(cx, 2) + std::pow(L_yf, 2) - p6 + std::pow(cy, 2) - std::pow(r, 2);

    double delta = std::pow(c2, 2) - 4 * c1 * c3;    

    if (delta < 0) // no itersections at all
    	return std::make_tuple(false, std::nan("1"), std::nan("1"), std::nan("1"), 
    		std::nan("1"), std::nan("1"), std::nan("1"));

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
    	i_x0 = L_x0 * t1 + L_xf * (1 - t1);
    	i_y0 = L_y0 * t1 + L_yf * (1 - t1);
    	i_th0 = std::atan2((i_y0 - cy), (i_x0 - cx));
    }

    if (t2 >= 0 && t2 <= 1 && t2 != t1){
    	i_x1 = L_x0 * t2 + L_xf * (1 - t2);
    	i_y1 = L_y0 * t2 + L_yf * (1 - t2);
    	i_th1 = std::atan2((i_y1 - cy), (i_x1 - cx));
    }

    /*// Check if the intersection pts belongs to the line // HERE
    bool i_flag1 = false;
    bool i_flag2 = false;

    if (!(std::isnan(i_x0) && std::isnan(i_y0)))
    	i_flag1 = isPtOnSegment(i_x0, i_y0, L_x0, L_y0, L_xf, L_yf);

    if (!(std::isnan(i_x1) && std::isnan(i_y1)))
    	i_flag2 = isPtOnSegment(i_x1, i_y1, L_x0, L_y0, L_xf, L_yf);

    return std::make_tuple((i_flag1 || i_flag2), i_x0, i_y0, i_th0, i_x1, i_y1, i_th1);*/

	// Check whether the segment is internal to the circle
	if (std::isnan(i_x0) && std::isnan(i_x1))
		return std::make_tuple(false, i_x0, i_y0, i_th0, i_x1, i_y1, i_th1);

    return std::make_tuple(true, i_x0, i_y0, i_th0, i_x1, i_y1, i_th1);
}

/*!
* Determine if there is an intersection between an arc of a circle and a segment. 
* If there is an intersection it returns also the coords.
* @param[in]  arc_x0 										Arc initial x-coord.
* @param[in]  arc_y0 										Arc initial y-coord.
* @param[in]  arc_th0 										Arc initial theta (w.r.t. robot).
* @param[in]  arc_xf 										Arc final x-coord.
* @param[in]  arc_yf 										Arc final y-coord.
* @param[in]  arc_L 										Arc lenght.
* @param[in]  arc_k 										Arc max curvature.
* @param[in]  L_x0 											Segment inital point x-coord.
* @param[in]  L_y0 											Segment inital point y-coord.
* @param[in]  L_xf 											Segment final point x-coord.
* @param[in]  L_yf 											Segment final point y-coord.
* @param[out] <flag, i_x0, i_y0, i_th0, i_x1, i_y1, i_th1> 	Tuple with bool true if intersection exists, 1st intersection (x-coord, y-coord), 1st intersection angle w.r.t. center, 2nd intersection (x-coord, y-coord), 2nd intersection angle w.r.t. center.
* @param[out] <flag, i_x0, i_y0, i_x1, i_y1> 	Tuple with bool true if intersection exists, 1st intersection (x-coord, y-coord), 2nd intersection (x-coord, y-coord).
*/
std::tuple<bool, double, double, double, double> intersArcLine(double arc_x0, double arc_y0, double arc_th0, 
	double arc_xf, double arc_yf, double arc_L, double arc_k,
	double L_x0, double L_y0, double L_xf, double L_yf){
	// Get the circle from the arc
	double cx, cy, r;
	std::tie(cx, cy, r) = getCricleFromArc(arc_x0, arc_y0, arc_th0, arc_xf, arc_yf, arc_L, arc_k);

	// Get intersections between circle and line if exist
	bool flag_i;
	double i_x0, i_y0, i_th0;
	double i_x1, i_y1, i_th1;
	std::tie(flag_i, i_x0, i_y0, i_th0, i_x1, i_y1, i_th1) = intersCircleLine(cx, cy, r, L_x0, L_y0, L_xf, L_yf);

	if (flag_i){
		// Get the angle of the extreme pts of the arc w.r.t. the circle center
		double c_arc_th0 = std::atan2((arc_y0 - cy), (arc_x0 - cx));
		double c_arc_thf = std::atan2((arc_yf - cy), (arc_xf - cx));
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
			plotAL(cx, cy, r, arc_L, arc_x0, arc_y0, arc_th0, arc_k, L_x0, L_y0, L_xf, L_yf, 
				i_x0, i_y0, i_x1, i_y1);
		#endif

		//return std::make_tuple(flag_a_i, i_x0, i_y0, i_th0, i_x1, i_y1, i_th1);
		return std::make_tuple(flag_a_i, i_x0, i_y0, i_x1, i_y1);
	}

	#ifdef DEBUG_IAL
		plotAL(cx, cy, r, arc_L, arc_x0, arc_y0, arc_th0, arc_k, L_x0, L_y0, L_xf, L_yf, 
			i_x0, i_y0, i_x1, i_y1);
	#endif

	// All previous checks have failed --> no intersection
	return std::make_tuple(false, std::nan("1"), std::nan("1"), std::nan("1"), std::nan("1")); // , std::nan("1"), std::nan("1")
}

//bool intersPtPolygon(double pt_x, double pt_y, Polygon p){}



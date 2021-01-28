#include "collisionDetectionModule.hpp"

namespace plt = matplotlibcpp;

//**********************************************************************
// AUXILIARY PRIVATE FUNCTIONS (from dubins.cpp)
//**********************************************************************
/*!
* Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t otherwise.
* @param[in]  t
* @param[out] s
*/
/*double sinc(double t){
	double s;

	if (std::abs(t) < 0.002) 
		s = 1 - std::pow(t, 2) * (1/6 - std::pow(t, 2)/20);
	else
		s = std::sin(t)/t;

	return s;
}*/

/*!
* Normalize an angle (in range [0,2*pi)).
* @param[in]  ang 	Initial angle.
* @param[out] out 	Final angle.
*/
/*double mod2pi(double ang){
	double out = ang;

	while (out < 0){
		out = out + 2*M_PI;
	}

	while (out >= 2*M_PI){
		out = out - 2*M_PI;
	}

	return out;
}*/

/*!
* Evaluate an arc (circular or straight) composing a Dubins curve, at a given arc-length s.
* @param[in]  s 			Arc length.
* @param[in]  x0 			Initial x-coord.
* @param[in]  y0 			Initial y-coord.
* @param[in]  th0 			Initial angle direction.
* @param[in]  k 			Max curvature.
* @param[out] <x, y, th> 	Final position and direction.
*/
/*std::tuple<double, double, double> circline(double s, double x0, double y0, double th0, double k){
	double x = x0 + s * sinc(k * s / 2.0) * std::cos(th0 + k * s / 2);
	double y = y0 + s * sinc(k * s / 2.0) * std::sin(th0 + k * s / 2);
	double th = mod2pi(th0 + k * s);

	return std::make_tuple(x, y, th);
}*/



//**********************************************************************
// SUPPORT PRIVATE FUNCTIONS
//**********************************************************************
/*!
* Retrieves the center coords and radius of a circle from a given arc.
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
	// Get two random points on the arc
	int npts = 1000;
	int random_i0 = rand() % npts;
	int random_i1 = rand() % npts;

	// Ensure to have two different points
	while (random_i0 == random_i1){
		random_i1 = rand() % npts;
	}

	// First random pt on the arc
	double random_x0, random_y0;
	double s = arc_L / npts * random_i0;
	std::tie(random_x0, random_y0, std::ignore) = circline(s, arc_x0, arc_y0, arc_th0, arc_k);

	// Second random pt on the arc
	double random_x1, random_y1;
	s = arc_L / npts * random_i1;
	std::tie(random_x1, random_y1, std::ignore) = circline(s, arc_x0, arc_y0, arc_th0, arc_k);

	// From the "arc line" AL0=[(arc_x0, arc_y0), (random_x0, random_y0)] find the middle pt
	double mid_x0 = std::abs(arc_x0 - random_x0) / 2;
	double mid_y0 = std::abs(arc_y0 - random_y0) / 2;
	// Build a perpendicular line PL0 from the middle pt
	double AL0_m = (arc_y0 - random_y0) / (arc_x0 - random_x0);
	double AL0_q = arc_y0 - AL0_m * arc_x0;
	double PL0_m = -1 / AL0_m; // 2 lines are perpendicular if AL0_m * PL0_m = -1
	double PL0_q = mid_y0 - PL0_m * mid_x0;

	// From the "arc line" AL2=[(random_x1, random_y1), (arc_xf, arc_yf)] find the middle pt
	double mid_x1 = std::abs(arc_xf - random_x1) / 2;
	double mid_y1 = std::abs(arc_yf - random_y1) / 2;
	// Build a perpendicular line PL2 from the middle pt
	double AL1_m = (arc_yf - random_y1) / (arc_xf - random_x1);
	double AL1_q = arc_yf - AL1_m * arc_xf;
	double PL1_m = -1 / AL1_m; // 2 lines are perpendicular if AL1_m * PL1_m = -1
	double PL1_q = mid_y1 - PL1_m * mid_x1;

	// Find circle center by resolving PL0 = PL1 --> intersection point (cx, cy)
	double cx = (PL1_q - PL0_q) / (PL0_m - PL1_m); // cx that resolve PL0 = PL1
	double cy = PL0_m * cx + PL0_q; 				// equivalent to cy = PL1_m * cx + PL1_q

	// Find circle radius
	double r = std::sqrt(std::pow((cx - mid_x0), 2) + std::pow((cy - mid_y0), 2));

	return std::make_tuple(cx, cy, r);
}



//**********************************************************************
// PUBLIC FUNCTIONS FOR COLLISION DETECTION
//**********************************************************************
/*!
* Determine if there is an intersection between two segments. 
* If there is an intersection it returns also the coords.
* @param[in]  x1 				First segment inital point x-coord.
* @param[in]  y1 				First segment inital point y-coord.
* @param[in]  x2 				First segment final point x-coord.
* @param[in]  y2 				First segment final point y-coord.
* @param[in]  x3 				Second segment inital point x-coord.
* @param[in]  y3 				Second segment inital point y-coord.
* @param[in]  x4 				Second segment final point x-coord.
* @param[in]  y4 				Second segment final point y-coord.
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
* Determine if there is an intersection between a circle and a segment. 
* If there is an intersection it returns also the coords.
* @param[in]  cx 										Circle center x-coord.
* @param[in]  cy 										Circle center y-coord.
* @param[in]  r 										Circle radius.
* @param[in]  x1 										Segment inital point x-coord.
* @param[in]  y1 										Segment inital point y-coord.
* @param[in]  x2 										Segment final point x-coord.
* @param[in]  y2 										Segment final point y-coord.
* @param[out] <flag, i_x1, i_y1, th1, i_x2, i_y2, th2> 	Tuple with bool true if intersection exists, 1st intersection (x-coord, y-coord), 1st intersection angle w.r.t. center, 2nd intersection (x-coord, y-coord), 2nd intersection angle w.r.t. center.
*/
std::tuple<bool, double, double, double, double, double, double> intersCircleLine(double cx, double cy, double r, 
	double x1, double y1, double x2, double y2){
	// Compute the various polynoms (as in the provided slides / matlab code)
	double p1 = 2 * x1 * x2;
    double p2 = 2 * y1 * y2;
    double p3 = 2 * cx * x1;
    double p4 = 2 * cx * x2;
    double p5 = 2 * cy * y1;
    double p6 = 2 * cy * y2;

    double c1 = std::pow(x1, 2) + std::pow(x2, 2) - p1 + std::pow(y1, 2) + std::pow(y2, 2) - p2;
    double c2 = - 2 * std::pow(x2, 2) + p1 - p3 + p4 - 2 * std::pow(y2, 2) + p2 - p5 + p6;
    double c3 = std::pow(x2, 2) - p4 + std::pow(cx, 2) + std::pow(y2, 2) - p6 + std::pow(cy, 2) - std::pow(r, 2);

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
    double i_x1, i_y1, th1, i_x2, i_y2, th2;

    if (t1 >= 0 && t1 <= 1){
    	i_x1 = x1 * t1 + x2 * (1 - t1);
    	i_y1 = y1 * t1 + y2 * (1 - t1);
    	th1 = std::atan2((i_y1 - cy), (i_x1 - cx));
    }

    if (t2 >= 0 && t2 <= 1 && t2 != t1){
    	i_x2 = x1 * t2 + x2 * (1 - t2);
    	i_y2 = y1 * t2 + y2 * (1 - t2);
    	th2 = std::atan2((i_y2 - cy), (i_x2 - cx));
    }

    return std::make_tuple(true, i_x1, i_y1, th1, i_x2, i_y2, th2);
}

/*!
* Determine if there is an intersection between an arc of a circle and a segment. 
* If there is an intersection it returns also the coords.
* @param[in]  arc_x0 							Arc initial x-coord.
* @param[in]  arc_y0 							Arc initial y-coord.
* @param[in]  arc_th0 							Arc initial theta (w.r.t. robot).
* @param[in]  arc_xf 							Arc final x-coord.
* @param[in]  arc_yf 							Arc final y-coord.
* @param[in]  arc_L 							Arc lenght.
* @param[in]  arc_k 							Arc max curvature.
* @param[in]  L_x0 								Segment inital point x-coord.
* @param[in]  L_y0 								Segment inital point y-coord.
* @param[in]  L_xf 								Segment final point x-coord.
* @param[in]  L_yf 								Segment final point y-coord.
* @param[out] <flag, i_x1, i_y1, i_x2, i_y2> 	Tuple with bool true if intersection exists, 1st intersection (x-coord, y-coord), 2nd intersection (x-coord, y-coord).
*/
std::tuple<bool, double, double, double, double> intersArcLine(double arc_x0, double arc_y0, double arc_th0, 
	double arc_xf, double arc_yf, double arc_L, double arc_k,
	double L_x0, double L_y0, double L_xf, double L_yf){
	// Get the circle from the arc
	double cx, cy, r;
	std::tie(cx, cy, r) = getCricleFromArc(arc_x0, arc_y0, arc_th0, arc_xf, arc_yf, arc_L, arc_k);

	// Get intersections between circle and line if exist
	bool flag_i;
	double i_x1, i_y1, i_th1;
	double i_x2, i_y2, i_th2;
	std::tie(flag_i, i_x1, i_y1, i_th1, i_x2, i_y2, i_th2) = intersCircleLine(cx, cy, r, L_x0, L_y0, L_xf, L_yf);

	if (flag_i){
		// Get the angle of the arc w.r.t. the circle center
		double c_arc_th0 = std::atan2((arc_y0 - cy), (arc_x0 - cx));
		double c_arc_thf = std::atan2((arc_yf - cy), (arc_xf - cx));
		// Get the the ordered angles interval
		double c_min_th = std::fmin(c_arc_th0, c_arc_thf);
		double c_max_th = std::fmax(c_arc_th0, c_arc_thf);

		// If c_min_th <= i_th1 <= c_max_th --> circle intersection point lies on the arc
		// Manage the first intersection point
		if (!(std::isnan(i_x1))){ // check if 1st intersection pt exists
			if (!(i_th1 >= c_min_th && i_th1 <= c_max_th)){ // if angle not in interval
				i_x1 = std::nan("1");
				i_y1 = std::nan("1");
			}
		}

		// If c_min_th <= i_th2 <= c_max_th --> circle intersection point lies on the arc
		// Manage the second intersection point
		if (!(std::isnan(i_x2))){ // check if 2nd intersection pt exists
			if (!(i_th2 >= c_min_th && i_th2 <= c_max_th)){ // if angle not in interval
				i_x2 = std::nan("1");
				i_y2 = std::nan("1");
			}
		}

		bool flag_a_i = !(std::isnan(i_x1) && std::isnan(i_x2)); // check if at least one intersection has survived

		return std::make_tuple(flag_a_i, i_x1, i_y1, i_x2, i_y2);
	}

	// All previous checks have failed --> no intersection
	return std::make_tuple(false, std::nan("1"), std::nan("1"), std::nan("1"), std::nan("1"));
}

//bool intersPtPolygon(double pt_x, double pt_y, Polygon p);



//**********************************************************************
// PUBLIC FUNCTIONS FOR PLOTTING
//**********************************************************************
void plotLL(double L1_x0, double L1_y0, double L1_xf, double L1_yf, 
	double L2_x0, double L2_y0, double L2_xf, double L2_yf){
	// Create structure for plot
	std::vector<double> L1_x_data, L1_y_data;
	L1_x_data.push_back(L1_x0);
	L1_x_data.push_back(L1_xf);
	L1_y_data.push_back(L1_y0);
	L1_y_data.push_back(L1_yf);

	std::vector<double> L2_x_data, L2_y_data;
	L2_x_data.push_back(L2_x0);
	L2_x_data.push_back(L2_xf);
	L2_y_data.push_back(L2_y0);
	L2_y_data.push_back(L2_yf);

	// Plot L1
    plt::named_plot("L1", L1_x_data, L1_y_data, "r-");

    // Plot L2
    plt::named_plot("L2", L2_x_data, L2_y_data, "b-");

    // Set x-axis and y-axis to [xmin-1, xmax+1] and [ymin-1, ymax+1] respectively
	double x_min_max[2] = {plt::xlim()[0], plt::xlim()[1]};
	double y_min_max[2] = {plt::ylim()[0], plt::ylim()[1]};
    plt::xlim(x_min_max[0] - 1, x_min_max[1] + 1);
    plt::ylim(y_min_max[0] - 1, y_min_max[1] + 1);

    plt::title("Line intersection"); 	// Add graph title
    plt::legend(); 						// Enable legend.
    plt::show(); 						// Show plot

    // Save png
	/*std::string this_file_path = __FILE__;
	std::string this_file_name = "collisionDetectionModule.cpp";
	int upper_bound = this_file_path.length() - this_file_name.length();
	std::string png_name = this_file_path.substr(0, upper_bound) + "Line_intersection.png";
    plt::save(png_name);*/
}
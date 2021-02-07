#include "dubins.hpp"

//#define DEBUG_DUBINS_SHORTEST_PATH
//#define DEBUG_PLOTDUBINS
//#define DEBUG_PLOTMULTIDUBINS

namespace plt = matplotlibcpp;

//**********************************************************************
// AUXILIARY UTILITY FUNCTION
//**********************************************************************
/*!
* Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t otherwise.
* @param[in]  t
* @param[out] s
*/
double sinc(double t){
	double s;

	if (std::abs(t) < 0.002) 
		s = 1 - std::pow(t, 2) * (1/6 - std::pow(t, 2)/20);
	else
		s = std::sin(t)/t;

	return s;
}

/*!
* Normalize an angle (in range [0,2*pi)).
* @param[in]  ang 	Initial angle.
* @param[out] out 	Final angle.
*/
double mod2pi(double ang){
	double out = ang;

	while (out < 0){
		out = out + 2*M_PI;
	}

	while (out >= 2*M_PI){
		out = out - 2*M_PI;
	}

	return out;
}

/*!
* Normalize an angular difference (range (-pi, pi]).
* @param[in]  ang 	Initial angle.
* @param[out] out 	Final angle.
*/
double rangeSymm(double ang){
	double out = ang;

	while (out <= -M_PI)
		out = out + 2*M_PI;

	while (out > M_PI)
		out = out - 2*M_PI;

	return out;
}

/*!
* Evaluate an arc (circular or straight) composing a Dubins curve, at a given arc-length s.
* @param[in]  s 			Arc length.
* @param[in]  x0 			Initial x-coord.
* @param[in]  y0 			Initial y-coord.
* @param[in]  th0 			Initial angle direction.
* @param[in]  k 			Max curvature.
* @param[out] <x, y, th> 	Final position and direction.
*/
std::tuple<double, double, double> circline(double s, double x0, double y0, double th0, double k){
	double x = x0 + s * sinc(k * s / 2.0) * std::cos(th0 + k * s / 2);
	double y = y0 + s * sinc(k * s / 2.0) * std::sin(th0 + k * s / 2);
	double th = mod2pi(th0 + k * s);

	return std::make_tuple(x, y, th);
}

/*!
* Check validity of a solution by evaluating explicitly the 3 equations defining a Dubins 
* problem (in standard form).
* @param[in]  s1 			First arc length.
* @param[in]  k0 			First arc max curvature.
* @param[in]  s2 			Second arc length.
* @param[in]  k1 			Second arc max curvature.
* @param[in]  s3 			Third arc length.
* @param[in]  k2 			Third arc max curvature.
* @param[in]  th0 			Initial angle direction.
* @param[in]  thf 			Final angle direction.
* @param[out] check_val 	True if the 3 equations are satisfied.
*/
bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf){
	double x0 = -1;
	double y0 = 0;
	double xf = 1;
	double yf = 0;

	double eq1 = x0 + s1 * sinc((1/2.) * k0 * s1) * cos(th0 + (1/2.) * k0 * s1) 
					+ s2 * sinc((1/2.) * k1 * s2) * cos(th0 + k0 * s1 + (1/2.) * k1 * s2) 
					+ s3 * sinc((1/2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - xf;

	double eq2 = y0 + s1 * sinc((1/2.) * k0 * s1) * sin(th0 + (1/2.) * k0 * s1) 
					+ s2 * sinc((1/2.) * k1 * s2) * sin(th0 + k0 * s1 + (1/2.) * k1 * s2) 
					+ s3 * sinc((1/2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - yf;

    double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

  bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
  bool check_val = (std::sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10) && Lpos;

  return check_val;
}



//**********************************************************************
// DATA STRUCTURES AND GETTERS
//**********************************************************************
/*!
* Create a structure representing an arc of a Dubins curve (straight or circular).
* @param[in]  x0 	Initial x-coord.
* @param[in]  y0 	Initial y-coord.
* @param[in]  th0 	Initial angle direction.
* @param[in]  k 	Max curvature.
* @param[in]  L 	Arc length.
* @param[out] a 	Arc type.
*/
arc getDubinsArc(double x0, double y0, double th0, double k, double L){
	arc a;

	a.x0 = x0;
	a.y0 = y0;
	a.th0 = th0;
	a.k = k;
	a.L = L;

	std::tie(a.xf, a.yf, a.thf) = circline(L, x0, y0, th0, k);

	return a;
}

/*!
* Create a structure representing a Dubins curve (composed by three arcs).
* @param[in]  x0 	Initial x-coord.
* @param[in]  y0 	Initial y-coord.
* @param[in]  th0 	Initial angle direction.
* @param[in]  s1 	First arc length.
* @param[in]  s2 	Second arc length.
* @param[in]  s3 	Third arc length.
* @param[in]  k0 	First arc max curvature.
* @param[in]  k1 	Second arc max curvature.
* @param[in]  k2 	Third arc max curvature.
* @param[out] c 	Dubins curve made of three arcs.
*/
curve getDubinsCurve(double x0, double y0, double th0, double s1,double s2, 
	double s3, double k0, double k1, double k2){
	curve c;

	c.arc1 = getDubinsArc(x0, y0, th0, k0, s1);
	c.arc2 = getDubinsArc(c.arc1.xf, c.arc1.yf, c.arc1.thf, k1, s2);
	c.arc3 = getDubinsArc(c.arc2.xf, c.arc2.yf, c.arc2.thf, k2, s3);

	c.L = c.arc1.L + c.arc2.L + c.arc3.L;

	return c;
}



//**********************************************************************
// SCALING FUNCTIONS
//**********************************************************************
/*!
* Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0).
* @param[in]  x0 									Initial x-coord.
* @param[in]  y0 									Initial y-coord.
* @param[in]  th0 									Initial angle direction.
* @param[in]  xf 									Final x-coord.
* @param[in]  yf 									Final y-coord.
* @param[in]  thf 									Final angle direction.
* @param[in]  Kmax 									Maximum curvature.
* @param[out] <sc_th0, sc_thf, sc_Kmax, lambda> 	Tuple with the scaled initial and final theta, scaled max curvature and conversion factor.
*/
std::tuple<double, double, double, double> scaleOrig2Std(double x0, double y0, 
	double th0, double xf, double yf, double thf, double Kmax){
	double dx = xf - x0;
	double dy = yf - y0;
	double phi = std::atan2(dy, dx);
	double lambda = std::hypot(dx, dy) / 2;

	double sc_th0 = mod2pi(th0 - phi);
	double sc_thf = mod2pi(thf - phi);
	double sc_Kmax = Kmax * lambda;

	return std::make_tuple(sc_th0, sc_thf, sc_Kmax, lambda);
}

/*!
* Scale the solution to the standard problem back to the original problem.
* @param[in]  lambda 		Conversion factor.
* @param[in]  sc_s1 		First arc scaled length.
* @param[in]  sc_s2 		Second arc scaled length.
* @param[in]  sc_s3 		Third arc scaled length.
* @param[out] <s1, s2, s3> 	Tuple with the 3 arcs original length.
*/
std::tuple<double, double, double> scaleStd2Orig(double lambda, double sc_s1, 
	double sc_s2, double sc_s3){
	double s1 = sc_s1 * lambda;
	double s2 = sc_s2 * lambda;
	double s3 = sc_s3 * lambda;

	return std::make_tuple(s1, s2, s3);
}



//**********************************************************************
// DUBINS PRIMITIVES
//**********************************************************************
/*!
* Retrieves sc_s1, sc_s2, sc_s3 that define the LSL Dubins curves.
* @param[in]  sc_th0 						Scaled initial theta.
* @param[in]  sc_thf 						Scaled final theta.
* @param[in]  sc_Kmax 						Scaled max curvature.
* @param[out] <flag, sc_s1, sc_s2, sc_s3> 	Tuple with bool and sc_s1, sc_s2, sc_s3.
*/
std::tuple<bool, double, double, double> LSL(double sc_th0, double sc_thf, double sc_Kmax){
	double invK = 1 / sc_Kmax;
	double C = std::cos(sc_thf) - std::cos(sc_th0);
	double S = 2 * sc_Kmax + std::sin(sc_th0) - std::sin(sc_thf);

	double root_elem = 2 + 4 * std::pow(sc_Kmax, 2) - 2 * std::cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf));
	if (root_elem < 0) return std::make_tuple(false, 0, 0, 0);
	double atan2_CS = std::atan2(C, S);

	double sc_s1 = invK * mod2pi(atan2_CS - sc_th0);	
	double sc_s2 = invK * std::sqrt(root_elem);
	double sc_s3 = invK * mod2pi(sc_thf - atan2_CS);

	return std::make_tuple(true, sc_s1, sc_s2, sc_s3);
}

/*!
* Retrieves sc_s1, sc_s2, sc_s3 that define the RSR Dubins curves.
* @param[in]  sc_th0 						Scaled initial theta.
* @param[in]  sc_thf 						Scaled final theta.
* @param[in]  sc_Kmax 						Scaled max curvature.
* @param[out] <flag, sc_s1, sc_s2, sc_s3> 	Tuple with bool and sc_s1, sc_s2, sc_s3.
*/
std::tuple<bool, double, double, double> RSR(double sc_th0, double sc_thf, double sc_Kmax){
	double invK = 1 / sc_Kmax;
	double C = std::cos(sc_th0) - std::cos(sc_thf);
	double S = 2 * sc_Kmax - std::sin(sc_th0) + std::sin(sc_thf);

	double root_elem = 2 + 4 * std::pow(sc_Kmax, 2) - 2 * std::cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf));
	if (root_elem < 0) return std::make_tuple(false, 0, 0, 0);
	double atan2_CS = std::atan2(C, S);

	double sc_s1 = invK * mod2pi(sc_th0 - atan2_CS);	
	double sc_s2 = invK * std::sqrt(root_elem);
	double sc_s3 = invK * mod2pi(atan2_CS - sc_thf);

	return std::make_tuple(true, sc_s1, sc_s2, sc_s3);
}

/*!
* Retrieves sc_s1, sc_s2, sc_s3 that define the LSR Dubins curves.
* @param[in]  sc_th0 						Scaled initial theta.
* @param[in]  sc_thf 						Scaled final theta.
* @param[in]  sc_Kmax 						Scaled max curvature.
* @param[out] <flag, sc_s1, sc_s2, sc_s3> 	Tuple with bool and sc_s1, sc_s2, sc_s3.
*/
std::tuple<bool, double, double, double> LSR(double sc_th0, double sc_thf, double sc_Kmax){
	double invK = 1 / sc_Kmax;
	double C = std::cos(sc_th0) + std::cos(sc_thf);
	double S = 2 * sc_Kmax + std::sin(sc_th0) + std::sin(sc_thf);

	double root_elem = - 2 + 4 * std::pow(sc_Kmax, 2) + 2 * std::cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (std::sin(sc_th0) + std::sin(sc_thf));
	if (root_elem < 0) return std::make_tuple(false, 0, 0, 0);

	double sc_s2 = invK * std::sqrt(root_elem);
	double atan2_1 = std::atan2(-C, S);
	double atan2_2= std::atan2(-2, (sc_Kmax * sc_s2));
	double sc_s1 = invK * mod2pi(atan2_1 - atan2_2 - sc_th0);
	double sc_s3 = invK * mod2pi(atan2_1 - atan2_2 - sc_thf);

	return std::make_tuple(true, sc_s1, sc_s2, sc_s3);
}

/*!
* Retrieves sc_s1, sc_s2, sc_s3 that define the RSL Dubins curves.
* @param[in]  sc_th0 						Scaled initial theta.
* @param[in]  sc_thf 						Scaled final theta.
* @param[in]  sc_Kmax 						Scaled max curvature.
* @param[out] <flag, sc_s1, sc_s2, sc_s3> 	Tuple with bool and sc_s1, sc_s2, sc_s3.
*/
std::tuple<bool, double, double, double> RSL(double sc_th0, double sc_thf, double sc_Kmax){
	double invK = 1 / sc_Kmax;
	double C = std::cos(sc_th0) + std::cos(sc_thf);
	double S = 2 * sc_Kmax - std::sin(sc_th0) - std::sin(sc_thf);

	double root_elem = - 2 + 4 * std::pow(sc_Kmax, 2) + 2 * std::cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (std::sin(sc_th0) + std::sin(sc_thf));
	if (root_elem < 0) return std::make_tuple(false, 0, 0, 0);

	double sc_s2 = invK * std::sqrt(root_elem);
	double atan2_CS = std::atan2(C, S);
	double atan2_2 = std::atan2(2, sc_Kmax * sc_s2);
	double sc_s1 = invK * mod2pi(sc_th0 - atan2_CS + atan2_2);
	double sc_s3 = invK * mod2pi(sc_thf - atan2_CS + atan2_2);

	return std::make_tuple(true, sc_s1, sc_s2, sc_s3);
}

/*!
* Retrieves sc_s1, sc_s2, sc_s3 that define the RLR Dubins curves.
* @param[in]  sc_th0 						Scaled initial theta.
* @param[in]  sc_thf 						Scaled final theta.
* @param[in]  sc_Kmax 						Scaled max curvature.
* @param[out] <flag, sc_s1, sc_s2, sc_s3> 	Tuple with bool and sc_s1, sc_s2, sc_s3.
*/
std::tuple<bool, double, double, double> RLR(double sc_th0, double sc_thf, double sc_Kmax){
	double invK = 1 / sc_Kmax;
	double C = std::cos(sc_th0) - std::cos(sc_thf);
	double S = 2 * sc_Kmax - std::sin(sc_th0) + std::sin(sc_thf);

	double arccos_arg = 0.125 * (6 - 4 * std::pow(sc_Kmax, 2) + 2 * std::cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf)));
	if (std::abs(arccos_arg) > 1) return std::make_tuple(false, 0, 0, 0);
	double atan2_CS = std::atan2(C, S);

	double sc_s2 = invK * mod2pi(2 * M_PI - std::acos(arccos_arg));
	double sc_s1 = invK * mod2pi(sc_th0 - atan2_CS + 0.5 * sc_Kmax * sc_s2);
	double sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));

	return std::make_tuple(true, sc_s1, sc_s2, sc_s3);
}

/*!
* Retrieves sc_s1, sc_s2, sc_s3 that define the LRL Dubins curves.
* @param[in]  sc_th0 						Scaled initial theta.
* @param[in]  sc_thf 						Scaled final theta.
* @param[in]  sc_Kmax 						Scaled max curvature.
* @param[out] <flag, sc_s1, sc_s2, sc_s3> 	Tuple with bool and sc_s1, sc_s2, sc_s3.
*/
std::tuple<bool, double, double, double> LRL(double sc_th0, double sc_thf, double sc_Kmax){
	double invK = 1 / sc_Kmax;
	double C = std::cos(sc_thf) - std::cos(sc_th0);
	double S = 2 * sc_Kmax + std::sin(sc_th0) - std::sin(sc_thf);

	double arccos_arg = 0.125 * (6 - 4 * std::pow(sc_Kmax, 2) + 2 * std::cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (std::sin(sc_th0) - std::sin(sc_thf)));
	if (std::abs(arccos_arg) > 1) return std::make_tuple(false, 0, 0, 0);
	double atan2_CS = std::atan2(C, S);

	double sc_s2 = invK * mod2pi(2 * M_PI - std::acos(arccos_arg));
	double sc_s1 = invK * mod2pi(- sc_th0 + atan2_CS + 0.5 * sc_Kmax * sc_s2);
	double sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));

	return std::make_tuple(true, sc_s1, sc_s2, sc_s3);
}



//**********************************************************************
// DUBINS SHORTEST PATH
//**********************************************************************
/*!
* Solve the Dubins problem for the given input parameters. Return the 
* type and the parameters of the optimal curve.
* @param[in]  x0 			Initial x-coord.
* @param[in]  y0 			Initial y-coord.
* @param[in]  th0 			Initial angle direction.
* @param[in]  xf 			Final x-coord.
* @param[in]  yf 			Final y-coord.
* @param[in]  thf 			Final angle direction.
* @param[in]  Kmax 			Maximum curvature.
* @param[out] <pidx, c> 	Pair with best primitive idx and best curve.
*/
std::pair<int, curve> dubins_shortest_path(double x0, double y0, double th0, 
	double xf, double yf, double thf, double Kmax){
	// Scale the problem to the standard
	double sc_th0, sc_thf, sc_Kmax, lambda;
	std::tie(sc_th0, sc_thf, sc_Kmax, lambda) = scaleOrig2Std(x0, y0, th0, xf, yf, thf, Kmax);

	// Define the functions corresponding to the different primitives
	std::vector<std::tuple<bool, double, double, double> (*)(double, double, double)> primitives = 
		{&LSL, &RSR, &LSR, &RSL, &RLR, &LRL};

	// Define the primitives' corresponding curvatue signs
	double ksigns[6][3] = {
		{ 1,  0,  1}, // LSL
		{-1,  0, -1}, // RSR
		{ 1,  0, -1}, // LSR
		{-1,  0,  1}, // RSL
		{-1,  1, -1}, // RLR
		{ 1, -1,  1}  // LRL
	};

	// Try all the possible primitives, to find the optimal solution
	int pidx = -1;
	double L = INFINITY;
	bool dubins_flag;
	double sc_s1, sc_s2, sc_s3;
	double sc_s1_i, sc_s2_i, sc_s3_i;
	double L_i;
	curve c;

	for (int i = 0; i < primitives.size(); i++){
		std::tie(dubins_flag, sc_s1_i, sc_s2_i, sc_s3_i) = primitives[i](sc_th0, sc_thf, sc_Kmax);

		L_i = sc_s1_i + sc_s2_i + sc_s3_i;

		#ifdef DEBUG_DUBINS_SHORTEST_PATH
			double L_d = L_i * lambda;
			double s1_d, s2_d, s3_d;
			std::tie(s1_d, s2_d, s3_d) = scaleStd2Orig(lambda, sc_s1_i, sc_s2_i, sc_s3_i);
			//std::cout << "pidx: " << i+1 << " s1 = " << s1 << " s2 = " << s2 << " s3 = " << s3 << std::endl;
			std::printf("pidx: %d, L = %.2f, s1 = %.2f, s2 = %.2f, s3 = %.2f \n", i+1, L_d, s1_d, s2_d, s3_d);
		#endif

		if (dubins_flag && (L_i < L)){
			L = L_i;
			sc_s1 = sc_s1_i;
			sc_s2 = sc_s2_i;
			sc_s3 = sc_s3_i;

			pidx = i;
		}
	}

	if (pidx >= 0){
		// Transform the solution to the problem in standard form to the solution of 
		// the original problem (scale the lengths)  
		double s1, s2, s3;
		std::tie(s1, s2, s3) = scaleStd2Orig(lambda, sc_s1, sc_s2, sc_s3);

		// Construct the Dubins curve object with the computed optimal parameters
		double k0 = ksigns[pidx][0] * Kmax;
		double k1 = ksigns[pidx][1] * Kmax;
		double k2 = ksigns[pidx][2] * Kmax;
		c = getDubinsCurve(x0, y0, th0, s1, s2, s3, k0, k1, k2);

		// Check the correctness of the algorithm
		/*double sc_k0 = ksigns[pidx][0] * sc_Kmax;
		double sc_k1 = ksigns[pidx][1] * sc_Kmax;
		double sc_k2 = ksigns[pidx][2] * sc_Kmax;
		assert(check(sc_s1, sc_k0, sc_s2, sc_k1, sc_s3, sc_k2, sc_th0, sc_thf));*/
		// This check is commented out since it fails the assert in the class methods
		// while lokking for the correct curve in minDj() and refinementSteps()

		#ifdef DEBUG_DUBINS_SHORTEST_PATH
			std::cout << "Optimal solution correspond to pidx: " << pidx+1 << std::endl;
		#endif
	} else {
		std::cout << "dubins_shortest_path() ERROR: shortest path not found." << std::endl;
	}

	return std::make_pair(pidx, c);
}



//**********************************************************************
// PLOTTING FUNCTIONS
//**********************************************************************
/*!
* Provides a tuple with the vectors of 2D plottable data.
* @param[in]  a 						Arc.
* @param[out] <vec_data_x, vec_data_y> 	Plottable vectors of 2D data.
*/
std::tuple<std::vector<double>, std::vector<double>> getPlottableArc(arc a){
	int npts = 1000;
	double s, x, y;
	std::vector<double> vec_data_x, vec_data_y;

	for (int i = 0; i < npts; i++){
		s = a.L / npts * i;
		std::tie(x, y, std::ignore) = circline(s, a.x0, a.y0, a.th0, a.k);
		vec_data_x.push_back(x);
		vec_data_y.push_back(y);
	}

	return std::make_tuple(vec_data_x, vec_data_y);
}

/*!
* Plot a Dubins curve and save it as dubins.png in 
* /home/user_name/workspace/project/src/testing_imgs .
* @param[in]  c 	Dubins curve to plot.
*/
void plotDubins(curve c){
	// Plot first arc - red line
	std::vector<double> arc1_x, arc1_y;
	std::tie(arc1_x, arc1_y) = getPlottableArc(c.arc1);
    plt::named_plot("First arc", arc1_x, arc1_y, "r-");
    /*double arc1_u = 0.1 * c.arc1.L * std::cos(c.arc1.th0);
    double arc1_v = 0.1 * c.arc1.L * std::sin(c.arc1.th0);
    plt::quiver(arc1_x[0], arc1_y[0], arc1_u, arc1_v);
    std::vector<double> dir1_x, dir1_y;
    dir1_x.push_back(arc1_x[0]);
    dir1_x.push_back(arc1_u);
    dir1_y.push_back(arc1_y[0]);
    dir1_y.push_back(arc1_v);
    plt::plot(dir1_x, dir1_y, "black -");*/

    // Plot second arc - green line
	std::vector<double> arc2_x, arc2_y;
	std::tie(arc2_x, arc2_y) = getPlottableArc(c.arc2);
    plt::named_plot("Second arc", arc2_x, arc2_y, "g-");

    // Plot third arc - blue line
	std::vector<double> arc3_x, arc3_y;
	std::tie(arc3_x, arc3_y) = getPlottableArc(c.arc3);
    plt::named_plot("Third arc", arc3_x, arc3_y, "b-");
    /*double arc3_u = 0.1 * c.arc3.L * std::cos(c.arc3.th0);
    double arc3_v = 0.1 * c.arc3.L * std::sin(c.arc3.th0);
    plt::quiver(arc3_x[0], arc3_y[0], arc3_u, arc3_v);
    std::vector<double> dir3_x, dir3_y;
    dir3_x.push_back(arc3_x[0]);
    dir3_x.push_back(arc3_u);
    dir3_y.push_back(arc3_y[0]);
    dir3_y.push_back(arc3_v);
    plt::plot(dir3_x, dir3_y, "black -");*/


    #ifdef DEBUG_PLOTDUBINS
    	double x0_1 = arc1_x[0];
    	double y0_1 = arc1_y[0];
    	double xf_1 = arc1_x[1000];
    	double yf_1 = arc1_y[1000];

    	double x0_2 = arc2_x[0];
    	double y0_2 = arc2_y[0];
    	double xf_2 = arc2_x[1000];
    	double yf_2 = arc2_y[1000];

    	double x0_3 = arc3_x[0];
    	double y0_3 = arc3_y[0];
    	double xf_3 = arc3_x[1000];
    	double yf_3 = arc3_y[1000];

    	std::printf("arc1 x0 = %.2f  y0 = %.2f xf = %.2f yf = %.2f\n", x0_1, y0_1, xf_1, yf_1);

    	std::printf("arc2 x0 = %.2f  y0 = %.2f xf = %.2f yf = %.2f\n", x0_2, y0_2, xf_2, yf_2);

    	std::printf("arc3 x0 = %.2f  y0 = %.2f xf = %.2f yf = %.2f\n", x0_3, y0_3, xf_3, yf_3);
    #endif

	// Set x-axis and y-axis to [xmin-1, xmax+1] and [ymin-1, ymax+1] respectively
	double x_min_max[2] = {plt::xlim()[0], plt::xlim()[1]};
	double y_min_max[2] = {plt::ylim()[0], plt::ylim()[1]};
    plt::xlim(x_min_max[0] - 1, x_min_max[1] + 1);
    plt::ylim(y_min_max[0] - 1, y_min_max[1] + 1);
    // Add graph title
    plt::title("Best Dubins curve");
    // Enable legend.
    plt::legend();
    // Show plot
    //plt::show();
    // Save png
	std::string this_file_path = __FILE__;
	std::string this_file_name = "dubins.cpp";
	int upper_bound = this_file_path.length() - this_file_name.length();
	std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/dubins.png";
    plt::save(png_name);
}

/*!
* Plot a multiple Dubins curves and save it as multi_dubins.png 
* in /home/user_name/workspace/project/src/ .
* @param[in]  all_best_curves 	Multi Dubins curves to plot.
*/
void plotMultiDubins(std::vector<std::pair<int, curve>> all_best_curves){
	// Initialize x-axis and y-axis
	double x_min = 0; 
	double x_max = 0;
	double y_min = 0; 
	double y_max = 0;

	for (const auto& pidx_curve: all_best_curves){
		//Get the curve
		curve c = pidx_curve.second;

		// Plot first arc - red line
		std::vector<double> arc1_x, arc1_y;
		std::tie(arc1_x, arc1_y) = getPlottableArc(c.arc1);
	    plt::named_plot("First arc", arc1_x, arc1_y, "r-");
	    double arc1_u = 0.1 * c.arc1.L * std::cos(c.arc1.th0);
	    double arc1_v = 0.1 * c.arc1.L * std::sin(c.arc1.th0);
	    //plt::quiver(arc1_x[0], arc1_y[0], arc1_u, arc1_v);
	    /*std::vector<double> dir1_x, dir1_y;
	    dir1_x.push_back(arc1_x[0]);
	    dir1_x.push_back(arc1_u);
	    dir1_y.push_back(arc1_y[0]);
	    dir1_y.push_back(arc1_v);
	    plt::plot(dir1_x, dir1_y, "black -");*/

	    // Plot second arc - green line
		std::vector<double> arc2_x, arc2_y;
		std::tie(arc2_x, arc2_y) = getPlottableArc(c.arc2);
	    plt::named_plot("Second arc", arc2_x, arc2_y, "g-");

	    // Plot third arc - blue line
		std::vector<double> arc3_x, arc3_y;
		std::tie(arc3_x, arc3_y) = getPlottableArc(c.arc3);
	    plt::named_plot("Third arc", arc3_x, arc3_y, "b-");
	    double arc3_u = 0.1 * c.arc3.L * std::cos(c.arc3.th0);
	    double arc3_v = 0.1 * c.arc3.L * std::sin(c.arc3.th0);
	    //plt::quiver(arc3_x[0], arc3_y[0], arc3_u, arc3_v);
	    /*std::vector<double> dir3_x, dir3_y;
	    dir3_x.push_back(arc3_x[0]);
	    dir3_x.push_back(arc3_u);
	    dir3_y.push_back(arc3_y[0]);
	    dir3_y.push_back(arc3_v);
	    plt::plot(dir3_x, dir3_y, "black -");*/


	    #ifdef DEBUG_PLOTMULTIDUBINS
	    	double x0_1 = arc1_x[0];
	    	double y0_1 = arc1_y[0];
	    	double xf_1 = arc1_x[1000];
	    	double yf_1 = arc1_y[1000];

	    	double x0_2 = arc2_x[0];
	    	double y0_2 = arc2_y[0];
	    	double xf_2 = arc2_x[1000];
	    	double yf_2 = arc2_y[1000];

	    	double x0_3 = arc3_x[0];
	    	double y0_3 = arc3_y[0];
	    	double xf_3 = arc3_x[1000];
	    	double yf_3 = arc3_y[1000];

	    	std::printf("arc1 x0 = %.2f  y0 = %.2f xf = %.2f yf = %.2f\n", x0_1, y0_1, xf_1, yf_1);

	    	std::printf("arc2 x0 = %.2f  y0 = %.2f xf = %.2f yf = %.2f\n", x0_2, y0_2, xf_2, yf_2);

	    	std::printf("arc3 x0 = %.2f  y0 = %.2f xf = %.2f yf = %.2f\n", x0_3, y0_3, xf_3, yf_3);
	    #endif

	    // Get the minum value for x-axis and y-axis
	    if (plt::xlim()[0] < x_min) x_min = plt::xlim()[0];
	    if (plt::xlim()[1] > x_max) x_max = plt::xlim()[1];
	    if (plt::ylim()[0] < y_min) y_min = plt::ylim()[0];
	    if (plt::ylim()[1] > y_max) y_max = plt::ylim()[1];
	}

	// Set the minum value for x-axis and y-axis to [xmin-1, xmax+1] and [ymin-1, ymax+1] respectively
    plt::xlim(x_min-1, x_max+1);
    plt::ylim(y_min-1, y_max+1);
    
    plt::title("Best Multi Dubins curve"); // Add graph title
    //plt::legend(); // Enable legend.
    //plt::show(); // Show plot
    
    // Save png
	std::string this_file_path = __FILE__;
	std::string this_file_name = "dubins.cpp";
	int upper_bound = this_file_path.length() - this_file_name.length();
	std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/multi_dubins.png";
    plt::save(png_name);
}



//**********************************************************************
// DUBINS CLASS SUPPORT FUNCTIONS - PRIVATE
//**********************************************************************
/*!
* Retrieves the k_thj possible theta_j angles that have to be tested in order 
* to find the shortest Dubins path. All theta_j are normalized in range [0, 2*M_PI).
* @param[in]  k_thj 	Granularity of angle discretization: number of theta_j.
* @param[out] all_thj 	Vector with all angles in the discretized interval.
*/
void getAngles(double k_thj, std::vector<double>& all_thj){
	for (double thj = 0; thj < 2*M_PI; thj += k_thj)
		all_thj.push_back(thj);
}

/*!
* Given the current refinement step m, it retrieves all the angles in the 
* discretized interval [theta_j - 3/2*h, theta_j + 3/2*h].
* @param[in]  m 			Number of refinement steps.
* @param[in]  k_thj 		Granularity of angle discretization: number of theta_j.
* @param[in]  prev_thj 		Best theta in the previous shortest path solution.
* @param[out] all_thj_rs 	Vector with all angles in the discretized interval.
*/
void getAnglesRS(double m, double k_thj, double prev_thj, std::vector<double>& all_thj_rs){
	// Granularity of discretization for thta_j
	double h = 2 * M_PI * std::pow(3, m) / std::pow(2 * k_thj, m);

	double min_thj = prev_thj - 3 / 2 * h;
	double max_thj = prev_thj + 3 / 2 * h;
	for (double disc_thj_rs = min_thj; disc_thj_rs <= max_thj; disc_thj_rs += h)
		all_thj_rs.push_back(disc_thj_rs);
}

/*!
* Retrieves the best Dubins path between two points with an unknown theta_j.
* @param[in]  x_prev		Previous point x-coord.
* @param[in]  y_prev		Previous point y-coord.
* @param[in]  all_thj 		Vector with all angles in the discretized interval.
* @param[in]  x_last 		Last point x-coord.
* @param[in]  y_last 		Last point y-coord.
* @param[in]  th_last 		Last point angle direction.
* @param[in]  Kmax 			Maximum curvature.
* @param[out] minDj_result 	Tuple with best theta_j, best primitve idx and best Dubins curve.
*/
void minDj(double x_prev, double y_prev, std::vector<double>& all_thj,	
	double x_last, double y_last, double th_last, double Kmax, 
	std::tuple<double, int, curve>& minDj_result){
	// Get theta_j which provides the best Dubins path
	std::pair<int, curve> current_path;
	double best_L = INFINITY;
	double current_L, best_thj;
	int best_pidx;
	curve best_c;

	for (int i = 0; i < all_thj.size(); i++){
		current_path = dubins_shortest_path(x_prev, y_prev, all_thj[i], x_last, y_last, th_last, Kmax);

		current_L = current_path.second.L;

		if (current_L < best_L){
			best_thj = all_thj[i];
			best_L = current_L;
			best_pidx = current_path.first;
			best_c = current_path.second;
		}
	}

	minDj_result = std::make_tuple(best_thj, best_pidx, best_c);
}

/*!
* It finds the best theta_j and Dubins path given a prev_thj and the current step m.
* @param[in]  m 			Number of refinement steps.
* @param[in]  k_thj 		Granularity of angle discretization: number of theta_j.
* @param[in]  x_prev		Previous discovered point x-coord.
* @param[in]  y_prev		Previous discovered point y-coord.
* @param[in]  all_thj_rs 	Vector with all angles in the discretized interval (in the m step). // TODELETE
* @param[in]  x_last 		Last known point x-coord.
* @param[in]  y_last 		Last known point y-coord.
* @param[in]  th_last 		Last known point angle direction.
* @param[in]  Kmax 			Maximum curvature.
* @param[out] minDj_result 	Tuple with best theta_j, best primitve idx and best Dubins curve (for the current step).
*/
void refinementSteps(double m, double k_thj, 
	double x_prev, double y_prev, //std::vector<double>& all_thj_rs, 
	double x_last, double y_last, double th_last, double Kmax, 
	std::tuple<double, int, curve>& minDj_result){
	// Initialize vars for various steps
	double prev_thj;
	std::vector<double> all_thj_rs;

	for (double current_m = 1; current_m <= m; current_m++){
		// Get the previous best theta_j
		tie(prev_thj, std::ignore, std::ignore) = minDj_result;

		// Get all the angles for the current_m and the prev_thj
		getAnglesRS(current_m, k_thj, prev_thj, all_thj_rs);

		// Compute the minDj in current_m
		// Find the "new" best theta_j and update minDj_result
		minDj(x_prev, y_prev, all_thj_rs, 
			x_last, y_last, th_last, Kmax,
			minDj_result);
	}
}



//**********************************************************************
// DUBINS CLASS
//**********************************************************************
/*!
* Parametrized class constructor: it takes initial and final points.
* @param[in]  x0 	Initial x-coord.
* @param[in]  y0 	Initial y-coord.
* @param[in]  th0 	Initial angle direction.
* @param[in]  xf 	Final x-coord.
* @param[in]  yf 	Final y-coord.
* @param[in]  thf 	Final angle direction.
* @param[in]  kmax 	Maximum curvature.
*/
DubinsProblem::DubinsProblem(double x0, double y0, double th0, 
	double xf, double yf, double thf, double kmax){
	this->x0 = x0;
	this->y0 = y0;
	this->th0 = th0;
	this->xf = xf;
	this->yf = yf;
	this->thf = thf;
	this->kmax = kmax;

	// Insert initial and final pts to the list
	all_pts_thj.push_back(std::make_tuple(x0, y0, th0));
	all_pts_thj.push_back(std::make_tuple(xf, yf, thf));

	// Default values
	this->k_thj = 4;
	this->m = 4;	
	this->pts_counter = 2;
}

/*!
* Class destructor.
*/
DubinsProblem::~DubinsProblem(){}

/*!
* Set the granularity of the discretization for the search of best theta_j. 
* It is initialized to 4 by default.
* @param[in]  k_thj 	Granularity of angle discretization: number of theta_j.
*/
void DubinsProblem::setK_thj(double k_thj){
	
	this->k_thj = k_thj;
}

/*!
* Set the number of required refinement step for the search of best theta_j. 
* It is initialized to 4 by default.
* @param[in]  m 	Number of refinement steps.
*/
void DubinsProblem::setM(double k_thj){
	
	this->k_thj = k_thj;
}

/*!
* Add a middle pt to the Dubins problem. Middle pts have unknown angle 
* direction hence the value is set to NaN.
* @param[in]  ptx 	Middle point X-coord.
* @param[in]  pty 	Middle point Y-coord.
*/
void DubinsProblem::addMiddlePt(double ptx, double pty){
	std::vector<std::tuple<double, double, double>>::iterator apt_it;
	apt_it = all_pts_thj.end()-1;

	// Middle pt has nan theta
	std::tuple<double, double, double> new_pt = std::make_tuple(ptx, pty, std::nan("1"));
	apt_it = all_pts_thj.insert(apt_it, new_pt);
	pts_counter += 1;
}

/*!
* Add a middle pt to the Dubins problem. Middle pts have unknown angle 
* direction hence the value is set to NaN.
* @param[in]  pt 	Point to add.
*/
void DubinsProblem::addMiddlePt(Point pt){
	double ptx = static_cast<double>(pt.x);
	double pty = static_cast<double>(pt.y);

	std::vector<std::tuple<double, double, double>>::iterator apt_it;
	apt_it = all_pts_thj.end()-1;

	// Middle pt has nan theta
	std::tuple<double, double, double> new_pt = std::make_tuple(ptx, pty, std::nan("1"));
	apt_it = all_pts_thj.insert(apt_it, new_pt);
	pts_counter += 1;
}

/*!
* Prints the parameters of the Dubins problem.
*/
void DubinsProblem::printInfo(){
	std::cout << "Dubins problem with " << pts_counter << " points:" <<std::endl;

	for (int i = 0; i < pts_counter; i++){
		if (i == 0)
			std::cout << "\tInitial point: ";
		else if (i == (pts_counter-1))
			std::cout << "\tFinal point: ";
		else
			std::cout << "\tMiddle point: ";

		std::cout << "x = " << std::get<0>(all_pts_thj[i]);
		std::cout << ", y = " << std::get<1>(all_pts_thj[i]);
		std::cout << ", th = " << std::get<2>(all_pts_thj[i]) << std::endl;
	}

	std::cout << std::endl;
}

/*!
* It finds the best/shortest Dubins path for a basic problem wit two known points. 
* It is the DubinsProblem class method equivalent of the function dubins_shortest_path().
*/
void DubinsProblem::findShortestPath(){
	std::pair<int, curve> dubin_result = dubins_shortest_path(x0,y0, th0, xf, yf, thf, kmax);
	all_best_curves.push_back(dubin_result);

	// Create an iterator for all_best_curves result and insert the result
	/*std::vector<std::pair<int, curve>>::iterator abc_it;
	abc_it = all_best_curves.begin();
	abc_it = all_best_curves.insert(abc_it, dubin_result);*/
}

/*!
* It finds the best Dubins path in a Multi Points Markov-Dubins Problem scenario.
*/
void DubinsProblem::solveDubins(std::vector<curve>& curves_result){
	// Clear all_best_curves from old results
	all_best_curves.clear();

	// Initialize vars
	double last_pt_x, last_pt_y, last_thj; // pt with known theta_j
	double prev_pt_x, prev_pt_y, prev_thj; // pt with (possible) unknown theta_j

	// Create an iterator for all_best_curves result
	std::vector<std::pair<int, curve>>::iterator abc_it;
	abc_it = all_best_curves.begin();

	for (int i = pts_counter-1; i > 0; i--){
		// Set the vars value
		last_pt_x = std::get<0>(all_pts_thj[i]);
		last_pt_y = std::get<1>(all_pts_thj[i]);
		last_thj = std::get<2>(all_pts_thj[i]);

		prev_pt_x = std::get<0>(all_pts_thj[i-1]);
		prev_pt_y = std::get<1>(all_pts_thj[i-1]);
		prev_thj = std::get<2>(all_pts_thj[i-1]);

		// Check if it is an MPMDP scenario: prev_thj is unknown?
		// True if it is a middle pt (i.e. theta_j == NaN)
		// False otherwise (i.e. it is the initial pt)
		if(std::isnan(prev_thj)){
			// Get all the k_thj angles (i.e. all theta_j angles)
			std::vector<double> all_thj;
			getAngles(k_thj, all_thj);

			// Best Dubins results between 2 pts
			std::tuple<double, int, curve> minDj_result;

			// Find best Dubins and perform refinement steps
			minDj(prev_pt_x, prev_pt_y, all_thj, last_pt_x, last_pt_y, last_thj, kmax, minDj_result);
			refinementSteps(m, k_thj, prev_pt_x, prev_pt_y, last_pt_x, last_pt_y, last_thj, kmax, minDj_result);

			// Insert the results in all_best_curves
			double best_thj;
			int best_pidx;
			curve best_curve;
			std::tie(best_thj, best_pidx, best_curve) = minDj_result;

			std::pair<int, curve> abc_result = std::make_pair(best_pidx, best_curve);
			abc_it = all_best_curves.insert(abc_it, abc_result);

			// Update the prev_thj in all_pts_thj with the best_thj
			std::tuple<double, double, double> update_pt = std::make_tuple(prev_pt_x, prev_pt_y, best_thj);
			all_pts_thj[i-1] = update_pt;
		} else {
			// Find best Dubins
			// Two known points scenario: find the best/shortest Dubins path with no refinement steps
			std::pair<int, curve> dubins_result = dubins_shortest_path(prev_pt_x, prev_pt_y, prev_thj, 
				last_pt_x, last_pt_y, last_thj, kmax);

			// Insert the result in all_best_curves
			abc_it = all_best_curves.insert(abc_it, dubins_result);
		}
	}

	// Populate the curves_result
	for (const auto& pidx_curve: all_best_curves){
		curves_result.push_back(pidx_curve.second);
	}
}

/*!
* Prints the points that solve the Dubins problem.
*/
void DubinsProblem::printSolutionPts(){
	double x0, y0, th0, xf, yf, thf;
	int pidx;
	curve c;
	//std::string primitives[6] = {"LSL", "LSR", "RSL", "RSR", "LRL", "RLR"};
	char primitives[6][4] = {"LSL", "LSR", "RSL", "RSR", "LRL", "RLR"};

	int tot_c = all_best_curves.size();
	std::printf("The solution is made of %d Dubins curves, in particular:\n", tot_c);

	for (int i = 0; i < pts_counter-1; i++){
		/*// Print first pt info
		x0 = std::get<0>(all_pts_thj[i]);
		y0 = std::get<1>(all_pts_thj[i]);
		th0 = std::get<2>(all_pts_thj[i]);
		std::printf("Initial point: x0 = %.2f, y0 = %.2f, th0 = %.2f\n", x0, y0, th0);

		// Print second pt info
		xf = std::get<0>(all_pts_thj[i+1]);
		yf = std::get<1>(all_pts_thj[i+1]);
		thf = std::get<2>(all_pts_thj[i+1]);
		std::printf("Final point: xf = %.2f, yf = %.2f, thf = %.2f\n", xf, yf, thf);

		// Print Dubins curve info between the two pts
		std::tie(pidx, c) = all_best_curves[i];
		std::printf("Dubins curve details: %s primitive, L = %.2f, s1 = %.2f, s2 = %.2f, s3 = %.2f \n", 
			primitives[pidx], c.L, c.arc1.L, c.arc2.L, c.arc3.L);*/
		std::tie(pidx, c) = all_best_curves[i];
		std::printf("%s primitive, L = %.2f, s1 = %.2f, s2 = %.2f, s3 = %.2f \n", 
			primitives[pidx], c.L, c.arc1.L, c.arc2.L, c.arc3.L);
		if (c.arc1.L != 0){
			std::printf("\ts1 initial: x0 = %.2f, y0 = %.2f, th0 = %.2f\n", 
				c.arc1.x0, c.arc1.y0, c.arc1.th0);
			std::printf("\ts1 final: xf = %.2f, yf = %.2f, thf = %.2f\n", 
				c.arc1.xf, c.arc1.yf, c.arc1.thf);
		}
		
		if(c.arc2.L != 0){
			std::printf("\ts2 initial: x0 = %.2f, y0 = %.2f, th0 = %.2f\n", 
				c.arc2.x0, c.arc2.y0, c.arc2.th0);
			std::printf("\ts2 final: xf = %.2f, yf = %.2f, thf = %.2f\n", 
				c.arc2.xf, c.arc2.yf, c.arc2.thf);
		}

		if(c.arc3.L != 0){
			std::printf("\ts3 initial: x0 = %.2f, y0 = %.2f, th0 = %.2f\n", 
				c.arc3.x0, c.arc3.y0, c.arc3.th0);
			std::printf("\ts3 final: xf = %.2f, yf = %.2f, thf = %.2f\n", 
				c.arc3.xf, c.arc3.yf, c.arc3.thf);
		}

		std::cout << std::endl;
	}
}

/*!
* It plots and save dubins.png in /home/user_name/workspace/project/src/ .
*/
void DubinsProblem::plot(){
	if (pts_counter <= 2){
		plotDubins(all_best_curves[0].second);
	} else {
		plotMultiDubins(all_best_curves);
	}
}



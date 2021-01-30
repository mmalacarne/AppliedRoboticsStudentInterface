#include "pathPlanning.hpp"

#define LOG_PLANPATH

//**********************************************************************
// PRIVATE FUNCTIONS
//**********************************************************************
/*!
* Description...
* @param[in]  param1 	Param 1 description.
* @param[in]  param2 	Param 2 description.
* @param[in]  param3 	Param 3 description.
* @param[out] return 	Return description.
*/


//**********************************************************************
// PUBLIC FUNCTIONS
//**********************************************************************
bool my_planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
	const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
	const float x, const float y, const float theta, Path& path, const std::string& config_folder){
	#ifdef LOG_PLANPATH
    	std::cout << "STUDENT FUNCTION - my_planPath" << std::endl;
    #endif

    // EX. slide
	/*double x0 = 0;
	double y0 = 0;
	double th0 = -M_PI/2;
	double xf = 4;
	double yf = 0;
	double thf = -M_PI/2;
	double Kmax = 1;*/

    /*// First test with basic implementation from lecture
    std::pair<int, curve> dubin_result = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax);
    plotDubins(dubin_result.second);*/

    // Class test
    /*DubinsProblem dubins(x0, y0, th0, xf, yf, thf, Kmax);
    //dubins.findShortestPath(); // class method for dubins_shortest_path()
    dubins.addMiddlePt(5, -1);
    dubins.printInfo();
    dubins.solveDubins();
    dubins.printSolutionPts();
    dubins.plot();*/

    // Arc and line intersection
    double arc_x0 = 0, arc_y0 = 5, arc_th0 = 0; 	// initial robot position facing right
    double arc_L = 5; 								// arc length
    double min_r = 5; 								// min turning radius
    double arc_k = -1/min_r; 						// min_r = 1/kmx -> kmx = 1/min_r
    double arc_xf, arc_yf; 							// final robot pts
    // final robot position
    std::tie(arc_xf, arc_yf, std::ignore) = circline(arc_L, arc_x0, arc_y0, arc_th0, arc_k);

    double L_x0 = -3, L_y0 = 5, L_xf = 3, L_yf = 5;
    bool i_AL; 
    double i_AL_x0, i_AL_y0, i_AL_x1, i_AL_y1;

    std::tie(i_AL, i_AL_x0, i_AL_y0, i_AL_x1, i_AL_y1) = intersArcLine(arc_x0, arc_y0, arc_th0, 
    	arc_xf, arc_yf, arc_L, arc_k, L_x0, L_y0, L_xf, L_yf);

    if (i_AL){
    	std::cout << "Intersection in ";
    	if (! std::isnan(i_AL_x0))
    		std::cout << "x1 = " << i_AL_x0 << " y1 = " << i_AL_y0;
    	if (! std::isnan(i_AL_x1))
    		std::cout << " and in x2 = " << i_AL_x1 << " y2 = " << i_AL_y1 << std::endl;
    } else {
    	std::cout << "No intersection found" << std::endl;
    	std::cout << "x1 = " << i_AL_x0 << " y1 = " << i_AL_y0;
    	std::cout << " - x2 = " << i_AL_x1 << " y2 = " << i_AL_y1 << std::endl;
    }

    std::cout << "my_planPath - END" << std::endl;

    return true;
}



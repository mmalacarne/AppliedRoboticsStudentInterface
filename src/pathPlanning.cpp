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

    // Circle and line intersection test
    // Define circle
    double cx = 0, cy = 0, r = 5;
    circle c = getCircle(cx, cy, r);

    // Define segment
    double L_x0 = 3, L_y0 = 12, L_xf = -5, L_yf = -2;
    segment L = getSegment(L_x0, L_y0, L_xf, L_yf);

    bool i_CL;
    double i_CL_x0, i_CL_y0, i_th0;
    double i_CL_x1, i_CL_y1, i_th1;

    std::tie(i_CL, i_CL_x0, i_CL_y0, i_th0, i_CL_x1, i_CL_y1, i_th1) = intersCircleLine(c, L);

    if (i_CL){
    	std::cout << "Intersection in x1 = " << i_CL_x0 << " y1 = " << i_CL_y0;
    	std::cout << " and in x2 = " << i_CL_x1 << " y2 = " << i_CL_y1 << std::endl;
    } else {
    	std::cout << "No intersection found" << std::endl;
    	std::cout << "x1 = " << i_CL_x0 << " y1 = " << i_CL_y0;
    	std::cout << " - x2 = " << i_CL_x1 << " y2 = " << i_CL_y1 << std::endl;
    }

    return true;
}



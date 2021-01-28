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

    // Test on intersection detection
    double L1_x0 = 3, L1_y0 = -12, L1_xf = -3, L1_yf = 5;
    double L2_x0 = -2, L2_y0 = 9, L2_xf = 3, L2_yf = -12;
    bool i_LL;
    double i_LL_x, i_LL_y;

    std::tie(i_LL, i_LL_x, i_LL_y) = intersLineLine(L1_x0, L1_y0, L1_xf, L1_yf, 
    	L2_x0, L2_y0, L2_xf, L2_yf);

    if (i_LL)
    	std::cout << "Intersection in x = " << i_LL_x << " y = " << i_LL_y << std::endl;
    else
    	std::cout << "No intersection found" << std::endl;

    plotLL(L1_x0, L1_y0, L1_xf, L1_yf, L2_x0, L2_y0, L2_xf, L2_yf);

    std::cout << "my_planPath - END" << std::endl;

    return true;
}



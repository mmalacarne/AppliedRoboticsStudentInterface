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
	double x0 = 0;
	double y0 = 0;
	double th0 = -M_PI/2;
	double xf = 4;
	double yf = 0;
	double thf = -M_PI/2;
	double Kmax = 1;

    /*// First test with basic implementation from lecture
    std::pair<int, curve> dubin_result = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax);
    plotDubins(dubin_result.second);*/

    // Class test
    DubinsProblem dubins(x0, y0, th0, xf, yf, thf, Kmax);
    //dubins.findShortestPath(); // class method for dubins_shortest_path()
    dubins.addMiddlePt(5, -1);
    dubins.printInfo();
    dubins.solveDubins();
    dubins.printSolutionPts();
    dubins.plot();

    return true;
}



#include "pathPlanning.hpp"

#define LOG_PLANPATH
//#define DEBUG_MAP
#define DEBUG_MAP_G

#define MISSION_1

namespace plt = matplotlibcpp;

//**********************************************************************
// PRIVATE DEBUGGINGG / PLOTTING FUNCTIONS
//**********************************************************************
void getPlottableBorders(const Polygon& borders){
	// Var definition
	double x0, y0 , xf, yf;
	segment L;
	std::vector<double> L_x_data, L_y_data;

    for (int i = 0; i <= borders.size()-1; i++){
    	x0 = static_cast<double>(borders[i].x);
	    y0 = static_cast<double>(borders[i].y);

    	if (i == borders.size()-1){
			xf = static_cast<double>(borders[0].x);
			yf = static_cast<double>(borders[0].y);
    	} else {
	    	xf = static_cast<double>(borders[i+1].x);
	    	yf = static_cast<double>(borders[i+1].y);
    	}
    	
    	L = getSegment(x0, y0, xf, yf);
    	std::vector<double> L_x_data, L_y_data;
		getPlottableSegment(L, L_x_data, L_y_data);

		plt::named_plot("Borders", L_x_data, L_y_data, "blue");

		L_x_data.clear();
		L_y_data.clear();
    }
}

void getPlottableObstacles(const std::vector<Polygon>& obstacle_list){
	// Var definition
	double x0, y0 , xf, yf;
	segment L;
	std::vector<double> L_x_data, L_y_data;

	for (const auto& obstacle: obstacle_list){
    	for (int i = 0; i <= obstacle.size()-1; i++){
    		x0 = static_cast<double>(obstacle[i].x);
    		y0 = static_cast<double>(obstacle[i].y);

    		if (i == obstacle.size()-1){
		    	xf = static_cast<double>(obstacle[0].x);
		    	yf = static_cast<double>(obstacle[0].y);
    		} else {
		    	xf = static_cast<double>(obstacle[i+1].x);
		    	yf = static_cast<double>(obstacle[i+1].y);
    		}

	    	L = getSegment(x0, y0, xf, yf);
			getPlottableSegment(L, L_x_data, L_y_data);

			plt::named_plot("Obstacles", L_x_data, L_y_data, "red");

			L_x_data.clear();
			L_y_data.clear();
    	}
    }
}

void getPlottableVictims(const std::vector<std::pair<int,Polygon>>& victim_list){
	// Var definition
	double x0, y0 , xf, yf;
	segment L;
	std::vector<double> L_x_data, L_y_data;

	for (const auto& victim: victim_list){
    	Polygon victim_pol = victim.second;
	   	for (int i = 0; i <= victim_pol.size()-1; i++){
	   		x0 = static_cast<double>(victim_pol[i].x);
	   		y0 = static_cast<double>(victim_pol[i].y);

	   		if (i == victim_pol.size()-1){
				xf = static_cast<double>(victim_pol[0].x);
				yf = static_cast<double>(victim_pol[0].y);
	   		} else {
	   			xf = static_cast<double>(victim_pol[i+1].x);
				yf = static_cast<double>(victim_pol[i+1].y);
	   		}
			L = getSegment(x0, y0, xf, yf);
			getPlottableSegment(L, L_x_data, L_y_data);

			plt::named_plot("Victims", L_x_data, L_y_data, "green");

			L_x_data.clear();
			L_y_data.clear();
		}
	}
}

void getPlottableGate(const Polygon& gate){
	// Var definition
	double x0, y0 , xf, yf;
	segment L;
	std::vector<double> L_x_data, L_y_data;

	for (int i = 0; i <= gate.size()-1; i++){
		x0 = static_cast<double>(gate[i].x);
		y0 = static_cast<double>(gate[i].y);

    	if (i == gate.size()-1){
			xf = static_cast<double>(gate[0].x);
			yf = static_cast<double>(gate[0].y);
    	} else {
			xf = static_cast<double>(gate[i+1].x);
			yf = static_cast<double>(gate[i+1].y);
    	}
		L = getSegment(x0, y0, xf, yf);
		getPlottableSegment(L, L_x_data, L_y_data);

		plt::named_plot("Gate", L_x_data, L_y_data, "green");

		L_x_data.clear();
		L_y_data.clear();
    }
}

void getPlottableRobot(const float& x, const float& y){
	double cx = static_cast<double>(x);
	double cy = static_cast<double>(y);
	double r = 0.01;

	circle c = getCircle(cx, cy, r);

	std::vector<double> cf_x, cf_y;
	getPlottableCircle(c, cf_x, cf_y);

	plt::named_plot("Pt", cf_x, cf_y, "blue");
}

void getPlottableNodes(std::map<Point,std::vector<Point>>& graph){
	// Manage vertex
	circle c;
	double cx, cy, r = 0.01;
	std::vector<double> cf_x, cf_y;

	std::map<Point,std::vector<Point>>::iterator g_it = graph.begin();

	for (; g_it != graph.end(); g_it++){
		cx = static_cast<double>(g_it->first.x);
		cy = static_cast<double>(g_it->first.y);

		c = getCircle(cx, cy, r);
		getPlottableCircle(c, cf_x, cf_y);

		plt::named_plot("Pt", cf_x, cf_y, "black");
		cf_x.clear();
		cf_y.clear();

		//std::cout << "knn = graph.second.size() = " << g_it->second.size() << std::endl;
		/*if (g_it == graph.begin()){
			std::cout << "Node = ( " << g_it->first.x << ", " << g_it->first.y << " )" << std::endl;
			for (int i = 0; i < g_it->second.size(); i++){
				std::cout << "Edge " << i << " -> ( " << g_it->second[i].x;
				std::cout << ", " << g_it->second[i].y << " )" << std::endl;
			}
		}*/
	}
}

void getPlottableEdges(std::map<Point,std::vector<Point>>& graph){
	std::map<Point,std::vector<Point>>::iterator g_it = graph.begin();
	std::vector<Point> neighbours = g_it->second;

	double x0 = static_cast<double>(g_it->first.x);
	double y0 = static_cast<double>(g_it->first.y);
	double xf, yf;
	segment edge;
	std::vector<double> L_x_data, L_y_data;

	for (const auto& node: neighbours){
		xf = static_cast<double>(node.x);
		yf = static_cast<double>(node.y);

		edge = getSegment(x0, y0, xf, yf);
		getPlottableSegment(edge, L_x_data, L_y_data);

		plt::named_plot("Edge", L_x_data, L_y_data, "orange");

		L_x_data.clear();
		L_y_data.clear();
	}
}

void getPlottableGraphPath(std::vector<Point>& graph_path){
	// Var definition
	double x0, y0 , xf, yf;
	segment L;
	std::vector<double> L_x_data, L_y_data;

	for (int i = 0; i < graph_path.size()-1; i++){
    	x0 = static_cast<double>(graph_path[i].x);
	    y0 = static_cast<double>(graph_path[i].y);
	    xf = static_cast<double>(graph_path[i+1].x);
	    yf = static_cast<double>(graph_path[i+1].y);

	    L = getSegment(x0, y0, xf, yf);
		getPlottableSegment(L, L_x_data, L_y_data);

		if (i == 0)
			plt::named_plot("Graph path", L_x_data, L_y_data, "blue");
		else
			plt::named_plot("Graph path", L_x_data, L_y_data, "orange");

		L_x_data.clear();
		L_y_data.clear();
	}
}

void getPlottableSmoothedGraphPath(std::vector<Point>& graph_path){
	// Var definition
	double x0, y0 , xf, yf;
	segment L;
	std::vector<double> L_x_data, L_y_data;

	for (int i = 0; i < graph_path.size()-1; i++){
    	x0 = static_cast<double>(graph_path[i].x);
	    y0 = static_cast<double>(graph_path[i].y);
	    xf = static_cast<double>(graph_path[i+1].x);
	    yf = static_cast<double>(graph_path[i+1].y);

	    L = getSegment(x0, y0, xf, yf);
		getPlottableSegment(L, L_x_data, L_y_data);

		plt::named_plot("Graph path", L_x_data, L_y_data, "purple");

		L_x_data.clear();
		L_y_data.clear();
	}
}



//**********************************************************************
// PRIVATE SUPPORT FUNCTIONS
//**********************************************************************
/*!
* Sorts the victims from the smaller ID to the biggest.
* @param[out] 	victims_id_bc 	Vector with ID - victim's baricenter coords sorted by ID.
*/
bool sortVictimsByID(const std::pair<int, Point>& i, const std::pair<int, Point>& j){

	return (i.first < j.first);
}

/*!
* Provided a Dubins path, it returns true if there is collision, flase otherwise.
* In case of collision it populate a vector of tuples made with dubins curve initial 
* pt, collision pt, Dubins curve final pt.
* @param[in] 	curves_result 		Vector with Dubins curves.
* @param[in] 	obstacle_list 		Vector with obstacles polygon.
* @param[in] 	victim_list 		Vector with pairs of ID and victims' baricenter.
* @param[out] 	collisions_list 	List with tuples made of Dubins curve initila pt, collision pt, Dubins curve final pt.
* @return[bool] 					True if there is any collision, false otherwise.
*/
/*bool checkCollisions(const std::vector<curve>& curves_result, 
	const std::vector<Polygon>& obstacle_list, 
	const std::vector<std::pair<int,Polygon>>& victim_list, 
	std::vector<std::tuple<Point,Point,Point>>& collisions_list){
	// Collision detected
	// Get path via Dijkstra and smooth the graph path
	// Update curves_result
}*/

/*!
* Given a victim's id it populates a vector with all the obstacles in the map and 
* all the victims with bigger id.
* @param[in] 	id 				Victim's id.
* @param[in] 	victim_list 	Vector with pairs of victim ID and polygon.
* @param[in] 	obstacle_list 	Vector with obstacles polygon.
* @param[out] 	expanded_list 	Vector with obstacles and all victims polygon with bigger id.
*/
void getExpandedList(const int id, const std::vector<std::pair<int,Polygon>>& victim_list, 
	const std::vector<Polygon>& obstacle_list, std::vector<Polygon>& expanded_list){
	expanded_list = obstacle_list;

	for (const auto& id_pol: victim_list){
		if (id_pol.first > id) 
			expanded_list.push_back(id_pol.second);
	}
}

/*!
* Given an arc from a Dubins curve it populates a Path struct with points 
* evaluated every 0.05.
* @param[in] 	a 		Dubins arc.
* @param[out] 	path 	Path object for the robot.
*/
void createArcPath(const arc& a, Path& path){
	double ds = 0.05;

	double xf, yf, thf;
	float _l, _xf, _yf, _thf, _k;

	for (double l = 0; l <= a.L; l += ds){
		std::tie(xf, yf, thf) = circline(l, a.x0, a.y0, a.th0, a.k);

		_l = static_cast<float>(l);
		_xf = static_cast<float>(xf);
		_yf = static_cast<float>(yf);
		_thf = static_cast<float>(thf);
		_k = static_cast<float>(a.k);

		path.points.emplace_back(_l, _xf, _yf, _thf, _k);
		//path.points.push_back(_l, _xf, _yf, _thf, _k);
	}

	// TODELETE
    // Professor interface
    /*float xc = 0, yc = 1.5, r = 1.4;
    float ds = 0.05;
    for (float theta = -M_PI/2, s = 0; theta<(-M_PI/2 + 1.2); theta+=ds/r, s+=ds) {
      path.points.emplace_back(s, xc+r*std::cos(theta), yc+r*std::sin(theta), theta+M_PI/2, 1./r);    
    }*/
}

/*!
* Given a vector of dubins curves it populate the path for the robot.
* @param[in] 	dubins_path 	Vector of dubins curves that describe the path.
* @param[out] 	path 			Path object for the robot.
*/
void createPath(const std::vector<curve>& dubins_path, Path& path){
	for (const curve& c: dubins_path){
		createArcPath(c.arc1, path);

		createArcPath(c.arc2, path);

		createArcPath(c.arc3, path);
	}
}



//**********************************************************************
// PUBLIC FUNCTIONS
//**********************************************************************
bool my_planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
	const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
	const float x, const float y, const float theta, Path& path, const std::string& config_folder){
	#ifdef LOG_PLANPATH
    	std::cout << "STUDENT FUNCTION - my_planPath" << std::endl;
    #endif

    //******************************************************************
    // Get gate baricenter
    //******************************************************************
    Point gate_bc;
    getBaricenter(gate, gate_bc);

    //******************************************************************
    // Define arrival theta (i.e. gate thf)
    //******************************************************************
    float theta_f = M_PI / 2; 		// UP
    //float theta_f = - M_PI / 2; 	// DOWN
    //float theta_f = 0.; 				// RIGHT
    //float theta_f = M_PI; 			// LEFT


    //******************************************************************
    // Get victims baricenter
    //******************************************************************
    std::vector<std::pair<int,Point>> new_victim_list;

    for (const auto& id_pol: victim_list){
    	Point victim_bc;
    	getBaricenter(id_pol.second, victim_bc);

    	new_victim_list.push_back(std::make_pair(id_pol.first, victim_bc));
    }

    //******************************************************************
    // Preprocessing phase
    // Create a prm graph (i.e. find all graph's verteces and edges) 
    // based on: borders, obstacles, victims and gate
    //******************************************************************
    Point robot_bc = Point(x,y);
    int n_pts = 150;
    int knn = 10;
    std::map<Point,std::vector<Point>> graph;
    getGraph(borders, obstacle_list, victim_list, gate, robot_bc, n_pts, knn, graph);

    #ifdef DEBUG_MAP_G
    	//std::cout << "graph.size() = 150 + 5 = " << graph.size() << std::endl;

    	getPlottableBorders(borders);
    	getPlottableObstacles(obstacle_list);
    	getPlottableVictims(victim_list);
    	getPlottableGate(gate);
    	getPlottableNodes(graph);
    	//getPlottableEdges(graph);
    	/*getPlottableGraphPath(graph_path);
    	getPlottableSmoothedGraphPath(smoothed_path);
    	getPlottableRobot(x, y);

    	plt::title("Map with pts in C_free");

    	// Set x-axis and y-axis to [xmin-1, xmax+1] and [ymin-1, ymax+1] respectively
		double x_min_max[2] = {plt::xlim()[0], plt::xlim()[1]};
		double y_min_max[2] = {plt::ylim()[0], plt::ylim()[1]};
	    plt::xlim(x_min_max[0] - 0.1, x_min_max[1] + 0.1);
	    plt::ylim(y_min_max[0] - 0.1, y_min_max[1] + 0.1);

	    // Save png
		std::string this_file_path = __FILE__;
		std::string this_file_name = "pathPlanning.cpp";
		int upper_bound = this_file_path.length() - this_file_name.length();
		std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Map_pts.png";
	    plt::save(png_name);*/
    #endif

	//******************************************************************
    // Define the Dubins problem
    //******************************************************************
	double x0 = static_cast<double>(x);
	double y0 = static_cast<double>(y);
	double th0 = static_cast<double>(theta);

	double xf = static_cast<double>(gate_bc.x);
	double yf = static_cast<double>(gate_bc.y);
	double thf = static_cast<double>(theta_f);

	double r = 0.05; 	// 5 cm
	double Kmax = 1./r;

	DubinsProblem dubins(x0, y0, th0, xf, yf, thf, Kmax);

    //******************************************************************
    // Mission 1
    //******************************************************************
    #ifdef MISSION_1
    	// Sort victims by ID
    	std::sort(new_victim_list.begin(), new_victim_list.end(), sortVictimsByID);

    	// Initialize a victim map <baricenter,ID>
    	std::map<Point,int> bc_id_victim_map;

    	// Create a temporary path - list of known middle pts
    	std::vector<Point> tmp_path_pts;

    	tmp_path_pts.push_back(robot_bc); // Add initial position

    	for (const auto& ib: new_victim_list){

    		tmp_path_pts.push_back(ib.second); // Add victims baricenter

    		bc_id_victim_map.insert({ib.second, ib.first}); // Populate map
    	}

    	tmp_path_pts.push_back(gate_bc);

    	// Initialize path search structures
    	std::vector<Point> dijkstra_path;
    	std::vector<Point> smoothed_path;
    	std::vector<Polygon> expanded_list;

    	for (int i = 0; i < tmp_path_pts.size()-1; i++){
    		// Initial and final query points
    		Point q_i = tmp_path_pts[i];
    		Point q_f = tmp_path_pts[i+1];

    		// Deal with obstacles
    		bool is_victim = (bc_id_victim_map.count(q_f) > 0);
    		if (is_victim){
    			int v_id = bc_id_victim_map.find(q_f)->second;
    			getExpandedList(v_id, victim_list, obstacle_list, expanded_list);
    		} else {
    			expanded_list = obstacle_list;
    		}

    		// Get the dijkstra path
    		getDijkstraPath(q_i, q_f, graph, expanded_list, dijkstra_path);

    		// Get dijkstra smoothed path
    		pathSmoother(dijkstra_path, expanded_list, smoothed_path);

    		// Additional middle points must be added to the Dubins problem
    		if (smoothed_path.size() > 2){
	        	// Exclude last pt (i.e. gate baricenter has been already provided dubins)
	        	int sp_bound;

	        	if (i == tmp_path_pts.size()-2) 
	        		sp_bound = smoothed_path.size()-1;
	        	else 
	        		sp_bound = smoothed_path.size();

	        	// Exclude first pt (i.e. on the first round it is the robot position)
	        	for (int j = 1; j < sp_bound; j++){
	        		dubins.addMiddlePt(smoothed_path[j]);
	        	}
	        } else {
	        	// Just add the final middle pt still excluding the gate (i.e. the last one)
	        	if (i != tmp_path_pts.size()-2) 
	        		dubins.addMiddlePt(smoothed_path[1]);
	        }

            #ifdef DEBUG_MAP_G
                getPlottableGraphPath(dijkstra_path);
                getPlottableSmoothedGraphPath(smoothed_path);
            #endif

            // Clear structures
            expanded_list.clear();
            dijkstra_path.clear();
            smoothed_path.clear();
    	}

    	#ifdef DEBUG_MAP_G
    		// Set x-axis and y-axis to [xmin-1, xmax+1] and [ymin-1, ymax+1] respectively
            double x_min_max[2] = {plt::xlim()[0], plt::xlim()[1]};
            double y_min_max[2] = {plt::ylim()[0], plt::ylim()[1]};
            plt::xlim(x_min_max[0] - 0.1, x_min_max[1] + 0.1);
            plt::ylim(y_min_max[0] - 0.1, y_min_max[1] + 0.1);

            plt::title("Mission 1");

            // Save png
            std::string this_file_path = __FILE__;
            std::string this_file_name = "pathPlanning.cpp";
            int upper_bound = this_file_path.length() - this_file_name.length();
            std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Mission_1.png";
            plt::save(png_name);

            // Print dubins info
            dubins.printInfo();
        #endif

        //**************************************************************
        // Solve Dubins, get the vector of curves and populate path
        //**************************************************************
        std::vector<curve> curves_result;
        dubins.solveDubins(curves_result);

        createPath(curves_result, path);
    #else
    //******************************************************************
    // Mission 2
    //******************************************************************
    	// TODO
    #endif

    return true;
}



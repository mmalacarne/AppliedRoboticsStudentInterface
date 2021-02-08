#include "pathPlanning.hpp"

#define LOG_PLANPATH
//#define DEBUG_MAP
#define DEBUG_MAP_G

//#define MISSION_1

#define N_PTS 150
#define KNN 10

#ifndef MISSION_1
#define INCREASE 0.3 // 30%
#endif

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

void getPlottableRescuePath(std::vector<Point>& rescue_path){
	// Var definition
	double x0, y0 , xf, yf;
	segment L;
	std::vector<double> L_x_data, L_y_data;

	for (int i = 0; i < rescue_path.size()-1; i++){
    	x0 = static_cast<double>(rescue_path[i].x);
	    y0 = static_cast<double>(rescue_path[i].y);
	    xf = static_cast<double>(rescue_path[i+1].x);
	    yf = static_cast<double>(rescue_path[i+1].y);

	    L = getSegment(x0, y0, xf, yf);
		getPlottableSegment(L, L_x_data, L_y_data);

		plt::named_plot("Graph path", L_x_data, L_y_data, "black");

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

/*! // TODELETE
* Given a vector of point, it populates another vector with vectors representing 
* all possible points permutations. (Heaps algorithm)
* @param[in] 	orig_vec 			Vector of points to permutate.
* @param[in] 	size 				Dimension of the subvector that has to be permuted.
* @param[in] 	tot 				Total number of points. // TODELETE this argument
* @param[out] 	all_permutations 	Vector with all possible permutations.
*/
/*void getAllPermutation(std::vector<Point>& orig_vec, int size, //int tot, 
	std::vector<Point>& all_permutations){
	if (size == 1){
		all_permutations.push_back(orig_vec);
		return;
	}

	for (int i = 0; i < size; i++){
		findAllPermutation(orig_vec, size-1, tot);

		// If size is even, swap ith and (size-1)th element
		if (size % 2 == 0){
			Point tmp = orig_vec[i];
			orig_vec[i] = orig_vec[size-1];
			orig_vec[size-1] = tmp;
		} else {
			// If size is odd, swap 0th and (size-1)th element
			Point tmp = orig_vec[0];
			orig_vec[0] = orig_vec[size-1];
			orig_vec[size-1] = tmp;

		}
	}
}*/

/*!
* Given a vector of point representing a path, it reurns the total length.
* @param[in] 		path 	Vector of points composing a path.
* @return[float] 			The total length of the path.
*/
float getPathDistance(const std::vector<Point> path){
	float x0, y0, xf, yf;
	float l = 0.;

	for (int i = 0; i < path.size()-1; i++){
		x0 = path[i].x;
		y0 = path[i].y;
		xf = path[i+1].x;
		yf = path[i+1].y;

		l += std::sqrt( std::pow((x0 - xf), 2) + std::pow((y0 - yf) , 2) );
	}

	return l;
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
    std::map<Point,std::vector<Point>> graph;
    getGraph(borders, obstacle_list, victim_list, gate, robot_bc, N_PTS, KNN, graph);

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

    	tmp_path_pts.push_back(robot_bc); // Add robot initial position

    	for (const auto& ib: new_victim_list){

    		tmp_path_pts.push_back(ib.second); // Add victims baricenter

    		bc_id_victim_map.insert({ib.second, ib.first}); // Populate map
    	}

    	tmp_path_pts.push_back(gate_bc); // Add gate baricenter

    	// Initialize path search structures
    	std::vector<Point> dijkstra_path;
    	std::vector<Point> smoothed_path;
    	std::vector<Point> all_mid_pts;
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

	        // Add to all_mid_pts all smoothed_path pts
	        // But exclude the first one (i.e. robot_bc in the first step)
	        for (int j = 1; j < smoothed_path.size(); j++){ all_mid_pts.push_back(smoothed_path[j]); }

            #ifdef DEBUG_MAP_G
                getPlottableGraphPath(dijkstra_path);
                getPlottableSmoothedGraphPath(smoothed_path);
            #endif

            // Clear structures
            expanded_list.clear();
            dijkstra_path.clear();
            smoothed_path.clear();
    	}

    	// Remove from all_mid_pts the last point (i.e. gate_bc)
    	all_mid_pts.erase(all_mid_pts.end());

    	// All middle points must be added to the Dubins problem
    	for (const auto& pt: all_mid_pts){ dubins.addMiddlePt(pt); }

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
    #else
    //******************************************************************
    // Mission 2 // TODO
    //******************************************************************
    	// Create a temporary path - list of known pts
    	std::vector<Point> tmp_path_pts;

    	tmp_path_pts.push_back(robot_bc); // Add robot initial position

    	for (const auto& ib: new_victim_list){

    		tmp_path_pts.push_back(ib.second); // Add victims baricenter
    	}

    	tmp_path_pts.push_back(gate_bc); // Add gate baricenter

    	// Compute a matrix of distances
    	int tot_pts = tmp_path_pts.size();
    	float paths_dist[tot_pts][tot_pts];

    	// Initialize path search structures
    	Point node_0, node_f;
    	std::vector<Point> dijkstra_path;
    	std::vector<Point> smoothed_path;

    	// Populate paths_dist matrix
    	for (int i = 0; i < tot_pts; i++){
    		node_0 = tmp_path_pts[i];

    		for (int j = i+1; j < tot_pts; j++){
    			node_f = tmp_path_pts[j];

    			// Get the shortest path to the nest node in tmp_path_pts via dijkstra
		    	getDijkstraPath(node_0, node_f, graph, obstacle_list, dijkstra_path);

		    	// Get a smoothed version of the shortest path
		    	pathSmoother(dijkstra_path, obstacle_list, smoothed_path);

		    	// Populate the matrix with the smoothed path distance
		    	paths_dist[i][j] = getPathDistance(smoothed_path);
		    	paths_dist[j][i] = paths_dist[i][j];

	    		// Clear structures
	    		dijkstra_path.clear();
	    		smoothed_path.clear();
    		}

    		// Populate the diagonal
    		paths_dist[i][i] = 0.;
    	}

    	// Set the maximum path length w.r.t. the shortest robot-gate path
    	float robot_gate_len = paths_dist[0][tot_pts-1];
    	float max_length = robot_gate_len + (robot_gate_len * INCREASE);

    	// Find the best increased path (kind of backtracking)
    	std::vector<Point> approx_rescue_path;
    	approx_rescue_path.push_back(robot_bc); // Add robot initial position

    	float current_length = 0.; 			// path len after every step
    	int current_node_idx = 0; 			// initial index (i.e. robot position)
    	int next_node_idx;					// next nearest node index
    	std::set<int> visited_idx = { 0 }; 	// set of visited indexes
    	float len_ij, len_jg, len_ijg;

    	while (current_length <= max_length){
    		len_ij = INFINITY;

    		for (int j = 0; j < tot_pts; j++){
    			if (visited_idx.count(j) == 0 && current_node_idx != j && paths_dist[current_node_idx][j] < len_ij){
    				// Path length from node i to nearest node j
    				len_ij = paths_dist[current_node_idx][j];

    				// Path length from node j to the gate
    				len_jg = paths_dist[j][tot_pts-1];

    				// Path length from node i to the gate passing via node j
    				len_ijg = len_ij + len_jg;

    				// Update the best next_node_idx
    				next_node_idx = j;
    			}
    		}

    		// Update current length in order to keep track for th next step
			current_length += len_ijg;

			// If path lenght is valid, add points
			if (current_length <= max_length){
				// Add node_j as middle point
				approx_rescue_path.push_back(tmp_path_pts[next_node_idx]);

				// Update current_node_idx for next step
				current_node_idx = next_node_idx;
				visited_idx.insert(next_node_idx);
			}
    	}

    	approx_rescue_path.push_back(gate_bc); // Add the gate

    	// Get the actual rescue path
    	std::vector<Point> rescue_path;
		Point q_i, q_f;

		for (int i = 0; i < approx_rescue_path.size()-1; i++){
			q_i = approx_rescue_path[i];
			q_f = approx_rescue_path[i+1];

			// Get the shortest path q_i - q_f via dijkstra
        	getDijkstraPath(q_i, q_f, graph, obstacle_list, dijkstra_path);

        	// Get a smoothed version of the shortest path
        	pathSmoother(dijkstra_path, obstacle_list, smoothed_path);

        	// Populate the rescue_path
        	for (int j = 0; j < smoothed_path.size()-1; j++){
        		rescue_path.push_back(smoothed_path[j]);
        	}

        	// Clear structures
        	dijkstra_path.clear();
        	smoothed_path.clear();
		}

		#ifdef DEBUG_MAP_G
			std::vector<Point> dp, sp;
        	getDijkstraPath(robot_bc, gate_bc, graph, obstacle_list, dp);
        	pathSmoother(dp, obstacle_list, sp);

        	getPlottableGraphPath(dp);
        	getPlottableSmoothedGraphPath(sp);

        	rescue_path.push_back(gate_bc);
        	getPlottableRescuePath(rescue_path);
        	rescue_path.erase(rescue_path.end());
    	#endif

    	// Remove first pt (i.e. robot_bc) from rescue_path
		rescue_path.erase(rescue_path.begin());

		// Add rescue_path points as middle points to the Dubins problem
		for (const auto& pt: rescue_path){ dubins.addMiddlePt(pt); }

    	#ifdef DEBUG_MAP_G
    		// Set x-axis and y-axis to [xmin-1, xmax+1] and [ymin-1, ymax+1] respectively
            double x_min_max[2] = {plt::xlim()[0], plt::xlim()[1]};
            double y_min_max[2] = {plt::ylim()[0], plt::ylim()[1]};
            plt::xlim(x_min_max[0] - 0.1, x_min_max[1] + 0.1);
            plt::ylim(y_min_max[0] - 0.1, y_min_max[1] + 0.1);

            plt::title("Mission 2");

            // Save png
            std::string this_file_path = __FILE__;
            std::string this_file_name = "pathPlanning.cpp";
            int upper_bound = this_file_path.length() - this_file_name.length();
            std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Mission_2.png";
            plt::save(png_name);

            // Print dubins info
            dubins.printInfo();
    	#endif
    #endif

    //**************************************************************
    // Solve Dubins, get the vector of curves and populate path
    //**************************************************************
    std::vector<curve> curves_result;
    dubins.solveDubins(curves_result);

    createPath(curves_result, path);

    return true;
}



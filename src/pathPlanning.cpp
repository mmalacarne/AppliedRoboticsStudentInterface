#include "pathPlanning.hpp"

#define LOG_PLANPATH
//#define DEBUG_MAP
#define DEBUG_MAP_G

/*!
* When defined it compile the logic for the mission 1, otherwise mission 2.
*/
#define MISSION_1

/*!
* Number of points to sample for the randmo sampling map.
*/
#define N_PTS 250

/*!
* Number of K Nearest Neighbors.
*/
#define KNN 10 // 5

#ifndef MISSION_1
/*!
* Allowed percentage of increased path.
*/
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
* Given a victim's id it populates a vector of obstacles with all the victims with bigger id.
* @param[in] 	id 				Victim's id.
* @param[in] 	victim_list 	Vector with pairs of victim ID and polygon.
* @param[out] 	expanded_list 	Vector with obstacles and all victims polygon with bigger id.
*/
void expandObstacleList(const int id, const std::vector<std::pair<int,Polygon>>& victim_list, 
	std::vector<Polygon>& expanded_list){

	for (const auto& id_pol: victim_list){
		if (id_pol.first > id) 
			expanded_list.push_back(id_pol.second);
	}
}

/*!
* Given a vector of dubins curves it populate the path for the robot. For each arc in the 
* Dubins curve it populates a Path struct with points evaluated every 0.05.
* @param[in] 	dubins_path 	Vector of dubins curves that describe the path.
* @param[out] 	path 			Path object for the robot.
*/
void createPath(const std::vector<curve>& dubins_path, Path& path){
	// Vector with all arcs that make the Dubins curve
	std::vector<arc> all_arcs;
	for (const curve& c: dubins_path){
		all_arcs.push_back(c.arc1);
		all_arcs.push_back(c.arc2);
		all_arcs.push_back(c.arc3);
	}	

	// Vars init in order to populate a Path struct with pts 
	// evaluated every 5 cm
	double ds = 0.05; // 5 cm
	double xf, yf, thf;
	float _l, _xf, _yf, _thf, _k;

	for (const arc& a: all_arcs){
		for (double l = 0; l <= a.L; l += ds){
			std::tie(xf, yf, thf) = circline(l, a.x0, a.y0, a.th0, a.k);

			_l = static_cast<float>(l);
			_xf = static_cast<float>(xf);
			_yf = static_cast<float>(yf);
			_thf = static_cast<float>(thf);
			_k = static_cast<float>(a.k);

			path.points.emplace_back(_l, _xf, _yf, _thf, _k);
		}
	}
}

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
/*!
* "Mask" for planPath. For more info look in student::planPath docs.
*/
bool my_planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
	const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
	const float x, const float y, const float theta, Path& path, const std::string& config_folder){
	#ifdef LOG_PLANPATH
    	std::cout << "STUDENT FUNCTION - my_planPath" << std::endl;
    #endif

    //******************************************************************
    // Get gate and robot baricenters
    //******************************************************************
    Point gate_bc;
    getBaricenter(gate, gate_bc);
    Point robot_bc = Point(x,y);

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
    // based on: borders, obstacles, victims, gate and robot position
    //******************************************************************
    std::map<Point,std::vector<Point>> graph;
    getGraph(borders, obstacle_list, victim_list, gate, robot_bc, N_PTS, KNN, graph);

    #ifdef DEBUG_MAP_G
    	getPlottableBorders(borders);
    	getPlottableObstacles(obstacle_list);
    	getPlottableVictims(victim_list);
    	getPlottableGate(gate);
    	getPlottableNodes(graph);
    	//getPlottableEdges(graph);
    	//getPlottableGraphPath(dijkstra_path);
    	//getPlottableSmoothedGraphPath(smoothed_path);
    	//getPlottableRobot(x, y);

    	/*plt::title("Map - 250 pts - 10KNN");

    	// Set x-axis and y-axis to [xmin-1, xmax+1] and [ymin-1, ymax+1] respectively
		double x_min_max[2] = {plt::xlim()[0], plt::xlim()[1]};
		double y_min_max[2] = {plt::ylim()[0], plt::ylim()[1]};
	    plt::xlim(x_min_max[0] - 0.1, x_min_max[1] + 0.1);
	    plt::ylim(y_min_max[0] - 0.1, y_min_max[1] + 0.1);

	    // Save png
		std::string this_file_path = __FILE__;
		std::string this_file_name = "pathPlanning.cpp";
		int upper_bound = this_file_path.length() - this_file_name.length();
		std::string png_name = this_file_path.substr(0, upper_bound) + "testing_imgs/Map_250pts_10knn.png";
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

        // Create a temporary path (i.e. a list of known path pts)
        std::vector<std::pair<Point,int>> tmp_path_pts; // <baricenter,ID>

        tmp_path_pts.push_back({robot_bc, -1}); // Add robot initial position with place holder ID

        for (const auto& ib: new_victim_list){ tmp_path_pts.push_back({ib.second, ib.first}); }

        tmp_path_pts.push_back({gate_bc, -1}); // Add gate baricenter with place holder ID

        // Initialize path search structures
        Point qi, qf;
        int qf_id;
        std::vector<Point> dijkstra_path, smoothed_path, all_mid_pts;
        std::vector<Polygon> expanded_list;

        for (int i = 0; i < tmp_path_pts.size()-1; i++){
            // Initial and final query points
            qi = tmp_path_pts[i].first;
            qf = tmp_path_pts[i+1].first;
            qf_id = tmp_path_pts[i+1].second;

            // Deal with obstacles
            expanded_list = obstacle_list;

            if (qf_id >= 0)
                expandObstacleList(qf_id, victim_list, expanded_list);

            // Get the dijkstra path
            getDijkstraPath(qi, qf, graph, expanded_list, dijkstra_path);

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
    // Mission 2
    //******************************************************************
    	// Create a temporary path - list of known pts
    	std::vector<Point> tmp_path_pts;

    	tmp_path_pts.push_back(robot_bc); // Add robot initial position

    	for (const auto& ib: new_victim_list){ tmp_path_pts.push_back(ib.second); } // Add victims baricenter

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
    	float rg_len = paths_dist[0][tot_pts-1]; // len of shortest path robot - gate 
    	float max_len = rg_len + (rg_len * INCREASE);
    	#ifdef DEBUG_MAP_G
    		std::cout << "Len robot-gate: " << rg_len << std::endl;
    		std::cout << "Max len: " << max_len << std::endl;
    	#endif

    	// Find the best increased path (kind of backtracking)
    	std::vector<Point> approx_rescue_path;
    	approx_rescue_path.push_back(robot_bc); // Add robot initial position

    	float current_len = 0.; 			// path len after every step
    	int current_node_idx = 0; 			// initial index (i.e. robot position)
    	int next_node_idx;					// next nearest node index
    	std::set<int> visited_idx = { 0 }; 	// set of visited indexes
    	float len_ij, len_jg, len_ijg;

    	while (current_len <= max_len ){
    		len_ij = INFINITY;

    		for (int j = 0; j < tot_pts-1; j++){
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

			// If path lenght is valid, add points
			if ((current_len + len_ijg) <= max_len){
				// Update current length up to node j
				current_len += len_ij;

				// Add node_j as middle point
				approx_rescue_path.push_back(tmp_path_pts[next_node_idx]);

				// Update current_node_idx for next step
				current_node_idx = next_node_idx;
				visited_idx.insert(next_node_idx);

				#ifdef DEBUG_MAP_G
					std::cout << "Point added: (" << tmp_path_pts[next_node_idx].x;
					std::cout << ", " << tmp_path_pts[next_node_idx].y << ")" << std::endl;
					std::cout << "Current len: " << current_len << std::endl;
				#endif
			} else {
				current_len += len_ijg; // current_len will be > max_len -> it will stop while loop
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



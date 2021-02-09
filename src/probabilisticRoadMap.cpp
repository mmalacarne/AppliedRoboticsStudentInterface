#include "probabilisticRoadMap.hpp"

//#define DEBUG_PIP

#define B_OFFSET 0.04 // 4 cm

namespace plt = matplotlibcpp;

//**********************************************************************
// PRIVATE FUNCTIONS
//**********************************************************************
/*!
* Returns true if the point is internal to a given polygon. It verifies the 
* condition of the query point via ray-casting algorithm. (PIP - Point In Polygon).
* @param[in] 	q_pt 	Query point coordinates.
* @param[in] 	s_pt 	Safe external point coordinates.
* @param[in] 	obj 	Polygon object.
* @return[bool] 		Bool true if the query point is internal, false otherwise. 		
*/
bool isPIP(const Point& q_pt, const Point& s_pt, const Polygon& obj){
	double x0 = static_cast<double>(s_pt.x);
	double y0 = static_cast<double>(s_pt.y);
	double xf = static_cast<double>(q_pt.x);
	double yf = static_cast<double>(q_pt.y);
	segment L = getSegment(x0, y0, xf, yf);

	// Intersection counter
	int counter = 0;
	double _x0, _y0, _xf, _yf;
	segment obj_edge;

	for (int i = 0; i <= obj.size()-1; i++){
		_x0 = static_cast<double>(obj[i].x);
		_y0 = static_cast<double>(obj[i].y);

		if (i == obj.size()-1){
			_xf = static_cast<double>(obj[0].x);
			_yf = static_cast<double>(obj[0].y);
		} else {
			_xf = static_cast<double>(obj[i+1].x);
			_yf = static_cast<double>(obj[i+1].y);
		}		
		
		obj_edge = getSegment(_x0, _y0, _xf, _yf);

		bool flag;
		double i_x, i_y;
		std::tie(flag, i_x, i_y) = intersLineLine(L, obj_edge);

		#ifdef DEBUG_PIP
			std::vector<double> E_x_data, E_y_data;
			getPlottableSegment(obj_edge, E_x_data, E_y_data);
			plt::named_plot("Obstacle edge", E_x_data, E_y_data, "red");
			E_x_data.clear();
			E_y_data.clear();

			if (flag && !std::isnan(i_x)){
				segment I = getSegment(i_x, i_y, 0., 0.);
				std::vector<double> i_x_data, i_y_data;
				getPlottableSegment(I, i_x_data, i_y_data);
				plt::named_plot("Intersection", i_x_data, i_y_data, "black");
			}
		#endif

		if (flag) counter += 1;
	}

	#ifdef DEBUG_PIP
		std::vector<double> L_x_data, L_y_data;
		getPlottableSegment(L, L_x_data, L_y_data);
		plt::named_plot("Pts", L_x_data, L_y_data, "blue");

		// Set x-axis and y-axis to [xmin-1, xmax+1] and [ymin-1, ymax+1] respectively
		double x_min_max[2] = {plt::xlim()[0], plt::xlim()[1]};
		double y_min_max[2] = {plt::ylim()[0], plt::ylim()[1]};
	    plt::xlim(x_min_max[0] - 1, x_min_max[1] + 1);
	    plt::ylim(y_min_max[0] - 1, y_min_max[1] + 1);


		std::string title;
		if ((counter % 2) == 0) title = "FALSE - pt not in polygon";
		else title = "TRUE - pt in polygon";

		plt::title(title);
		plt::show();

		std::cout << "counter = " << counter << " counter % 2 = " << counter%2 << std::endl;
	#endif

	// If counter is even -> q_pt is external w.r.t. obj
	return ( (counter % 2 == 0) ? false : true );
}

/*!
* Computes the euclidean distance between two given points.
* @param[in] 	pt0 	First point coords.
* @param[in] 	pt1 	Second point coords.
* @param[out] 	d 		Computed euclidean distance.		
*/
void getEuclidianDist(const Point& pt0, const Point& pt1, double& d){
	double x0 = static_cast<double>(pt0.x);
	double y0 = static_cast<double>(pt0.y);

	double x1 = static_cast<double>(pt1.x);
	double y1 = static_cast<double>(pt1.y);

	d = std::sqrt( std::pow((x0 - x1), 2) + std::pow((y0 - y1) , 2) );
}

/*!
* Given two graph's edeges it returns true if they do not collide with any of the 
* provided obstacles, false otherwise.
* @param[in] 	node0 			First graph's node.
* @param[in] 	node0 			Second graph's node.
* @param[in] 	obstacle_list 	List of Polygon obstacles.
* @return[bool] 				Bool true if graph's edge do not collide with any obstacle, false otherwise. 		
*/
bool isClearEdge(const Point& node0, const Point& node1, const std::vector<Polygon>& obstacle_list){
	// Graph edge definition
	double x0 = static_cast<double>(node0.x);
	double y0 = static_cast<double>(node0.y);
	double xf = static_cast<double>(node1.x);
	double yf = static_cast<double>(node1.y);
	segment g_edge = getSegment(x0, y0, xf, yf);

	// Obstacle edge initialization
	bool collision = false;
	double _x0, _y0, _xf, _yf;
	segment o_edge;

	for (int i = 0; (! collision) && (i < obstacle_list.size()); i++){
		Polygon obstacle = obstacle_list[i];

		for(int j = 0; (! collision) && (j <= obstacle.size()-1); j++){
			_x0 = static_cast<double>(obstacle[i].x);
			_y0 = static_cast<double>(obstacle[i].y);

			if (j == obstacle.size()-1){
				_xf = static_cast<double>(obstacle[0].x);
				_yf = static_cast<double>(obstacle[0].y);
			} else {
				_xf = static_cast<double>(obstacle[j+1].x);
				_yf = static_cast<double>(obstacle[j+1].y);
			}

			o_edge = getSegment(_x0, _y0, _xf, _yf);

			std::tie(collision, std::ignore, std::ignore) = intersLineLine(g_edge, o_edge);
		}
	}

	return (! collision);
}

/*!
* Returns true if the second element of the first pair is less than the second 
* element of the second pair.
* @param[in] 	a 	First pair.
* @param[in] 	b 	Second pair.
* @return[bool] 	Bool true if a second element is less than b second element, false otherwise. 		
*/
bool map_cmp(const std::pair<Point,double>& a, const std::pair<Point,double>& b){
    
    return (a.second < b.second);
}

/*!
* Given the a set of nodes and the distances map of Dijkstra algorithm, it provides the 
* node with minimum distance.
* @param[in] 	all_nodes_set 	Set with all nodes that need to be sorted.
* @param[in] 	distances 		Map of distances: distances[u] = n -> distance n from q_i to u.
* @param[out] 	node_u 			Node in distances map with minimum distance.		
*/
void getMinDistanceNode(const std::set<Point>& all_nodes_set, const std::map<Point,double>& distances, 
	Point& node_u){
	// Sort the map w.r.t. values by means of a relay vector
	std::vector<std::pair<Point,double>> distances_vec;

	for (const auto& p_d: distances){
		if (all_nodes_set.count(p_d.first) != 0)
			distances_vec.push_back(p_d);
	}

	std::sort(distances_vec.begin(), distances_vec.end(), map_cmp);

	node_u = distances_vec[0].first;
}



//**********************************************************************
// PUBLIC FUNCTIONS
//**********************************************************************
/*!
* Returns a graph with points uniformly distributed. The graph G(V, E) is 
* a key-value structure with verteces as keys and edges as value.
* @param[in] 	borders 		Borders' points coord.
* @param[in] 	obstacle_list 	List of obstacles polygon representing the not free space.
* @param[in] 	victim_list 	List of victims id and polygon.
* @param[in] 	gate 			Gate polygon.
* @param[in] 	robot_bc 		Robot baricenter.
* @param[in] 	n_pts 			Number of points to generate in the map.
* @param[out] 	graph			Map structure defining the graph.
*/
void getGraph(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
	const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
	const Point& robot_bc, const int n_pts, const int knn, 
	std::map<Point,std::vector<Point>>& graph){
	//******************************************************************
    // Create an expanded list -> obstacles + victims + add gate
	// Since victims area and gate zone is free space it is unecessary 
	// to produce points in that space
	// Then, get victims and gate baricenters and add them to pt_idx_map
	// Add also robot_bc to pt_idx_map
    //******************************************************************
    // TODO - convert pt_idx_map to std::set
    std::vector<Polygon> expanded_list;
	std::map<Point,int> pt_idx_map;
    int idx = 0;

    // Manage victims
    for (const auto& id_pol: victim_list){
    	Point victim_bc;
    	getBaricenter(id_pol.second, victim_bc);
    	pt_idx_map.insert({victim_bc, idx});
    	idx += 1;

    	expanded_list.push_back(id_pol.second);
    }

    // Manage obstacles
    for (const auto& obstacle: obstacle_list){

    	expanded_list.push_back(obstacle);
    }

    expanded_list.push_back(gate);

    // Add gate
	Point gate_bc;
	getBaricenter(gate, gate_bc);
	pt_idx_map.insert({gate_bc, idx});
	idx += 1;

	// Add robot baricenter
	pt_idx_map.insert({robot_bc, idx});
	idx += 1;

	//****************************************************************
	// Define the max and min value for the uniform distributed points
	//****************************************************************
	float min_x = INFINITY, max_x = - INFINITY;
	float min_y = INFINITY, max_y = - INFINITY;
	for (const auto& pt: borders){
		min_x = std::fmin(min_x, pt.x);
		max_x = std::fmax(max_x, pt.x);

		min_y = std::fmin(min_y, pt.y);
		max_y = std::fmax(max_y, pt.y);
	}
	//******************************************************************
    // "Offset" borders -> better be safe than sorry;)
    //******************************************************************
	min_x += B_OFFSET;
	max_x -= B_OFFSET;

	min_y += B_OFFSET;
	max_y -= B_OFFSET;

	//*********************************************************
	// Initialize uniform_real_distribution class for both axis
	//*********************************************************
	std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution_x(min_x, max_x);
    std::uniform_real_distribution<float> distribution_y(min_y, max_y);

    //**************************************************
    // Find a "safe point" w.r.t. an obstacle baricenter
    //**************************************************
    Point obstacle_bc;
    getBaricenter(expanded_list[0], obstacle_bc);

    Point safe_pt;
    bool found = false;

    while (! found){
    	safe_pt.x = distribution_x(generator);
    	safe_pt.y = distribution_y(generator);

    	// Segment from safe_pt to obstacle_bc must intersect with 
    	// the reference obstacle (i.e. safe_pt is out of reference obstacle)
    	found = isPIP(safe_pt, obstacle_bc, expanded_list[0]);

		// Segment from safe_pt to obstacle_bc must not intersect with
		// all other obstacles
		for (int i = 1; found && i < expanded_list.size(); i++){

			found = ( !isPIP(safe_pt, obstacle_bc, expanded_list[i]) );
		}
    }

	//***************************************************
	// Generate n_pts random points uniformly distributed 
	//***************************************************
	// Points generation loop
    Point new_pt;
    int tot_pts = pt_idx_map.size() + n_pts;

    while ( idx != tot_pts ){
    	new_pt.x = distribution_x(generator);
    	new_pt.y = distribution_y(generator);

    	// Check if the new_pt is internal to any obstacle
    	bool flag_pip = false;

    	for (int i = 0; (! flag_pip) && (i < expanded_list.size()); i++){

    		flag_pip = isPIP(new_pt, safe_pt, expanded_list[i]);
    	}

    	// A point is valid iff it is in free space (i.e. no Point In Polygon) 
    	// and it is not in pt_idx_map yet
    	if ( (!flag_pip) && (pt_idx_map.count(new_pt) == 0)){

    		pt_idx_map.insert({new_pt, idx});

    		idx += 1;
    	}

    	// NOTE
    	// std::map keeps its elements sorted from the smaller to the bigger.
    	// Hence pt_idx_map is already sorted.
    }

    //*************************************************************
	// Find the k nearest and reachable neighbours for all vertexes
	//*************************************************************
	// Compute the distance (each current_node vs all other pts)
	std::map<double,std::vector<Point>> dist_pt_map; // NOTE - dist_pt_map will be sorted by distance from smaller to bigger

	for (const auto& pt_idx: pt_idx_map){
		Point current_node = pt_idx.first;

		for (const auto& _pt_idx: pt_idx_map){
			Point new_node = _pt_idx.first;

			// Do not consider the distance from the node to itself
			// Check if the edge is colliding with any obstacle
			if (current_node != new_node && isClearEdge(current_node, new_node, obstacle_list)){
				double dist;
				getEuclidianDist(current_node, new_node, dist);

				// If dist already present -> update dist_pt_map[dist]
				// Otherwise insert new element in dist_pt_map
				if (dist_pt_map.count(dist) > 0)
					dist_pt_map[dist].push_back(new_node);
				else{
					std::vector<Point> pt_vec;
					pt_vec.push_back(new_node);

					dist_pt_map.insert({dist, pt_vec});
				}
			}
		}

		// Get the k nearest neighbours of current_node
		int knn_counter = 0;
		std::vector<Point> k_neighbors;
		std::map<double,std::vector<Point>>::iterator dp_it = dist_pt_map.begin();

		for ( ; (knn_counter != knn) && (dp_it != dist_pt_map.end()); dp_it++){
			// Vector of points at a certain distance from current_node
			std::vector<Point> dist_nn = dp_it->second;

			for (int i = 0; (knn_counter != knn) && (i < dist_nn.size()); i++){
				k_neighbors.push_back(dist_nn[i]);
				knn_counter += 1;
			}
		}

		// Insert in graph the new element with current_node as key 
		// and k_neighbours as value
		graph.insert({current_node, k_neighbors});

		// Clear dist_pt_map from all current_node's distances
		dist_pt_map.clear();
	}
}

/*!
* Returns, via Dijkstra algorithm, a vector of ordered nodes that determins the 
* shortest path in the graph from the initial query point to the final query point.
* @param[in] 	q_i 			Query initial point.
* @param[in] 	q_f 			Query final point.
* @param[in] 	graph 			Map structure defining the graph.
* @param[in] 	obstacle_list 	List of obstacles polygon representing the not free space.
* @param[out] 	g_path 			Vector of point describing the path from q_i to q_f.
*/
void getDijkstraPath(const Point& q_i, const Point& q_f, 
	const std::map<Point,std::vector<Point>>& graph, 
	const std::vector<Polygon>& obstacle_list, std::vector<Point>& g_path){
	// Q set
	std::set<Point> all_nodes_set;
	for (const auto& n_knn: graph){ all_nodes_set.insert(n_knn.first); }

	// S set
	std::set<Point> visited_nodes_set;

	// Distances from q_i to all other nodes in graph
	std::map<Point,double> distances; 		// distances[n] = u -> shortest path from q_i to n with length u
	// Backtracking structure
	std::map<Point,Point> backtrack_map; 	// bacltrack_map[n] = u -> shortest path from q_i to n via u

	// Initialize distances of all nodes at infinity, except q_i at 0
	for (const auto& node: all_nodes_set){ distances.insert({node, INFINITY}); }
	distances[q_i] = 0.;
	
	while (! all_nodes_set.empty()){
		// Take the node with min distance from q_i
		// On the first round it will be the source (i.e. node_u == q_i)
		Point node_u;
		getMinDistanceNode(all_nodes_set, distances, node_u);

		// Remove node_u from q and add it to visited_nodes_set - S
		all_nodes_set.erase(node_u);
		visited_nodes_set.insert(node_u);

		// Initialize the var for distance node_u - node_n
		double dist_u_n;
		std::vector<Point> neighbours = graph.find(node_u)->second;

		for (const auto& node_n: neighbours){
			if (isClearEdge(node_u, node_n, obstacle_list)){
				getEuclidianDist(node_u, node_n, dist_u_n);

				if ( (distances[node_u] + dist_u_n) < distances[node_n] ){
					// q_i can get to node_n via node_u with cost = distances[node_u] + dist_u_n
					distances[node_n] = distances[node_u] + dist_u_n;
					backtrack_map[node_n] = node_u;
				}
			}
		}
	}

	// Security check
	assert(all_nodes_set.empty() && (visited_nodes_set.size() == graph.size()));

	// Retrieve the path to q_f via backtracking
	Point node_bt = q_f;
	while (node_bt != q_i){
		// Insert node at begin of the path
		g_path.insert(g_path.begin(), node_bt);

		// Update to next node
		node_bt = backtrack_map[node_bt];
	}
	// Insert q_i node at begin of the path
	g_path.insert(g_path.begin(), q_i);
}

/*!
* Given a graph path, it provides a smoothed version.
* @param[in] 	g_path 			Graph path found with Dijkstra (at least 3 points).
* @param[in] 	obstacle_list 	List of obstacles polygon representing the not free space.
* @param[out] 	s_path 			Smoothed path.
*/
void pathSmoother(const std::vector<Point>& g_path, 
	const std::vector<Polygon>& obstacle_list, std::vector<Point>& s_path){
	// Insert the initial point
	s_path.push_back(g_path[0]);

	// Initialize smoothed initial and final points
	Point s_pt0 = g_path[0];
	Point s_ptf;

	// Initialize idx loop
	int idx = 2;

	while (idx < g_path.size()){
		// Set smoothed final pt 
		s_ptf = g_path[idx];

		if (! isClearEdge(s_pt0, s_ptf, obstacle_list)){
			// Add to the smoothed path the previous s_ptf
			s_path.push_back(g_path[idx-1]);

			// Update s_pt0
			s_pt0 = g_path[idx-1];
		}

		// Update idx - try shortcut to next pt
		idx += 1;
	}

	// Insert the final point
	s_path.push_back(g_path[g_path.size()-1]);
}



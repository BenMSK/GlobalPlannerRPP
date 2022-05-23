#include "../include/CatmullRomSpline.h"
#include "../include/GlobalPlanner.h"
#include "../include/Dijkstra.h"
#include <string>
#include <algorithm>


#define CHANGE 0 // move the way point to right-dir
#define M_PI 3.14159265358979323846
#define DEBUG false
#define UNSEARCHED_EDGE 3
#define USING_MODIFIED_HIERHOLZER true

#define GAMMA_cost1 0.9
#define GAMMA_cost2 0.5
#define forward_rate 1.
#define backward_rate 0.9
#define cpp_weight 10000000
#define fitness_weight 1
#define gear_weight 2.7778*2

using namespace std;

namespace globalplanner{

// MATH FUNCTION:
// Calculate a distance between point1(x1, y1) and point2(x2, y2)
float GlobalPlanner::distance(float point1_x, float point1_y, float point2_x, float point2_y) {
    return sqrt(pow(point1_x - point2_x, 2) + pow(point1_y - point2_y, 2));
}
// Round function
float GlobalPlanner::round(float value, int pos) {	
    float temp_number;
    temp_number = value * pow(10, pos);
    temp_number = floor(temp_number + 0.5);
    temp_number *= pow(10, -pos);
    return temp_number;
}


GlobalPlanner::GlobalPlanner(ros::NodeHandle node, ros::NodeHandle private_node) {
    // PUBLISH
    Pub_pure_path = node.advertise<nav_msgs::Path>("pure_globalPathData", 10);
    Pub_path = node.advertise<nav_msgs::Path>("globalPathData", 10);
    Pub_marker = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    // SUBSCRIBE
    Sub_localization = node.subscribe("/LocalizationData", 10, &GlobalPlanner::CallbackLocalizationData, this);// Subscribes vehicle's localization data
    Sub_globalCall = node.subscribe("/globalPathCall", 10, &GlobalPlanner::CallbackGlobalPath, this);// Request the global path from MASTER

    m_flag = true;
    filename = "";//Initialize
    node.param("/load_graph", filename, std::string(""));//added at 181109, Get a param from a launch
    _graph = ReadWeightedGraph(filename);//ReadWeightedGraph returns a pair (Graph, cost)
    node.getParam("/carla_reset_pos", usingCARLA);
    
    cout << "Activated" << endl;
    G = _graph._G;
    RG  = _graph._G;// Original Graph
    cost = _graph._cost;
    node_information = _graph._node_information;
    directed = _graph._directed;
    parkable = _graph._park;
    
    srand ( time(NULL) );// Make random routes,    
    // ======================= Experiments for ITSC2019
    //exp_log
    // fstream mfile;
    // string log_name = "/home/dyros-vehicle/catkin_ws/src/global_planner_rpp/exp_data/map1_proposed-";
    // log_name += to_string(max_generation);
    // log_name += " ";
    // log_name += to_string(er_count);
    // log_name += " and ";
    // log_name += to_string(eo_count);
    // log_name += ".csv";
    

    // mfile.open(log_name, std::fstream::in | std::fstream::out | std::fstream::app);
    // for(int i = 0; i < exp_count; i++) {
    //     RPP_final_path = RPP(RG, cost, node_information);
    //     mfile << fcm << ", " << cpu_time_used << endl;
    // }
    // mfile.close();

    // double sum_fcm;
    // for(int i = 0; i < fcm_stack.size(); i++)
    //     sum_fcm += fcm_stack[i];
    // double avg_fcm;
    // avg_fcm = sum_fcm / (double) fcm_stack.size();
    // cout << "avg_fcm: " << avg_fcm << endl;
}

double GlobalPlanner::cost1(Graph & G, list<int> cycle, const vector< vector<float> > & node_information,
                 const float car_pos_x, const float car_pos_y) {
    double cost1_i;

    double array_sum;//numerator of cost1
    double total_sum;//denominator of cost1
    double final_cost1;

    int idx = 0;
    for(list<int>::iterator it = cycle.begin(); it != cycle.end(); it++) {
			cost1_i = distance(node_information[*it][1], node_information[*it][2], car_pos_x, car_pos_y);
            array_sum += cost1_i*pow(GAMMA_cost1, idx);
            total_sum += cost1_i;
            idx++;
		}
    
    final_cost1 = array_sum / total_sum;
    return final_cost1;
    }

double GlobalPlanner::cost2(Graph & G, vector< pair<int, int> > solution_edge, vector<bool> parkable, const vector< vector<float> > & node_information) {
    double cost2_i;
    double decay_rate;
    double norm_sol;
    double final_cost2 = 0;
    norm_sol = solution_edge.size();

    for(int idx=0; idx < solution_edge.size(); idx++) {
        if(parkable[G.GetEdgeIndex(solution_edge[idx].first, solution_edge[idx].second)]==false) {
            // cout << "NOPE"<< endl;
            // cout << solution_edge[idx].first << ", " << solution_edge[idx].second << endl;
            continue;
        }
        else {
            cost2_i = distance(node_information[solution_edge[idx].first][1], node_information[solution_edge[idx].first][2], 
                                    node_information[solution_edge[idx].second][1], node_information[solution_edge[idx].second][2]);
            // final_cost2 += cost2_i / (0.1*idx+1);
            decay_rate = (norm_sol - idx) / norm_sol;
            // cout << decay_rate << endl;
            final_cost2 += decay_rate *cost2_i;
        }
    }
    
    if (solution_edge[0].first == reasonable_start_edge[0] && solution_edge[0].second == reasonable_start_edge[1])
        final_cost2 *= forward_rate;
    else
        final_cost2 *= backward_rate;

    // 전，　후진　판단　＋　forward or backward rate 곱하기
    return final_cost2;
}

void GlobalPlanner::main() {//This function is run, continuously and I have to update the explored in the GRAPH for Re-Planning (TODO) 

    // // This is for using CARLA
    // if(usingCARLA) {
    //     // cout << "USING CARLA!"<<endl;
    //     car_pos_x = 90.7691421509;
    // 	car_pos_y = 316.689971924;
    // 	car_pos_heading = -89.9998397827;
    // }
    // else {
    //     car_pos_x = m_car.x;
    //     car_pos_y = m_car.y;
    //     car_pos_heading = m_car.th;
    // }

    // Get localization data
    // car_pos_x = m_car.x;
	// car_pos_y = m_car.y;
	// car_pos_heading = m_car.th;
	car_pos_x = -10;
	car_pos_y = 0;
	car_pos_heading = 0;
	// car_pos_waypt_idx = 0;

    RPP_final_path = RPP(RG, cost, node_information);

    // Initialize
    // if(run_main == 0) {
    solution_edge.clear();
    explored.first.clear();
    explored.second.clear();
    car_pos_waypt_idx = 0;//Assume that the vehicle is on the first waypoint
    // }
    // else {
    //     passed = 0;
    //     for(int i = 0; i < explored.first.size(); i++) {//explored[pass or not, IndicesOfEdges]
    //         if(explored.second[i] < car_pos_waypt_idx)
    //             explored.first[i] = true;
    //     }
    // }

    cycle_count = 0;//initialize
    // if(explored.first.size() - progress < UNSEARCHED_EDGE) {// If the vehicle pass 80% of the route, then...
		
    //     std::cout << "Global plan is re-planned" << std::endl;
	// 	if(true)
	// 		cout << "RESET! " << endl;
        
    //     progress = 0;
    //     solution_edge.clear();
    //     explored.first.clear();
    //     explored.second.clear();
        

    //     clock_t begin = clock();
        
    //     solution = ChinesePostman(G, cost, node_information, car_pos_x, car_pos_y, car_pos_heading);

    //     clock_t end = clock();
        
    //     double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

        
    //     ////////////////////////////////////////////////////////
    //     // Find the maximum cost, EulerTour in the candidates
    //     int maxElementIndex = std::max_element(candidate_cost.begin(),candidate_cost.end()) - candidate_cost.begin();
    //     EulerTour = candidate_cycle[maxElementIndex];
    //     fcost2 = candidate_cost[maxElementIndex];

	// 	// Print the solution // It compares the ParkingRPP with the original algorithm, CPP
    //     cout << "===== CPP Final Solution:" << endl;
	// 	for(list<int>::iterator it = EulerTour.begin(); it != EulerTour.end(); it++)
	// 		cout << *it << " ";
	// 	cout << endl;
    //     // cout << "Cost is " << fcost2 << endl;
    //     cout << "Check time: " << elapsed_secs << " [Seconds]"<<endl;
	// }
	// else {// If the vehicle pass below 80% of the route, then
	// 	for(int i = 0; i < explored.first.size(); i++) {
	// 		if(explored.first[i] == true)
	// 			passed++;//how many edges the agent pass.
	// 	}
	// 	progress += passed;// Assume that all solution(CPP routes) has same number of edges.

	// 	for(int i = 0; i <= passed; i++) {// Returns a new EulerTour for exploring unexplored edges, first.
	// 		if(passed==0)// Car doesn't explore any route of given EulerTour
	// 			break;
	// 		if(i==0) {
	// 			EulerTour.pop_front();
	// 			continue;
	// 		}
	// 		if(i==passed) {
	// 			EulerTour.push_back(EulerTour.front());
	// 			continue;
	// 		}
	// 		int temp = EulerTour.front();
	// 		EulerTour.pop_front();
	// 		EulerTour.push_back(temp);	
	// 	}

	// 	// Now you have to initialize every parameterfs for the new EulerTour.
	// 	solution_edge.clear();
	// 	explored.first.clear();
	// 	explored.second.clear();
	// 	for(list<int>::iterator it = EulerTour.begin(); it != --EulerTour.end(); it++) {
	// 		pair<int, int> temp_pair;
	// 		list<int>::iterator it2 = it;
	// 		advance(it2, 1);

	// 		temp_pair.first = *it;
	// 		temp_pair.second = *it2;
	// 		solution_edge.push_back(temp_pair);
	// 		explored.first.push_back(false);//replanning
	// 	}
	// }
    
    // CPP -> EulerTour | RPP -> RPP_final_path
    vector< vector<float> > way_point_final;//initialize
    way_point_final = WayptPath(G, RPP_final_path, directed, node_information, interval); // Get a RPP with a cost function path;; It will return the global plan for visiting the parking roads with a priority
    // way_point_final = WayptPath(G, EulerTour, directed, node_information, interval); // Get a Euler Path solved by normal RPP problem
    way_point_final = ms_spline(way_point_final, interval, spline_weight);

	// Adding indices of waypoints
	// int j = 0;
	for(int i = 0; i < way_point_final.size(); i++)
		if(i != 0 && way_point_final[i][2] == 1)    explored.second.push_back(i);


    //CatmullRomSpline
    vector<Vector2d> nodeVec, nodeVecSpline;
    int interval_point = 3;
    for(int i =0; i < way_point_final.size(); i += interval_point)
        nodeVec.push_back(Vector2d(way_point_final[i][0], way_point_final[i][1]));

    CatmullRomSpline spline;
    nodeVecSpline = spline.PathToCurve(nodeVec, 1, interval*3*2);

    //ROS
    //Get way points and get it into the 'nav_msgs::Path' and PUBLISH
    vector< geometry_msgs::PoseStamped> waypoints;
    for(int i = 0; i < nodeVecSpline.size(); i++) {
        if (i % 5 != 0)
            continue;
        geometry_msgs::PoseStamped waypt;
        waypt.header.frame_id = "map";
        waypt.header.stamp = ros::Time::now();
        waypt.pose.position.x = nodeVecSpline[i][0];
        waypt.pose.position.y = nodeVecSpline[i][1];
        waypt.pose.position.z = 0;

        waypoints.push_back(waypt);

        // for exp
        nav_msgs::Path msg2;
        msg2.header.frame_id = "map";
        msg2.header.stamp = ros::Time::now();

        msg2.poses.resize(waypoints.size());

        for(unsigned int i = 0; i < waypoints.size(); i++)
            msg2.poses[i] = waypoints[i];
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = waypoints.back().pose.position.x;
        marker.pose.position.y = waypoints.back().pose.position.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 3;
        marker.scale.y = 3;
        marker.scale.z = 0.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        Pub_marker.publish( marker );
        Pub_path.publish(msg2);//node and few waypoints being published
    }

}


GlobalPlanner::~GlobalPlanner() {}



bool GlobalPlanner::Connected(Graph & G) {// Check that a graph is fully-connected which meanns there is no redundant edges or vertices of Topological Map( graph map )

    map<int, bool> visited;
    for(int j = 0; j < G.GetNumVertices(); j++) {
        // cout << j << ": " << G.GetVertices()[j]<<endl;
        visited[G.GetVertices()[j]] = false;
    }

    int n = 0;
    list<int> L;

    L.push_back(0);
    while(not L.empty()) {
        int u = L.back();
        // cout << "u: " << u <<endl;
        L.pop_back();

        if(visited[u]) continue;
        
        visited[u] = true;
        n++;

        for(list<int>::const_iterator it = G.AdjList(u).begin(); it != G.AdjList(u).end(); it++) {
            int v = *it;
            // cout << "v: " << v << endl;
            L.push_back(v);
        }
    }

    return G.GetNumVertices() == n;
}

list<int> GlobalPlanner::RPP(Graph & aG, const vector<double> & cost, const vector< vector<float> > & node_information) {
    
    // Initialize parameters for a genetic algorithm
    int childs = 100;
    int dominant = 0;
         
    clock_t start, end;
    
    // Count non-Req(optional) Edge
    int count_optionalE = 0;
    vector< pair<int, int> > optionalE;

    for(int e = 0; e < G.GetNumEdges(); e++)
        if (parkable[G.GetEdgeIndex(G.GetEdge(e).first, G.GetEdge(e).second)] == false)
            optionalE.push_back(G.GetEdge(e));
    count_optionalE = optionalE.size();

    // Genetic Algorithm
    vector< vector<bool> > vec_chorosome;
    vector< vector<bool> > vec_chorosome_child;
    vector<double> chorosome_cost;
    vector<size_t> chorosome_idx;
    vector<bool> unit;// one genome
    
    start = clock();
    while(true) {    
        // Make one generation
        for(int a = 0; a < vec_chorosome_child.size(); a++)
            vec_chorosome.push_back(vec_chorosome_child[a]);// push the best parents back

        for(int i = vec_chorosome_child.size(); i < childs; i++) {
            // cout << "random??????????:       " << randomly_choose << endl;
            for(int j = 0; j < count_optionalE; j++) {
                double randomly_choose = ((double)rand() / (RAND_MAX));
                if( randomly_choose > 0.8 || (vec_chorosome_child.size()==0) || 0.1*max_generation > dominant ) {
                    double randomBit = rand() % 2;
                    unit.push_back(randomBit);//mutation
                    continue;
                }
                //uniform crossover                
                double randomBit = ((double) rand() / (RAND_MAX));
                if (randomBit > 0.5)
                    unit.push_back(vec_chorosome_child[0][j]);
                else
                    unit.push_back(vec_chorosome_child[1][j]);                
            }
            
            vec_chorosome.push_back(unit);
            unit.clear();
        }
                    
        
        for(int i = 0; i < vec_chorosome.size(); i++) {
            //Making a new graph
            Graph tempG = aG;
            for(int j = 0; j < vec_chorosome[i].size(); j++)
                if(vec_chorosome[i][j] == false)
                    tempG.RemoveEdge(optionalE[j].first, optionalE[j].second);
        
            Graph _tempG(tempG.GetNumVertices());
            _tempG.initialize_vertices();
            vector<double> _tempG_cost(tempG.GetNumEdges());
            vector<bool> _tempG_park(tempG.GetNumEdges());
            
            // Make pairs with new and original one.
            map<int, int> matchingGn2o;//<new, original>
            map<int, int> matchingGo2n;//<original, new>        
            int mat = 0;
            for(int _mat = 0; _mat < _tempG.GetVertices().size(); _mat++)
                while(mat < tempG.GetVertices().size()) {
                    if(tempG.GetVertices()[mat] != -1) {
                        matchingGn2o.insert(make_pair(_tempG.GetVertices()[_mat], tempG.GetVertices()[mat]));
                        matchingGo2n.insert(make_pair(tempG.GetVertices()[mat], _tempG.GetVertices()[_mat]));
                        mat++;
                        break;
                    }
                    mat++;
                }

            double max_d = 99999.0;
            rpp_start = 0;
            for (int i = 0; i < _tempG.GetNumVertices(); i++) {
                double newo = (double) distance(node_information[matchingGn2o[i]][1], node_information[matchingGn2o[i]][2], car_pos_x, car_pos_y);
                if (newo < max_d) {
                    max_d = newo;
                    rpp_start = i;
                }
            }
            
            for(int i= 0; i< tempG.edges.size(); i++)  {
                double c;
                int u, v;
                u = matchingGo2n[tempG.edges[i].first];//new
                v = matchingGo2n[tempG.edges[i].second];//new
                _tempG.AddEdge(u, v);
                c = (double) distance(node_information[tempG.edges[i].first][1], node_information[tempG.edges[i].first][2],
                                    node_information[tempG.edges[i].second][1], node_information[tempG.edges[i].second][2]);
      
                _tempG_cost[_tempG.GetEdgeIndex(u, v)] = c + 0.00001*(double)rand()/RAND_MAX;// WHY....
                _tempG_park[_tempG.GetEdgeIndex(u, v)] = parkable[G.GetEdgeIndex(matchingGn2o[u], matchingGn2o[v])];
            }     
            
            pair<list<int>, double > sol_rpp = ChinesePostmanCost(_tempG, _tempG_cost,_tempG_park); // solving CPP with a subgraph

            chorosome_cost.push_back(sol_rpp.second);
            chorosome_idx.push_back(i);
            continue;

        }

        // sorting for selecting parent nodes
        std::sort(
            chorosome_idx.begin(),
            chorosome_idx.end(),
            [&chorosome_cost](size_t i1, size_t i2) {
                return chorosome_cost[i1] < chorosome_cost[i2];
            }
        );
    
        vec_chorosome_child.clear();

        // Best two childs are going to be parents
        for(int i = vec_chorosome.size()-1; i >= vec_chorosome.size()-2; i--)
            vec_chorosome_child.push_back(vec_chorosome[chorosome_idx[i]]);
        
        std::cout << "CURRENT GENRATION: " << dominant << std::endl;
        if(max_generation == dominant) break;
        dominant++;
        vec_chorosome.clear();
        chorosome_cost.clear();
        chorosome_idx.clear();
    }

    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
    cout << "computation time of GA: " << cpu_time_used;
    // cout << "\n============== sub_optimal rpp tour ==============!"<< endl;
    // cout << "Final Subgraph: ";
    // for(int j = 0; j < vec_chorosome_child.front().size(); j++)
    //     cout << vec_chorosome_child.front()[j];
    // cout << endl;
    // cout << "Additional Edge" << endl;
    // for(int j = 0; j < vec_chorosome_child.front().size(); j++)
    //     if (vec_chorosome_child.front()[j] == true)
    //         cout << "(" << optionalE[j].first << ", " << optionalE[j].second<<"), ";
    // cout << endl;

    
    // Making a new graph
    Graph temp2G = aG;
    for(int j = 0; j < vec_chorosome_child.front().size(); j++)
        if(vec_chorosome_child.front()[j] == false)
            temp2G.RemoveEdge(optionalE[j].first, optionalE[j].second);
        // if(exptmp[j] == false)
    
    //Making a new graph
    Graph _temp2G(temp2G.GetNumVertices());
    _temp2G.initialize_vertices();
    vector<double> _temp2G_cost(temp2G.GetNumEdges());
    vector<string> _temp2G_directed(temp2G.GetNumEdges());
    vector<bool> _temp2G_park(temp2G.GetNumEdges());

    map<int, int> matchingGn2o;//<new, original>
    map<int, int> matchingGo2n;//<original, new>
            
    int mat = 0;
    for(int _mat = 0; _mat < _temp2G.GetVertices().size(); _mat++)
        while(mat < temp2G.GetVertices().size()) {
            if(temp2G.GetVertices()[mat] != -1) {
                matchingGn2o.insert(make_pair(_temp2G.GetVertices()[_mat], temp2G.GetVertices()[mat]));
                matchingGo2n.insert(make_pair(temp2G.GetVertices()[mat], _temp2G.GetVertices()[_mat]));
                mat++;
                break;
            }
            mat++;
        }
    double max_d = 99999.0;
    rpp_start = 0;
    for (int i = 0; i < _temp2G.GetNumVertices(); i++) {
        double newo = (double) distance(node_information[matchingGn2o[i]][1], node_information[matchingGn2o[i]][2], car_pos_x, car_pos_y);
        if (newo < max_d) {
            max_d = newo;
            rpp_start = i;
        }
    }
    // cout << "rpp_start: " << rpp_start << endl;
    for(int i= 0; i< temp2G.edges.size(); i++)
    {
        double c;
        int u, v;
        u = matchingGo2n[temp2G.edges[i].first];
        v = matchingGo2n[temp2G.edges[i].second];
        _temp2G.AddEdge(u, v);
        c = (double) distance(node_information[temp2G.edges[i].first][1], node_information[temp2G.edges[i].first][2],
                                node_information[temp2G.edges[i].second][1], node_information[temp2G.edges[i].second][2]);
        _temp2G_cost[_temp2G.GetEdgeIndex(u, v)] = c;
        _temp2G_directed[_temp2G.GetEdgeIndex(u, v)] = "no";
        _temp2G_park[_temp2G.GetEdgeIndex(u,v)] = parkable[G.GetEdgeIndex(matchingGn2o[u], matchingGn2o[v])];
    }
    cout << endl;


    // Calculate the Fitness
    pair<list<int>, double> sol_rpp = ChinesePostmanCost(_temp2G, _temp2G_cost,_temp2G_park);
    list<int> sol_rpp_final = sol_rpp.first;
    double sol_rpp_final_cost = sol_rpp.second;
    
    cout << "Gear Switching: " << gear_switch << endl;
    cout << "Fitness to Objective: " << fcm << endl;
    fcm_stack.push_back(fcm);
    list<int> rpp_tour;// Change to the orignal one. (or change cooridnates)
    for(list<int>::iterator w =sol_rpp_final.begin(); w != sol_rpp_final.end(); w++)
        rpp_tour.push_back(matchingGn2o[*w]);

    list<int> return_tour = rpp_tour;

    for(list<int>::iterator w = return_tour.begin(); w != return_tour.end(); w++)
        cout << *w << " ";
    cout << endl;

    return return_tour;
}


/*
Solves the chinese postman problem
returns a pair containing a list and a double
the list is the sequence of vertices in the solution
the double is the solution cost
*/

pair<list<int>, double > GlobalPlanner::ChinesePostmanCost(Graph & Rgraph, const vector<double> & Rcost, const vector<bool> & Rpark) {


    // Check if the graph if connected
    if(not Connected(Rgraph)) {
        // cout << "NOT CONNECTED" << endl;
        list<int> _;
        return pair<list<int>, double >(_, 0.0);
    }
    

    // srand ( time(NULL) );// Make random routes,
    
    map< int, list<int> > A;//map A = [node name(key), adjacent node(value)]
    
    for(int u = 0; u < Rgraph.GetNumVertices(); u++)
        A.insert( pair< int, list<int> >(Rgraph.GetVertices()[u], Rgraph.AdjList(Rgraph.GetVertices()[u])) );
    

    vector<int> odd;// Contains an odd degree vertices
    for(map<int, list<int>>::iterator it = A.begin(); it != A.end(); it++)
        if((it->second).size() % 2) {
            odd.push_back(it->first);
            // cout << it->first << " ";
        }
    // cout << "\nodd size: " << odd.size() << endl;

    if(not odd.empty()) {

        Graph O(odd.size());// It makes a graph which only has vertices having odd degree

        for(int u = 0; u < (int)odd.size(); u++)// Make a comlete graph
            for(int v = u+1; v < (int)odd.size(); v++)
                O.AddEdge(u, v);
        // O.GraphInfo(O);
        vector<double> costO(O.GetNumEdges());// Make the costO vector which has empty rooms of number of edges

        /// Find the shortest paths between all odd degree vertices ///
        vector< map<int, int> > shortestPath(O.GetNumVertices());

        for(int u = 0; u < (int)odd.size(); u++) {

            pair< map<int, int>, map<int, double> > sp = Dijkstra(Rgraph, odd[u], Rcost);//Dijkstra(Graph, origin, cost) returns a pair (father, pathCost)
            shortestPath[u] = sp.first ;//father means shortest path of that vertex

            //The cost of an edge [u, v] in O will be the cost of the corresponding shortest path in G
            for(int v = 0; v < (int)odd.size(); v++)
                if(v != u)
                    costO[ O.GetEdgeIndex(u, v) ] = sp.second[odd[v]];
        }

        //Find the minimum cost perfect matching of the graph of the odd degree vertices
        Matching M(O);//O: A graph
        pair< list<int>, double > p = M.SolveMinimumCostPerfectMatching(costO);//it will return perfect matching in a graph
        if (p.second == -1.0) {
            list<int> _;
            return pair<list<int>, double >(_, 0.0);
        }
        // cout << "done, matching" << endl;
        list<int> matching = p.first;
        
        
        //If an edge [u, v] is in the matching, the edges in the shortest path from u to v should be duplicated in G, i.o. additional edges b/w odd_degree vertices
        for(list<int>::iterator it = matching.begin(); it != matching.end(); it++) {

            pair<int, int> p = O.GetEdge(*it);
            int u = p.first, v = odd[p.second];
            int w = shortestPath[u][v];
            while(w != -1) {// Store orders of nodes which are awkward(filtering the path which makes a vehicle drive backward)
                A[w].push_back(v);// Find the edges we have to pass, twice.
                A[v].push_back(w);
                v = w;
                w = shortestPath[u][v];
            }
        }
    }

    //This is to keep tracking how many times we can traverse an edge
    vector<int> traversed(Rgraph.GetNumEdges(), 0);
    for(map< int, list<int> >::iterator itr = A.begin(); itr != A.end(); itr++) {
        for(list<int>::iterator it = (itr->second).begin(); it != (itr->second).end(); it++) {
            int v = *it;
            if(v < (itr->first)) continue;
            traversed[Rgraph.GetEdgeIndex((itr->first), v)]++;
        }
    }

    double total_edges = (double) traversed.size();
    if (dbg)
        cout << "total edges: " << total_edges << endl;
    // Matching을 사용한 후에는, 무조건적으로 모든 node의 degree가 짝수는 아니다.
    // 특정 node의 degree가 홀수가 나온다. 이때 홀수 degree를 갖는 node는 2개가 나오고,
    // 시작 위치(source_node)가 아닐 경우, source_node를 바꿔야 되는 작업이 필요하다.
    // cout << "Make a cycle, from here" << endl;
    // Make a cycle, Euler Tour using 'Fleury algorithm'
    list<int> cycle;

    // ===================== Choose the source point!!!!!!!!!!!!!!!!!=========================
    // This source point becomes the start point of the global plan
    vector<int> odd_check;// Contains an odd degree vertices
    for(map<int, list<int>>::iterator it = A.begin(); it != A.end(); it++)
        if((it->second).size() % 2)
            odd_check.push_back(it->first);
    // cout << "odd_Check: " << odd_check.size() << endl;
    if (not odd_check.empty()) {
        list<int> _;
        return pair<list<int>, double >(_, 0.0);
    }
    
    cycle.push_back(rpp_start);
    // cycle.push_back(0);//map1
    // cycle.push_back(22);//map5

    double current_number = -1.0;
    tmp_Rpark0 = Rpark;

    int previous_u = 0;

    list<int>::iterator itp = cycle.begin();// Define an iterator, itp. Now itp has only one element, source point
    float obj = 0;//calculate the total cost of the route
    while(itp != cycle.end()) {

        int u = *itp;// Let u be the current vertex in the cycle
        int randomN;// Generate random numbers for making random routes. Euler Tour has many solutions
        
        list<int>::iterator jtp = itp;
        jtp++;
        current_number += 1;
        if (dbg)
            cout << "current number: " << current_number << endl;
        
        while(not A[u].empty()) {//This loop is running untill the A is empty.

            while(not A[u].empty() and traversed[ Rgraph.GetEdgeIndex(u, A[u].back()) ] == 0)
                A[u].pop_back();//remove the last element of the container   

            if(not A[u].empty()) {
                if (USING_MODIFIED_HIERHOLZER) {
                    ////////////////////////////////// Modified Hierholzer's //////////////////////////////////////
                    // Contributions of the paper
                    vector<double> temp_cost_list;
                    double temp_cost;
                    // cost function
                    for(list<int>::iterator adj_node = A[u].begin();adj_node != A[u].end();adj_node++) {
                        temp_cost = 0;
                        // if (*adj_node == previous_u)
                            // temp_cost -= gear_weight;
                        double decay_factor = (total_edges - current_number)/total_edges;
                        decay_factor = 1;//exp
                        double nonpassedP = (double) tmp_Rpark0[Rgraph.GetEdgeIndex(u, *adj_node)];
                        
                        if (nonpassedP == 0.0)
                            nonpassedP = -1.0;                        
                        temp_cost += decay_factor * nonpassedP * Rcost[ Rgraph.GetEdgeIndex(u, *adj_node) ];
                        temp_cost_list.push_back(temp_cost);
                    }
                    int maxElementIndex = std::max_element(temp_cost_list.begin(),temp_cost_list.end()) - temp_cost_list.begin();
                    randomN = maxElementIndex;
                    //////////////////////////////////////////////////////////////////////////////////////////////
                }
                else
                    randomN = rand() % A[u].size();//pick random index
                
                std::list<int>::iterator it = A[u].begin();
                std::advance(it, randomN);
                int v = *it;
                int previous_u = u;
                
                if (USING_MODIFIED_HIERHOLZER && tmp_Rpark0[Rgraph.GetEdgeIndex(u, v)] == true) // already passed,
                    tmp_Rpark0[Rgraph.GetEdgeIndex(u, v)] = false;
                

                A[u].erase(it);
                cycle.insert(jtp, v);
                
                traversed[Rgraph.GetEdgeIndex(u, v)]--;// Check that we visited the edge(u, v)
                obj += Rcost[ Rgraph.GetEdgeIndex(u, v) ];// Add cost of the edge(u, v) to obj
                u = v;// CHANGE the vertex
                for(list<int>::iterator wow = A[u].begin(); wow != A[u].end(); wow++)
                    if(*wow == previous_u) {
                        A[u].erase(wow);
                        break;
                    }
            }
        }
        itp++;
    }

    // cout << "Done Cycle"<<endl;
    // for(list<int>::iterator it = cycle.begin(); it != cycle.end(); it++)
    //     cout << *it << " ";
    // cout << endl;
    

    if (dbg)
        cout << "CHECK!" << endl;
    double cpp_cost = 0;
    
    double tour_cost = 0;
    double cycle_length = cycle.size() - 1;

    int cnt = 0;
    gear_switch = 0;

    vector<bool> tmp_Rpark = Rpark;
    for(list<int>::iterator s = cycle.begin(); s != --cycle.end(); s++) {
        list<int>::iterator t = s;
        list<int>::iterator t2 = s;
    
        advance(t, 1);
        advance(t2, 2);
        if ( *s == *t2 && t2 != cycle.end() ) 
            gear_switch += 1;
        cpp_cost += Rcost[Rgraph.GetEdgeIndex(*s, *t)];//lower is better
        
        double decay = double(cycle_length - cnt)/cycle_length;
        double parky = double(tmp_Rpark[Rgraph.GetEdgeIndex(*s, *t)]);
        if (parky == 1)
            tmp_Rpark[Rgraph.GetEdgeIndex(*s, *t)] = false;
        else
            parky = -1.0;
        if(dbg) {
            cout << "u, v: " << *s << ", " << *t << " -cost: ";
            cout << parky*Rcost[Rgraph.GetEdgeIndex(*s, *t)] << endl;
        }
        tour_cost += decay*parky*Rcost[Rgraph.GetEdgeIndex(*s, *t)];

        cnt++;
    }
    fcm = fitness_weight * tour_cost - gear_weight * gear_switch;

    double candidate_cost = fcm;
    
    return pair<list<int>, double >(cycle, candidate_cost);
}


/*
Solves the chinese postman problem
returns a pair containing a list and a double
the list is the sequence of vertices in the solution
the double is the solution cost
*/

pair< list<int>, double > GlobalPlanner::ChinesePostman(Graph & G, const vector<double> & cost, const vector< vector<float> > & node_information,
                                                        const float car_pos_x, const float car_pos_y, const float car_pos_heading) {

    srand ( time(NULL) );// Make random routes,
    bool again = true;// Checks continuously untill we get the route we want.
    int minsoo = 0;// Checks that how many times it searchs


    // 1. Get the starting point considering a localization data of a vehicle
    vector<int> heading_reasonable_node;
    int source_node;// starting node considering vehicle's localization data (x, y, th)

    // 181017
    vector< vector<int> > pointsOfEdge;
    vector< vector<int> > reasonable_pointsOfEdge;
    vector<int> cp_points;

    for(int i = 0; i < G.edges.size(); i++)
    {
        int first_node = G.edges[i].first;
        int second_node = G.edges[i].second;
        double temp_edge_length = distance(node_information[first_node][1], node_information[first_node][2],
                node_information[second_node][1], node_information[second_node][2]);
        int cp_number_of_pt = 20;
        double cp_interval = temp_edge_length / cp_number_of_pt;
        double cp_theta = atan2(node_information[second_node][2] - node_information[first_node][2],
                node_information[second_node][1] - node_information[first_node][1]);//edge_heading
        for(int cp_j = 1; cp_j<cp_number_of_pt; cp_j++) {
            double cp_ptx = round(node_information[first_node][1] + cp_j*cp_interval*cos(cp_theta), 3);
            double cp_pty = round(node_information[first_node][2] + cp_j*cp_interval*sin(cp_theta), 3);

            cp_points.push_back(i);
            cp_points.push_back(cp_ptx);
            cp_points.push_back(cp_pty);
            pointsOfEdge.push_back(cp_points);
            cp_points.clear();
        }
    }
    // cout << "edges!!! " << G.GetEdgeIndex(1, 3) << endl;

    for(int i = 0; i<pointsOfEdge.size(); i++) {
        // cout << "THAT is: " << pointsOfEdge[i][1] - car_pos_x << endl;
        float heading_diff = atan2(pointsOfEdge[i][2] - car_pos_y, pointsOfEdge[i][1] - car_pos_x);
        float c_h = 0;

        if (fabs(car_pos_heading - heading_diff) > M_PI)
            c_h = fabs(2*M_PI - fabs(car_pos_heading - heading_diff));
        else
            c_h = fabs(car_pos_heading - heading_diff);

        if (c_h < (M_PI/2))
            reasonable_pointsOfEdge.push_back(pointsOfEdge[i]);
    }

    double cp_dis = 9999;
    int source_edge_idx=0;
    for (int apoint = 0; apoint < reasonable_pointsOfEdge.size(); apoint++) {// Find the closest node from a vehicle
        float _dis = sqrt( pow(reasonable_pointsOfEdge[apoint][1] - car_pos_x,2) + pow(reasonable_pointsOfEdge[apoint][2] - car_pos_y,2) );
        if(cp_dis >= _dis)	{
            cp_dis = _dis;
            source_edge_idx=reasonable_pointsOfEdge[apoint][0];
        }
    }

    float first_heading = atan2(node_information[G.GetEdge(source_edge_idx).first][2] - car_pos_y,
                                node_information[G.GetEdge(source_edge_idx).first][1] - car_pos_x);
    float second_heading = atan2(node_information[G.GetEdge(source_edge_idx).second][2] - car_pos_y,
                                 node_information[G.GetEdge(source_edge_idx).second][1] - car_pos_x);
    float c_h1 = 0;
    float c_h2 = 0;
    if (fabs(car_pos_heading - first_heading) > M_PI)
        c_h1 = fabs(2*M_PI - fabs(car_pos_heading - first_heading));
    else
        c_h1 = fabs(car_pos_heading - first_heading);
    if (fabs(car_pos_heading - second_heading) > M_PI)
        c_h2 = fabs(2*M_PI - fabs(car_pos_heading - second_heading));
    else
        c_h2 = fabs(car_pos_heading - second_heading);
    if(c_h1 < c_h2)
        source_node = G.GetEdge(source_edge_idx).first;
    else
        source_node = G.GetEdge(source_edge_idx).second;
    ////

    // Find a reasonable 'start edge', considering vehicle's heading
    list<int> adj_to_source_node = G.AdjList(source_node);// Stores vertices which is adjacent to the source node.
    int next_node;// It is the adjacent node to the starting node
    float min_heading = 9999;
    float temp_min_heading = 0;
    float diff = 0;

    // vector<int> reasonable_start_edge;

    for(list<int>::iterator node = adj_to_source_node.begin(); node != adj_to_source_node.end(); node++) {

        float edge_heading = atan2(node_information[source_node][2] - node_information[*node][2], node_information[source_node][1] - node_information[*node][1]);
        if(fabs(car_pos_heading - edge_heading) > M_PI)
            diff = fabs(2*M_PI - fabs(car_pos_heading - edge_heading));
        else
            diff = fabs(car_pos_heading - edge_heading);

        if(diff <= min_heading)	{
            min_heading = diff;
            next_node = *node;
        }
    }
    if(DEBUG)
        cout << "Next Node: " << next_node << endl;

    //SWITCH
    reasonable_start_edge.push_back(next_node);
    reasonable_start_edge.push_back(source_node);
    // [next_node, source_node]

    source_node = next_node;
    source_node = 0;
    cout << "Source Node: " << source_node << endl;// Print out the source node, This source node describes the node which the vehicle is looking,
    // 여기서　자연스러운　간선을　찾고난　후，　그　간선부터　경로를　시작한다．　왜냐하면，초기에　정해진　시작노드까지　차량이　현재　위치에서　진행할　수　있는　웨이포인트가　존재하지　않는다．　하지만　문제는　차량의　초기　시작　노드에　인접해　있는　노드가　단　하나일　때，　문제　발생．
    // 이는　state machine의　형태로　차량의　초기　시작　노드의　인접한　노드의　개수가　하나일　때　그냥　알고리즘을　수행하는　쪽으로！ <- IMPORTANT

    // // RPP
    // vector<int> count_required;
    // for(int e = 0; e < G.GetNumEdges(); e++) {
    //     if (parkable[G.GetEdgeIndex(G.GetEdge(e).first, G.GetEdge(e).second)] == true) {
    //         count_required.push_back(G.GetEdge(e).first);
    //         count_required.push_back(G.GetEdge(e).second);
    //     }
    // }
    // sort( count_required.begin(), count_required.end() );
    // count_required.erase( unique( count_required.begin(), count_required.end() ), count_required.end() );

    // Graph R(count_required.size());

    //2. Find a proper route, CPP
    while(again) {// It is running until we got the route we want, that means the route satisfying the reasonable node and non-awkward(?) driving
        minsoo++;//calculate how many times we've search

        // Check if the graph if connected
        if(not Connected(G)) {
            cout << "NOT CONNECTED" << endl;
            throw "Error: Graph is not connected";// if it is satisfied, the execptional situation, it "throw", which means this graph is not completed
        }


        map< int, list<int> > A;//map A = [node name(key), adjacent node(value)]
        for(int u = 0; u < G.GetNumVertices(); u++)
            A.insert( pair< int, list<int> >(G.GetVertices()[u], G.AdjList(G.GetVertices()[u])) );
        // Returns the adjacency list of a vertex
        // That means, substitutes adjacency list of the node, u from G into the A with index, u
        // A =[ [node0, adjacency vertices of node 0], [node1, adjacency vertices of node 1], ... ]

        // FILTER1: Inspection to find Awkward Route
        vector< vector<int> > never_passed;// It is a kind of filter which makes us to decide which routes are reasonable. (non-awkward for a vehicle)
        vector<int> temp;

        vector<int> odd;// Contains an odd degree vertices
        for(map<int, list<int>>::iterator it = A.begin(); it != A.end(); it++)
            if((it->second).size() % 2)
                odd.push_back(it->first);
        
        // Euler Path
        if (A[source_node].size() % 2) {//source node has odd degree
            if (odd.size()==2)
                odd.clear();
            // else if (odd.size() % 2 == 0) {
            //     while(true) {//erase one of the odd nodes except source_node
            //         int _idx = rand() % odd.size();
            //         if (odd[_idx] != source_node) {
            //             odd.erase(odd.begin() + _idx);
            //             break;
            //         }
            //     }
                
            //     for(int _idx = 0; _idx < odd.size(); _idx++)// erase the source node in odd
            //         if (odd[_idx] == source_node) {
            //             odd.erase(odd.begin() + _idx);
            //             break;
            //         }
            // }
        }
    
        // If there are odd degree vertices
        if(not odd.empty()) {
            // Create a graph with the odd degree vertices
            // In other words, makes an augmented graph
            Graph O(odd.size());// It makes a graph which only has vertices having odd degree
            // This is beacuse we have to put additional edges to the graph like you did in Python
            
            // Total number of vertices of O is # of odd_degree
            // Make a complete graph, O. [Completed Graph: All vertices are connected with each other]
            for(int u = 0; u < (int)odd.size(); u++)
                for(int v = u+1; v < (int)odd.size(); v++)
                    O.AddEdge(u, v);

            vector<double> costO(O.GetNumEdges());// Make the costO vector which has empty rooms of number of edges

            /// Find the shortest paths between all odd degree vertices ///
            vector< map<int, int> > shortestPath(O.GetNumVertices());

            for(int u = 0; u < (int)odd.size(); u++) {

                pair< map<int, int>, map<int, double> > sp = Dijkstra(G, odd[u], cost);//Dijkstra(Graph, origin, cost) returns a pair (father, pathCost)
                shortestPath[u] = sp.first ;//father means shortest path of that vertex

                //The cost of an edge [u, v] in O will be the cost of the corresponding shortest path in G
                for(int v = 0; v < (int)odd.size(); v++)
                    if(v != u)
                        costO[ O.GetEdgeIndex(u, v) ] = sp.second[odd[v]];
            }

            if (DEBUG) {
                cout << "START_shortestPath"<< endl;
                for(int mi = 0; mi < (int)shortestPath.size(); mi++)
                    for(int ni = 0;ni < shortestPath[mi].size(); ni++)
                        cout << shortestPath[mi][ni] <<  endl;
                cout << "ENDHERE_shortestPath"<< endl;
            }
            
            //Find the minimum cost perfect matching of the graph of the odd degree vertices
            Matching M(O);//O: A graph
            pair< list<int>, double > p = M.SolveMinimumCostPerfectMatching(costO);//it will return perfect matching in a graph
            list<int> matching = p.first;
            
            //If an edge [u, v] is in the matching, the edges in the shortest path from u to v should be duplicated in G, i.o. additional edges b/w odd_degree vertices
            for(list<int>::iterator it = matching.begin(); it != matching.end(); it++) {

                pair<int, int> p = O.GetEdge(*it);
                int u = p.first, v = odd[p.second];
                int w = shortestPath[u][v];

                while(w != -1) {// Store orders of nodes which are awkward(filtering the path which makes a vehicle drive backward)

                    A[w].push_back(v);// Find the edges we have to pass, twice.
                    A[v].push_back(w);

                    // FILTER1: contains vectors into 'never_passed'
                    temp.push_back(v);
                    temp.push_back(w);
                    temp.push_back(v);
                    never_passed.push_back(temp);
                    temp.clear();

                    temp.push_back(w);
                    temp.push_back(v);
                    temp.push_back(w);
                    never_passed.push_back(temp);
                    temp.clear();

                    v = w;
                    w = shortestPath[u][v];
                }

            }
        }

        //This is to keep tracking how many times we can traverse an edge
        vector<int> traversed(G.GetNumEdges(), 0);
        for(map< int, list<int> >::iterator itr = A.begin(); itr != A.end(); itr++) {
            for(list<int>::iterator it = (itr->second).begin(); it != (itr->second).end(); it++) {
                int v = *it;
                if(v < (itr->first)) continue;
                traversed[G.GetEdgeIndex((itr->first), v)]++;
            }
        }

        // DECISION: Replanning, bug I'm not sure that I am going to use it
        // for(int i = 0; i < explored.first.size(); i++) {
        // 	if(explored.first[i] != true)
        // 		break;

        // 	for(list<int>::iterator it = A[solution_edge[i].first].begin(); it != A[solution_edge[i].first].end(); it++) {
        // 		if(*it == solution_edge[i].second) {
        // 			A[solution_edge[i].first].erase(it);
        // 			break;
        // 		}
        // 	}
        // 	for(list<int>::iterator it = A[solution_edge[i].second].begin(); it != A[solution_edge[i].second].end(); it++) {
        // 		if(*it == solution_edge[i].first) {
        // 			A[solution_edge[i].second].erase(it);
        // 			break;
        // 		}
        // 	}
        // }

        // Make a cycle, Euler Tour using 'Fleury algorithm'
        list<int> cycle;
        cycle.push_back(source_node);// Euler Tour will be started at the source node we've got
        list<int>::iterator itp = cycle.begin();// Define an iterator, itp. Now itp has only one element, source point
        float obj = 0;//calculate the total cost of the route
        // cout << "try: " << minsoo << endl;
        int previous_u = 0;
        while(itp != cycle.end()) {

            int u = *itp;// Let u be the current vertex in the cycle
            int randomN;// Generate random numbers for making random routes. Euler Tour has many solutions
            // cout << "U: " << u << endl;


            list<int>::iterator jtp = itp;
            jtp++;

            while(not A[u].empty()) {//This loop is running untill the A is empty.

                while(not A[u].empty() and traversed[ G.GetEdgeIndex(u, A[u].back()) ] == 0) {
                    A[u].pop_back();//remove the last element of the container
                    cout << "ACTIVATED!!!!" << endl;
                }
                if(not A[u].empty()) {

                    vector<double> temp_cost_list;
                    double temp_cost;
                    for(list<int>::iterator adj_node = A[u].begin();adj_node != A[u].end();adj_node++) {
                        temp_cost = 0;
                        if (*adj_node == previous_u && A[u].size() != 1)// not what da got da
                            continue;
                        if (parkable[G.GetEdgeIndex(u, *adj_node)] && traversed[G.GetEdgeIndex(u, *adj_node)] != 0)//next edge is parkable
                            temp_cost += cost[ G.GetEdgeIndex(u, *adj_node) ];
                        for(list<int>::iterator reqQ = A[*adj_node].begin(); reqQ != A[*adj_node].end(); reqQ ++)
                            if (parkable[G.GetEdgeIndex(*adj_node, *reqQ)] && traversed[G.GetEdgeIndex(*adj_node, *reqQ)] != 0)//next x 2 edge is parkable
                                temp_cost += 0.5*cost[ G.GetEdgeIndex(u, *adj_node) ];
                        temp_cost_list.push_back(temp_cost);
                    }
                    int maxElementIndex = std::max_element(temp_cost_list.begin(),temp_cost_list.end()) - temp_cost_list.begin();
                    randomN = maxElementIndex;
                    // randomN = rand() % A[u].size();//pick random index
                    // cout << "randomN: " << randomN <<endl;
                    std::list<int>::iterator it = A[u].begin();
                    std::advance(it, randomN);
                    int v = *it;
                    int previous_u = u;
                    A[u].erase(it);

                    cycle.insert(jtp, v);

                    // cout << "Cycle: ";
                    // for(list<int>::iterator aha = cycle.begin(); aha != cycle.end(); aha++){
                    // 	cout << *aha << ", ";
                    // }
                    // cout << endl;

                    traversed[G.GetEdgeIndex(u, v)]--;// Check that we visited the edge(u, v)
                    obj += cost[ G.GetEdgeIndex(u, v) ];// Add cost of the edge(u, v) to obj
                    u = v;// CHANGE the vertex

                    for(list<int>::iterator wow = A[u].begin(); wow != A[u].end(); wow++) {
                        if(*wow == previous_u) {
                            A[u].erase(wow);
                            break;
                        }
                    }

                }
            }
            //go to the next node in the cycle and do the same
            itp++;
        }

        // 3. Let's CHECK which the route has awkward orders of nodes.

        // FIRST: Compare reasonable start edge, That means we are going to check that the first and second nodes' sequence is reasonable.
        // Considering the localization data
        vector<int> cycle_start_edge;
        list<int>::iterator cycle_start_edge_i = cycle.begin();
        cycle_start_edge.push_back(*cycle_start_edge_i);
        std::advance(cycle_start_edge_i, 1);
        cycle_start_edge.push_back(*cycle_start_edge_i);

        // reasonalbe_start_edge -> heading과　맞는　방향의　start edge
        // if(reasonable_start_edge != cycle_start_edge) {// This route doesn't satisfy our preference.
        //     cycle_start_edge.clear();
        //     continue;//Find another route
        // }

        // Second: Compare it!, These lines compare that sequences we've got with the Tour
        // FILTER1
        vector<int> never_passed_sequence;// Filtering awkward combinations

        bool running = true;

        // running = FILTER1(never_passed, cycle);
        if(not running) {
            cycle.clear();
            odd.clear();
            continue;
        }


        cycle_count += 1;
        // for(list<int>::iterator it = cycle.begin(); it != cycle.end(); it++)
        //     cout << *it << " ";
        // cout << endl;

        vector< pair<int, int> > cycle_edge;
        // Making a sequence of edges of the solution and initialize explored_list. explored_list contains info. of which edge is explored.
        for(list<int>::iterator it = cycle.begin(); it != --cycle.end(); it++) {
            pair<int, int> temp_pair;
            list<int>::iterator it2 = it;
            advance(it2, 1);

            temp_pair.first = *it;
            temp_pair.second = *it2;
            cycle_edge.push_back(temp_pair);
        }
        
        fcost2 = cost2(G, cycle_edge, parkable, node_information);
        // cout << "# of candidate_cycle: " << candidate_cycle.size()<<endl;
        if (candidate_cycle.size() == 0) {
            candidate_cycle.push_back(cycle);
            candidate_cost.push_back(fcost2);
        }

        bool over_lap;
        over_lap = false;
        
        for(int j=0; j < candidate_cycle.size(); j++) {
            if(candidate_cycle[j] == cycle) {
                over_lap = true;
                break;
            }
        }
        
        if(!over_lap) {
            candidate_cycle.push_back(cycle);
            candidate_cost.push_back(fcost2);
        }

        if(cycle_count < search_times) {
            cycle.clear();
            odd.clear();
            continue;
        }

        return pair< list<int>, double >(cycle, obj);
    }//while
}


// Write down the raw set of way points without SPLINE
// Below function consider three edges, previous, current and next.
// It also considers the edges are directed or not. There are about 12 cases we have to consider.
vector< vector<float> > GlobalPlanner::WayptPath(const Graph & G, list<int> cycle, vector<string> directed, const vector< vector<float> > & node_information, const float interval) {// Write down the csv file for storing way points 

    vector< vector<float> > way_point;// Contains the waypoint_set (x, y, label)

    // Super complicated part that calculates these all way points. Fasten your seatbelt and make sure that you're ready.
    for(list<int>::iterator node_s = cycle.begin(); node_s != --cycle.end(); node_s++) {
        list<int>::iterator last_node = --cycle.end();
        advance(last_node, -1);

        // Initialize
        temp_node_information = node_information;// COPY
        edge_heading = 0;
        next_edge_heading = 0;
        previous_edge_heading = 0;

        theta = 0;
        theta1 = 0;
        theta2 = 0;
        edge_length = 0;
        previous_node = node_s;
        node_f = node_s;
        advance(node_f, 1);
        list<int>::iterator next_node;

        next_node = node_s;

        if(node_s != cycle.begin())
            advance(previous_node, -1);

        if(node_s != --cycle.end())
            advance(next_node, 2);

        edge_heading = atan2(node_information[*node_f][2] - node_information[*node_s][2],
                node_information[*node_f][1] - node_information[*node_s][1]);

        if(directed[G.GetEdgeIndex(*node_s, *node_f)] == "yes") {//Current edge is directed

            //1
            if(node_s != cycle.begin() && node_s != last_node \
                    && directed[G.GetEdgeIndex(*previous_node, *node_s)] == "no" \
                    && directed[G.GetEdgeIndex(*node_f, *next_node)] == "yes") { // previousE: nope, nextE: directed
                // cout << "1" <<endl;
                next_edge_heading = atan2(node_information[*next_node][2] - node_information[*node_f][2],
                        node_information[*next_node][1] - node_information[*node_f][1]);
                theta = fabs(next_edge_heading - edge_heading)/2;
                temp_node_information[*node_s][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_s][2] -= CHANGE*cos(edge_heading);
                temp_node_information[*node_f][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_f][2] -= CHANGE*cos(edge_heading);

                if(next_edge_heading - edge_heading >= 0) {
                    temp_node_information[*node_f][1] += CHANGE*tan(theta)*cos(edge_heading);
                    temp_node_information[*node_f][2] += CHANGE*tan(theta)*sin(edge_heading);
                }
                else {
                    temp_node_information[*node_f][1] -= CHANGE*tan(theta)*cos(edge_heading);
                    temp_node_information[*node_f][2] -= CHANGE*tan(theta)*sin(edge_heading);
                }
            }

            //2
            else if(node_s != cycle.begin() && node_s != last_node \
                    && directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes" \
                    && directed[G.GetEdgeIndex(*node_f, *next_node)] == "no") { // previousE: directed, nextE: nope
                // cout << "2" <<endl;
                previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
                        node_information[*node_s][1] - node_information[*previous_node][1]);
                theta = fabs(edge_heading - previous_edge_heading)/2;
                temp_node_information[*node_s][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_s][2] -= CHANGE*cos(edge_heading);
                temp_node_information[*node_f][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_f][2] -= CHANGE*cos(edge_heading);

                if(edge_heading - previous_edge_heading >= 0) { //LEFT
                    temp_node_information[*node_s][1] -= CHANGE*tan(theta)*cos(edge_heading);
                    temp_node_information[*node_s][2] -= CHANGE*tan(theta)*sin(edge_heading);
                }
                else {
                    temp_node_information[*node_s][1] += CHANGE*tan(theta)*cos(edge_heading);
                    temp_node_information[*node_s][2] += CHANGE*tan(theta)*sin(edge_heading);
                }

            }

            //3
            else if(node_s != cycle.begin() && node_s != last_node \
                    && directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes" \
                    && directed[G.GetEdgeIndex(*node_f, *next_node)] == "yes") { // previousE: directed, nextE: directed
                // cout << "3" <<endl;
                next_edge_heading = atan2(node_information[*next_node][2] - node_information[*node_f][2],
                        node_information[*next_node][1] - node_information[*node_f][1]);
                previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
                        node_information[*node_s][1] - node_information[*previous_node][1]);

                theta1 = fabs(edge_heading - previous_edge_heading)/2;
                theta2 = fabs(edge_heading - next_edge_heading)/2;

                temp_node_information[*node_s][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_s][2] -= CHANGE*cos(edge_heading);
                temp_node_information[*node_f][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_f][2] -= CHANGE*cos(edge_heading);

                if(edge_heading - previous_edge_heading >= 0) {//LEFT
                    temp_node_information[*node_s][1] -= CHANGE*tan(theta1)*cos(edge_heading);
                    temp_node_information[*node_s][2] -= CHANGE*tan(theta1)*sin(edge_heading);

                    if(next_edge_heading - edge_heading >= 0) {
                        temp_node_information[*node_f][1] += CHANGE*tan(theta2)*cos(edge_heading);
                        temp_node_information[*node_f][2] += CHANGE*tan(theta2)*sin(edge_heading);
                    }
                    else {
                        temp_node_information[*node_f][1] -= CHANGE*tan(theta2)*cos(edge_heading);
                        temp_node_information[*node_f][2] -= CHANGE*tan(theta2)*sin(edge_heading);
                    }
                }

                else {
                    temp_node_information[*node_s][1] += CHANGE*tan(theta1)*cos(edge_heading);
                    temp_node_information[*node_s][2] += CHANGE*tan(theta1)*sin(edge_heading);

                    if(next_edge_heading - edge_heading >= 0) {
                        temp_node_information[*node_f][1] += CHANGE*tan(theta2)*cos(edge_heading);
                        temp_node_information[*node_f][2] += CHANGE*tan(theta2)*sin(edge_heading);
                    }
                    else {
                        temp_node_information[*node_f][1] -= CHANGE*tan(theta2)*cos(edge_heading);
                        temp_node_information[*node_f][2] -= CHANGE*tan(theta2)*sin(edge_heading);
                    }
                }

            }

            //4
            else if(node_s == cycle.begin() && directed[G.GetEdgeIndex(*node_f, *next_node)] == "yes") {
                // cout << "4" <<endl;
                next_edge_heading = atan2(node_information[*next_node][2] - node_information[*node_f][2],
                        node_information[*next_node][1] - node_information[*node_f][1]);
                temp_node_information[*node_s][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_s][2] -= CHANGE*cos(edge_heading);
                temp_node_information[*node_f][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_f][2] -= CHANGE*cos(edge_heading);
                theta = fabs(next_edge_heading - edge_heading)/2;

                if(next_edge_heading - edge_heading >= 0) {
                    temp_node_information[*node_f][1] += CHANGE*tan(theta)*cos(edge_heading);
                    temp_node_information[*node_f][2] += CHANGE*tan(theta)*sin(edge_heading);
                }
                else {
                    temp_node_information[*node_f][1] -= CHANGE*tan(theta)*cos(edge_heading);
                    temp_node_information[*node_f][2] -= CHANGE*tan(theta)*sin(edge_heading);
                }

            }

            //5
            else if(node_s == last_node && directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes") {
                // cout << "5" <<endl;
                previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
                        node_information[*node_s][1] - node_information[*previous_node][1]);
                temp_node_information[*node_s][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_s][2] -= CHANGE*cos(edge_heading);
                temp_node_information[*node_f][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_f][2] -= CHANGE*cos(edge_heading);
                theta = fabs(edge_heading - previous_edge_heading)/2;

                if(edge_heading - previous_edge_heading >= 0) {
                    temp_node_information[*node_s][1] -= CHANGE*tan(theta)*cos(edge_heading);
                    temp_node_information[*node_s][2] -= CHANGE*tan(theta)*sin(edge_heading);
                }
                else {
                    temp_node_information[*node_s][1] += CHANGE*tan(theta)*cos(edge_heading);
                    temp_node_information[*node_s][2] += CHANGE*tan(theta)*sin(edge_heading);
                }

            }

            //6
            else {//NOTHING, only current edge is directed
                temp_node_information[*node_s][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_s][2] -= CHANGE*cos(edge_heading);
                temp_node_information[*node_f][1] += CHANGE*sin(edge_heading);
                temp_node_information[*node_f][2] -= CHANGE*cos(edge_heading);
            }



            // Stores way points
            edge_length = distance(temp_node_information[*node_s][1], temp_node_information[*node_s][2], temp_node_information[*node_f][1], temp_node_information[*node_f][2]);

            // START_NODE
            point_info.push_back(temp_node_information[*node_s][1]);// x coordinate
            point_info.push_back(temp_node_information[*node_s][2]);// y coordinate
            point_info.push_back(1);
            way_point.push_back(point_info);

            point_info.clear();//intialize

            number_of_pt = (int)(edge_length / interval);
//            number_of_pt = (int)(edge_length / edge_length);
            theta = atan2(temp_node_information[*node_f][2] - temp_node_information[*node_s][2],
                    temp_node_information[*node_f][1] - temp_node_information[*node_s][1]);//edge_heading
            for(int i = 1; i<=number_of_pt; i++) {
                ptx = round(temp_node_information[*node_s][1] + i*interval*cos(theta), 3);
                pty = round(temp_node_information[*node_s][2] + i*interval*sin(theta), 3);

                if(i == number_of_pt) {
                    if(ptx == temp_node_information[*node_f][1] and pty == temp_node_information[*node_f][2])// same point with the last one
                        break;
                }

                point_info.push_back(ptx);
                point_info.push_back(pty);
                point_info.push_back(0);
                way_point.push_back(point_info);
                point_info.clear();
            }

            if(node_s == last_node) {//last edge
                point_info.push_back(temp_node_information[*node_f][1]);
                point_info.push_back(temp_node_information[*node_f][2]);
                point_info.push_back(1);
                way_point.push_back(point_info);
                point_info.clear();
            }

            temp_node_information.clear();
        }


        else if(node_s != last_node && directed[G.GetEdgeIndex(*node_f, *next_node)] == "yes") {// Next Edge is DIRECTED, the current Edge is not directed

            next_edge_heading = atan2(node_information[*next_node][2] - node_information[*node_f][2],
                    node_information[*next_node][1] - node_information[*node_f][1]);


            temp_node_information[*node_f][1] += CHANGE*sin(next_edge_heading);
            temp_node_information[*node_f][2] -= CHANGE*cos(next_edge_heading);


            if(node_s!= cycle.begin() && directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes") {// + Previous EDGE is DIRECTED
                previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
                        node_information[*node_s][1] - node_information[*previous_node][1]);
                temp_node_information[*node_s][1] += CHANGE*sin(previous_edge_heading);
                temp_node_information[*node_s][2] -= CHANGE*cos(previous_edge_heading);
            }

            // Store way points
            edge_length = distance(temp_node_information[*node_s][1], temp_node_information[*node_s][2], temp_node_information[*node_f][1], temp_node_information[*node_f][2]);
            point_info.push_back(temp_node_information[*node_s][1]);
            point_info.push_back(temp_node_information[*node_s][2]);
            point_info.push_back(1);
            way_point.push_back(point_info);

            point_info.clear();//intialize


            number_of_pt = (int)(edge_length / interval);
            theta = atan2(temp_node_information[*node_f][2] - temp_node_information[*node_s][2],
                    temp_node_information[*node_f][1] - temp_node_information[*node_s][1]);//edge_heading
            for(int i = 1; i<=number_of_pt; i++) {
                ptx = round(temp_node_information[*node_s][1] + i*interval*cos(theta), 3);
                pty = round(temp_node_information[*node_s][2] + i*interval*sin(theta), 3);

                if(i == number_of_pt) {
                    if(ptx == temp_node_information[*node_f][1] and pty == temp_node_information[*node_f][2])// same point with the last one
                        break;
                }

                point_info.push_back(ptx);
                point_info.push_back(pty);
                point_info.push_back(0);
                way_point.push_back(point_info);
                point_info.clear();
            }

            temp_node_information.clear();
        }



        else if(node_s != cycle.begin() && directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes") {//Previous Edge is directed, but not current and next one

            previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
                    node_information[*node_s][1] - node_information[*previous_node][1]);

            temp_node_information[*node_s][1] += CHANGE*sin(previous_edge_heading);
            temp_node_information[*node_s][2] -= CHANGE*cos(previous_edge_heading);

            // Store way points
            edge_length = distance(temp_node_information[*node_s][1], temp_node_information[*node_s][2], temp_node_information[*node_f][1], temp_node_information[*node_f][2]);
            point_info.push_back(temp_node_information[*node_s][1]);
            point_info.push_back(temp_node_information[*node_s][2]);
            point_info.push_back(1);
            way_point.push_back(point_info);

            point_info.clear();//intialize

            number_of_pt = (int)(edge_length / interval);
            theta = atan2(temp_node_information[*node_f][2] - temp_node_information[*node_s][2],
                    temp_node_information[*node_f][1] - temp_node_information[*node_s][1]);//edge_heading
            for(int i = 1; i<=number_of_pt; i++) {
                ptx = round(temp_node_information[*node_s][1] + i*interval*cos(theta), 3);
                pty = round(temp_node_information[*node_s][2] + i*interval*sin(theta), 3);

                if(i == number_of_pt) {
                    if(ptx == temp_node_information[*node_f][1] and pty == temp_node_information[*node_f][2])// same point with the last one
                        break;
                }

                point_info.push_back(ptx);
                point_info.push_back(pty);
                point_info.push_back(0);
                way_point.push_back(point_info);
                point_info.clear();
            }

            if(node_s == last_node) {//if the current edge is the last one
                point_info.push_back(temp_node_information[*node_f][1]);
                point_info.push_back(temp_node_information[*node_f][2]);
                point_info.push_back(1);
                way_point.push_back(point_info);
                point_info.clear();
            }

            temp_node_information.clear();
        }


        else {//all edges are not 'directed'

            edge_length = distance(node_information[*node_s][1], node_information[*node_s][2], node_information[*node_f][1], node_information[*node_f][2]);
            point_info.push_back(node_information[*node_s][1]);
            point_info.push_back(node_information[*node_s][2]);
            point_info.push_back(1);
            way_point.push_back(point_info);

            point_info.clear();//intialize

            number_of_pt = (int)(edge_length / interval);
            theta = atan2(temp_node_information[*node_f][2] - temp_node_information[*node_s][2],
                    temp_node_information[*node_f][1] - temp_node_information[*node_s][1]);//edge_heading
            for(int i = 1; i<=number_of_pt; i++) {
                ptx = round(temp_node_information[*node_s][1] + i*interval*cos(theta), 3);
                pty = round(temp_node_information[*node_s][2] + i*interval*sin(theta), 3);

                if(i == number_of_pt) {
                    if(ptx == temp_node_information[*node_f][1] and pty == temp_node_information[*node_f][2])// same point with the last one
                        break;
                }

                point_info.push_back(ptx);
                point_info.push_back(pty);
                point_info.push_back(0);
                way_point.push_back(point_info);
                point_info.clear();
            }

            if(node_s == last_node) {
                point_info.push_back(node_information[*node_f][1]);
                point_info.push_back(node_information[*node_f][2]);
                point_info.push_back(1);
                way_point.push_back(point_info);
                point_info.clear();
            }
            temp_node_information.clear();
        }
    }

    return way_point;
}


// This function's input is a raw set of waypoints and it applies radius(?) spline to the nodes execpt the start and last nodes
vector< vector<float> > GlobalPlanner::ms_spline(vector< vector<float> > way_point, const float interval, const int spline_weight) {


    int idx_counting = 0;// Get indices
    vector<int> node_idx;
    for(int i = 0; i < way_point.size(); i++) {
        if(way_point[i][2] == 1) {
            node_idx.push_back(i);
            // cout << "i is " << i << endl;
        }
    }
    radius = 0;// initialize
    rear_edge_heading = 0;// rear_edge heading <previous node, current node>
    _rear_edge_heading = 0;// rear edge heading <current node, previous node> != rear_edge_heading
    front_edge_heading = 0;// front edge heading <current node, next node>
    node_angle = 0;// angle of <previous node, current node, next node>

    pair<double, double> center_point;// center point of the circle we defined
    pair<double, double> move_point;// new points which are moved from the original one.

    spline_theta = 0;// angle of the center point

    dis = 0;
    right = true;// it checks, in the current vertex, finds out  a vehicle is turning right or left.

    number_of_spline_point = 0;// how many points we have to do 'spline'

    for(int point_idx = 0; point_idx < way_point.size()-1; point_idx++) {

        if(point_idx != 0 && way_point[point_idx][2] == 1) {//EACH NODE

            //spline_constraint: 181011
            idx_counting++;
            double temp_dis1 = distance(way_point[node_idx[idx_counting]][0], way_point[node_idx[idx_counting]][1], way_point[node_idx[idx_counting-1]][0], way_point[node_idx[idx_counting-1]][1]);
            double temp_dis2 = distance(way_point[node_idx[idx_counting]][0], way_point[node_idx[idx_counting]][1], way_point[node_idx[idx_counting+1]][0], way_point[node_idx[idx_counting+1]][1]);
            int temp_spline_weight = spline_weight;
            bool ben = true;

            if(idx_counting == node_idx.size()-1)
                ben = false;
            while(ben) {
                if(temp_spline_weight*interval > temp_dis1*0.4 || temp_spline_weight*interval > temp_dis2*0.4)
                    temp_spline_weight -=1;
                else
                    break;
            }



            rear_edge_heading = atan2(way_point[point_idx][1] - way_point[point_idx-1][1], way_point[point_idx][0] - way_point[point_idx-1][0]);
            _rear_edge_heading = atan2(way_point[point_idx-1][1] - way_point[point_idx][1], way_point[point_idx-1][0] - way_point[point_idx][0]);
            front_edge_heading = atan2(way_point[point_idx+1][1] - way_point[point_idx][1], way_point[point_idx+1][0] - way_point[point_idx][0]);

            // if( fabs( front_edge_heading - rear_edge_heading) < 0.001)
            //     continue;	// A vehicle is going to be straight which means this node doesn't need to be splined
            // else if( fabs(front_edge_heading - _rear_edge_heading) > M_PI)
            //     node_angle = fabs(2*M_PI - fabs(front_edge_heading - _rear_edge_heading));
            // else
            //     node_angle = fabs(front_edge_heading - _rear_edge_heading);

            if( fabs(front_edge_heading - _rear_edge_heading) > M_PI)
                node_angle = fabs(2*M_PI - fabs(front_edge_heading - _rear_edge_heading));
            else
                node_angle = fabs(front_edge_heading - _rear_edge_heading);
            
            if(node_angle < 0.01 || fabs(node_angle - M_PI) < 0.001) // node_angle ~=180
                continue;

            spline_theta = M_PI - node_angle;
            radius = temp_spline_weight * interval * tan(node_angle/2);

            //Check that, car is going to turn right or left, this is important because we have to know where the center point are
            if (rear_edge_heading >=0 && front_edge_heading >= 0) {//++
                // cout << " + + " << endl;
                if (front_edge_heading - rear_edge_heading > 0)
                    right = false;
                else
                    right = true;
            }
            else if (rear_edge_heading >= 0 && front_edge_heading <=0) { //+-
                // cout << " + - " << endl;
                if ((M_PI + front_edge_heading) - rear_edge_heading < 0)
                    right = false;
                else
                    right = true;
            }

            else if (rear_edge_heading <= 0 && front_edge_heading >= 0) {//-+
                // cout << " - + " << endl;
                if ((M_PI + rear_edge_heading) - front_edge_heading > 0)
                    right = false;
                else
                    right = true;
            }

            else {//--
                // cout << " - - " << endl;
                if (front_edge_heading - rear_edge_heading > 0)
                    right = false;
                else
                    right = true;
            }

            if (right) {//RIGHT
                // 	center_point.first = way_point[point_idx][0] - spline_weight*interval*cos(rear_edge_heading) + radius*cos(M_PI/2 - rear_edge_heading);// x
                // 	center_point.second = way_point[point_idx][1] - spline_weight*interval*sin(rear_edge_heading) - radius*sin(M_PI/2 - rear_edge_heading);// y
                center_point.first = way_point[point_idx][0] - temp_spline_weight*interval*cos(rear_edge_heading) + radius*cos(M_PI/2 - rear_edge_heading);// x
                center_point.second = way_point[point_idx][1] - temp_spline_weight*interval*sin(rear_edge_heading) - radius*sin(M_PI/2 - rear_edge_heading);// y

                count1 = 0;
                count2 = 0;
                length1 = 0;
                length2 = 0;
                while(true) {// Check how many points we make SPLINE on a left-side

                    length1 = distance(way_point[point_idx][0], way_point[point_idx][1], way_point[point_idx-count1-1][0], way_point[point_idx-count1-1][1]);
                    if (length1 > radius/tan(node_angle/2) )// || length1 > temp_distance )
                        break;

                    count1 +=1;
                }
                while(true) {// Check how many points we make SPLINE on a right-side

                    length2 = distance(way_point[point_idx][0], way_point[point_idx][1], way_point[point_idx+count2+1][0], way_point[point_idx+count2+1][1]);
                    if (length2 > radius/tan(node_angle/2) )// || length2 > temp_distance )
                        break;

                    count2 +=1;
                }

                number_of_spline_point = count1 + count2 + 1;

                // using NAEBOONJUM
                for(int i = 0; i< number_of_spline_point; i++) {// point idx -> node_idx

                    dis = distance(way_point[point_idx-count1+i][0], way_point[point_idx-count1+i][1], center_point.first, center_point.second);
                    double n = dis - radius;

                    move_point.first = (radius * way_point[point_idx - count1 + i][0] + n * center_point.first)/(radius + n);
                    move_point.second = (radius * way_point[point_idx - count1 + i][1] + n * center_point.second)/(radius + n);
                    way_point[point_idx - count1 + i][0] = move_point.first;
                    way_point[point_idx - count1 + i][1] = move_point.second;
                }
            }

            else {//LEFT
                // center_point.first = way_point[point_idx][0] - spline_weight*interval*cos(rear_edge_heading) - radius*cos(M_PI/2 - rear_edge_heading);
                // center_point.second = way_point[point_idx][1] - spline_weight*interval*sin(rear_edge_heading) + radius*sin(M_PI/2 - rear_edge_heading);
                center_point.first = way_point[point_idx][0] - temp_spline_weight*interval*cos(rear_edge_heading) - radius*cos(M_PI/2 - rear_edge_heading);
                center_point.second = way_point[point_idx][1] - temp_spline_weight*interval*sin(rear_edge_heading) + radius*sin(M_PI/2 - rear_edge_heading);
                count1 = 0;
                count2 = 0;
                length1 = 0;
                length2 = 0;

                while(true) {
                    length1 = distance(way_point[point_idx][0], way_point[point_idx][1], way_point[point_idx-count1-1][0], way_point[point_idx-count1-1][1]);
                    if (length1 > radius/tan(node_angle/2))// || length1 > temp_distance )
                        break;

                    count1 +=1;
                }
                while(true) {
                    length2 = distance(way_point[point_idx][0], way_point[point_idx][1], way_point[point_idx+count2+1][0], way_point[point_idx+count2+1][1]);
                    if (length2 > radius/tan(node_angle/2))// || length2 > temp_distance )
                        break;

                    count2 +=1;
                }

                number_of_spline_point = count1 + count2 + 1;

                for(int i = 0; i< number_of_spline_point; i++) {

                    dis = distance(way_point[point_idx-count1 + i][0], way_point[point_idx-count1 + i][1], center_point.first, center_point.second);
                    double n = dis - radius;

                    move_point.first = (radius * way_point[point_idx - count1 + i][0] + n * center_point.first)/(radius + n);
                    move_point.second = (radius * way_point[point_idx - count1 + i][1] + n * center_point.second)/(radius + n);
                    way_point[point_idx - count1 + i][0] = move_point.first;/////////////////////IT HAS TO BE IMPROVED
                    way_point[point_idx - count1 + i][1] = move_point.second;
                }
            }

        }
    }


    return way_point;// Returns the splined set of waypoints
}



GlobalPlanner::g_info GlobalPlanner::ReadWeightedGraph(string filename) {//This function reads a graph_map.txt for initialization
    //Please see Graph.h for a description of the interface

    file.open(filename.c_str());//open a file as an string,
    getline(file, s);// get an array of the sentence in a file
    stringstream ss(s);//it is very useful to get data we want from the string array
    ss >> vertices;// #VERTICES

    getline(file, s);
    ss.str(s);//copy the string s to the stream ss
    ss.clear();
    ss >> edges;// #EDGES

    if (DEBUG) {
        cout << "VERTICES: " << vertices << endl;
        cout << "EDGES: " << edges << endl;
    }

    //Stores node_information, node_number and a position
    for(int i = 0; i < vertices; i++) {
        getline(file, s);
        ss.str(s);
        ss.clear();
        float ReadNode, ReadNode_x, ReadNode_y;
        ss >> ReadNode >> ReadNode_x >> ReadNode_y;
        node_info_set.push_back(ReadNode);
        node_info_set.push_back(ReadNode_x);
        node_info_set.push_back(ReadNode_y);
        node_info.push_back(node_info_set);

        node_info_set.clear();
    }

    Graph graph(vertices);// define a graph having #VERTICES, n
    graph.initialize_vertices();// add the vertices name(number) into the list INITIALIZE
    vector<double> cost(edges);// define a vector named 'cost(m)', each cost has weight. In this case, it means 'DISTANCE' b/w adjacent nodes
    vector<string> directed(edges);
    vector<bool> park(edges);// Parking edge

    // Initializae the graph
    for(int i = 0; i < edges; i++) {
        getline(file, s);
        ss.str(s);
        ss.clear();
        int u, v;
        double c;// Cost (= Distance)
        string dir;
        bool par;
        ss >> u >> v >> dir >> par;

        // Calcuate the cost for this
        c = (double) distance(node_info[u][1], node_info[u][2], node_info[v][1], node_info[v][2]);
        graph.AddEdge(u, v);// add an edge(u, v)
        cost[graph.GetEdgeIndex(u, v)] = c;// append a cost of the edge using GetEdgeIndex
        directed[graph.GetEdgeIndex(u, v)] = dir;
        park[graph.GetEdgeIndex(u, v)] = par;
        if (par == true)
            er_count += 1;
    }
    eo_count = edges - er_count;
    cout << "er: " << er_count << " eo: " << eo_count << endl;
    file.close();

    //DEBUGGGGGGGGGGGGGGG################################### & A TEST,,
    //This test is for checking whether 'RemoveEdge' is worked well.

    // cout << "info" <<endl;
    // graph.GraphInfo(graph);
    // first exp
    // graph.RemoveEdge(0, 6);
    // graph.RemoveEdge(2, 6);
    //second exp
    // graph.RemoveEdge(0, 1);
    // third exp
    // graph.RemoveEdge(11, 12);
    // graph.RemoveEdge(12, 8);
    // graph.RemoveEdge(10, 11);
    // graph.RemoveEdge(10, 9);
    // graph.RemoveEdge(8, 9);
    // graph.RemoveEdge(8, 7);
    // graph.RemoveEdge(7, 0);
    // cout << "ha!";
    // graph.GraphInfo(graph);

    // for(int i = 0; i<graph.GetVertices().size();i++){
    // cout << graph.GetVertices()[i] << ", ";
    // }

    // graph.RemoveVertex(6);
    // graph.RemoveEdge(1, 2);
    // graph.RemoveEdge(0, 1);// Experiment!
    // graph.AddEdge(0, 1);
    // cout << "After removing edge(12, 8)" <<endl;
    // cout << "edges!" << endl;
    // for(int i= 0; i< graph.edges.size(); i++)
    // {
    // 	cout << graph.edges[i].first << ", " << graph.edges[i].second << endl;
    // }

    // cout << "edgeIndex!" << endl;
    // for(int i= 0; i< graph.edgeIndex.size(); i++)
    // {
    // 	cout << i << ": ";
    // 	for(int j = 0; j < graph.edgeIndex[i].size(); j++)
    // 	{
    // 		cout << graph.edgeIndex[i][j] << ", ";
    // 	}
    // 	cout << endl;
    // }
    //##########################################################

    graph_information._node_information = node_info;
    graph_information._G = graph;
    graph_information._cost = cost;
    graph_information._directed = directed;
    graph_information._park = park;

    return graph_information;
}


GlobalPlanner::g_info GlobalPlanner::RePlanning(g_info Graph) {
    //Removing edges we've passed using EXPLORED term
}


bool GlobalPlanner::FILTER1(const vector< vector<int> > never_passed, const list<int> cycle) {

    vector<int> never_passed_sequence;// Filtering awkward combinations
    bool DECISION;
    for(int i = 0; i < never_passed.size(); i++) {

        for(int j = 0; j < cycle.size() - 2; j++) {
            int size = 0;
            list<int>::const_iterator iter_ = cycle.begin();
            std::advance(iter_, j);
            never_passed_sequence.push_back(*iter_);
            std::advance(iter_, 1);//move to next iter
            never_passed_sequence.push_back(*iter_);
            std::advance(iter_, 1);//move to next iter
            never_passed_sequence.push_back(*iter_);

            if(never_passed[i] == never_passed_sequence) {// Check that there is an awkward path
                DECISION = false;
                return DECISION;
            }

            never_passed_sequence.clear();
        }
    }

    DECISION = true;
    return DECISION;
}


void GlobalPlanner::CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg)  {
    // cout<<"Get LocalizationData"<<endl;
    m_car.x = msg->data.at(0);
    m_car.y = msg->data.at(1);
    m_car.th = msg->data.at(2);
    m_car.vel = msg->data.at(3);
}

void GlobalPlanner::CallbackCarIdx(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    m_car.waypt_idx = msg->data.at(0);
}

void GlobalPlanner::CallbackGlobalPath(const std_msgs::Bool::ConstPtr& msg) {
    // cout << "CALL!" << endl;
    if (msg->data && m_flag)
        main();
        
}

}

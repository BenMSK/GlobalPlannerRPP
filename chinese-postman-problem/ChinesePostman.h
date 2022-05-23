#include "./Minimum-Cost-Perfect-Matching/Matching.h"
#include "./Minimum-Cost-Perfect-Matching/Graph.h"
#include "Dijkstra.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <iterator>
#include <math.h>
#include <string>
#include <sstream>
#define M_PI 3.14159265358979323846
// #include <tuple>

using namespace std;


// typedef get_point_data{
// 	string csvdata1;
// 	string csvdata2;
// 	string csvdata3;
// } CSVreader;

bool Connected(const Graph & G)
{
    vector<bool> visited(G.GetNumVertices(), false);
    list<int> L;
    
    int n = 0;
    L.push_back(0);
    while(not L.empty())
    {
        int u = L.back();
        L.pop_back();
        if(visited[u]) continue;
        
        visited[u] = true;
        n++;
        
        for(list<int>::const_iterator it = G.AdjList(u).begin(); it != G.AdjList(u).end(); it++)
		{
			int v = *it;
		    L.push_back(v);
		}
    }
   
    return G.GetNumVertices() == n;
}

/*
Solves the chinese postman problem
returns a pair containing a list and a double
the list is the sequence of vertices in the solution
the double is the solution cost
*/

// it is using Fluery's Algorithm
pair< list<int>, double > ChinesePostman(const Graph & G, const vector<double> & cost, const vector< vector<float> > & node_information,
										 const float car_pos_x, const float car_pos_y, const float car_pos_heading)//ChinesePostman(Graph, cost), returns < list, double >
{
	srand ( time(NULL) );// Make it, random

	bool again = true;
	int minsoo = 0;
	// Get the starting point
	vector<int> heading_reasonable_node;
	int source_node;

	for(int i = 0; i<G.GetNumVertices();i++)
	{	float heading_diff = atan2(node_information[i][2] - car_pos_y, node_information[i][1] - car_pos_x);
		float c_h = 0;
		if(fabs(car_pos_heading - heading_diff) > M_PI)
		{	c_h = fabs(2*M_PI - fabs(car_pos_heading - heading_diff));	}
		else
		{	c_h = fabs(car_pos_heading - heading_diff);   }

		cout << "car_pos_heading - heading_diff :" << c_h <<endl;
		if( c_h < (M_PI/2) )
		{
			cout << (M_PI/2);
			cout << i << endl;
			heading_reasonable_node.push_back(i);
		} 
	}
	for(vector<int>::iterator iter = heading_reasonable_node.begin(); iter != heading_reasonable_node.end(); iter++)
	{
		cout << *iter << endl;
	}
	float dis = 9999;
	for(vector<int>::iterator node = heading_reasonable_node.begin(); node != heading_reasonable_node.end();node++)
	{
		float _dis = sqrt( pow(node_information[*node][1] - car_pos_x,2) + pow(node_information[*node][2] - car_pos_y,2) );
		cout << *node << ": " << _dis<<endl;
		if(dis >= _dis)
		{	dis = _dis;
			source_node = *node;}
	}
	cout << "Source Node: ";
	cout << source_node << endl;
	cout << endl;


	// Make the reasonable edge set for starts
	list<int> adj_to_source_node = G.AdjList(source_node);

	int next_node;
	float min_heading = 9999;
	float temp_min_heading = 0;
	float diff = 0;

	vector<int> reasonable_start_edge;
	for(list<int>::iterator node = adj_to_source_node.begin(); node != adj_to_source_node.end(); node++)
	{
		float edge_heading = atan2(node_information[*node][2] - node_information[source_node][2], node_information[*node][1] - node_information[source_node][1]);
		cout << "node: " << *node <<endl;
		cout << "car_pos_heading:" << car_pos_heading<<endl;
		cout << "edge_Heading: " << edge_heading <<endl;
		if(fabs(car_pos_heading - edge_heading) > M_PI)
		{	
			diff = fabs(2*M_PI - fabs(car_pos_heading - edge_heading));
			cout << "1"<<endl;
			cout << "diff: " << diff<<endl;
		}
		else
		{
			diff = fabs(car_pos_heading - edge_heading);
			cout << "2"<<endl;
			cout << "diff: " << diff<<endl;
		}
		cout << "diff: " << diff << endl;
		cout << endl;
		// float diff = fabs(car_pos_heading - edge_heading);
		// cout << "heading diff of node, " << *node <<": "<<diff<<endl;
		if(diff <= min_heading)
		{
			min_heading = diff;
			next_node = *node;
		}
	}

	reasonable_start_edge.push_back(source_node);
	reasonable_start_edge.push_back(next_node);
	// cout << adj_to_source_node[]

	cout << reasonable_start_edge[0] << reasonable_start_edge[1] <<endl;




	while(again)
	{
		minsoo++;

	//Check if the graph if connected
	if(not Connected(G))
		throw "Error: Graph is not connected";// if it is satisfied, the execptional situation, it "throw", which means this graph is not completed

	//Build adjacency lists using edges in the graph
	//Sequence Container -> vector, list, deque
	//vector.push_back(값): 맨 뒤에 값 추가
	//vector<type>::iterator:맴버 타입으로 정의되어이T고, vector.begin(), vector.end()를 리턴
	//vector.insert(location, element): in the front of the location, add the element
	//vector.erase <-> vec.insert
	//list의 경우는 양방향 견결 구조를 가진 자료형, list에서는 특정한 위치의 원소를 가리킬 수 없다.
	//list: 맨 R트이 아닌 중간에 원소들을 추가하거나 제거하는 일을 많이하고 원소를 순차적으로 접근한다면, 리스트 사용.
	//deque -> it is more fast than vector but using lots of memories
	vector< list<int> > A(G.GetNumVertices(), list<int>());//A = (empty_list, empty_list, ....)#vertices
	for(int u = 0; u < G.GetNumVertices(); u++)
	    A[u] = G.AdjList(u);//Returns the adjacency list of a vertex
	//That means, substitutes adjacency list of the node, u from G into the A with index, u
	// A = [adjacency vertices of node 0, adjacency vertices of node 1, ... ]



	vector< vector<int> > never_passed;// The sequence of that code
	vector<int> temp;// just saying

	//Find vertices with odd degree & put into a vector, odd
	vector<int> odd;
	for(int u = 0; u < G.GetNumVertices(); u++)
		if(A[u].size() % 2)
			odd.push_back(u);//put the odd vertices into the vector, 'odd', it contains odd nodes' name into the list, odd


    //If there are odd degree vertices
	if(not odd.empty())
	{
		//Create a graph with the odd degree vertices
		//In other words, makes an augmented graph
		Graph O(odd.size());//it makes a graph which only has vertices having odd degree
							//This is beacuse we have to put additional edges to the graph like you did in Python
							//Total number of vertices of O is # of odd_degree
		// Make a complete graph, O
		for(int u = 0; u < (int)odd.size(); u++)
			for(int v = u+1; v < (int)odd.size(); v++)
				O.AddEdge(u, v);
        vector<double> costO(O.GetNumEdges());// Make the costO vector which has empty rooms of number of edges

		///////////////////////////////////////////////////////////////
        /// Find the shortest paths between all odd degree vertices ///
		vector< vector<int> > shortestPath(O.GetNumVertices());
		for(int u = 0; u < (int)odd.size(); u++)
		{
			pair< vector<int>, vector<double> > sp = Dijkstra(G, odd[u], cost);//Dijkstra(Graph, origin, cost)
							//Dijkstra returns a pair (father, pathCost)
			shortestPath[u] = sp.first ;//father means shortest path of that vertex
			
			//The cost of an edge uv in O will be the cost of the corresponding shortest path in G
			for(int v = 0; v < (int)odd.size(); v++)
			    if(v != u)
    			    costO[ O.GetEdgeIndex(u, v) ] = sp.second[odd[v]];
		}

		////////////////////////////////
	    //Find the minimum cost perfect matching of the graph of the odd degree vertices
	    Matching M(O);//O: a graph
	    pair< list<int>, double > p = M.SolveMinimumCostPerfectMatching(costO);//it will return perfect matching in a graph
	    list<int> matching = p.first;

	    //If an edge uv is in the matching, the edges in the shortest path from u to v should be duplicated in G, i.o. additional edges b/w odd_degree vertices
	    for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
	    {
		    pair<int, int> p = O.GetEdge(*it);
		    int u = p.first, v = odd[p.second];

		    int w = shortestPath[u][v];
			// cout << "\nw is " << w << "\n";
		    while(w != -1)
		    {
		        A[w].push_back(v);
		        A[v].push_back(w);

				// contains vectors into 'never_passed'
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

		// DEBUG
		// cout << 'DEBUGnever_passed' << endl;
		// for(int i = 0; i<never_passed.size(); i++)
		// {
		// 	for(int j = 0; j<3; j++)
		// 	{
		// 		cout<<never_passed[i][j]<< " ";
		// 	}
		// 	cout<<endl;
		// // cout << itrr->first << " " << itrr->second << endl;
		// // cout << std::get<0>(*itrr) << " " << get<1>(*itrr) << " " << get<2>(*itrr);
		// }
		// cout << endl;
	}

	////////////////////////////////////////////////////////////////////////////
	/////// Find an Eulerian cycle in the graph implied by A

	list<int> cycle;//cycle -> EulerTour
	
	//This is to keep track of how many times we can traverse an edge
	vector<int> traversed(G.GetNumEdges(), 0);// size: NumEdges, values: 0

	// 'A' contains an adjacent edges of the vertex
	for(int u = 0; u < G.GetNumVertices(); u++)//of every vertex
	{	//u -> each vertex
		for(list<int>::iterator it = A[u].begin(); it != A[u].end(); it++)
		{
			int v = *it;//substitue the value of u to v
			
			//we do this so that the edge is not counted twice
			if(v < u) continue;

			traversed[G.GetEdgeIndex(u, v)]++;//지나간 edge를 몇번 지나갔는지 계산
		}
	}


	// Make a cycle
	//chcle.push_back(source_node)
	cycle.push_back(source_node);//append zero into a cycle, that means it is going to start at a source, 0
	list<int>::iterator itp = cycle.begin();//define an iterator, itp. Now itp has only one element, source point
	double obj = 0;//calculate the cost

	while(itp != cycle.end())
	{
		//Let u be the current vertex in the cycle, starting at the first
		int count = 0;
		int u = *itp;
		int randomN;//ms

		// cout << "u is " << u <<"\n";
		list<int>::iterator jtp = itp;
		jtp++;

		//if there are non-traversed edges incident to u, find a subcycle starting at u
		//replace u in the cycle by the subcycle
		//A = {(1, 3), (0, 2), (1, 3, 5, 3), (0, 2, 4, 2), (3, 5), (2, 4)}

		while(not A[u].empty())//This loop is running untill the A is empty.
		{

			count++;
			// cout << "count: " << count << "\n";
			// cout << "==========Debug, be not run==========\n";
			while(not A[u].empty() and traversed[ G.GetEdgeIndex(u, A[u].back()) ] == 0)//list.back() -> a last element of a list
				// cout << "A[u].pop_back1\n";
				// check that edge is already passed
				{A[u].pop_back();//remove the last element of the container
				// cout << "A[u].pop_back2\n";
				}		
			// DEBUG, msk
			// Check A
			// list<int> _msk;
			// for(vector< list<int> >::iterator i = A.begin(); i != A.end(); i++){
			// 	_msk = *i;
			// 	for(list<int>::iterator j = _msk.begin(); j != _msk.end(); j++)
			// 	{
			// 		cout << " element of A now is " << *j << "\n";
			// 	}
			// 	cout << "\n==============================\n";
			// }
			// cout << "Above numbers are from 'A'\n";
			// Each value of A  means vertices of current value has

			if(not A[u].empty())
			{
				// cout << "A[u]size: " << A[u].size() << "\n";
				randomN = rand() % A[u].size();//pick random index
				// cout << "random!: " <<randomN << "\n";

				std::list<int>::iterator it = A[u].begin();//pick randomly choose
				std::advance(it, randomN);
				int v = *it;
				
				int previous_u = u;
				// int v = A[u].back();
				// cout << "v is " << v << "\n";
				// A[u].pop_back();
				// A[u].remove(v);
				A[u].erase(it);
				
				// cout<<"yo ";
				cycle.insert(jtp, v);
				// cout<<"ho\n";
				traversed[G.GetEdgeIndex(u, v)]--;//check that we visited the edge(u, v)
		        obj += cost[ G.GetEdgeIndex(u, v) ];//Add cost of the edge(u, v) to obj
				u = v;//change the vertex

				// if(traversed[G.GetEdgeIndex(u, previous_u)] == 0)
				// {
					// Get an index(wow)

				for(list<int>::iterator wow = A[u].begin(); wow != A[u].end(); wow++){
					// cout << *wow;
					if(*wow == previous_u)
					{
						A[u].erase(wow);
						break;
					}
				}
			}

			// DEBUG, list cycle
			// cout << "cycle is ";
			// for(list<int>::iterator q = cycle.begin(); q != cycle.end(); q++){
			// 	cout << *q << " ";
			// }
			// cout << "\n";


			// DEBUG, msk
			// list<int> msk;
			// for(vector< list<int> >::iterator i = A.begin(); i != A.end(); i++){
			// 	msk = *i;
			// 	for(list<int>::iterator j = msk.begin(); j != msk.end(); j++)
			// 	{
			// 		cout << " element of A now is " << *j << "\n";
			// 	}
			// 	cout << "\n\n";
			// }
			// cout << "Above numbers are from 'A'\n";
			// Each value of A  means vertices of current value has

		}

		//go to the next vertex in the cycle and do the same
		itp++;
	}


	// first set: Compare reasonable start edge;
	vector<int> cycle_start_edge;
	list<int>::iterator cycle_start_edge_i = cycle.begin();
	cycle_start_edge.push_back(*cycle_start_edge_i);
	std::advance(cycle_start_edge_i, 1);
	cycle_start_edge.push_back(*cycle_start_edge_i);

	if(reasonable_start_edge != cycle_start_edge){	continue;	}
	// vector<int> cycle_vector{ std::begin(cycle), std::end(cycle) };//change the cycle(list) to the cycle(vector)
	// Compare it!, These lines compare that sequences we've got with the Tour
	vector<int> never_passed_sequence;
	// vector<int> never_passed_sequence_;
	
	bool running = true;
	bool satisfied = true;

	// int i, j = 0;
	for(int i = 0; i < never_passed.size(); i++)
	{

		// never_passed_sequence.push_back(never_passed[i][0]);//never_passed is a vector containing the sequences
		// never_passed_sequence.push_back(never_passed[i][1]);
		// never_passed_sequence.push_back(never_passed[i][2]);
		for(int j = 0; j < cycle.size() - 2; j++)
		{
			// list<int> iter_ = cycle.begin();
			// for(list<int>::iterator s = cycle.begin(); s != cycle.end(); s++)
			// 	{cout << *s << " ";
			// 	cout << endl;}
			// cout << "j: " << j << endl;
			if(j == 30)
			{
				return pair< list<int>, double >(cycle, obj);
			}
			// cout << "cycle_size: " << cycle.size() << endl;
			int size = 0;
			list<int>::iterator iter_ = cycle.begin();
			std::advance(iter_, j);
			never_passed_sequence.push_back(*iter_);
			std::advance(iter_, 1);
			never_passed_sequence.push_back(*iter_);
			std::advance(iter_, 1);
			never_passed_sequence.push_back(*iter_);


			// cout << "DEBUG" << endl;
			// for(vector<int>::iterator itr = never_passed[i].begin(); itr != never_passed[i].end(); itr++)
			// {
			// 	cout << *itr << " ";
			// }
			// cout << "\nCheck the tour\n";
			// for(vector<int>::iterator  itr = never_passed_sequence.begin(); itr != never_passed_sequence.end(); itr++)
			// {
			// 	cout << *itr << " ";
			// }
			// cout << endl;

			if(never_passed[i] == never_passed_sequence)// Check that there is an akward path
			{
				cout << "Got ya"<< endl;// Found out something wrong.
				running = false;
				break;
			}

			never_passed_sequence.clear();
			// cout << endl;
		}

		if(not running)
		{
			cycle.clear();
			never_passed_sequence.clear();

			satisfied = false;
			break;
		}

	}

	if(satisfied)
	{
		cout << "how many it search, " << minsoo << endl;
		return pair< list<int>, double >(cycle, obj);
	}

	}

}



//ms adds
float distance(float point1_x, float point1_y, float point2_x, float point2_y)
{
	return sqrt(pow(point1_x - point2_x, 2) + pow(point1_y - point2_y, 2));
}

float round(float value, int pos)
{	float temp;
	temp = value * pow(10, pos);
	temp = floor(temp + 0.5);
	temp *= pow(10, -pos);
	return temp;	}
// CYCLE -> (1, 2, 3, 1)...
void write_down(const Graph & G, list<int> cycle, vector<string> directed, const vector< vector<float> > & node_information)// Write down the csv file for storing way points 
{
	float _dis;
	float interval = 1;
	// char yes[] = "yes";
	// char no[] = "no";
	// string yes = "yes";
	// string no = "no";
	vector< vector<float> > way_point;// Contains the waypoint_set (x, y, label)
	vector< vector<float> > temp_node_information;
	// for(list<int>::iterator iter = cycle.begin();iter != cycle.end(); iter++)
	// {
	// 	cout << node_information[*iter][0] << endl;
	// 	cout << node_information[*iter][1] << endl;
	// 	cout << node_information[*iter][2] << endl;
	// 	_dis = distance(node_information[*iter][1], node_information[*iter][2], node_information[*iter+1][1], node_information[*iter+1][2]);
	// 	std::advance(iter, 0);
	// 	cout << "DANCING QUEEN: " << _dis;
	// 	break;
	// }
	// // cout << directed[0] << endl;
	// cout << directed[1] << endl;
	// cout << directed[G.GetEdgeIndex(6, 0)] <<endl;
	// cout << directed[G.GetEdgeIndex(1, 2)] <<endl;


	// Let's get it
	// Super complicated part that calculates these all way points. Fasten your seatbelt and make sure that you're ready.
	for(list<int>::iterator node_s = cycle.begin(); node_s != --cycle.end(); node_s++)
	{
		list<int>::iterator last_node = --cycle.end();
		advance(last_node, -1);
		
		temp_node_information = node_information;// Initialize
		float edge_heading = 0;
		float next_edge_heading = 0;
		float previous_edge_heading = 0;
		float theta = 0;
		float theta1 = 0;
		float theta2 = 0;
		float change = 1.63;// move the way point to right-dir
		float edge_length = 0;
		vector<float> point_info;
		
		list<int>::iterator previous_node = node_s;
		list<int>::iterator node_f = node_s;
		advance(node_f, 1);
		list<int>::iterator next_node = node_s;

		if(node_s != cycle.begin())
		{	
			advance(previous_node, -1);
		}
		// cout << *node_s << " " << *node_f;
		if(node_s != --cycle.end())
		{
			advance(next_node, 2);
		}

		// cout << "node_s: " <<*node_s <<endl;
		// cout << "next_node: " << *next_node<<endl;


		// cout << "hell"<<endl;

		// cout << "O"<<endl;



		edge_heading = atan2(node_information[*node_f][2] - node_information[*node_s][2],
							 node_information[*node_f][1] - node_information[*node_s][1]);
		// cout << edge_heading <<endl;
		if(directed[G.GetEdgeIndex(*node_s,*node_f)] == "yes")//Current edge is directed!
		{
			//1
			cout << "CURRENT EDGE IS DIRECTED!" << endl;
			if(node_s != cycle.begin() && node_s != last_node \
			&& directed[G.GetEdgeIndex(*node_f, *next_node)] == "yes" \
			&& directed[G.GetEdgeIndex(*previous_node, *node_s)] == "no")
			{
				// cout << "1" <<endl;
				next_edge_heading = atan2(node_information[*next_node][2] - node_information[*node_f][2],
										node_information[*next_node][1] - node_information[*node_f][1]);
				theta = fabs(next_edge_heading - edge_heading)/2;
				temp_node_information[*node_s][1] += change*sin(edge_heading);
				temp_node_information[*node_s][2] -= change*cos(edge_heading);
				temp_node_information[*node_f][1] += change*sin(edge_heading);
				temp_node_information[*node_f][2] -= change*cos(edge_heading);

				if(next_edge_heading - edge_heading >= 0)
				{	temp_node_information[*node_f][1] += change*tan(theta)*cos(edge_heading);
					temp_node_information[*node_f][2] += change*tan(theta)*sin(edge_heading);}
				else
				{	temp_node_information[*node_f][1] -= change*tan(theta)*cos(edge_heading);
					temp_node_information[*node_f][2] -= change*tan(theta)*sin(edge_heading);}
			}
			//2
			else if(node_s != cycle.begin() && node_s != last_node \
			&& directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes" \
			&& directed[G.GetEdgeIndex(*node_f, *next_node)] == "no")
			{
				// cout << "2" <<endl;				
				previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
										node_information[*node_s][1] - node_information[*previous_node][1]);
				theta = fabs(edge_heading - previous_edge_heading)/2;
				temp_node_information[*node_s][1] += change*sin(edge_heading);
				temp_node_information[*node_s][2] -= change*cos(edge_heading);
				temp_node_information[*node_f][1] += change*sin(edge_heading);
				temp_node_information[*node_f][2] -= change*cos(edge_heading);

				if(edge_heading - previous_edge_heading >= 0)//LEFT
				{	temp_node_information[*node_s][1] -= change*tan(theta)*cos(edge_heading);
					temp_node_information[*node_s][2] -= change*tan(theta)*sin(edge_heading);}
				else
				{	temp_node_information[*node_s][1] += change*tan(theta)*cos(edge_heading);
					temp_node_information[*node_s][2] += change*tan(theta)*sin(edge_heading);}

			}
			//3
			else if(node_s != cycle.begin() && node_s != last_node \
			&& directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes" \
			&& directed[G.GetEdgeIndex(*node_f, *next_node)] == "yes")
			{
				// cout << "3" <<endl;				
				next_edge_heading = atan2(node_information[*next_node][2] - node_information[*node_f][2],
										  node_information[*next_node][1] - node_information[*node_f][1]);
				previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
											  node_information[*node_s][1] - node_information[*previous_node][1]);
				
				theta1 = fabs(edge_heading - previous_edge_heading)/2;
				theta2 = fabs(edge_heading - next_edge_heading)/2;

				temp_node_information[*node_s][1] += change*sin(edge_heading);
				temp_node_information[*node_s][2] -= change*cos(edge_heading);
				temp_node_information[*node_f][1] += change*sin(edge_heading);
				temp_node_information[*node_f][2] -= change*cos(edge_heading);

				if(edge_heading - previous_edge_heading >= 0)//LEFT
				{	temp_node_information[*node_s][1] -= change*tan(theta1)*cos(edge_heading);
					temp_node_information[*node_s][2] -= change*tan(theta1)*sin(edge_heading);
					if(next_edge_heading - edge_heading >= 0)
					{	temp_node_information[*node_f][1] += change*tan(theta2)*cos(edge_heading);
						temp_node_information[*node_f][2] += change*tan(theta2)*sin(edge_heading);}
					else
					{	temp_node_information[*node_f][1] -= change*tan(theta2)*cos(edge_heading);
						temp_node_information[*node_f][2] -= change*tan(theta2)*sin(edge_heading);}   }
				else
				{	temp_node_information[*node_s][1] += change*tan(theta1)*cos(edge_heading);
					temp_node_information[*node_s][2] += change*tan(theta1)*sin(edge_heading);
					if(next_edge_heading - edge_heading >= 0)
					{	temp_node_information[*node_f][1] += change*tan(theta2)*cos(edge_heading);
						temp_node_information[*node_f][2] += change*tan(theta2)*sin(edge_heading);}
					else
					{	temp_node_information[*node_f][1] -= change*tan(theta2)*cos(edge_heading);
						temp_node_information[*node_f][2] -= change*tan(theta2)*sin(edge_heading);}   }

			}
			//4
			else if(node_s == cycle.begin() && directed[G.GetEdgeIndex(*node_s, *node_f)] == "yes")
			{
				// cout << "4" <<endl;				
				next_edge_heading = atan2(node_information[*next_node][2] - node_information[*node_f][2],
										  node_information[*next_node][1] - node_information[*node_f][1]);
				temp_node_information[*node_s][1] += change*sin(edge_heading);
				temp_node_information[*node_s][2] -= change*cos(edge_heading);
				temp_node_information[*node_f][1] += change*sin(edge_heading);
				temp_node_information[*node_f][2] -= change*cos(edge_heading);
				theta = fabs(next_edge_heading - edge_heading)/2;

				if(next_edge_heading - edge_heading >= 0)
				{	temp_node_information[*node_f][1] += change*tan(theta)*cos(edge_heading);
					temp_node_information[*node_f][2] += change*tan(theta)*sin(edge_heading);	}
				else
				{	temp_node_information[*node_f][1] -= change*tan(theta)*cos(edge_heading);
					temp_node_information[*node_f][2] -= change*tan(theta)*sin(edge_heading);	}
			}
			//5
			else if(node_s == last_node && directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes")
			{
				// cout << "5" <<endl;				
				previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
											  node_information[*node_s][1] - node_information[*previous_node][1]);
				temp_node_information[*node_s][1] += change*sin(edge_heading);
				temp_node_information[*node_s][2] -= change*cos(edge_heading);
				temp_node_information[*node_f][1] += change*sin(edge_heading);
				temp_node_information[*node_f][2] -= change*cos(edge_heading);
				theta = fabs(edge_heading - previous_edge_heading)/2;

				if(edge_heading - previous_edge_heading >= 0)
				{	temp_node_information[*node_s][1] -= change*tan(theta)*cos(edge_heading);
					temp_node_information[*node_s][2] -= change*tan(theta)*sin(edge_heading);	}
				else
				{	temp_node_information[*node_s][1] += change*tan(theta)*cos(edge_heading);
					temp_node_information[*node_s][2] += change*tan(theta)*sin(edge_heading);	}
			}

			//6
			else
			{	temp_node_information[*node_s][1] += change*sin(edge_heading);
				temp_node_information[*node_s][2] -= change*cos(edge_heading);
				temp_node_information[*node_f][1] += change*sin(edge_heading);
				temp_node_information[*node_f][2] -= change*cos(edge_heading);	}
			


			// Stores way points
			edge_length = distance(temp_node_information[*node_s][1], temp_node_information[*node_s][2], temp_node_information[*node_f][1], temp_node_information[*node_f][2]);
			point_info.push_back(temp_node_information[*node_s][1]);
			point_info.push_back(temp_node_information[*node_s][2]);
			point_info.push_back(1);
			way_point.push_back(point_info);

			point_info.clear();//intialize
			
			int number_of_pt = (int)(edge_length / interval);
			theta = atan2(temp_node_information[*node_f][2] - temp_node_information[*node_s][2], temp_node_information[*node_f][1] - temp_node_information[*node_s][1]);
			for(int i = 0; i<number_of_pt; i++)
			{	float ptx = round(temp_node_information[*node_s][1] + (i+1)*interval*cos(theta), 3);
				float pty = round(temp_node_information[*node_s][2] + (i+1)*interval*sin(theta), 3);
				if(i == number_of_pt - 1)
				{	if(ptx == temp_node_information[*node_f][1] and pty == temp_node_information[*node_f][2])
					{	break;	}	}
				point_info.push_back(ptx);
				point_info.push_back(pty);
				point_info.push_back(0);
				way_point.push_back(point_info);
				point_info.clear();
			}
			
			if(node_s == last_node)
			{	point_info.push_back(temp_node_information[*node_f][1]);
				point_info.push_back(temp_node_information[*node_f][2]);
				point_info.push_back(1);
				way_point.push_back(point_info);
				point_info.clear();   }

			temp_node_information.clear();

		}

		else if(node_s != last_node && directed[G.GetEdgeIndex(*node_f, *next_node)] == "yes")// Next Edge is DIRECTED, the current Edge is not directed
		{
			next_edge_heading = atan2(node_information[*next_node][2] - node_information[*node_f][2],
										node_information[*next_node][1] - node_information[*node_f][1]);
			previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
											node_information[*node_s][1] - node_information[*previous_node][1]);

			temp_node_information[*node_f][1] += change*sin(next_edge_heading);
			temp_node_information[*node_f][2] -= change*cos(next_edge_heading);
			if(node_s!= cycle.begin() && directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes")
			{	temp_node_information[*node_s][1] += change*sin(previous_edge_heading);
				temp_node_information[*node_s][2] -= change*cos(previous_edge_heading);   }
			
			// Store way points
			edge_length = distance(temp_node_information[*node_s][1], temp_node_information[*node_s][2], temp_node_information[*node_f][1], temp_node_information[*node_f][2]);
			point_info.push_back(temp_node_information[*node_s][1]);
			point_info.push_back(temp_node_information[*node_s][2]);
			point_info.push_back(1);
			way_point.push_back(point_info);

			point_info.clear();//intialize
			
			int number_of_pt = (int)(edge_length / interval);
			theta = atan2(temp_node_information[*node_f][2] - temp_node_information[*node_s][2], temp_node_information[*node_f][1] - temp_node_information[*node_s][1]);
			for(int i = 0; i<number_of_pt; i++)
			{	float ptx = round(temp_node_information[*node_s][1] + (i+1)*interval*cos(theta), 3);
				float pty = round(temp_node_information[*node_s][2] + (i+1)*interval*sin(theta), 3);
				if(i == number_of_pt - 1)
				{	if(ptx == temp_node_information[*node_f][1] and pty == temp_node_information[*node_f][2])
					{	break;	}	}
				point_info.push_back(ptx);
				point_info.push_back(pty);
				point_info.push_back(0);
				way_point.push_back(point_info);
				point_info.clear();
			}
			
			temp_node_information.clear();				
		}

		else if(node_s != cycle.begin() && directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes")
		{
			previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
										  node_information[*node_s][1] - node_information[*previous_node][1]);
			
			temp_node_information[*node_s][1] += change*sin(previous_edge_heading);
			temp_node_information[*node_s][2] -= change*cos(previous_edge_heading);

			// Store way points
			edge_length = distance(temp_node_information[*node_s][1], temp_node_information[*node_s][2], temp_node_information[*node_f][1], temp_node_information[*node_f][2]);
			point_info.push_back(temp_node_information[*node_s][1]);
			point_info.push_back(temp_node_information[*node_s][2]);
			point_info.push_back(1);
			way_point.push_back(point_info);

			point_info.clear();//intialize
			
			int number_of_pt = (int)(edge_length / interval);
			theta = atan2(temp_node_information[*node_f][2] - temp_node_information[*node_s][2], temp_node_information[*node_f][1] - temp_node_information[*node_s][1]);
			for(int i = 0; i<number_of_pt; i++)
			{	float ptx = round(temp_node_information[*node_s][1] + (i+1)*interval*cos(theta), 3);
				float pty = round(temp_node_information[*node_s][2] + (i+1)*interval*sin(theta), 3);
				if(i == number_of_pt - 1)
				{	if(ptx == temp_node_information[*node_f][1] and pty == temp_node_information[*node_f][2])
					{	break;	}	}
				point_info.push_back(ptx);
				point_info.push_back(pty);
				point_info.push_back(0);
				way_point.push_back(point_info);
				point_info.clear();
			}

			if(node_s == last_node)
			{	point_info.push_back(temp_node_information[*node_f][1]);
				point_info.push_back(temp_node_information[*node_f][2]);
				point_info.push_back(1);
				way_point.push_back(point_info);
				point_info.clear();   }		

			temp_node_information.clear();		
		}

		else if(node_s == cycle.begin() && directed[G.GetEdgeIndex(*node_s, *node_f)] == "yes")
		{
			next_edge_heading = atan2(node_information[*next_node][2] - node_information[*node_f][2],
									  node_information[*next_node][1] - node_information[*node_f][1]);			
			
			temp_node_information[*node_f][1] += change*sin(next_edge_heading);
			temp_node_information[*node_f][2] += change*cos(next_edge_heading);
			// Store way points
			edge_length = distance(temp_node_information[*node_s][1], temp_node_information[*node_s][2], temp_node_information[*node_f][1], temp_node_information[*node_f][2]);
			point_info.push_back(temp_node_information[*node_s][1]);
			point_info.push_back(temp_node_information[*node_s][2]);
			point_info.push_back(1);
			way_point.push_back(point_info);

			point_info.clear();//intialize
			
			int number_of_pt = (int)(edge_length / interval);
			theta = atan2(temp_node_information[*node_f][2] - temp_node_information[*node_s][2], temp_node_information[*node_f][1] - temp_node_information[*node_s][1]);
			for(int i = 0; i<number_of_pt; i++)
			{	float ptx = round(temp_node_information[*node_s][1] + (i+1)*interval*cos(theta), 3);
				float pty = round(temp_node_information[*node_s][2] + (i+1)*interval*sin(theta), 3);
				if(i == number_of_pt - 1)
				{	if(ptx == temp_node_information[*node_f][1] and pty == temp_node_information[*node_f][2])
					{	break;	}	}
				point_info.push_back(ptx);
				point_info.push_back(pty);
				point_info.push_back(0);
				way_point.push_back(point_info);
				point_info.clear();
			}
			
			temp_node_information.clear();				
		}

		else if(node_s == last_node && directed[G.GetEdgeIndex(*previous_node, *node_s)] == "yes")
		{
			previous_edge_heading = atan2(node_information[*node_s][2] - node_information[*previous_node][2],
										  node_information[*node_s][1] - node_information[*previous_node][1]);
			temp_node_information[*node_s][1] += change*sin(previous_edge_heading);
			temp_node_information[*node_s][2] -= change*cos(previous_edge_heading);

			// Store way points
			edge_length = distance(temp_node_information[*node_s][1], temp_node_information[*node_s][2], temp_node_information[*node_f][1], temp_node_information[*node_f][2]);
			point_info.push_back(temp_node_information[*node_s][1]);
			point_info.push_back(temp_node_information[*node_s][2]);
			point_info.push_back(1);
			way_point.push_back(point_info);

			point_info.clear();//intialize
			
			int number_of_pt = (int)(edge_length / interval);
			theta = atan2(temp_node_information[*node_f][2] - temp_node_information[*node_s][2], temp_node_information[*node_f][1] - temp_node_information[*node_s][1]);
			for(int i = 0; i<number_of_pt; i++)
			{	float ptx = round(temp_node_information[*node_s][1] + (i+1)*interval*cos(theta), 3);
				float pty = round(temp_node_information[*node_s][2] + (i+1)*interval*sin(theta), 3);
				if(i == number_of_pt - 1)
				{	if(ptx == temp_node_information[*node_f][1] and pty == temp_node_information[*node_f][2])
					{	break;	}	}
				point_info.push_back(ptx);
				point_info.push_back(pty);
				point_info.push_back(0);
				way_point.push_back(point_info);
				point_info.clear();
			}

			if(node_s == last_node)
			{	point_info.push_back(temp_node_information[*node_f][1]);
				point_info.push_back(temp_node_information[*node_f][2]);
				point_info.push_back(1);
				way_point.push_back(point_info);
				point_info.clear();   }		

			temp_node_information.clear();
		}

		else
		{
			edge_length = distance(node_information[*node_s][1], node_information[*node_s][2], node_information[*node_f][1], node_information[*node_f][2]);
			point_info.push_back(node_information[*node_s][1]);
			point_info.push_back(node_information[*node_s][2]);
			point_info.push_back(1);
			way_point.push_back(point_info);

			point_info.clear();//intialize
			
			int number_of_pt = (int)(edge_length / interval);
			theta = atan2(node_information[*node_f][2] - node_information[*node_s][2], node_information[*node_f][1] - node_information[*node_s][1]);
			for(int i = 0; i<number_of_pt; i++)
			{	float ptx = round(node_information[*node_s][1] + (i+1)*interval*cos(theta), 3);
				float pty = round(node_information[*node_s][2] + (i+1)*interval*sin(theta), 3);
				if(i == number_of_pt - 1)
				{	if(ptx == node_information[*node_f][1] and pty == node_information[*node_f][2])
					{	break;	}	}
				point_info.push_back(ptx);
				point_info.push_back(pty);
				point_info.push_back(0);
				way_point.push_back(point_info);
				point_info.clear();
			}

			if(node_s == last_node)
			{	point_info.push_back(node_information[*node_f][1]);
				point_info.push_back(node_information[*node_f][2]);
				point_info.push_back(1);
				way_point.push_back(point_info);
				point_info.clear();   }		
		}

		// cout << directed[G.GetEdgeIndex(*node_s, *node_f)]<<endl;
	}
	cout << endl;
	cout << way_point.size() << endl;

	// for(int i = 0; i < way_point.size(); i++)
	// {
	// 	cout << way_point[i][0] << ", " << way_point[i][1] << ", "<<way_point[i][2] <<endl;
	// }
}
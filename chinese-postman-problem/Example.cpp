// #include <ros/ros.h>
#include "./Minimum-Cost-Perfect-Matching/Graph.h"
#include "ChinesePostman.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
using namespace std;

//pair class는 2가지 변수를 하나로 취급, in other words, literally PAIR
//(first, second)
//vector: 동적 배열, 크기가 자동으로 관리되는 배열로 이해
// ms adds

typedef struct g_info// Make a type containing the information of a graph
{
	vector< vector<float> > _node_information;
	Graph _G;
	vector<double> _cost;
	vector<string> _directed;
};

// pair< Graph, vector<double> > ReadWeightedGraph(string filename)
g_info ReadWeightedGraph(string filename)
{
	//Please see Graph.h for a description of the interface

	ifstream file;//파일 읽기
	file.open(filename.c_str());//open a file as an string,
	// we have to understand the string as an sequence of characters, in other words, ARRAY


	string s;
	getline(file, s);// get an array of the sentence in a file
	stringstream ss(s);//it is very useful to get data we want from the string array
	int n;
	ss >> n;//VERTICES
	cout << "n: " << n << endl;
	getline(file, s);
	ss.str(s);//copy the string s to the stream ss
	ss.clear();
	int m;
	ss >> m;//EDGES
	cout << "m: " << m << endl;

	//ms adds
	vector< vector<float> > node_info;
	vector<float> node_info_set;
	for(int i = 0; i < n; i++)
	{
		getline(file, s);
		ss.str(s);//copy the line in the txt file as the stringstream
		ss.clear();
		float q, w, e;
		ss >> q >> w >> e;
		node_info_set.push_back(q);
		node_info_set.push_back(w);
		node_info_set.push_back(e);
		node_info.push_back(node_info_set);

		node_info_set.clear();

		// cout << "+++" << endl;
	}
	cout << node_info.size() <<endl;
	// cout << "hahahaha" <<endl;////////ms.fins




	Graph G(n);// define a graph having #vertices n
	vector<double> cost(m);// define a vector named 'cost(m)', each cost has weight
	double edge_distance;
	vector<string> directed(m);
	for(int i = 0; i < m; i++)
	{
		getline(file, s);
		ss.str(s);
		ss.clear();
		int u, v;
		double c;
		string dir;
		ss >> u >> v >> dir;

		// Calcuate the cost for this
		c = (double) distance(node_info[u][1], node_info[u][2], node_info[v][1], node_info[v][2]);
		G.AddEdge(u, v);// add an edge(u, v)
		cost[G.GetEdgeIndex(u, v)] = c;// append a cost of the edge using GetEdgeIndex
		directed[G.GetEdgeIndex(u, v)] = dir;
	}

	file.close();

	g_info graph_information;// Declare the type of g_info as 'graph_information'
	graph_information._node_information = node_info;
	graph_information._G = G;
	graph_information._cost = cost;
	graph_information._directed = directed;

	return graph_information;
	// return make_pair(G, cost);// make the pair with G and cost, we are going to think them, together.
}

int main(int argc, char * argv[])
{


	string filename = "";//define a string named filename

	int i = 1;
	while(i < argc)
	{
		string a(argv[i]);
		if(a == "-f")
			filename = argv[++i];
		i++;
	}

	if(filename == "")
	{
		cout << endl << "usage: ./example -f <filename>" << endl << endl;
		cout << "-f followed by file name to specify the input file." << endl << endl;
		
		cout << "File format:" << endl;
		cout << "The first two lines give n (number of vertices) and m (number of edges)," << endl;
		cout << "Each of the next m lines has a tuple (u, v [, c]) representing an edge," << endl;
	   	cout << "where u and v are the endpoints (0-based indexing) of the edge and c is its cost" << endl;
		return 1;
	}

	try// catching an exeptional situation.(try ~ catch, throw)
	{
	    Graph G;
	    vector<double> cost;
		vector<string> directed;
		vector< vector<float> > node_information;
		float car_pos_x = 10.1;
		float car_pos_y = 1.63;
		float car_pos_heading = 0.5;// -pi <= car_pos_heading <= pi

	
		g_info _graph = ReadWeightedGraph(filename);//ReadWeightedGraph returns a pair (Graph, cost)
		G = _graph._G;
		cost = _graph._cost;
		node_information = _graph._node_information;
		directed = _graph._directed;
		// node_information contains the node index, point_x, point_y as a vector
	    //Read the graph
	    // pair< Graph, vector<double> > p = ReadWeightedGraph(filename);//ReadWeightedGraph returns a pair (Graph, cost)
	    // G = p.first;
	    // cost = p.second;

	    //Solve the problem
     	pair< list<int> , double > sol = ChinesePostman(G, cost, node_information, car_pos_x, car_pos_y, car_pos_heading);
		cout << "Car_x: " << car_pos_x << endl;
		cout << "Car_y: " << car_pos_y << endl;
		cout << "Car_heading: " << car_pos_heading << endl;
		cout << "Solution cost: " << sol.second << endl;

		list<int> s = sol.first;

        //Print edges in the solution
		cout << "Solution:" << endl;
		for(list<int>::iterator it = s.begin(); it != s.end(); it++)
			cout << *it << " ";
		cout << endl;

		write_down(G, s, directed, node_information);
		cout << "\nMINSOO\n";
	}
	catch(const char * msg)
	{
		cout << msg << endl;
		return 1;
	}

	return 0;
}





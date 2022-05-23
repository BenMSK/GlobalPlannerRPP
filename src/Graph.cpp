#include "../include/Graph.h"

Graph::Graph(int n, const list< pair<int, int> > & edges):
	n(n),
	m(edges.size()),
	adjMat(n, vector<bool>(n, false)),
	adjList(n),
	edges(),
	edgeIndex(n, vector<int>(n, -1))

{
	for(list< pair<int, int> >::const_iterator it = edges.begin(); it != edges.end(); it++)
	{
		int u = (*it).first;
		int v = (*it).second;

		AddEdge(u, v);
	}
}


pair<int, int> Graph::GetEdge(int e) const
{
	if(e > (int)edges.size())
		throw "Error: edge does not exist";

	return edges[e];
}

int Graph::GetEdgeIndex(int u, int v) const
{
	// if( u > n or
	// 	v > n )
	// 	throw "Error: vertex does not exist";//THIS IS FOR CACULATING THE VERTICES

	if(edgeIndex[u][v] == -1)
		throw "Error: edge does not exist";

	return edgeIndex[u][v];
}

void Graph::AddVertex()
{
	for(int i = 0; i < n; i++)
	{
		adjMat[i].push_back(false);
		edgeIndex[i].push_back(-1);
	}
	n++;
	adjMat.push_back( vector<bool>(n, false) );
	edgeIndex.push_back( vector<int>(n, -1) );
	adjList.push_back( list<int>() );
}

void Graph::AddEdge(int u, int v)
{
	if( u > n or
		v > n )
		throw "Error: vertex does not exist";

	if(adjMat[u][v]) return;

	adjMat[u][v] = adjMat[v][u] = true;
	adjList[u].push_back(v);
	adjList[v].push_back(u);

	edges.push_back(pair<int, int>(u, v));
	edgeIndex[u][v] = edgeIndex[v][u] = m++;
}

// MS
void Graph::initialize_vertices()
{	
	for(int i = 0; i < n; i++) {
		vertices_list.push_back(i);
	}
}

// int Graph::GetVertice(int u)
// {
// 	if (u > n)
// 		throw "Error: vertex does not exist";
// 	return vertices_list(u);
// }
void Graph::GraphInfo(Graph & graph)
{
	cout << "Number of VERTICES: " << n << endl;
	cout << "Number of EDGES: "  << m << endl;

	cout << "edges!" << endl;
	for(int i= 0; i< graph.edges.size(); i++)
	{
		cout << graph.edges[i].first << ", " << graph.edges[i].second << endl;
	}
	cout << endl;

	cout << "edgeIndex!" << endl;
	for(int i= 0; i< graph.edgeIndex.size(); i++)
	{	
		cout << i << ": ";
		for(int j = 0; j < graph.edgeIndex[i].size(); j++)
		{
			cout << graph.edgeIndex[i][j] << ", ";
		}
		cout << endl;
	}
	cout << endl;

	cout << "AdjMat" << endl;
	for(int i=0; i<n; i++)
	{
		cout << graph.GetVertices()[i] << ": ";
		for(int j=0; j< graph.AdjMat()[graph.GetVertices()[i]].size(); j++)
		{
			cout << graph.AdjMat()[graph.GetVertices()[i]][j] << ", ";
		}
		cout << endl;
	}
	cout << endl;
	
	cout << "AdjList" << endl;
	for(int i=0; i<n; i++)
	{
		cout << graph.GetVertices()[i] << ": ";
		for(list<int>::const_iterator itr = graph.AdjList(graph.GetVertices()[i]).begin(); itr != graph.AdjList(graph.GetVertices()[i]).end(); itr++)
		{
			cout << *itr << ", ";
		}
		cout << endl;
	}
}

void Graph::RemoveVertex(int u)
{
	n--;
}

void Graph::RemoveEdge(int u, int v)//MS
{
	// if( u > n or
	// 	v > n )
	// 	throw "Error: vertex does not exist";
	// cout << "haha0" << endl;

	if(adjMat[u][v] != true) return;//the edge(u, v) doesn't exist

	adjMat[u][v] = adjMat[v][u] = false;
	adjList[u].remove(v);
	adjList[v].remove(u);

	for(vector< pair<int, int> >::iterator itr = edges.begin(); itr != edges.end(); itr++)
	{
		if( *itr == pair<int, int>(u, v) || *itr == pair<int, int>(v, u) )
		{
			edges.erase(itr);
			break;;
		}
	}
	edgeIndex[u][v] = edgeIndex[v][u] = -1;
	m--;//number of edge --

	for(int i = 0; i<vertices_list.size(); i++)
		if(adjList[vertices_list[i]].size() == 0) {
			vertices_list[i] = -1;
			n--;
			break;
		}

	// for(vector<int>::iterator it = vertices_list.begin(); it != vertices_list.end(); it ++) {
	// 	if(adjList[*it].size() == 0) {//Which means the vertice, *it, is not connected any of vertices in a graph.
	// 		// vertices_list.erase(it);
	// 		*it = -1;
	// 		n--;
	// 		break;
	// 	}
	// }

}

const list<int> & Graph::AdjList(int v) const
{
	// if(v > n)
	// 	throw "Error: vertex does not exist";

	return adjList[v];
}

const vector< vector<bool> > & Graph::AdjMat() const
{
	return adjMat;
}


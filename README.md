# GlobalPlannerRPP
Implementation of Global Planner with RPP for parking global plan.  

This repository is based on [the paper](https://ieeexplore.ieee.org/abstract/document/8917318) below.  
Kim, Minsoo, Joonwoo Ahn, and Jaeheung Park. "Global Planning Method for Visiting Roads with Parking Spaces in Priority using Rural Postman Problem." 2019 IEEE Intelligent Transportation Systems Conference (ITSC). IEEE, 2019.  
All the details of the algorithm can be found from this paper.  
This code is implemented with ROS.  

## Requirements
* ROS
* Boost
* Eigen

## How to use
1) clone this code in catkin_ws/src
2) build and launch global_planner_.launch
3) All parking lot is formulated as a graph, and the graph's information is contained in a .txt file.  
4) If you want to use a custom map, you should formulate the map into a graph. The edge of the graph consists of required, and optional edges.  

## Graph form in .txt
'the number of vertices'  
'the number of edges'  
{'node_idx x_coords y_coords'} // coordinates of each node  
...  
{'node_a node_b directed_or_undirected(not_used...) required_edge(1)_or_optional_edge(0)} // informations of each edge  





#include <ros/ros.h>
#include "../include/GlobalPlanner.h"

int main(int argc, char **argv)// argc: argument count! argv: argument vector
{

	//ROS part.
    ros::init(argc, argv, "global_planner_node");
	ros::NodeHandle node;
	ros::NodeHandle private_node("~");

	// ros::Rate r(10);
	// while(ros::ok())
	
    globalplanner::GlobalPlanner msk(node, private_node);
		
	cout << "WORKING" <<endl;
	ros::spin();

	return 0;
}
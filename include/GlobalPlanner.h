#include <ros/ros.h>
#include <ros/package.h>
#include "./Matching.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <iterator>
#include <math.h>
#include <string>
#include <sstream>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <time.h>


using namespace std;

namespace globalplanner{
class GlobalPlanner
{
public:

    GlobalPlanner(ros::NodeHandle node, ros::NodeHandle private_node);
    ~GlobalPlanner();

    bool Connected(Graph & G);
    // bool Connected(const Graph & G);
    pair< list<int>, double > ChinesePostman(Graph & G, const vector<double> & cost, const vector< vector<float> > & node_information,
                                             const float car_pos_x, const float car_pos_y, const float car_pos_heading);
    pair< list<int>, double > RuralPostman(Graph & G, const vector<double> & cost, const vector< vector <float> > & node_information,
                                             const float car_pos_x, const float car_pos_y, const float car_pos_heading);
    list<int> RPP(Graph & G, const vector<double> & cost, const vector< vector<float> > & node_information);
    pair< list<int>, double > ChinesePostmanCost(Graph & G, const vector<double> & cost, const vector<bool> & park);
    //  pair<vector<bool>, vector<int>> explored, vector< pair<int, int> > solution_edge);//ChinesePostman(Graph, cost), returns < list, double >
    bool FILTER1(const vector< vector<int> > never_passed, const list<int> cycle);
    vector< vector<float> > WayptPath(const Graph & G, list<int> cycle, vector<string> directed,
                                      const vector< vector<float> > & node_information, const float interval);
    vector< vector<float> > ms_spline(vector< vector<float> > way_point, const float interval, const int spline_weight);
    
    float distance(float point1_x, float point1_y, float point2_x, float point2_y);
    float round(float value, int pos);
    void main();

    void PrintElements(const std::vector<double>& numbers, const std::vector<size_t>& indexes);
    
    //COST
    double cost1(Graph & G, list<int> cycle, const vector< vector<float> > & node_information,
                 const float car_pos_x, const float car_pos_y);
    double cost2(Graph & G, vector< pair<int, int> > solution_edge, vector<bool> parkable, const vector< vector<float> > & node_information);

    void CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void CallbackGlobalPath(const std_msgs::Bool::ConstPtr& msg);
    void CallbackCarIdx(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void CallbackCarlaResetPos(const std_msgs::Bool::ConstPtr& msg); // CARLA

private:


    struct g_info
    {
        vector< vector<float> > _node_information;// containing x and y coordinates
        Graph _G;
        vector<double> _cost;
        vector<string> _directed;
        vector<bool> _park;
    };
    g_info ReadWeightedGraph(string filename);
    g_info RePlanning(g_info G);

    struct CARPOSE
    {
        double x;
        double y;
        double th;
        double vel;
        int waypt_idx;
    };
    CARPOSE m_car;

    /*_____ReadWeightedGraph_____*/
    double edge_distance;
    ifstream file;//파일　읽기
    string s;
    int vertices;
    int edges;
    vector< vector<float> > node_info;
    vector<float> node_info_set;
    g_info graph_information;// Declare the type of g_info as 'graph_information'
    // You have to put it into a same place where the sturct exists, private-private or public-public


    /*_____Main_____*/
    string filename = "";//define a string named filename, and it will contain 'TOPOLOGICAL MAP INFO'
    bool usingCARLA;// using CARLA or not

    Graph G;
    Graph RG;
    vector<double> cost;
    vector<string> directed;
    vector<bool> parkable;
    vector< vector<float> > node_information;

    float car_pos_x = 50.1;
    float car_pos_y = 40.1;
    float car_pos_heading;// -pi <= car_pos_heading <= pi
    const float interval = .02;
    const int spline_weight = 300;

    pair< list<int> , double > solution;
    vector< pair<int, int> > solution_edge;
    list<int> EulerTour;
    g_info _graph;
    bool m_flag;


    /*_____CPP_____*/
    float _dis;
    vector< vector<float> > temp_node_information;

    float edge_heading;
    float next_edge_heading;
    float previous_edge_heading;
    vector<int> reasonable_start_edge;

    /*_____RPP_____*/
    list<int> RPP_final_path;
    Graph RPP_G;
    vector<double> RPP_cost;
    vector<string> RPP_directed;
    double fcm;
    vector<double> fcm_stack;
    double gear_switch;
    int max_generation = 60;
    int exp_count = 1;
    int rpp_start;
    bool dbg = false;
    double cpu_time_used;
    int er_count = 0;
    int eo_count = 0;
    vector<bool> tmp_Rpark0;


    /*____wayptPath_____*/
    float theta;
    float theta1;
    float theta2;
    float edge_length;
    vector<float> point_info;

    list<int>::iterator previous_node;
    list<int>::iterator node_f;

    int number_of_pt;
    float ptx;
    float pty;

    /*_____ms_spline function_____*/
    double radius;// initialize
    double rear_edge_heading;// rear_edge heading <previous node, current node>
    double _rear_edge_heading;// rear edge heading <current node, previous node> != rear_edge_heading
    double front_edge_heading;// front edge heading <current node, next node>
    double node_angle;// angle of <previous node, current node, next node>
    double spline_theta;

    float dis;
    bool right;// it cheSub_globalCall or left.

    int number_of_spline_point;// how many points we have to do 'spline'

    int count1;
    int count2;
    float length1;
    float length2;
    pair<double, double> center_point;// center point of the circle we defined
    pair<double, double> move_point;// new points which are moved fron the original one.


    /*_____Using for REPLANNING_____*/
    bool replanning;
    int car_pos_waypt_idx;
    pair< vector<bool>, vector<int> > explored;//(passed or not, last index of each edge)
    int run_main = 0;
    int passed = 0;
    int progress = 0;//Means how far the vehicle goes in a CPP route, initialized here
    int temp_car_pos_waypt_idx;
    pair< vector<bool>, vector<int> > copied_explored;

    double fcost1;
    double fcost2;    

    vector< list<int> > candidate_cycle;
    vector<int> candidate_cost;
    vector< list<int> > Rcandidate_cycle;
    vector<int> Rcandidate_cost;
    vector< list<int> > Zcandidate_cycle;
    vector<int> Zcandidate_cost;
    
    int cycle_count=0;
    int Rsearch_times = 2;
    int search_times = 1;

    /*_____ROS_____*/
    ros::Publisher Pub_path;
    ros::Publisher Pub_pure_path;
    ros::Publisher Pub_marker;

    ros::Subscriber Sub_localization;
    ros::Subscriber Sub_globalCall;
    ros::Subscriber Sub_carIdx;

};

}

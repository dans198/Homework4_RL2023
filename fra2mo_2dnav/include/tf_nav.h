#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <vector>
#include <future>

class TF_NAV {

    public:
        TF_NAV();
        void run();
        void tf_listener_fun();
        void position_pub();
        void goal_listener();
        void send_goal();
        
        void tf_listener_fun_camera();
        void send_goal_4B();
        void run2();

    private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;

        Eigen::Vector3d _home_pos;

        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;

        Eigen::Vector3d _goal_pos;
        Eigen::Vector4d _goal_or;
        
        std::vector<int> goalOrder;
        std::string goal_c;
        
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


	//Inclusioni per il single_goal.cpp
	Eigen::VectorXd aruco_pose_map;

};

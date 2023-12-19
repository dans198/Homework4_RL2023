#include "../include/tf_nav.h"
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Global variables
std::vector<double> aruco_pose(7,0.0);
bool aruco_pose_available = false;

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
}

TF_NAV::TF_NAV() {

    goalOrder = {3, 4, 2, 1};
    goal_c="goal"+std::to_string(goalOrder[0]);

    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );
    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 0.0;
    _goal_pos << 0.0, 0.0, 0.0;
    _goal_or << 0.0, 0.0, 0.0, 0.0;
    _home_pos << -18.0, 2.0, 0.0;
}

void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() )
    {
        try {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform );

        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }

}

void TF_NAV::position_pub() {

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];

    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];

    _position_pub.publish(pose);
}

void TF_NAV::goal_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        
        try
        {
            listener.waitForTransform( "map", goal_c, ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", goal_c, ros::Time( 0 ), transform );

        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        r.sleep();
    }    
}

 /*
        //PUNTO 2B

void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    move_base_msgs::MoveBaseGoal goal;

    while ( ros::ok() )
    {   
        int cmd;
        std::cout<<"\nInsert 1 to send goal 1"<<std::endl;
        std::cout<<"Insert 2 to send goal 2"<<std::endl;
        std::cout<<"Insert 3 to send goal 3"<<std::endl;
        std::cout<<"Insert 4 to send goal 4"<<std::endl;
        std::cout<<"Insert 5 to send home position goal "<<std::endl;
        std::cout<<"Inser your choice"<<std::endl;
        std::cin>>cmd;



        if ( cmd <= 4) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos[0];
            goal.target_pose.pose.position.y = _goal_pos[1];
            goal.target_pose.pose.position.z = _goal_pos[2];

            goal.target_pose.pose.orientation.w = _goal_or[0];
            goal.target_pose.pose.orientation.x = _goal_or[1];
            goal.target_pose.pose.orientation.y = _goal_or[2];
            goal.target_pose.pose.orientation.z = _goal_or[3];


            ROS_INFO("Sending goal");

            std::cout << "Header Frame ID: " << goal.target_pose.header.frame_id << std::endl;
            std::cout << "Header Timestamp: " << goal.target_pose.header.stamp << std::endl;
            std::cout << "Position (x, y, z): ("
                << _goal_pos[0] << ", "
                << _goal_pos[1] << ", "
                << _goal_pos[2] << ")" << std::endl;
            std::cout << "Orientation (w, x, y, z): (" 
                << _goal_or[0] << ", "
                << _goal_or[1] << ", "
                << _goal_or[2] << ", "
                << _goal_or[3] << ")" << std::endl;

            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");
        }

        else if ( cmd == 2 ) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;

            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move for some reason");
        }

         else {
            ROS_INFO("Wrong input!");
        }
        r.sleep();
    }
    
}

*/
        //PUNTO 2C

void TF_NAV::send_goal() {

    ros::Rate r( 5 );
    move_base_msgs::MoveBaseGoal goal;
    int cmd;
    int cont=0;
    std::cout<<"The goal order is 3-4-2-1"<<std::endl;

    while ( ros::ok() )
    {
        std::cout << "\nInsert 1 to send goal from TF " << std::endl;
        std::cout << "Insert 2 to send home position goal " << std::endl;
        std::cout<<"Insert your choice"<<std::endl;
        std::cin>>cmd;



        if ( cmd == 1) {
            while(cont <= 4){
                MoveBaseClient ac("move_base", true);
                while(!ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
                }
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                
                goal.target_pose.pose.position.x = _goal_pos[0];
                goal.target_pose.pose.position.y = _goal_pos[1];
                goal.target_pose.pose.position.z = _goal_pos[2];

                goal.target_pose.pose.orientation.w = _goal_or[0];
                goal.target_pose.pose.orientation.x = _goal_or[1];
                goal.target_pose.pose.orientation.y = _goal_or[2];
                goal.target_pose.pose.orientation.z = _goal_or[3];

                ROS_INFO("Sending goal");
                ac.sendGoal(goal);

                ac.waitForResult();

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("The mobile robot arrived in the TF goal");
                cont=cont+1;
                goal_c="goal" + std::to_string(goalOrder[cont]);
                std::cout<<"Il goal e': "<<goal_c<<std::endl;
                }
                else
                    ROS_INFO("The base failed to move for some reason");

            }
        }
        else if ( cmd == 2 ) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;

            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move for some reason");
        }

         else {
            ROS_INFO("Wrong input!");
        }
        r.sleep();
               
    }
    
}

    //PUNTO 4B

/*
void TF_NAV::send_goal() {

    ros::Rate r( 5 );
    move_base_msgs::MoveBaseGoal goal;
    int cmd;
    int cont=0;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    static tf::TransformBroadcaster br;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, -3.14);  // Roll, Pitch, Yaw in rad

    while ( ros::ok() )
    {
        std::cout << "\nInsert 1 to send goal from TF " << std::endl;
        std::cout << "Insert 2 to send home position goal " << std::endl;
        std::cout<<"Insert your choice"<<std::endl;
        std::cin>>cmd;
        
        if ( cmd == 1) {
            while(cont < 1){
                MoveBaseClient ac("move_base", true);

                while(!ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
                }

                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                
                goal.target_pose.pose.position.x = -12.0;
                goal.target_pose.pose.position.y = 7.0;
                goal.target_pose.pose.orientation = tf2::toMsg(quaternion);

                ROS_INFO("Sending goal");
                ac.sendGoal(goal);

                ac.waitForResult();

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("The mobile robot arrived in the TF goal");
                cont=cont+2;
                }
                else
                    ROS_INFO("The base failed to move for some reason");
            }
        }
        while ( cont >= 1 ) {

            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }


            try
        {
            listener.waitForTransform( "map", "aruco_marker_frame", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "aruco_marker_frame", ros::Time( 0 ), transform );

        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = transform.getOrigin().x()+1;
            goal.target_pose.pose.position.y = transform.getOrigin().y();

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "aruco_115_frame"));
            
            std::cout << "Header Frame ID: " << goal.target_pose.header.frame_id << std::endl;
            std::cout << "Header Timestamp: " << goal.target_pose.header.stamp << std::endl;
            std::cout << "Position (x, y, z): ("
                << goal.target_pose.pose.position.x << ", "
                << goal.target_pose.pose.position.y << ", "
                << goal.target_pose.pose.position.z << ")" << std::endl;
            std::cout << "Orientation (w, x, y, z): (" 
                << goal.target_pose.pose.orientation.x  << ", "
                << goal.target_pose.pose.orientation.y << ", "
                << goal.target_pose.pose.orientation.z << ", "
                << goal.target_pose.pose.orientation.w << ")" << std::endl;



            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move for some reason");
        }

        r.sleep();
               
    }
    
}
*/
void TF_NAV::run() {
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );
    boost::thread tf_listener_goal_t( &TF_NAV::goal_listener, this );
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
    ros::spin();
}



int main( int argc, char** argv ) {
    ros::init(argc, argv, "tf_navigation");
    ros::NodeHandle n;
    // Subscribers
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);

    TF_NAV tfnav;
    tfnav.run();
    

    return 0;
}
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Global variables
std::vector<double> aruco_pose(7,0.0);
bool aruco_pose_available = false;

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);

    transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) );
    tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "aruco_marker_frame"));
}
 
int main(int argc, char** argv){
  
  ros::init(argc, argv, "simple_navigation_goals");
  std::cout<<"SONO NEL MAIN"<<std::endl;  
  ros::NodeHandle n;
  ros::Rate r( 5 );

  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, -3.14);  // Roll, Pitch, Yaw in rad
  bool goal_r = false;
  
  // Subscribers
  ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);

  move_base_msgs::MoveBaseGoal goal;

  while( ros::ok() ){
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    
    if(goal_r == false){

      //wait for the action server to come up
      while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
      }

      //we'll send a goal to the robot to move 1 meter forward
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
    
      goal.target_pose.pose.position.x = -13.0;
      goal.target_pose.pose.position.y = 7.0;
      goal.target_pose.pose.orientation = tf2::toMsg(quaternion);

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved");
        goal_r = true;
      }
        
      else
        ROS_INFO("The base failed to move for some reason");
    }

    if(goal_r == true){
      std::cout<<"SONO NEL SECONDO IF"<<std::endl;

      //wait for the action server to come up
      while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
      }

      try {
        listener.waitForTransform( "map", "camera_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform( "map", "camera_link", ros::Time(0), transform );
      }
      catch( tf::TransformException &ex ) {
        ROS_ERROR("%s", ex.what());
        r.sleep();
        continue;
      }
          
      Eigen::VectorXd  map_base_tf;
      map_base_tf << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

      //Conversione da vettore std a eigen per poter fare il prodotto per il cambio di riferimento
      Eigen::VectorXd aruco_pose_eigen = Eigen::Map<Eigen::VectorXd>(aruco_pose.data(), aruco_pose.size());
      Eigen::VectorXd aruco_pose_map = map_base_tf * aruco_pose_eigen;

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
    
      goal.target_pose.pose.position.x = 1+ aruco_pose_map.x();
      goal.target_pose.pose.position.y = aruco_pose_map.y();

    
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved");
      else
        ROS_INFO("The base failed to move for some reason");

    }

  r.sleep();
  }

  return 0;
}
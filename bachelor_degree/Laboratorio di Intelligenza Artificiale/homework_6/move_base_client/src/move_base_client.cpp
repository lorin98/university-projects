#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
	ros::init(argc, argv, "move_base_client");
	ros::NodeHandle n;

	// trying to save the initial pose of the robot
	// this operation will be done once
	boost::shared_ptr<nav_msgs::Odometry const> msg_ptr;
	nav_msgs::Odometry msg;
	msg_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
	if(msg_ptr != NULL){
		msg = *msg_ptr;
	}
	
	ROS_INFO("initial pose saved...the robot is now in %f, %f, %f", 
				msg.pose.pose.position.x,
				msg.pose.pose.position.y,
				msg.pose.pose.position.z);

	MoveBaseClient ac("move_base", true);
	
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("waiting for the move_base action server to come up");
	}
	
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now(); 
	goal.target_pose.pose.position.x =  7.125;
	goal.target_pose.pose.position.y = 1.771;
	goal.target_pose.pose.orientation.z = 1.0;

	ROS_INFO("sending a first goal");
	ac.sendGoal(goal);
	
	// now I'm waiting for some seconds and then I cancel the goal...
	ac.waitForResult(ros::Duration(30.0));
	// ...so I force the robot to stop
	ac.cancelAllGoals();
	ROS_INFO("canceling the previous goal...the robot must stop!!!");
	
	// finally I told the robot to come back to its initial pose (saved in 'msg')
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose = msg.pose.pose;
	
	ROS_INFO("sending a second goal to its initial pose");
	ac.sendGoal(goal);
	
	// get the final result of this action
	ac.waitForResult(ros::Duration(40.0));
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("arrived at the goal!!!");
	else
		ROS_INFO("tried to come back, the robot is near its goal...bye!");
		
	return 0;
}
	

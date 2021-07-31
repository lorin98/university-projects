#include <ros/ros.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/Circle.h>
#include <turtlesim/GetCircles.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>

turtlesim::Pose current_pose;
uint8_t crossed_circle = 0;
std::vector<turtlesim::Circle> circles;

void killCircle(uint8_t id){
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<turtlesim::RemoveCircle>("tutorial_delete_circle");
	turtlesim::RemoveCircle srv;
	srv.request.id = id;
	if (client.call(srv)){
		ROS_INFO("sending a request to remove my circles\n");	
	}
	else {
		ROS_INFO("failed to call service tutorial_delete_circle\n");
		return;
	}
	circles = srv.response.circles;
	if (circles.size() == 0){
		ROS_INFO("there are no spawned circles left...my work is done!!!\n");
		exit(0);
	}
	ROS_INFO("there are still %lu spawned circles left\n", circles.size());	
}

void poseCallback(const turtlesim::Pose& pose){
	// first I update my circles vector ...
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<turtlesim::GetCircles>("tutorial_get_circles");
	turtlesim::GetCircles get_srv;
	
	if (!client.call(get_srv)){
		ROS_INFO("failed to call service tutorial_get_circles\n");
		return;		
	}
	circles = get_srv.response.circles;
	
	// ... and then I start searching other turtles
	current_pose = pose;
	for (int i=0; i<circles.size(); i++){

		if (((circles[i].x - 0.25 <= current_pose.x) && (current_pose.x <= circles[i].x + 0.25))
				&& ((circles[i].y - 0.25 <= current_pose.y) && (current_pose.y <= circles[i].y + 0.25))){
			crossed_circle = circles[i].id;
			ROS_INFO("found circle %d\n", crossed_circle);
			killCircle(crossed_circle);
		}
	}
	
}


int main(int argc, char** argv){
	
	ros::init(argc, argv, "tutorial_delete_circle_client");
	ros::NodeHandle n;
	
	ros::Subscriber s = n.subscribe("turtle1/pose", 1000, poseCallback);
	
	ros::spin();

	return 0;
	
}

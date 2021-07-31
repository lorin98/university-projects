#include <ros/ros.h>
#include <turtlesim/Circle.h>
#include <turtlesim/GetCircles.h>
#include <turtlesim/SpawnCircle.h>
#include <turtlesim/Pose.h>

std::vector<turtlesim::Circle> circles;


bool getCirclesCallback(turtlesim::GetCircles::Request &req, turtlesim::GetCircles::Response &res){
	ROS_INFO("received a request, i'll try to handle it\n");
	ros::NodeHandle n;

	/* I need to get the coordinates of my spawned circles
	 * I set x and y to -1 so that the Spawn Circle server recognizes me*/

	ros::ServiceClient client = n.serviceClient<turtlesim::SpawnCircle>("tutorial_draw_circle");
	turtlesim::SpawnCircle srv;
	srv.request.x = -1;
	srv.request.y = -1;
	if (client.call(srv)){
			ROS_INFO("get turtles\n");
	}
	else {
		ROS_INFO("failed to call service kill\n");
		return false;		
	}
	circles = srv.response.circles;
	res.circles = circles;
	
	return true;
}


int main(int argc, char** argv){
	
	ros::init(argc, argv, "tutorial_get_circle_server");
	ros::NodeHandle n;

	ROS_INFO("ready to get circles\n");
	//service for get circles
	ros::ServiceServer draw_service = n.advertiseService("tutorial_get_circles", getCirclesCallback);
	
	ros::spin();
	
	return 0;
}

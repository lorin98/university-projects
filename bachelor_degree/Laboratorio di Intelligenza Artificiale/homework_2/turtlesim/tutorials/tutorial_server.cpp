#include <ros/ros.h>
#include <turtlesim/Circle.h>
#include <turtlesim/SpawnCircle.h>
#include <turtlesim/Spawn.h>
#include <vector>
#include <turtlesim/GetCircles.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Pose.h>

#include <stdio.h>	//for sprintf

#define PI 3.14159265

int n_spawns = 0;
std::vector<turtlesim::Circle> circles;

// draw routine
bool drawCircleCallback(turtlesim::SpawnCircle::Request &req, turtlesim::SpawnCircle::Response &res){
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn srv;
	srv.request.x = req.x;
	srv.request.y = req.y;
	// I also want to rotate my new spawned turtle...
	srv.request.theta = PI/2 * n_spawns;

	if (client.call(srv)){
			ROS_INFO("spawn turtle %d\n", n_spawns);
	}
	else {
		ROS_INFO("failed to call service spawn\n");
		return false;		
	}
	turtlesim::Circle c;
	c.id = n_spawns + 2;
	c.x = req.x;
	c.y = req.y;
	circles.push_back(c);
	// store my new circle in the service response
	res.circles = circles;
	n_spawns++;
	return true;
}

// get routine
bool getCirclesCallback(turtlesim::GetCircles::Request &req, turtlesim::GetCircles::Response &res){
	res.circles = circles;
	
	return true;
}

// delete routine
bool deleteCircleCallback(turtlesim::RemoveCircle::Request &req, turtlesim::RemoveCircle::Response &res){
	ros::NodeHandle n;
	ros::ServiceClient client;
	
	
	uint8_t id_to_remove = req.id;
	char id_to_string[8];
	sprintf(id_to_string, "%d", id_to_remove);
	std::string name_to_remove = "turtle";
	name_to_remove += id_to_string;

	// I have to kill this turtle...
	client = n.serviceClient<turtlesim::Kill>("kill");
	turtlesim::Kill srv;
	srv.request.name = name_to_remove;
	
	if (client.call(srv)){
			ROS_INFO("kill turtle %d\n", id_to_remove);
	}
	else {
		ROS_INFO("failed to call service kill\n");
		return false;		
	}
	// ...and remove it from circles
	for (int i=0; i<circles.size(); i++){
		if (circles[i].id == id_to_remove)
			circles.erase(circles.begin() + i);
	}
	res.circles = circles;
	
	return true;
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "tutorial_server");
	ros::NodeHandle n;
	ROS_INFO("Hi, I'm a centralized server and I'll try to handle all your requests...\n");

	ROS_INFO("ready to spawn circles\n");
	//service for draw circles
	ros::ServiceServer draw_service = n.advertiseService("tutorial_draw_circle", drawCircleCallback);
	
	ROS_INFO("ready to get circles\n");
	//service for get circles
	ros::ServiceServer get_service = n.advertiseService("tutorial_get_circles", getCirclesCallback);
	
	ROS_INFO("ready to delete circles");
	//service to delete circles 
	ros::ServiceServer delete_service = n.advertiseService("tutorial_delete_circle", deleteCircleCallback);
	
	
	ros::spin();
	
	return 0;
}

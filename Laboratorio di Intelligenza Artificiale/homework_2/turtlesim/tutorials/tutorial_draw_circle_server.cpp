#include <ros/ros.h>
#include <turtlesim/Circle.h>
#include <turtlesim/SpawnCircle.h>
#include <turtlesim/Spawn.h>
#include <vector>
#define PI 3.14159265

int n_spawns = 0;
std::vector<turtlesim::Circle> circles;

bool drawCircleCallback(turtlesim::SpawnCircle::Request &req, turtlesim::SpawnCircle::Response &res){
	// "false" callback: the request comes from get circles server
	if (req.x == -1 && req.y == -1){
		ROS_INFO("received a request from get circles server, i'll try to handle it\n");
		res.circles = circles;
		return true;
	}
	// normal callback
	ROS_INFO("received a request to draw, i'll try to handle it\n");
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


int main(int argc, char** argv){
	
	ros::init(argc, argv, "tutorial_draw_circle_server");
	ros::NodeHandle n;

	ROS_INFO("ready to spawn circles\n");
	//service for draw circles
	ros::ServiceServer draw_service = n.advertiseService("tutorial_draw_circle", drawCircleCallback);
	
	ros::spin();
	
	return 0;
}

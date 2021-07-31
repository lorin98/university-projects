#include <ros/ros.h>
#include <turtlesim/Circle.h>
#include <turtlesim/GetCircles.h>
#include <turtlesim/RemoveCircle.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Kill.h>

#include <stdio.h>	//for sprintf

std::vector<turtlesim::Circle> circles;


bool deleteCircleCallback(turtlesim::RemoveCircle::Request &req, turtlesim::RemoveCircle::Response &res){
	ROS_INFO("received a request to delete, i'll try to handle it\n");
	ros::NodeHandle n;
	ros::ServiceClient client;
	/* if circles is not initialized I have to get the spawned circles vector,
	 * this operation will be done once*/
	if (circles.size() == 0){
		client = n.serviceClient<turtlesim::GetCircles>("tutorial_get_circles");
		turtlesim::GetCircles get_srv;
	
		if (client.call(get_srv)){
			ROS_INFO("get turtles\n");
		}
		else {
			ROS_INFO("failed to call service tutorial_get_circles\n");
			return false;		
		}
		circles = get_srv.response.circles;
	}
	
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
	
	ros::init(argc, argv, "tutorial_delete_circle_server");
	ros::NodeHandle n;
	
	ROS_INFO("ready to delete circles");
	
	//service to delete circles 
	ros::ServiceServer delete_service = n.advertiseService("tutorial_delete_circle", deleteCircleCallback);
	
	ros::spin();
	
	return 0;
}

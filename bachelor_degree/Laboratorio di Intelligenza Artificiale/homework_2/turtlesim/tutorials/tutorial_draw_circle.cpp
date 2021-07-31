#include <ros/ros.h>
#include <turtlesim/SpawnCircle.h>

#include <stdlib.h> //for atoi and rand
#include <ctime>	//for a better random generator

int main(int argc, char** argv){
	//check if the number of args is correct
	if (argc < 2){
		ROS_INFO("missing arguements: you have to specify how many "
				"turtles you'd like to spawn\n");
		return 1;
	}
	else if (argc > 2){
		ROS_INFO("too many arguments for this node\n");
		return 1;
	}
	
	ros::init(argc, argv, "tutorial_draw_circle_client");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<turtlesim::SpawnCircle>("tutorial_draw_circle");
	turtlesim::SpawnCircle srv;
	
	srand((unsigned) time(0));
	int num_circles = atoi(argv[1]);
	
	float x, y;
	
	for (int i=0; i<num_circles; i++){
		/* the turtlesim window is a cartesian plan where the values of
		 * x and y are included in [0, 11] so use a rand() between 1 and 
		 * 10 could be fine for this target*/
		x = (float)(rand() % 10) +1;
		y = (float)(rand() % 10) +1;
		
		srv.request.x = x;
		srv.request.y = y;
		if (client.call(srv)){
			ROS_INFO("sending a request to spawn a circle\n");
		}
		else {
			ROS_INFO("failed to call service tutorial_draw_circle\n");
			return 1;
		}
	}
	
	return 0;
	
}

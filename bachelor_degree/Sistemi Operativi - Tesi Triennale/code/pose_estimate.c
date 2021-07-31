#include "pose_estimate.h"

//ln personal note: it's my responsibility to deallocate every matd_t* I create

const double camera_to_robot_R_data[9] = {
	0, 0, 1,
	-1, 0, 0,
	0, -1, 0
};

const double camera_to_robot_t_data[3] = {
	TX, TY, TZ
};

//ln rotation matrix and translation vector from camera frame to robot frame
matd_t* camera_to_robot_R = NULL;
matd_t* camera_to_robot_t = NULL;

//ln allocating common matrices
void matd_init(){
	camera_to_robot_R = matd_create_data(3, 3, camera_to_robot_R_data);
	camera_to_robot_t = matd_create_data(3, 1, camera_to_robot_t_data);
#ifdef _VERBOSE_
	printf("camera to robot R and t:\n");
	matd_print(camera_to_robot_R, "%lf");
	matd_print(camera_to_robot_t, "%lf");
#endif

}

matd_t* camera_to_robot_transform(matd_t* pose){
	matd_t* pose_wrt_robot;
	
	//ln p' = R * p + t
	pose_wrt_robot = matd_multiply(camera_to_robot_R, pose);
	matd_add_inplace(pose_wrt_robot, camera_to_robot_t);
#ifdef _VERBOSE_
	printf("camera to robot transform:\n");
	matd_print(pose_wrt_robot, "%lf");
#endif
	//ln remember to destroy it in the caller function
	return pose_wrt_robot; 
}

matd_t* detection_to_pose(apriltag_detection_t *det){
	//ln estimate tag pose
	apriltag_detection_info_t info;
	info.det = det;
	info.tagsize = TAGSIZE;
	info.fx = FX;
	info.fy = FY;
	info.cx = CX;
	info.cy = CY;
	apriltag_pose_t pose;
	estimate_tag_pose(&info, &pose);
#ifdef _VERBOSE_
	printf("translation marix:\n");
	matd_print(pose.t, "%lf");
	printf("rotational matrix:\n");
	matd_print(pose.R, "%lf");
#endif
	//ln I only want the translation
	matd_destroy(pose.R);
	return pose.t;
}

matd_t* get_tag_global_pose(odom_t* current_odom, matd_t* tag_to_robot){
	//ln creating a rotation matrix and a translation vector wrt the world
	double R_data[4] = {
		(double) cosf(current_odom->theta), (double) -sinf(current_odom->theta),
		(double) sinf(current_odom->theta), (double) cosf(current_odom->theta)
	};
	matd_t* rot_matrix = matd_create_data(2, 2, R_data);
	
	double t_data[2] = {
		(double) (current_odom->cxy->data[0]),
		(double) (current_odom->cxy->data[1])
	};
	matd_t* tr_vec = matd_create_data(2, 1, t_data);
	
	matd_t* tag_to_world;
	
	//ln p' = R * p + t
	tag_to_world = matd_multiply(rot_matrix, tag_to_robot);
	matd_add_inplace(tag_to_world, tr_vec);
	
	matd_destroy(tr_vec);
	matd_destroy(rot_matrix);
	//ln remember to destroy it in the caller function
	return tag_to_world;
}

//ln deallocating common matrices
void matd_finish(){
	matd_destroy(camera_to_robot_R);
	matd_destroy(camera_to_robot_t);
#ifdef _VERBOSE_
	printf("matrices deallocated\n");
#endif

}

//ln to create a odom_t struct with given values
odom_t* odom_init(float x, float y, float theta, int id){
	odom_t* odom = (odom_t*) malloc(sizeof(odom_t));
	double data[2] = { (double) x, (double) y };
	matd_t* cxy = matd_create_data(2, 1, data);
	odom->cxy = cxy;
	odom->theta = theta;
	odom->id = id;
	return odom;
}

//ln current coordinates with respect to the previous frame
matd_t* prev_to_current_transform(odom_t* prev_odom, odom_t* current_odom){
	matd_t* res;
	//ln creating a rotation matrix and a translation vector
	double R_data[4] = {
		(double) cosf(prev_odom->theta), (double) -sinf(prev_odom->theta),
		(double) sinf(prev_odom->theta), (double) cosf(prev_odom->theta)
	};
	matd_t* rot_matrix = matd_create_data(2, 2, R_data);

	//ln transform = R^T * (current_odom->cxy - prev_odom->cxy)
	matd_t* rot_matrix_T = matd_transpose(rot_matrix);
	double aux_data[2] = {
		current_odom->cxy->data[0] - prev_odom->cxy->data[0],
		current_odom->cxy->data[1] - prev_odom->cxy->data[1]
	};
	matd_t* aux = matd_create_data(2, 1, aux_data);
	res = matd_multiply(rot_matrix_T, aux);
	
	matd_destroy(aux);
	matd_destroy(rot_matrix_T);
	matd_destroy(rot_matrix);
	
	return res;
}

//ln safely release of an odom_t struct
void odom_destroy(odom_t* odom){
	matd_destroy(odom->cxy);
	free(odom);
}

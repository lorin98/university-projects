#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <apriltag/common/matd.h>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/zarray.h>
#include <apriltag/apriltag_pose.h>

//ln default variations of odometry between two pose estimations
#define DX 0.2
#define DY 0.2
#define DTHETA 0.7854	//ln pi/4

//ln camera_to_robot translation vector components
//ln (thanks to the professor for manual measurements :) )
#define TX -0.255
#define TY -0.0
#define TZ 0.435

//ln tag size in meters
#define TAGSIZE 0.106

//ln camera intrinsic parameters after calibration
#define FX 7.4354009405064676e+02
#define FY 7.4354009405064676e+02
#define CX 3.1950000000000000e+02
#define CY 2.3950000000000000e+02

typedef struct {
	matd_t* cxy;
	float theta;
	int id;
} odom_t;

void matd_init();
void matd_finish();

matd_t* camera_to_robot_transform(matd_t* pose);
matd_t* detection_to_pose(apriltag_detection_t *det);
matd_t* get_tag_global_pose(odom_t* current_odom, matd_t* tag_to_robot);

odom_t* odom_init(float x, float y, float theta, int id);
void odom_destroy(odom_t* odom);

matd_t* prev_to_current_transform(odom_t* prev_odom, odom_t* current_odom);

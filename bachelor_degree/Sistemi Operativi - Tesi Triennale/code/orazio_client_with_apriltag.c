#include <sys/select.h>
#include <termios.h>
#include <pthread.h>
#include <linux/joystick.h>

#include "orazio_client.h"
#include "orazio_print_packet.h"
#include "../orazio_client_test_getkey.h"
//ln including my headers
#include "camera_tools.h"
#include "pose_estimate.h"

#define NUM_JOINTS_MAX 4

typedef struct {
	int fd;
	double max_tv;
	double max_rv;
	int tv_axis;
	int rv_axis;
	int boost_button;
	int halt_button;
	const char* joy_device;
} JoyArgs;


const char* banner[] = {
	"orazio_client_test",
	" minimal program to connect to a configured orazio",
	" use arrow keys or joystick to steer the robot",
	" keys:",
	" 's':    toggles system mode",
	" 'j':    toggles joint mode",
	" 'd':    toggles drive mode",
	" 'ESC'   quits the program",
	"",
	"usage: orazio_client_test [options]",
	"options: ",
	"-serial-device <device>, (default: /dev/ttyACM0)",
	"-joy-device    <device>, (default: /dev/input/js2)",
	"-camera-device <device>, (default: /dev/video0)",
	0
};

static int num_joints=0;

static void printMessage(const char** msg){
	while (*msg){
		printf("%s\n",*msg);
		++msg;
	}
}

typedef enum {
	System=0,
	Joints=1,
	Drive=2,
	None=6,
	Start=-1,
	Stop=-2
} Mode;

// display mode;
Mode mode=Start;
int current_joint=0;

static struct OrazioClient* client=0;

//ln camera struct
camera_t* camera;

// variables filled by the client when we ask for stuff (OrazioClient_get)
// or filled by us and sent to the client when we want to
// issue a command
// they are global since the keyboard/joystick threads might want to alter them
static DifferentialDriveControlPacket drive_control={
	.header.type=DIFFERENTIAL_DRIVE_CONTROL_PACKET_ID,
	.header.size=sizeof(DifferentialDriveControlPacket),
	.header.seq=0,
	.translational_velocity=0,
	.rotational_velocity=0
};

JointControlPacket joint_control[NUM_JOINTS_MAX] = {
  {// Joint 0
    {
      .type=JOINT_CONTROL_PACKET_ID,
      .size=sizeof(JointControlPacket),
      .seq=0,
      .index=0
    },
    {
      .speed=0,
      .mode=JointPWM
    }
  } ,
  {// Joint 1
    {
      .type=JOINT_CONTROL_PACKET_ID,
      .size=sizeof(JointControlPacket),
      .seq=0,
      .index=1
    },
    {
      .speed=0,
      .mode=JointPWM
    }
  },
  {// Joint 2
    {
      .type=JOINT_CONTROL_PACKET_ID,
      .size=sizeof(JointControlPacket),
      .seq=0,
      .index=2
    },
    {
      .speed=0,
      .mode=JointPWM
    }
  } ,
  {// Joint 3
    {
      .type=JOINT_CONTROL_PACKET_ID,
      .size=sizeof(JointControlPacket),
      .seq=0,
      .index=3
    },
    {
      .speed=0,
      .mode=JointPWM
    }
  }
};

// to stop the robot we just schedule the sending
// of a drive message with zero velocities
// and we 
void stopRobot(void){
	for (int i=0; i<num_joints; ++i){
		joint_control[i].control.mode=0;
		joint_control[i].control.speed=0;
		joint_control[i].header.seq=0;
		OrazioClient_sendPacket(client, (PacketHeader*) &joint_control[i], 0);
	}
}

/** this stuff is to read the joystick**/
void* joyThread(void* args_){
	JoyArgs* args=(JoyArgs*)args_;
	args->fd = open (args->joy_device, O_RDONLY|O_NONBLOCK);
	if (args->fd<0) {
		printf ("no joy found on [%s]\n", args->joy_device);
		return 0;
	}
	printf("joy opened\n");
	printf("\t tv_axis: %d\n", args->tv_axis);
	printf("\t rv_axis: %d\n", args->rv_axis);
	printf("\t boost_button: %d\n", args->boost_button);
	printf("\t halt_button: %d\n", args->halt_button);
	printf("\t max_tv=%f\n", args->max_tv);
	printf("\t max_rv=%f\n", args->max_rv);

	float tv = 0;
	float rv = 0;
	float tvscale = args->max_tv/32767.0;
	float rvscale = args->max_rv/32767.0;
	float gain = 1;

	struct js_event e;
	while (mode!=Stop) {
	if (read (args->fd, &e, sizeof(e)) > 0 ){
		fflush(stdout);
		int axis = e.number;
		int value = e.value;
		int type = e.type;
		if (axis == args->tv_axis && type == 2) {
			tv = -value * tvscale;
		}
		if (axis == args->rv_axis && type == 2) {
			rv = -value *rvscale;
		}
		if (axis == args->halt_button && type==1){
			tv = 0;
			rv = 0;
		} else if (axis == args->boost_button && type == 1) {
			if(value)
				gain = 2.0;
			else
				gain = 1.0;
		} 
		drive_control.rotational_velocity=rv*gain;
		drive_control.translational_velocity=tv*gain;
		drive_control.header.seq=1;	//ln here there was a 0 that didn't
									//ln let me send any DDCPacket
	}
	usleep(10000); // 10 ms
	}
	close(args->fd);
	return 0;
};

void* keyThread(void* arg){
	setConioTerminalMode();
	while(mode!=Stop) {
		KeyCode key_code=getKey();
		switch (key_code){
			case KeyEsc:
				mode=Stop;
				break;
			case KeyS:
				mode=System;
				break;
			case KeyJ:
				mode=Joints;
				break;
			case KeyD:
				mode=Drive;
				break;
			case KeyX:
				mode=None;
				break;
			default:
				stopRobot();
		}
	}
	resetTerminalMode();
	return 0;
};

//ln camera stuff
void* cameraThread(void* arg){
	char* camera_device = (char*) arg;
	printf("%s\n", camera_device);
	camera = camera_open(camera_device, 640, 480);
	camera_init(camera);
	camera_start(camera);
	
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	
	while (mode != Stop) {
		//ln the real work for the camera is here
		camera_frame(camera, timeout);
	}
	
	camera_stop(camera);
	camera_finish(camera);
	camera_close(camera);
	
	return 0;
}


void toggleMode(Mode previous_mode, int previous_joint_index) {
	if (mode==previous_mode && previous_joint_index==current_joint)
		return;
		printf("previous mode: %d, current_mode:%d\n", previous_mode, mode);
	//make room to read params
	SystemParamPacket system_params = {
	.header.type=SYSTEM_PARAM_PACKET_ID,
	.header.size=sizeof(SystemParamPacket)
	};

	DifferentialDriveParamPacket drive_params = {
	.header.type=DIFFERENTIAL_DRIVE_PARAM_PACKET_ID,
	.header.size=sizeof(DifferentialDriveParamPacket)
	};

	JointParamPacket joint_params[NUM_JOINTS_MAX];

	for (int i=0; i<NUM_JOINTS_MAX; ++i) {
		joint_params[i].header.type=JOINT_PARAM_PACKET_ID,
		joint_params[i].header.size=sizeof(JointParamPacket),
		joint_params[i].header.index=i;
	}

	ParamPacketHeader* params=0;
	ParamPacketHeaderIndexed* indexed_params=0;
	switch (previous_mode){
		case System:
			params=(ParamPacketHeader*)&system_params;
			break;
		case Drive:
			params=(ParamPacketHeader*)&drive_params;
			break;
		case Joints:
			indexed_params=(ParamPacketHeaderIndexed*)&joint_params[previous_joint_index];
			params=(ParamPacketHeader*) indexed_params;
			break;
		default:;
	}

	//shut up the subsystem
	if (params) {
		printf("shutting down old subsystem, type: %d\n", params->type);
		OrazioClient_get(client, (PacketHeader*)params);
	if (indexed_params) {
		printf("shutting down old subsystem, index: %d\n", indexed_params->index);
		indexed_params->update_enabled=0;
	} else {
		params->update_enabled=0;
	}
	int retries=10;
	OrazioClient_sendPacket(client, (PacketHeader*)params, retries);
	}
      
	params=0;
	indexed_params=0;
	switch (mode){
		case System:
			params=(ParamPacketHeader*)&system_params;
			break;
		case Drive:
			params=(ParamPacketHeader*)&drive_params;
			break;
		case Joints:
			indexed_params=(ParamPacketHeaderIndexed*)&joint_params[current_joint];
			params=(ParamPacketHeader*) indexed_params;
			break;
		default:;
	}

	//make the the subsystem talking
	if (params) {
	// fetch the params
		printf("enabling subsystem, type: %d\n", params->type);

		OrazioClient_get(client, (PacketHeader*)params);
		if (indexed_params) {
			printf("enabling subsystem, index: %d\n", indexed_params->index);
			indexed_params->update_enabled=1;
		} else {
			params->update_enabled=1;
		}
		int retries=10;
		printf("setting params\n");
		OrazioClient_sendPacket(client, (PacketHeader*)params, retries);
	}
}


char* default_joy_device = "/dev/input/js2";
char* default_serial_device = "/dev/ttyACM0";
//ln adding a default camera device
char* default_camera_device = "/dev/video0";


int main(int argc, char** argv) {
	Orazio_printPacketInit();
	// parse the command line arguments
	int c=1;
	char* joy_device = default_joy_device;
	char* serial_device = default_serial_device;
	char* camera_device = default_camera_device;
  
	while(c<argc){
		if (! strcmp(argv[c],"-h")){
			printMessage(banner);
			exit(0);
		} else if (! strcmp(argv[c],"-serial-device")){
			++c;
			if (c<argc)
				serial_device=argv[c];
		} else if (! strcmp(argv[c],"-joy-device")){
			++c;
			if (c<argc)
				joy_device=argv[c];
		}
		else if (! strcmp(argv[c],"-camera-device")){
			++c;
			if (c<argc)
				camera_device=argv[c];
		}
		++c;
	}
	printf("Starting %s with the following parameters\n", argv[0]);
	printf("-serial-device %s\n", serial_device);
	printf("-joy-device %s\n", joy_device);
	printf("-camera-device %s\n", camera_device);
	
  
	// these variables are used locally in this thread
	// to interact with orazio client
	// it will read/write from/to these variables

	SystemStatusPacket system_status={
	.header.type=SYSTEM_STATUS_PACKET_ID,
	.header.size=sizeof(SystemStatusPacket)
	};

	SystemParamPacket system_params={
	.header.type=SYSTEM_PARAM_PACKET_ID,
	.header.size=sizeof(SystemParamPacket)
	};

	DifferentialDriveStatusPacket drive_status = {
	.header.type=DIFFERENTIAL_DRIVE_STATUS_PACKET_ID,
	.header.size=sizeof(DifferentialDriveStatusPacket)
	};

	JointStatusPacket joint_status[NUM_JOINTS_MAX];

	// 1. create an orazio object and initialize it
	client=OrazioClient_init(serial_device, 115200);
	if (! client) {
		printf("cannot open client on device [%s]\nABORTING", serial_device);
		return -1;
	}

  
	// 2. synchronize the serial protocol
	printf("Syncing ");
	for (int i=0; i<50; ++i){
		OrazioClient_sync(client,1);
		printf(".");
		fflush(stdout);
	}
	printf(" Done\n");

	// 3. read the configuration
	if (OrazioClient_readConfiguration(client, 100)!=Success){
		return -1;
	}

	// 4. ask how many motors are on the platform
	//    and initialize the index of each joint
	//    the client will read the index from the destination
	//    packet to know which joint is queried.
	num_joints=OrazioClient_numJoints(client);
	int retries=10;

	// 5. at the beginning we disable sending all packets, to minimize the burden on the serial line
	//    a: ask the clients for the system params (read during readConfiguration)
	OrazioClient_get(client, (PacketHeader*)&system_params);

	//    c: send the packet immediately, and try "retries" times, 
	OrazioClient_sendPacket(client, (PacketHeader*)&system_params, retries);
	//    d: get the parameters refreshed
	OrazioClient_get(client, (PacketHeader*)&system_params);



	// 6. we start a keyboard thread and a joystick thread
	//    these threads will write on
	//    drive_control and joint_control
	pthread_t key_thread;
	pthread_create(&key_thread, 0, keyThread, 0);

	pthread_t joy_thread;
	JoyArgs joy_args = {
		.max_tv=0.3,
		.max_rv=0.5,
		.tv_axis=1,
		.rv_axis=3,
		.boost_button=4,
		.halt_button=5,
		.joy_device=joy_device
	};
	pthread_create(&joy_thread, 0, joyThread,  &joy_args);
	
	//ln starting a camera thread too
	pthread_t camera_thread;
	pthread_create(&camera_thread, 0, cameraThread, camera_device);
	
	//ln create log file
	int fd = open("log.txt", O_CREAT | O_RDWR | O_TRUNC, 0666);
	if (fd < 0){
		printf("error in the open\n");
		exit(EXIT_FAILURE);
	}
	printf("log file opened\n");
	//ln here I store every data to log
	char* to_write = (char*) malloc(sizeof(char)*100);
	int res;
	
	Mode previous_mode=mode;
	int previous_joint=0;
	// 7 the one below is the main loop
	//   the structure is:
	//   a. send in output, all controls you need
	//   b. sync
	//   c. get the variables/status from OrazioClient,
	//      by Orazio_get(cl, dest, packet_idx);
	//   If you need an immediate response after scheduling a command to be sent (with sendPacket)
	//   you need to issue the sync
	while(mode!=Stop){
		// we use the seq field of the drive_control packet
		// to notify we have to send a new control
		// if different from 0, we post the control packet

		// a. send the control
		if(drive_control.header.seq) {
			int result = OrazioClient_sendPacket(client, (PacketHeader*)&drive_control, 0);
			if (result)
				printf("Fail %d\n",(int)result);
			//drive_control.header.seq=0;
		}

		for (int i=0; i<num_joints; ++i){
			if(joint_control[i].header.seq){
				/* printf("\nsending j[%d]: mode=%d speed=%d status=", */
				/*        (int) joint_control[i].header.index, */
				/*        (int) joint_control[i].control.mode, */
				/*        (int)joint_control[i].control.speed); */
				PacketStatus result = OrazioClient_sendPacket(client, (PacketHeader*)&joint_control[i], 0);
				if (result)
					printf("Fail %d\n",(int)result);
				joint_control[i].header.seq=0;
			}
		}

		//here we control the packets sent by the host, depending on the current mode
		toggleMode(previous_mode,previous_joint);
		previous_mode=mode;
		previous_joint=current_joint;

		// b. sync;
		OrazioClient_sync(client,1);

		for (int i=0; i<num_joints; ++i) {
			joint_status[i].header.type=JOINT_STATUS_PACKET_ID;
			joint_status[i].header.size=sizeof(JointStatusPacket);
			joint_status[i].header.index=i;
		}

		// c. read the output (to visualize the data)
		char output_buffer[1024];
		switch(mode){
		case System:
			OrazioClient_get(client, (PacketHeader*)&system_status);
			Orazio_printPacket(output_buffer,(PacketHeader*)&system_status);
			printf("\r\033[2K%s",output_buffer);
			break;
		case Joints:
			OrazioClient_get(client, (PacketHeader*)&joint_status[current_joint]);
			Orazio_printPacket(output_buffer,(PacketHeader*)&joint_status[current_joint]);
			printf("\r\033[2K%s-%d-",output_buffer,current_joint);
			break;
		case Drive:
			OrazioClient_get(client, (PacketHeader*)&drive_status);
			Orazio_printPacket(output_buffer,(PacketHeader*)&drive_status);
			printf("\r\033[2K%s",output_buffer);
			break;
		default:;
		}
		
		//ln get timestamp
		struct timeval timer;
		res = gettimeofday(&timer, NULL);
		if (res < 0){
			printf("error in the gettimeofday\n");
			exit(EXIT_FAILURE);
		}
		//ln record odom on file
		OrazioClient_get(client, (PacketHeader*)&drive_status);
		sprintf(to_write, "<%ld.%ld> ODOM %f %f %f\n", timer.tv_sec, timer.tv_usec,
				drive_status.odom_x, drive_status.odom_y, drive_status.odom_theta);
#ifdef _VERBOSE_
		printf("%s\n", to_write);
#endif
		res = write(fd, to_write, strlen(to_write));
		if (res < 0){
			printf("error in the write\n");
			exit(EXIT_FAILURE);
		}
		//ln get current tag detections
		zarray_t* tag_detections = get_current_detections();
		if (tag_detections != NULL) {
			for (int i=0; i<zarray_size(tag_detections); i++){
				apriltag_detection_t *det;
				zarray_get(tag_detections, i, &det);
				matd_t* tag_to_camera = detection_to_pose(det);
				//ln record tag info on file too
				//ln n.b. current odom and current detected tags timestamps (if any) are the same
				sprintf(to_write, "<%ld.%ld> TAG %d %lf %lf %lf\n", timer.tv_sec, timer.tv_usec,
					det->id, tag_to_camera->data[0], tag_to_camera->data[1], tag_to_camera->data[2]);
#ifdef _VERBOSE
				printf("%s", to_write);
#endif
				res = write(fd, to_write, strlen(to_write));
				if (res < 0){
					printf("error in the write\n");
					exit(EXIT_FAILURE);
				}
				
				matd_destroy(tag_to_camera);
			}
		}
		
	}
	
	//ln closing log file
	free(to_write);
	res = close(fd);
	if (res < 0){
		printf("error in the close\n");
		exit(EXIT_FAILURE);
	}
	printf("log file closed\n");
	
	printf("Terminating\n");
	if (joy_args.fd>0){
		close(joy_args.fd);
	}

	// we reach this when we terminate
	void* arg;
	pthread_join(key_thread, &arg);
	pthread_join(joy_thread, &arg);
	pthread_join(camera_thread, &arg);
	
	printf("Stopping Robot");
	stopRobot();
	for (int i=0; i<10;++i){
		printf(".");
		fflush(stdout);
		OrazioClient_sync(client,10);
	}
	printf("Done\n");
	OrazioClient_destroy(client);
}

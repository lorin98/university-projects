#include "pose_estimate.h"
#include "g2o_graph_build.h"

odom_t* curr_odom = NULL;
odom_t* prev_odom = NULL;
long int curr_time_sec = 0.0;
long int curr_time_nsec = 0.0;

//ln here I "save" the tags I detect in time
//ln n.b. there are max 587 36h11 AprilTags
int known_tags[587];

const char* banner[] = {
	"log_parser",
	" minimal program to parse a text file with odom and tags data",
	"usage: ./log_parser <log_file> (e.g. log.txt) [options]",
	"options: ",
	"-dx <size_in_meters>, (default: 0.2)",
	"-dy    <size_in_meters>, (default: 0.2)",
	"-dtheta <angle_in_radians>, (default: 1.5708)",
	0
};

//ln default bounds between two pose estimations
double dx = DX;
double dy = DY;
double dtheta = DTHETA;

//ln here I store the informations for g2o_graph_build
LogArgs* args;

static void printMessage(const char** msg){
	while (*msg){
		printf("%s\n",*msg);
		++msg;
	}
}

void parse_line(int fd, char* line){
	long int ts_sec, ts_nsec;
	char type[4];
	int id;
	double x, y, theta, z;
	
	sscanf(line, "<%ld.%ld> %s", &ts_sec, &ts_nsec, type);

	//ln if I'm parsing an odom...
	if (strcmp(type, "ODOM") == 0){
		sscanf(line, "<%ld.%ld> %s %lf %lf %lf", &ts_sec, &ts_nsec, type, &x, &y, &theta);
		//ln check if this odom is over the bounds
		curr_odom->cxy->data[0] = x;
		curr_odom->cxy->data[1] = y;
		curr_odom->theta = theta;
		if (fabsf(curr_odom->cxy->data[0] - prev_odom->cxy->data[0]) >= dx ||
				fabsf(curr_odom->cxy->data[1] - prev_odom->cxy->data[1]) >= dy ||
				fabsf(curr_odom->theta - prev_odom->theta) >= dtheta) {
			curr_odom->id++;
			//ln writing a VERTEX_SE2
			args->key = VERTEX_SE2;
			args->id1 = curr_odom->id;
			args->x = curr_odom->cxy->data[0];
			args->y = curr_odom->cxy->data[1];
			args->theta = curr_odom->theta;
			g2o_log(fd, args);	
			
			matd_t* prev_to_current = prev_to_current_transform(prev_odom, curr_odom);
#ifdef _VERBOSE_
			printf("curr pose with respect to prev pose:\n");
			matd_print(prev_to_current, "%lf");
#endif
			//ln writing an EDGE_SE2
			args->key = EDGE_SE2;
			args->id1 = prev_odom->id;
			args->id2 = curr_odom->id;
			args->x = prev_to_current->data[0];
			args->y = prev_to_current->data[1];
			args->theta = curr_odom->theta - prev_odom->theta;
			g2o_log(fd, args);
			
			matd_destroy(prev_to_current);
			
			//ln updating previos odometry...
			prev_odom->cxy->data[0] = curr_odom->cxy->data[0];
			prev_odom->cxy->data[1] = curr_odom->cxy->data[1];
			prev_odom->theta = curr_odom->theta;
			prev_odom->id = curr_odom->id;
			//ln ...and current timestamp
			curr_time_sec = ts_sec;
			curr_time_nsec = ts_nsec;
		}
		
	}
	//ln ...or a tag
	else {
		//ln verifying current timestamp matches with the one I'm reading
		if (curr_time_sec == ts_sec && curr_time_nsec == ts_nsec) {
			sscanf(line, "<%ld.%ld> %s %d %lf %lf %lf", &ts_sec, &ts_nsec, type, &id, &x, &y, &z);
			double data[3] = { x, y, z };
			matd_t* tag_to_camera = matd_create_data(3, 1, data);
			matd_t* tag_to_robot = camera_to_robot_transform(tag_to_camera);
			
			//ln only if this tag is unknown
			if (known_tags[id] == 0){
				//ln get global pose of the detected tag and write a VERTEX_XY
				matd_t* tag_to_global = get_tag_global_pose(curr_odom, tag_to_robot);
				
				args->key = VERTEX_XY;
				args->id1 = id;
				args->x = tag_to_global->data[0];
				args->y = tag_to_global->data[1];
				g2o_log(fd, args);
				matd_destroy(tag_to_global);
				//ln now I know where this tag is, so I "label" it
				known_tags[id] = 1;
			}
			//ln writing an EDGE_SE2_XY
			args->key = EDGE_SE2_XY;
			args->id1 = curr_odom->id;
			args->id2 = id;
			args->x = tag_to_robot->data[0];
			args->y = tag_to_robot->data[1];
			g2o_log(fd, args);
			
			matd_destroy(tag_to_robot);
			matd_destroy(tag_to_camera);
		}	
	}
	
}

int main(int argc, char** argv){
	if (!strcmp(argv[1],"-h")){
		printMessage(banner);
		exit(0);
	}
	//ln opening the file
	char* line;
	FILE* fptr = fopen(argv[1], "r");
	if (fptr == NULL){
		printf("error in the fopen\n");
		exit(EXIT_FAILURE);
	}
	printf("file opened\n");
	
	//ln parse the command line arguments (taken from orazio_client)
	int c=1;
	while(c<argc){
		if (! strcmp(argv[c],"-dx")){
			++c;
			if (c<argc)
				dx = atof(argv[c]);
		} else if (! strcmp(argv[c],"-dy")){
			++c;
			if (c<argc)
				dy = atof(argv[c]);
		}
		else if (! strcmp(argv[c],"-dtheta")){
			++c;
			if (c<argc)
				dtheta = atof(argv[c]);
		}
		++c;
	}
	printf("Starting %s with the following parameters\n", argv[0]);
	printf("file to parse: %s\n", argv[1]);
	printf("-dx %lf\n", dx);
	printf("-dy %lf\n", dy);
	printf("-dtheta %lf\n", dtheta);
	
	//ln initializing g2o_graph_build, odometries and common matd_t*
	int g2o_fd = g2o_init();
	curr_odom = odom_init(0.0, 0.0, 0.0, 100);
	prev_odom = odom_init(0.0, 0.0, 0.0, 100);
	memset(known_tags, 0, sizeof(known_tags));
	matd_init();
	args = (LogArgs*) malloc(sizeof(LogArgs));
	
	//ln first log the initial odom
	args->key = VERTEX_SE2;
	args->id1 = curr_odom->id;
	args->x = curr_odom->cxy->data[0];
	args->y = curr_odom->cxy->data[1];
	args->theta = curr_odom->theta;
	g2o_log(g2o_fd, args);
	int res;
	size_t len = 0;
	
	//ln start reading every line and parsing it until EOF
	printf("start parsing...\n");
	while (getline(&line, &len, fptr) != -1) {
#ifdef _VERBOSE_
		printf("read line: %s", line);
#endif
		parse_line(g2o_fd, line);
	}
	
	printf("finish parsing\nTerminating...\n");
	//ln releasing everything
	free(args);
	matd_finish();
	odom_destroy(curr_odom);
	odom_destroy(prev_odom);
	g2o_close(g2o_fd);
	free(line);
	res = fclose(fptr);
	if (res != 0){
		printf("error in the close\n");
		exit(EXIT_FAILURE);
	}
}

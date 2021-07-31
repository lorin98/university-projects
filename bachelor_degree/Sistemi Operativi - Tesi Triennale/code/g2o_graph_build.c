#include "g2o_graph_build.h"

int g2o_init(){
	int fd = open("out.g2o", O_CREAT | O_RDWR | O_TRUNC, 0666);
	if (fd < 0){
		printf("error in the open\n");
		exit(EXIT_FAILURE);
	}
#ifdef _VERBOSE_
	printf("file opened\n");
#endif
	return fd;
}


void g2o_log(int fd, LogArgs* args){
	char* to_write = (char*) malloc(sizeof(char)*100);
	switch (args->key) {
		//ln VERTEX_XY
		case VERTEX_XY:
			sprintf(to_write, "VERTEX_XY %d %lf %lf\n", args->id1, args->x, args->y);
			break;
			
		//ln VERTEX_SE2
		case VERTEX_SE2:
			sprintf(to_write, "VERTEX_SE2 %d %lf %lf %lf\n", args->id1, args->x, args->y, (double) args->theta);
			break;
			
		//ln EDGE_SE2
		case EDGE_SE2:
			sprintf(to_write, "EDGE_SE2 %d %d %lf %lf %lf 100 0 0 100 0 1000\n", args->id1, 
					args->id2, args->x, args->y, (double) args->theta);
			break;
			
		//ln EDGE_SE2_XY
		case EDGE_SE2_XY:
			sprintf(to_write, "EDGE_SE2_XY %d %d %lf %lf 100 0 100\n", args->id1, args->id2, args->x, args->y);
			break;
		
		//ln invalid key
		default:
			printf("invalid key...could not log on file\n");
			free(to_write);
			return;
	}
	//ln write the created string on file
	ssize_t res = write(fd, to_write, strlen(to_write));
	if (res < 0){
		printf("error in the write\n");
		exit(EXIT_FAILURE);
	}
#ifdef _VERBOSE_
	printf("write on file: %s\n", to_write);
#endif
	free(to_write);
}

void g2o_close(int fd){
	int res = close(fd);
	if (res < 0){
		printf("error in the close\n");
		exit(EXIT_FAILURE);
	}
}

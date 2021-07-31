#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

typedef enum {
	VERTEX_XY = 0,
	VERTEX_SE2 = 1,
	EDGE_SE2 = 2,
	EDGE_SE2_XY = 3
} TYPES;

typedef struct {
	TYPES key;
	int id1;
	int id2;		//ln this field will not be set in VERTEX_*
	double x;
	double y;
	float theta;	//ln this field will not be set in VERTEX_XY 
					//ln or in EDGE_SE2_XY
} LogArgs;

int g2o_init();
void g2o_log(int fd, LogArgs* args);
void g2o_close(int fd);

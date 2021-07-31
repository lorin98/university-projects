#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <asm/types.h>
#include <linux/videodev2.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/getopt.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <apriltag/apriltag_pose.h>

#include <stdio.h>	//ln for sprintf


#define FALSE 0
#define TRUE  1

void quit(const char * msg);
int xioctl(int fd, int request, void* arg);

typedef struct {
  uint8_t* start;
  size_t length;
} buffer_t;

typedef struct {
  int fd;
  uint32_t width;
  uint32_t height;
  buffer_t head;        // buffer for the current image

  size_t buffer_count;
  buffer_t* buffers;    // image buffers four nimage buffers
} camera_t;

camera_t* camera_open(const char * device, uint32_t width, uint32_t height);
void camera_init(camera_t* camera);
void camera_start(camera_t* camera);
void camera_stop(camera_t* camera);
void camera_finish(camera_t* camera);
void camera_close(camera_t* camera);
int camera_capture(camera_t* camera);
int camera_frame(camera_t* camera, struct timeval timeout);
//ln this function will be used by orazio client to get current tag detections
zarray_t* get_current_detections();

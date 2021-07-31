#include "camera_tools.h"
#include "pose_estimate.h"

//ln here I'll store my tags detection
zarray_t *detections = NULL;

//ln using 'capture_camera' seen at lesson as starting point

void quit(const char * msg) {
	fprintf(stderr, "[%s] %d: %s\n", msg, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

int xioctl(int fd, int request, void* arg) {
	for (int i = 0; i < 100; i++) {
		int r = ioctl(fd, request, arg);
		if (r != -1 || errno != EINTR) return r;
	}
	return -1;
}

/*
  Opens the camera device and stores the requested image size in the camera struct
*/

camera_t* camera_open(const char * device, uint32_t width, uint32_t height) {
	int fd = open(device, O_RDWR | O_NONBLOCK, 0);
	if (fd == -1) quit("open camera");
	camera_t* camera = malloc(sizeof (camera_t));
	camera->fd = fd;
	camera->width = width;
	camera->height = height;
	camera->buffer_count = 0;
	camera->buffers = NULL;
	camera->head.length = 0;
	camera->head.start = NULL;
	printf("device opened\n");
	return camera;
}

/*
  1. queries the capability of he camera
  2. checks if device supports cropping
  3. allocates memory buffers for dma operation
  4. sets up mmap with the requested buffers
*/
void camera_init(camera_t* camera) {
	struct v4l2_capability cap;
	if (xioctl(camera->fd, VIDIOC_QUERYCAP, &cap) == -1) quit("VIDIOC_QUERYCAP");
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) quit("no capture");
	if (!(cap.capabilities & V4L2_CAP_STREAMING)) quit("no streaming");
	printf("camera supports capture and streaming\n");

	struct v4l2_cropcap cropcap;
	memset(&cropcap, 0, sizeof cropcap);
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(camera->fd, VIDIOC_CROPCAP, &cropcap) == 0) {
		struct v4l2_crop crop;
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect;
		if (xioctl(camera->fd, VIDIOC_S_CROP, &crop) == -1) {
		// cropping not supported
		}
	}
	printf("camera supports cropping\n");

	struct v4l2_format format;
	memset(&format, 0, sizeof format);
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = camera->width;
	format.fmt.pix.height = camera->height;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	if (xioctl(camera->fd, VIDIOC_S_FMT, &format) == -1) quit("VIDIOC_S_FMT");
	printf("set format to %d x %d\n", camera->width, camera->height);

	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof req);
	req.count = 1;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (xioctl(camera->fd, VIDIOC_REQBUFS, &req) == -1) quit("VIDIOC_REQBUFS");
	camera->buffer_count = req.count;
	camera->buffers = calloc(req.count, sizeof (buffer_t));
	printf("allocated %d buffers\n", req.count);

	//here we do a mmap for the buffer
	size_t buf_max = 0;
	struct v4l2_buffer buf;
	memset(&buf, 0, sizeof buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = 0;
	if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buf) == -1)
		quit("VIDIOC_QUERYBUF");
	if (buf.length > buf_max) buf_max = buf.length;
	camera->buffers[0].length = buf.length;
	camera->buffers[0].start = 
      mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, 
           camera->fd, buf.m.offset);
    if (camera->buffers[0].start == MAP_FAILED) quit("mmap");
    printf("mmapping buffer\n");
  
	camera->head.start = malloc(buf_max);
}

// starts the streaming (one single xioctl)
void camera_start(camera_t* camera) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1) quit("VIDIOC_QBUF"); // query for a buffer
    
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(camera->fd, VIDIOC_STREAMON, &type) == -1) 
		quit("VIDIOC_STREAMON");
}

// stops the streaming
void camera_stop(camera_t* camera) {
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(camera->fd, VIDIOC_STREAMOFF, &type) == -1) 
		quit("VIDIOC_STREAMOFF");
}

// unmaps the buffers
void camera_finish(camera_t* camera) {
    munmap(camera->buffers[0].start, camera->buffers[0].length);
    
	free(camera->buffers);
	camera->buffer_count = 0;
	camera->buffers = NULL;
	free(camera->head.start);
	camera->head.length = 0;
	camera->head.start = NULL;
	
	//ln destroy detections too
	apriltag_detections_destroy(detections);
}

// closes the device
void camera_close(camera_t* camera) {
	if (close(camera->fd) == -1) quit("close");
	free(camera);
}


// captures a frame from the current buffer
int camera_capture(camera_t* camera) {
	struct v4l2_buffer buf;
	memset(&buf, 0, sizeof buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	if (xioctl(camera->fd, VIDIOC_DQBUF, &buf) == -1) return FALSE; // buffer exchange with the driver - full
	
	//ln opencv integration
	IplImage* frame;
	CvMat cvmat = cvMat(640, 480, CV_8UC3, (void*) camera->buffers[0].start);
	frame = cvDecodeImage(&cvmat, 1);
	IplImage* gray_frame = cvCreateImage(cvGetSize(frame), 8, 1);
	cvCvtColor(frame, gray_frame, CV_BGR2GRAY);
	
	//ln apriltag integration
	apriltag_family_t *tf = NULL;
	tf = tag36h11_create();
	apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
	
	//ln convert Mat to image_u8_t
	image_u8_t im = { .width = gray_frame->width,
            .height = gray_frame->height,
            .stride = gray_frame->width,
            .buf = (uint8_t*) gray_frame->imageData
        };
	detections = apriltag_detector_detect(td, &im);
#ifdef _VERBOSE_
	printf("tags detected: %d\n", zarray_size(detections));
#endif
	for (int i=0; i<zarray_size(detections); i++){
		//ln starting from 'opencv_demo.cc' provided from apriltag and
		//ln change it a little
		apriltag_detection_t *det;
		zarray_get(detections, i, &det);
		cvLine(frame, cvPoint(det->p[0][0], det->p[0][1]),
				 cvPoint(det->p[1][0], det->p[1][1]),
				 CV_RGB(0, 0xff, 0), 2, 8, 0);
		cvLine(frame, cvPoint(det->p[0][0], det->p[0][1]),
				 cvPoint(det->p[3][0], det->p[3][1]),
				 CV_RGB(0xff, 0, 0), 2, 8, 0);
		cvLine(frame, cvPoint(det->p[1][0], det->p[1][1]),
				 cvPoint(det->p[2][0], det->p[2][1]),
				 CV_RGB(0, 0, 0xff), 2, 8, 0);
		cvLine(frame, cvPoint(det->p[2][0], det->p[2][1]),
				 cvPoint(det->p[3][0], det->p[3][1]),
				 CV_RGB(0, 0, 0xff), 2, 8, 0);
				 
		//ln stuff to identify the tag id...
		char* text = (char*) malloc(sizeof(int));
		sprintf(text, "%d", det->id);
		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0f, 1.0f, 0, 3, 8);
		int baseline;
		CvSize text_size;
		cvGetTextSize(text, &font, &text_size, &baseline);
#ifdef _VERBOSE_
		printf("tag id: %s\n", text);
#endif
		//ln ...and "print" it on the frame 	 
		cvPutText(frame, text, cvPoint(det->c[0]-text_size.width/2,
                                       det->c[1]+text_size.height/2),
                   &font, CV_RGB(0, 0x99, 0xff));
		
		free(text);
		
	}
	cvNamedWindow("Apriltag Detector", CV_WINDOW_AUTOSIZE);
	cvShowImage("Apriltag Detector", frame);
	
	apriltag_detector_destroy(td);
	tag36h11_destroy(tf);
	
	cvWaitKey(10);
	cvReleaseImage(&frame);
	cvReleaseImage(&gray_frame);
	if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1) return FALSE; // buffer exchange with the driver - empty
	return TRUE;
}

int camera_frame(camera_t* camera, struct timeval timeout) {
	// waits fror a new frame, when camera ready
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(camera->fd, &fds);
	int r = select(camera->fd + 1, &fds, 0, 0, &timeout);
	if (r == -1) quit("select");
	if (r == 0) return FALSE;
	return camera_capture(camera);
}

zarray_t* get_current_detections(){
	return detections;
}


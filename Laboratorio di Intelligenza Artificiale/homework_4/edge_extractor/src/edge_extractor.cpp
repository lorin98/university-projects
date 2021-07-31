#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

const std::string FIRST_WINDOW = "Original image window";
const std::string SECOND_WINDOW = "Filtered image window";


void imageCallback(const sensor_msgs::CompressedImage &msg){
	// setting original image
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv::imshow(FIRST_WINDOW, cv_ptr->image);

	// convert to grayscale
	cv::Mat gray_img;
	cv::cvtColor(cv_ptr->image, gray_img, CV_BGR2GRAY);

	// thresholding to "clean" the grayscale image
	double low_threshold = 127;
	double high_threshold = 255;
	cv::threshold(gray_img, gray_img, low_threshold, high_threshold, CV_THRESH_BINARY);
	
	// applying a Gaussian Blur
	unsigned int kernel_size = 3;
	cv::Mat blur_img;
	cv::GaussianBlur(gray_img, blur_img, cv::Size(kernel_size, kernel_size), 0.0, 0.0);
	
	// applying the Canny corner detector
	cv::Mat canny_img;
	cv::Canny(blur_img, canny_img, low_threshold, high_threshold, kernel_size);
	
	// showing the final filtered image 
	cv::imshow(SECOND_WINDOW, canny_img);
	cv::waitKey(10);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "edge_extractor");
	ros::NodeHandle n;
	
	ros::Subscriber s = n.subscribe("/default/camera_node/image/compressed", 1000, imageCallback);

	ros::spin();
	
	return 0;
}

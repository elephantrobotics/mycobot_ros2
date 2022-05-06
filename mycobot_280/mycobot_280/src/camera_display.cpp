#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv, "image_listener");
	auto camera = std::make_shared
	ros::NodeHandle nh;
	cv::namedWindow("view");
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
	rclcpp::spin();
	cv::destroyWindow("view");
}

/// ROS
#include <ros/ros.h>

/// Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

/// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>


ros::NodeHandle *nh;

cv::Mat inputimage, rgbImage;
cv_bridge::CvImagePtr cv_ptr_raw;

bool firsttime = true;


/// Callback when we get an image
void rawImageCallback(const sensor_msgs::Image::ConstPtr& raw_image_msg){

		//convert out of scary cv_bridge land
    try
    {
        cv_ptr_raw = cv_bridge::toCvCopy(raw_image_msg,"8UC3");
    }
    catch (cv_bridge::Exception& error)
    {
        ROS_ERROR("cv_bridge exception: %s", error.what());
        return;
    }

    /// Keep a copy of the raw image
    cv_ptr_raw->image.copyTo(inputimage);

		if(firsttime){
			firsttime = false;
			std::cout << "image size: " << inputimage.cols << ", " << inputimage.rows << "\n";
		}

		//show it for now
    cv::imshow("input", inputimage);
		cv::waitKey(15);

}

int main (int argc, char** argv)
{
    /// Initialize ROS
    ros::init (argc, argv, "simple_capture");
    nh = new ros::NodeHandle("~");

    // subscribe
    ros::Subscriber sub = nh->subscribe("/usb_cam/image_raw", 1, rawImageCallback);

    // Spin
    ros::spin ();

}

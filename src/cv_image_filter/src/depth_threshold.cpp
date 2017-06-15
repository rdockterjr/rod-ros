
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


#define depthsize_x 480
#define depthsize_y 360
#define colorsize_x 640
#define colorsize_y 480


ros::Publisher scan_pub;
ros::NodeHandle *nh;

cv::Mat masker;
cv::Mat depthimage, rgbimage, coloredimage, scaledimage;
cv_bridge::CvImagePtr cv_ptr_raw_depth, cv_ptr_raw_color;

bool firsttime = true;


//this is nessecary to be in a useful range for CV_8UC1 images (otherwise everything is white)
cv::Mat scaleDepth(cv::Mat input){
	//https://software.intel.com/en-us/forums/realsense/topic/538066
	// https://software.intel.com/en-us/forums/realsense/topic/595121 gives 0.5 to 3.5 meters
	// http://stackoverflow.com/questions/6909464/convert-16-bit-depth-cvmat-to-8-bit-depth

	// range distance for the camera (changing these values changes your working range (units mm)
	unsigned short min = 500.0; //mm
	unsigned short max = 3500.0; //mm
	cv::Mat imgscaled = cv::Mat::zeros(input.size(), CV_8UC1);
	double scalef = 255.0 / (max - min); //for scaling
	//double scalef = 1.0 / 256.0; //=0.00390625 pure 16 bit to 8 bit scaling factor

	//should also try this
	//cv::Mat norm16;
	//cv::normalize(input, norm16, 0, 255, cv::NORM_MINMAX);
	//input.convertTo(norm16, CV_8UC1);

	//convert to 8UC1 with a scale factor
	input.convertTo(imgscaled, CV_8UC1, scalef);

	return imgscaled;
}


/// Callback when we get an image
void depthImageCallback(const sensor_msgs::Image::ConstPtr& raw_image_msg){

		//convert out of scary cv_bridge land
    try
    {
        cv_ptr_raw_depth = cv_bridge::toCvCopy(raw_image_msg,"16UC1");
    }
    catch (cv_bridge::Exception& error)
    {
        ROS_ERROR("cv_bridge exception: %s", error.what());
        return;
    }

    /// Keep a copy of the raw image
    cv_ptr_raw_depth->image.copyTo(depthimage);

		if(firsttime){
			//firsttime = false;
			std::cout << "image size: " << depthimage.cols << ", " << depthimage.rows << "\n";
		}

		//scale it
		scaledimage = cv::Mat::zeros(depthimage.size(), CV_8UC1);
		scaledimage = scaleDepth(depthimage);

		//find the mid range
		cv::Mat maskersmall;
		
		cv::inRange(scaledimage,40, 100,maskersmall);
		cv::resize(maskersmall, masker, cv::Size(colorsize_x, colorsize_y) );
		

		//show it for now
    cv::imshow("scaledimage", scaledimage);
		cv::imshow("masker", masker);
		cv::waitKey(15);

}

/// Callback when we get an image
void colorImageCallback(const sensor_msgs::Image::ConstPtr& raw_image_msg){

		//convert out of scary cv_bridge land
    try
    {
        cv_ptr_raw_color = cv_bridge::toCvCopy(raw_image_msg,"8UC3");
    }
    catch (cv_bridge::Exception& error)
    {
        ROS_ERROR("cv_bridge exception: %s", error.what());
        return;
    }

    /// Keep a copy of the raw image
    cv_ptr_raw_color->image.copyTo(rgbimage);

		cv::cvtColor(rgbimage, coloredimage, CV_RGB2BGR);

		if(firsttime){
			firsttime = false;
			std::cout << "image size: " << coloredimage.cols << ", " << coloredimage.rows << ", " << coloredimage.channels() << "\n";
		}

		cv::Mat depthmasked;
		coloredimage.copyTo(depthmasked, masker);


		//show it for now
    cv::imshow("coloredimage", coloredimage);
		cv::imshow("depthmasked", depthmasked);
		cv::waitKey(15);

}


int main (int argc, char** argv)
{
    /// Initialize ROS
    ros::init (argc, argv, "depthimage_filter");
    nh = new ros::NodeHandle("~");


    // subscribe
    ros::Subscriber sub1 = nh->subscribe("/camera/depth/image_raw", 1, depthImageCallback);
		ros::Subscriber sub2 = nh->subscribe("/camera/color/image_raw", 1, colorImageCallback);

    // Spin
    ros::spin ();

}

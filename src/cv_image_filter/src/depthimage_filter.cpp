
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

ros::Publisher scan_pub;
ros::NodeHandle *nh;

cv::Mat inputimage, scaledimage;
cv_bridge::CvImagePtr cv_ptr_raw;

bool firsttime = true;

#define num_readings 480 //image is 480 x 360
int midcol = 180; //360 / 2;
double laser_frequency = 40;
double ranges[num_readings];
double intensities[num_readings];

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
void rawImageCallback(const sensor_msgs::Image::ConstPtr& raw_image_msg){

		//convert out of scary cv_bridge land
    try
    {
        cv_ptr_raw = cv_bridge::toCvCopy(raw_image_msg,"16UC1");
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

		//scale it
		scaledimage = cv::Mat::zeros(inputimage.size(), CV_8UC1);
		scaledimage = scaleDepth(inputimage);

		//populate the LaserScan message
		ros::Time scan_time = ros::Time::now();
		sensor_msgs::LaserScan scan;
		scan.header.stamp = scan_time;
		scan.header.frame_id = "laser_frame";
		scan.angle_min = -1.02/2.0; //59 degree cone
		scan.angle_max = 1.02/2.0;
		scan.angle_increment = 1.02 / num_readings;
		scan.time_increment = 30.0 / (num_readings);
		scan.range_min = 500.0;
		scan.range_max = 3500.0;

		//scan it
		scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
			uchar val = inputimage.at<uchar>(midcol, i);
			scan.ranges[i] = double(val);
			scan.intensities[i] = 1;
    }

		
    scan_pub.publish(scan);

		//show it for now
    cv::imshow("scaledimage", scaledimage);
		cv::waitKey(15);

}

int main (int argc, char** argv)
{
    /// Initialize ROS
    ros::init (argc, argv, "depthimage_filter");
    nh = new ros::NodeHandle("~");

    // advertise
		scan_pub = nh->advertise<sensor_msgs::LaserScan>("scan", 50);

    // subscribe
    ros::Subscriber sub = nh->subscribe("/camera/depth/image_raw", 1, rawImageCallback);

    // Spin
    ros::spin ();

}

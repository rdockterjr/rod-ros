#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <iostream>

bool add(beginner_tutorials::AddTwoInts::Request &req, beginner_tutorials::AddTwoInts::Response &res)
{
	long int a = (long int) req.a;
	long int b = (long int) req.b;

	long int summer = a + b;
	res.sum = summer;

	std::cout << "a= " << a << " b= " << b << "\n";
	std::cout << "sum= " << summer << "\n";

	return true;
}


int main(int argc, char **argv){
	ros::init(argc,argv,"add_two_ints_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("add_two_ints",add);
	std::cout << "ready to add two ints \n";

	ros::spin();
	
	return 1;

}

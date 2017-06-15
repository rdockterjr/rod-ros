#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>
#include <iostream>


int main(int argc, char **argv){

	ros::init(argc,argv,"add_two_ints_client");
	if(argc != 3){
		std::cout << "usage add_two_ints_client X Y \n";
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");

	beginner_tutorials::AddTwoInts srv;
	srv.request.a = atoi(argv[1]);
	srv.request.b = atoi(argv[2]);
	if(client.call(srv)){
		std::cout << " response sum = " << srv.response.sum << "\n";
	}
	else{
		std::cout << "uh oh error \n";
		return 1;
	}
	
	return 0;
}

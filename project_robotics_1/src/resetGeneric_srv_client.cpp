//ROBOTICS PROJECT 1
//10617369, Gabriele Ribolla


#include "ros/ros.h"
#include "project_1/ResetGen.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "resetGeneric_service_client");

    if (argc != 4)

    {
        ROS_INFO("usage: resetGeneric_service_client X [m] Y [m] Theta [rad]");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<project_1::ResetGen>("resetGen");
    project_1::ResetGen srv;
    srv.request.x = std::stof(argv[1]);
    srv.request.y = std::stof(argv[2]);
    srv.request.theta = std::stof(argv[3]);

    if (client.call(srv)) {
        ROS_INFO("Position reset at (%f,%f) with orientation theta: %f", srv.request.x, srv.request.y, srv.request.theta);
    } else {
        ROS_ERROR("Failed to call service resetGen");
        return 1;
    }
    return 0;
}

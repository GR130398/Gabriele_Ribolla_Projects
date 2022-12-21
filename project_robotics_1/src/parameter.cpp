//ROBOTICS PROJECT 1
//10617369, Gabriele Ribolla


#include "ros/ros.h"

#include <dynamic_reconfigure/server.h>
#include <project_1/parameters_Config.h>
#include "project_1/parameters_msgs.h"


class robot_parameter {

public:

    robot_parameter() {


        this->pub = this->n.advertise<project_1::parameters_msgs>("/parameters", 1000);
        this->f = boost::bind(&robot_parameter::callback_param, this, _1);
        this->server.setCallback(f);

    }

    void callback_param(project_1::parameters_Config &config){

        /*
         Header header
         float64 r
         float64 lx
         float64 ly
         float64 T
         float64 N
        */

        ros::Time t = ros::Time::now();
        parameters_chosen.header.stamp = t;
        parameters_chosen.N = config.N;
        parameters_chosen.T = config.T;
        parameters_chosen.r = config.r;
        parameters_chosen.ly = config.ly;
        parameters_chosen.lx = config.lx;
    }

    void main_loop() {
        ros::Rate loop_rate(49); // avg hz of cmd_vel

        while (ros::ok()) {

            ros::spinOnce();

            this->pub.publish(parameters_chosen);

            loop_rate.sleep();
        }
    }

private:

    ros::NodeHandle n;
    ros::Publisher pub;
    dynamic_reconfigure::Server<project_1::parameters_Config> server;
    dynamic_reconfigure::Server<project_1::parameters_Config>::CallbackType f;
    project_1::parameters_msgs parameters_chosen;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "parameter");

    robot_parameter my_robot_parameter;

    my_robot_parameter.main_loop();

    return 0;
}
//ROBOTICS PROJECT 1
//10617369, Gabriele Ribolla

#include <ros/ros.h>
#include "project_1/Wheel_rpm.h" // to include the custom message generated
#include "geometry_msgs/TwistStamped.h"
#include "project_1/parameters_msgs.h"

class computeWhlRpm_calculator

{

public:

    computeWhlRpm_calculator()

    {

        this->pub =  this->n.advertise<project_1::Wheel_rpm>("/wheels_rpm", 1);


        this->sub =this->n.subscribe("/cmd_vel", 1, &computeWhlRpm_calculator::callback, this);


        this->n.getParam("/r", this->r);
        this->n.getParam("/lx", this->lx);
        this->n.getParam("/ly", this->ly);
        this->n.getParam("/T", this->gearRatio);
        this->n.getParam("/N", this->N);


        this->sub_param=this->n.subscribe("/parameters", 1, &computeWhlRpm_calculator::callback_param_msg, this);
        this->wheel2enc = (60*this->gearRatio);

    }
    void callback_param_msg(const project_1::parameters_msgs &param)
    {

        this->param_chosen = param;
        this->r = param_chosen.r;
        this->lx=this->param_chosen.lx;
        this->ly=this->param_chosen.ly;
        this->gearRatio=this->param_chosen.T;
        this->N=this->param_chosen.N;
        this->wheel2enc = (60*this->gearRatio);

    }

    void callback(const geometry_msgs::TwistStamped& cmd_vel)
    {

        project_1::Wheel_rpm wheel_rpm_output;
        this->vx = cmd_vel.twist.linear.x;
        this->vy = cmd_vel.twist.linear.y;
        this->w =cmd_vel.twist.angular.z;


        wheel_rpm_output.header.stamp.sec = cmd_vel.header.stamp.sec;
        wheel_rpm_output.header.stamp.nsec = cmd_vel.header.stamp.nsec;
        wheel_rpm_output.header.frame_id ="base_link";
        wheel_rpm_output.rpm_fl = this->wheel2enc * (1/this->r) * ((-this->lx-this->ly) * this->w + this->vx - this->vy); //w1
        wheel_rpm_output.rpm_fr = this->wheel2enc * (1/this->r) * ((this->lx+this->ly) * this->w + this->vx + this->vy);  //w2
        wheel_rpm_output.rpm_rr = this->wheel2enc * (1/this->r) * ((this->lx+this->ly) * this->w + this->vx - this->vy);  //w3
        wheel_rpm_output.rpm_rl = this->wheel2enc * (1/this->r) * ((-this->lx-this->ly) * this->w + this->vx + this->vy); //w4

        this->pub.publish(wheel_rpm_output);

    }

private:

    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber sub_param;

    double r;
    double lx;
    double ly;
    double gearRatio;
    double N;
    double wheel2enc;

    double vx;
    double vy;
    double w;
    project_1::parameters_msgs param_chosen;

};//End of class computeWhlRpm_calculator

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "compute_Whl_Rpm");


    computeWhlRpm_calculator my_computeWhlRpm;

    ros::spin();

    return 0;
}

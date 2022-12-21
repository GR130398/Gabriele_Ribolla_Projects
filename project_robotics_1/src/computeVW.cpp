//ROBOTICS PROJECT 1
//10617369, Gabriele Ribolla


#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "project_1/time_msg.h"
#include "std_msgs/String.h"
#include <project_1/wh_parametersConfig.h>
#include <dynamic_reconfigure/server.h>
#include <project_1/time_msg.h>
#include "project_1/parameters_msgs.h"

class computeVW_calculator {

public:

    computeVW_calculator() {

        this->pub = this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
        this->custom = this->n.advertise<std_msgs::String>("/input_method", 1000);



        this->f = boost::bind(&computeVW_calculator::callback_param, this, _1);
        this->server.setCallback(f);

        //uncommented to use param from file .launch
        /*
        this->n.getParam("/r", this->r);
        this->n.getParam("/lx", this->lx);
        this->n.getParam("/ly", this->ly);
        this->n.getParam("/T", this->gearRatio);
        this->n.getParam("/N", this->N);
        this->msEnc2RadMwh = 2*M_PI*(1/this->N)*(1/this->gearRatio);
        this->enc2wheel = 1/(60*this->gearRatio);
        */

        //Topic you want to subscribe
        this->sub =this-> n.subscribe("/wheel_states", 1, &computeVW_calculator::callback, this);
        this->sub_param =this->n.subscribe("/parameters", 1, &computeVW_calculator::callback_param_msg, this);

    }


    void callback_param_msg(const project_1::parameters_msgs &param)
    {

        //comment to use param from file .launch instead of dynamic_reconfigure
        this->param_chosen = param;
        this->r = param_chosen.r;
        this->lx=this->param_chosen.lx;
        this->ly=this->param_chosen.ly;
        this->gearRatio=this->param_chosen.T;
        this->N=this->param_chosen.N;
        this->msEnc2RadMwh = 2*M_PI*(1/this->N)*(1/this->gearRatio);
        this->enc2wheel = 1/(60*this->gearRatio);

    }

    void callback(const sensor_msgs::JointState& msg)
    {

        geometry_msgs::TwistStamped cmd_vel;


        if (this->Method_chosen.data == "from_enc_velocity")

            //RPM, con rad/min to rad/s
            {

                double radMfl = msg.velocity[0];
                double radMfr = msg.velocity[1];
                double radMrl = msg.velocity[2];
                double radMrr = msg.velocity[3];
                this->wfl = radMfl * this->enc2wheel;
                this->wfr = radMfr * this->enc2wheel;
                this->wrl = radMrl * this->enc2wheel;
                this->wrr = radMrr * this->enc2wheel;

                //ROS_INFO("in rad/s");
                //ROS_INFO("I heard: [%f][%f][%f][%f]", wfl, wfr, wrl, wrr);

            }

            else {

                this->t_k = this->t_k_1; // previous value

                this->t_k_1.sec = msg.header.stamp.sec;
                this->t_k_1.nsec = msg.header.stamp.nsec;


                if (!this->firstTime) {

                  this->Ts = this->t_k_1.sec - this->t_k.sec + (this->t_k_1.nsec -this->t_k.nsec) * (exp10(-9));

                    if (this->Ts < 0)
                        this->firstTime = true;

                }

                if (this->firstTime) {

                    this->Ts = 0.020; // dato da 1/48.8 hz
                    this->tick_k_fl = msg.position[0];
                    this->tick_k_fr = msg.position[1];
                    this->tick_k_rl = msg.position[2];
                    this->tick_k_rr = msg.position[3];

                    firstTime = false;

                } else {

                    this->tick_k_1_fl = msg.position[0];
                    this->tick_k_1_fr = msg.position[1];
                    this->tick_k_1_rl = msg.position[2];
                    this->tick_k_1_rr = msg.position[3];

                    this->dTicks_rl = this->tick_k_1_rl -this->tick_k_rl;
                    this->dTicks_rr = this->tick_k_1_rr - this->tick_k_rr;
                    this->dTicks_fl = this->tick_k_1_fl - this->tick_k_fl;
                    this->dTicks_fr = this->tick_k_1_fr - this->tick_k_fr;

                    this->tick_k_rl =this->tick_k_1_rl;
                    this->tick_k_rr = this->tick_k_1_rr;
                    this->tick_k_fl = this->tick_k_1_fl;
                    this->tick_k_fr = this->tick_k_1_fr;


                    this->wfl = (this->dTicks_fl / this->Ts) * this->msEnc2RadMwh;
                    this->wfr = (this->dTicks_fr / this->Ts) * this->msEnc2RadMwh;
                    this->wrr = (this->dTicks_rr / this->Ts) *this-> msEnc2RadMwh;
                    this->wrl = (this->dTicks_rl / this->Ts) *this-> msEnc2RadMwh;
                }
            }


        double vx = (this->wfl+this->wfr+this->wrl+this->wrr)*(this->r/4);
        double vy = (-this->wfl + this->wfr+this->wrl-this->wrr)*(this->r/4);
        double w =  (-this->wfl+this->wfr-this->wrl+this->wrr)*(this->r/(4*(this->lx+this->ly)));


        cmd_vel.header.stamp.sec = msg.header.stamp.sec;
        cmd_vel.header.stamp.nsec = msg.header.stamp.nsec;
        cmd_vel.header.frame_id = "base_link";

        cmd_vel.twist.linear.x = vx;
        cmd_vel.twist.linear.y = vy;
        cmd_vel.twist.linear.z = 0;

        cmd_vel.twist.angular.x = 0;
        cmd_vel.twist.angular.y = 0;
        cmd_vel.twist.angular.z = w;

        this->pub.publish(cmd_vel);

        std_msgs::String r;

        this->n.getParam("/compute_VW/method_input_wh_states", this->Method_chosen.data);

        if (this->Method_chosen.data == "from_enc_velocity")
            r.data = "_from_enc_velocity";
        else
            r.data = "_from_enc_position";


        this->custom.publish(r);

    }
    void callback_param(project_1::wh_parametersConfig &config) {
        this->Method_chosen.data = config.method_input_wh_states;

        if (this->Method_chosen.data == "from_enc_velocity")
            ROS_INFO("Method chosen from_enc_velocity\n\n");
        else if (this->Method_chosen.data == "from_enc_position")
            ROS_INFO("Method chosen from_enc_position\n\n");
        else
            ROS_INFO(
                    "Incorrect parameter /compute_odom/method! Automatically set to from_enc_position!\nUse 'from_enc_position' or 'from_enc_position'\n\n");

    }

private:

    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber sub_param;
    ros::Publisher custom;

    bool firstTime = true;

    double r;
    double lx;
    double ly;
    double gearRatio;
    double N;

    double msEnc2RadMwh;
    double wfl;  // front left pos 0
    double wfr; // front right pos 1
    double wrr; // rear right pos 3
    double wrl; // rear left pos 2
    double Ts;

    project_1::time_msg t_k_1;
    project_1::time_msg t_k;

    double dTicks_rl;
    double dTicks_rr;
    double dTicks_fl;
    double dTicks_fr;
    double tick_k_1_fl;
    double tick_k_1_fr;
    double tick_k_1_rl;
    double tick_k_1_rr;

    double tick_k_rl;
    double tick_k_rr;
    double tick_k_fl;
    double tick_k_fr;
    project_1::parameters_msgs param_chosen;

    std_msgs::String Method_chosen;
    dynamic_reconfigure::Server <project_1::wh_parametersConfig> server;
    dynamic_reconfigure::Server<project_1::wh_parametersConfig>::CallbackType f;

    double enc2wheel;

};//End of class computeVW_calculator

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "compute_VW");


    computeVW_calculator my_computeVW;

    ros::spin();

    return 0;
}

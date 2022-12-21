//ROBOTICS PROJECT 1
//10617369, Gabriele Ribolla


#include "ros/ros.h"
#include "project_1/recap.h"
#include "project_1/ResetGen.h"
#include <project_1/parametersConfig.h>
#include <dynamic_reconfigure/server.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include "math.h"
#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <project_1/time_msg.h>
#include "std_msgs/String.h"


class odom_calculator {

public:

    odom_calculator() {
        this->firstTime = true;
        this->srvResGen = n.advertiseService("resetGen", &odom_calculator::rstGen, this);

        this->f = boost::bind(&odom_calculator::callback_param, this, _1);
        this->server.setCallback(f);

        this->k_1 = this->n.advertise<nav_msgs::Odometry>("/odom", 100);
       this->Vel = this->n.subscribe("/cmd_vel", 1, &odom_calculator::callback_vel, this);

        this->custom = this->n.advertise<project_1::recap>("/cstm_msg", 1000);
        this->robot_pose = this->n.subscribe("/robot/pose", 1, &odom_calculator::callback_robotPose, this);
        this->pub_time =this->n.advertise<project_1::time_msg>("/time_msg", 1000);
        this->pubInitialPose= this->n.advertise<nav_msgs::Odometry>("/initialPose", 100);

    }


    void callback_robotPose(const geometry_msgs::PoseStampedConstPtr &robotPose)
    {

          this->initial_pose.header.stamp = ros::Time::now();
          this->initial_pose.header.frame_id = "odom";
          this->initial_pose.child_frame_id = "base_link";

        this->initial_pose.pose.pose.position.x=0; //robotPose->pose.position.x;
        this->initial_pose.pose.pose.position.y=0; //robotPose->pose.position.y;
        this->initial_pose.pose.pose.position.z=0; //robotPose->pose.position.z;

        this->initial_pose.pose.pose.orientation.x = 0; //robotPose->pose.orientation.x;
        this->initial_pose.pose.pose.orientation.y = 0; //robotPose->pose.orientation.y;
        this->initial_pose.pose.pose.orientation.z = robotPose->pose.orientation.z;
        this->initial_pose.pose.pose.orientation.w = robotPose->pose.orientation.w;



    }

    void callback_vel(const geometry_msgs::TwistStamped::ConstPtr &cmd_vel) {

        // set the time
        this->t_k = this->t_k_1;

        // set the poses
        this->old_pose =this->new_pose;
        this->old_theta = this->new_theta;

        this->t_k_1.sec = cmd_vel->header.stamp.sec;
        this->t_k_1.nsec = cmd_vel->header.stamp.nsec;

        // set linear velocity from cmd_vel
        this->old_pose.twist.twist.linear.x = cmd_vel->twist.linear.x;
        this->old_pose.twist.twist.linear.y = cmd_vel->twist.linear.y;
        this->old_pose.twist.twist.linear.z = cmd_vel->twist.linear.z;

        // set angular velocity from cmd_vel
        this->old_pose.twist.twist.angular.x= cmd_vel->twist.angular.x ;
        this->old_pose.twist.twist.angular.y =cmd_vel->twist.angular.y ;
        this->old_pose.twist.twist.angular.z = cmd_vel->twist.angular.z;

        this->vx = cmd_vel->twist.linear.x;
        this->vy = cmd_vel->twist.linear.y;
        this->w = cmd_vel->twist.angular.z;

       this->compute_odometry();

    }

    void compute_odometry() {

        double Ts;
        double delta_x;
        double delta_y;
        double delta_th;

        if (!this->firstTime) {

            Ts = this->t_k_1.sec - this->t_k.sec + (this->t_k_1.nsec - this->t_k.nsec) * (exp10(-9));

           //ROS_INFO(" Ts %f \n\n", Ts);

            if (Ts < 0) this->firstTime = true;
        }


        if (this->firstTime) {

            // set initial new pose
            this->new_pose.pose.pose.position.x=0;
            this->new_pose.pose.pose.position.y=0;
            this->new_pose.pose.pose.position.z=0;

            // set initial new pose orientation
            this->new_pose.pose.pose.orientation.x=0;
            this->new_pose.pose.pose.orientation.y=0;
            this->new_pose.pose.pose.orientation.z=0;
            this->new_pose.pose.pose.orientation.w=0;

            // this-> old_pose = this->initial_pose; // da commentare se si usano i valori dal launch

            //   to uncomment to use them instead to use the way to speed up the switch betweeen bags


                // set initial pose from launch
              this->n.getParam("/p_x", this->old_pose.pose.pose.position.x);
              this->n.getParam("/p_y", this->old_pose.pose.pose.position.y);
              this->n.getParam("/p_z", this->old_pose.pose.pose.position.z);

              // set initial orientation from launch
              this->n.getParam("/o_x", this->old_pose.pose.pose.orientation.x);
              this->n.getParam("/o_y", this->old_pose.pose.pose.orientation.y);
              this->n.getParam("/o_z", this->old_pose.pose.pose.orientation.z);
              this->n.getParam("/o_w", this->old_pose.pose.pose.orientation.w);
           // */



            this->pubInitialPose.publish(this->old_pose);
            this->n.getParam("/odom_comp/method", this->Method_chosen.data);

           // set initial linear and angular velocities
            this->new_pose.twist.twist.linear.x =0;
            this->old_pose.twist.twist.linear.x =0;
            this->new_pose.twist.twist.linear.y =0;
            this->old_pose.twist.twist.linear.y =0;
            this->new_pose.twist.twist.linear.z =0;
            this->old_pose.twist.twist.linear.z=0;

            // set initial angular velocity
            this->new_pose.twist.twist.angular.x =0;
            this->old_pose.twist.twist.angular.x=0;
            this->new_pose.twist.twist.angular.y = 0;
            this->old_pose.twist.twist.angular.y =0;
            this->new_pose.twist.twist.angular.z =0;
            this->old_pose.twist.twist.angular.z=0;


            this->vx=0;
            this->vy=0;
            this->w=0;

            delta_x=0;
            delta_y=0;
            delta_th=0;

            this->old_theta=0;
            this->new_theta = 0;



            pub_odom_tf(this->old_pose);

            // define the value of yaw from initial quaternion taken by launch file
            tf2::Quaternion q(this->old_pose.pose.pose.orientation.x, this->old_pose.pose.pose.orientation.y,
                             this-> old_pose.pose.pose.orientation.z, this->old_pose.pose.pose.orientation.w);
            this->old_theta = tf2::impl::getYaw(q);
            this->th = this->old_theta;
            this->firstTime = false;

            //due the fact that all the speeds are equal to zero, at the end of the computation we will have;
            this->new_pose = this->old_pose;

        } else {


            if (this->Method_chosen.data == "Euler") {

                delta_x = (this->vx * cos(this->th) -this->vy * sin(this->th)) * Ts;
                delta_y = (this->vx * sin(this->th) + this->vy * cos(this->th)) * Ts;


                this->new_pose.pose.pose.position.x =this->old_pose.pose.pose.position.x + delta_x;
                this->new_pose.pose.pose.position.y =this->old_pose.pose.pose.position.y + delta_y;

                // ROS_INFO("Euler");

            } else {

                double appRK =((this->old_pose.twist.twist.angular.z *Ts) / 2);


                delta_x = (this->vx * cos(this->th+appRK) - vy * sin(this->th+appRK)) * Ts;
                delta_y = (this->vx * sin(this->th+appRK) + this->vy * cos(this->th+appRK)) * Ts;


                this->new_pose.pose.pose.position.x =this->old_pose.pose.pose.position.x + delta_x;
                this->new_pose.pose.pose.position.y =this->old_pose.pose.pose.position.y + delta_y;

                // ROS_INFO("Runge-Kutta");

            }


            delta_th =this->w * Ts;

            this->new_theta = this->old_theta + delta_th;

            this->th = this->old_theta + delta_th;
            this->new_theta = this-> th;

            // define the quaternion from the value of theta, roll=pitch=0  (2D motion)
            tf2::Quaternion q;
            q.setRPY(0, 0, this->new_theta);

            // no variation along z
            this->new_pose.pose.pose.position.z = old_pose.pose.pose.position.z;

            // set  pose
            this->new_pose.pose.pose.orientation.x = q.getX();
            this->new_pose.pose.pose.orientation.y = q.getY();
            this->new_pose.pose.pose.orientation.z = q.getZ();
            this->new_pose.pose.pose.orientation.w = q.getW();

            // set  linear velocity
            this->new_pose.twist.twist.linear.x = this->old_pose.twist.twist.linear.x;
            this->new_pose.twist.twist.linear.y = this->old_pose.twist.twist.linear.y;
            this->new_pose.twist.twist.linear.z = this->old_pose.twist.twist.linear.z;

            // set angular velocities
            this->new_pose.twist.twist.angular.x = this->old_pose.twist.twist.angular.x;
            this->new_pose.twist.twist.angular.y = this->old_pose.twist.twist.angular.y;
            this->new_pose.twist.twist.angular.z = this->old_pose.twist.twist.angular.z;

            pub_odom_tf(this->new_pose);

        }

    }

    void pub_odom_tf(nav_msgs::Odometry odom) {

        ros::Time t = ros::Time::now();

        this->transformStamped.header.stamp = t;
        this->transformStamped.header.frame_id = "odom";
        this->transformStamped.child_frame_id = "base_link";

       this->transformStamped.transform.translation.x = odom.pose.pose.position.x;
       this->transformStamped.transform.translation.y = odom.pose.pose.position.y;
        this->transformStamped.transform.translation.z = odom.pose.pose.position.z;

        this->transformStamped.transform.rotation.x = odom.pose.pose.orientation.x;
        this->transformStamped.transform.rotation.y = odom.pose.pose.orientation.y;
        this->transformStamped.transform.rotation.z = odom.pose.pose.orientation.z;
        this->transformStamped.transform.rotation.w = odom.pose.pose.orientation.w;

        this->br2.sendTransform(this->transformStamped);

        odom.header.stamp = t;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";


        this->k_1.publish(odom);

        project_1::recap r;
        r.odom = odom;
        this->n.getParam("/odom_comp/method", this->Method_chosen.data);

        if (this->Method_chosen.data == "Euler")
            r.method.data = "euler";
        else
            r.method.data = "rk";

        this->custom.publish(r);

    }

    bool rstGen(project_1::ResetGen::Request &req, project_1::ResetGen::Response &res) {

        this->new_pose.pose.pose.position.x = req.x;
        this->new_pose.pose.pose.position.y = req.y;
        this->new_theta = req.theta;

        tf2::Quaternion q;
        q.setRPY(0, 0, this->new_theta);

        this->new_pose.pose.pose.orientation.x = q.getX();
        this->new_pose.pose.pose.orientation.y = q.getY();
        this->new_pose.pose.pose.orientation.z = q.getZ();
        this->new_pose.pose.pose.orientation.w = q.getW();

        this->pub_odom_tf(this->new_pose);

        //ROS_INFO("Odometry reset to x: %f, y: %f, theta: %f\n\n", req.x, req.y, req.theta);
        return true;
    }


    void callback_param(project_1::parametersConfig &config) {
        this->Method_chosen.data = config.method;


        if (this->Method_chosen.data == "Euler")
            ROS_INFO("Method chosen Euler\n\n");
        else if (this->Method_chosen.data == "Runge-Kutta")
            ROS_INFO("Method chosen Runge-Kutta\n\n");
        else
            ROS_INFO(
                    "Incorrect parameter /odom_comp/method! Automatically set to Runge-Kutta!\nUse 'Euler' or 'Runge-Kutta'\n\n");

    }

private:

    ros::NodeHandle n;

    ros::Subscriber Vel;
    ros::Subscriber robot_pose;
    ros::Publisher k_1;
    ros::Publisher custom;
    ros::Publisher pub_time;
    ros::ServiceServer srvResGen;

    ros::Publisher pubInitialPose;
    dynamic_reconfigure::Server <project_1::parametersConfig> server;
    dynamic_reconfigure::Server<project_1::parametersConfig>::CallbackType f;

    nav_msgs::Odometry old_pose;
    nav_msgs::Odometry new_pose;
    nav_msgs::Odometry initial_pose;

    double new_theta;
    double old_theta;

    project_1::time_msg t_k_1;
    project_1::time_msg t_k;

    bool firstTime ;

    std_msgs::String Method_chosen;

    double vx;
    double vy;
    double th;
    double w;

    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::TransformBroadcaster br2;

};


int main(int argc, char **argv) {

    ros::init(argc, argv, "odom_compute");

    odom_calculator my_odom_comp;

    ros::spin();

    return 0;

}
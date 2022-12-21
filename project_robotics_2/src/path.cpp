//
// Created by gr130398 on 01/07/22.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class Path_class
{
    nav_msgs::Path path_msg;

private:
    ros::NodeHandle n;
    int i = 0;
    ros::Subscriber sub;
    ros::Publisher  pub;
    ros::Timer timer_a;

public :
    Path_class()
    {
    sub = n.subscribe("/amcl_pose",1000,&Path_class::path_callback,this);
    pub = n.advertise<nav_msgs::Path>("/path",1000);
    }

    void path_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = msg->header.frame_id;
    pose.pose.position.x = msg->pose.pose.position.x;
    pose.pose.position.y = msg->pose.pose.position.y;

    path_msg.header.seq = msg->header.seq;
    path_msg.header.stamp.sec = msg->header.stamp.sec;
    path_msg.header.stamp.nsec = msg->header.stamp.nsec;
    path_msg.header.frame_id = msg->header.frame_id;
    path_msg.poses.push_back(pose);
    pub.publish(path_msg);

    }

};
int main (int argc,char **argv)
{
    ros::init(argc,argv,"ps_path");
    Path_class my_path;

    ros::spin();

    return 0;
}
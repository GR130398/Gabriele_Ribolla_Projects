#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"

class tf_sub_pub_odometry {
public:
    tf_sub_pub_odometry() {
        sub = n.subscribe("/odom", 1000, &tf_sub_pub_odometry::callback, this);
    }

    void callback(const nav_msgs::OdometryConstPtr &base) {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_footprint";
        transformStamped.transform.translation.x = base->pose.pose.position.x;
        transformStamped.transform.translation.y = base->pose.pose.position.y;
        transformStamped.transform.translation.z = base->pose.pose.position.z;
        transformStamped.transform.rotation.x = base->pose.pose.orientation.x;
        transformStamped.transform.rotation.y = base->pose.pose.orientation.y;
        transformStamped.transform.rotation.z = base->pose.pose.orientation.z;
        transformStamped.transform.rotation.w = base->pose.pose.orientation.w;
        br.sendTransform(transformStamped);
    }

private:
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    ros::Subscriber sub;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "transf_base");
    tf_sub_pub_odometry my_tf_sub_pub_odm;
    ros::spin();
    return 0;
}

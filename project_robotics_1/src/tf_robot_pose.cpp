//ROBOTICS PROJECT 1
//10617369, Gabriele Ribolla


#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseStamped.h"

class tf_sub_pub_robot_pose {

public:

    tf_sub_pub_robot_pose() {

        this->sub = n.subscribe("/robot/pose", 1000, &tf_sub_pub_robot_pose::callback, this);

    }

    void callback(const geometry_msgs::PoseStampedConstPtr &wrld) {

        this->transformStamped.header.stamp = ros::Time::now();
        this->transformStamped.header.frame_id = "world";
        this->transformStamped.child_frame_id = "robot/pose_pose";
        this->transformStamped.transform.translation.x = wrld->pose.position.x;
        this->transformStamped.transform.translation.y = wrld->pose.position.y;
        this->transformStamped.transform.translation.z = wrld->pose.position.z;
        this->transformStamped.transform.rotation.x = wrld->pose.orientation.x;
        this->transformStamped.transform.rotation.y = wrld->pose.orientation.y;
        this->transformStamped.transform.rotation.z = wrld->pose.orientation.z;
        this->transformStamped.transform.rotation.w = wrld->pose.orientation.w;
        this->br.sendTransform(transformStamped);

    }

private:

    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    ros::Subscriber sub;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "tf_robot_pose");
    tf_sub_pub_robot_pose my_tf_sub_pub_robot_pose;
    ros::spin();
    return 0;

}

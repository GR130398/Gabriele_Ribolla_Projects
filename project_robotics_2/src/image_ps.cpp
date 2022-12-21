//
// Created by gr130398 on 01/07/22.
//
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;
using namespace cv;



class Image_ps
{
public:
    Image_ps()
    {
        image_transport::ImageTransport it_(this->n_);

       image_pub_ = it_.advertise("/image_topic", 1);
        this->sub_ = n_.subscribe("/amcl_pose", 1, &Image_ps::callback, this);
        n_.getParam("/dir", this->directory);
        this->image= imread(directory,IMREAD_COLOR);
    }

    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = msg->header.stamp;
        pose.header.frame_id = msg->header.frame_id;
        pose.pose.position.x = msg->pose.pose.position.x;
        pose.pose.position.y = msg->pose.pose.position.y;


       if (msg->header.seq!=0){
            Point p1(x,y), p2((msg->pose.pose.position.x+31.2)/0.05, 600-(msg->pose.pose.position.y+10)/0.05);
            int thickness = 1;
            // Line drawn using 8 connected Bresenham algorithm
            line(this->image, p1, p2, Scalar(255, 0, 0),thickness, LINE_8);

        }
        this->x = (msg->pose.pose.position.x+31.2)/0.05;
        this->y = 600-(msg->pose.pose.position.y+10)/0.05;


        ros::Time time = ros::Time::now();
        picture = image;
        cv::namedWindow("image", CV_WINDOW_NORMAL);
        //cv::resizeWindow("image", 1024,1024);
        //cv::imshow("image", image);
        cv::waitKey(1);
        cv::destroyWindow("image");

        cv_ptr->encoding = "bgr8";
        cv_ptr->header.stamp = time;
        cv_ptr->header.frame_id = "/image_topic";

        cv_ptr->image = image;
        this->image_pub_.publish(cv_ptr->toImageMsg());
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    image_transport::Publisher image_pub_ ;
    Mat image;
    cv::Mat picture;
    std::string im_name;
    nav_msgs::Path path_msg;
    std::string directory;

    int x;
    int y;
};//End of class Image_ps

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_ps");


    Image_ps image_pub;

    ros::spin();

    return 0;
}
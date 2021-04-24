#include "ros/ros.h"

//Remove ones that are not used
#include "ar_track_alvar/AlvarMarkers.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//Cleanup--------------------------------------------------------------
#include <iostream>
#include <string>
// #include <thread>
// #include <chrono>
// #include <mutex>
#include <random>
#include <cmath>
// #include <condition_variable>
#include <vector>

//Delete what is not used-------------------------------------------------
class BlockDetection
{
public:
    BlockDetection(ros::NodeHandle nh);
    ~BlockDetection();

    void detection(void);

    ros::NodeHandle nh_;

private:
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);

    cv_bridge::CvImagePtr cvPtr_; //Delete if not used------------

    cv::Mat image_;//delete if not used--------------
    ros::Subscriber odom_sub_;
    ros::Subscriber laser_sub_;
    ros::Publisher cmd_vel_pub_;
    image_transport::ImageTransport it_; //Not used?---------------------------
    image_transport::Publisher image_pub_; //Check if used- use for debugging------
    image_transport::Subscriber image_sub_;
    geometry_msgs::Pose tagPose;
}
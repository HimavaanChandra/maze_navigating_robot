#include "block_detection.h"

//Check message types and addons in cmake and package.xml

TurtleFollow::TurtleFollow(ros::NodeHandle nh)
    : nh_(nh), it_(nh)//can probs get rid of the it_----------------------------------------------------
{
    //need to initialise all the variable values

    //Topic robot_0 might be different------------------------------------------------------
    //Passing by reference (&TurtleFollow) might be a problem-------------------------------
    odom_sub_ = nh_.subscribe("/robot_0/odom", 10, &TurtleFollow::odomCallback, this);
    laser_sub_ = nh_.subscribe("/robot_0/base_scan", 10, &TurtleFollow::laserCallback, this);
    
    image_transport::ImageTransport it(nh); //Can probs remove and use it_ in header
    //Check topic name if should use compressed or not----------------------------------------------------------------------------------
    image_sub_ = it.subscribe("/camera/rgb/image_raw", 1, &Pursuit::imageCallback, this);

    //Topic robot_0 might be different------------------------------------------------------
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);
}

TurtleFollow::~TurtleFollow()
{
}

//Might not use-----------------------------------------------------------------------------
void TurtleFollow::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    robot_.ranges_ = msg->ranges;
}

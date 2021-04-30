#include "block_detection.h"

//Check message types and addons in cmake and package.xml

BlockDetection::BlockDetection(ros::NodeHandle nh)
    : nh_(nh), it_(nh) //can probs get rid of the it_----------------------------------------------------
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

BlockDetection::~BlockDetection()
{
}

//Might not use-----------------------------------------------------------------------------
void BlockDetection::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    robot_.ranges_ = msg->ranges;
}

//May not need------------------------------------------------------------------------
void BlockDetection::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
}

void BlockDetection::detection(void)
{
	//Convert image
	//BGR2HSV
	cv::Mat image_ = cv::imread("D:/Documents/UTS/Mechatronics/Advanced robotics/Assignment 3/maze_navigating_robot/imageTuning/image2.jpg");
	cv::Mat hsv; //Make class variable?-------
	cv::cvtColor(image_, hsv, cv::COLOR_BGR2HSV);

	//https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
	//Threshold to isolate Red
	cv::Mat redMask;
	//Sim
	// cv::inRange(hsv, cv::Scalar(0, 127, 50), cv::Scalar(6, 255, 255), redMask);
	//Real Robot
	cv::inRange(hsv, cv::Scalar(0, 127, 50), cv::Scalar(6, 255, 255), redMask);
	//Draw box around red blob
	cv::Mat edges;
	cv::Canny(redMask, edges, 400, 1400, 3);
	//Find contours
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	//Draw contours
	cv::drawContours(image_, contours, 0, 255, -1);
	// cv::drawContours( image_, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
	// cv::drawContours(output_image, hulls, 0, 255, -1);
	//Get centre position of blob
	//Get moments of contours
	cv::Moments m = cv::moments(contours, true);
	//centre of mass
	int cx = m.m10 / m.m00;
	int cy = m.m01 / m.m00;

	cv::imshow("image", image_);
	cv::waitKey();
    //Publish image to topic for debugging and report

//Method 1:
    //Use lidar to work out where block is in frame
    //Plot on map

//Method 2:
    //Override turtlebot control --Will need to blear waypoints or set a bool to override
    //Turn towards red blob
    //Drive towards blob untill it reaches a certain scale in frame
    //Stop the robot
    
}

void Pursuit::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    //New
    image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    ///

    //Vid sub
    try
    {
        cv::imshow("video_subscriber", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    ////////////////////////////////////////////////////////////////////////////////

    namespace enc = sensor_msgs::image_encodings;
    
    cv_bridge::CvImagePtr cvPtr_;

    try
    {
        if (enc::isColor(msg->encoding))
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
        else
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    imageDataBuffer_.imageDeq_.push_back(cvPtr_->image);
}

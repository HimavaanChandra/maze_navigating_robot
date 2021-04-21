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

    //Threshold to isolate Red

    //Draw box around red blob

    //Get centre position of blob

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
    //Python
    detection_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                          rows,
    columns, res = detection_image.shape

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

    //Vid pub
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    ///////////////////

    namespace enc = sensor_msgs::image_encodings;

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

    imageDataBuffer_.mtx_.lock();
    imageDataBuffer_.imageDeq_.push_back(cvPtr_->image);

    if (imageDataBuffer_.imageDeq_.size() > 2)
    {
        imageDataBuffer_.imageDeq_.pop_front();
    }
    imageDataBuffer_.mtx_.unlock();
}
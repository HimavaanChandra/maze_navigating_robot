#include "brick_search.h"

void BrickSearch::pathPlanning(double x, double y)
{
    if (getGoalReachedStatus() == 3 || lock == false) // Only navigate to new goal if the current goal has been reached (goal reached status == 3, goal not reached status == 1) and robot is localised
    {
        lock = true;

        goal = {};

        // i++;
        goal.pose.position.x = x;
        goal.pose.position.y = y;

        robot_pose_2d = getPose2d().theta;
        quaternion.setRPY(0, 0, robot_pose_2d);

        goal.pose.orientation.w = quaternion.getW();
        goal.pose.orientation.x = quaternion.getX();
        goal.pose.orientation.y = quaternion.getY();
        goal.pose.orientation.z = quaternion.getZ();

        goal.header.frame_id = "map";

        move_base_simple_goal_.publish(goal);

        ROS_INFO("Sending goal...");
    }
}

void BrickSearch::mainLoop()
{
    // Wait for the TurtleBot to localise
    ROS_INFO("Localising...");
    while (ros::ok())
    {

        // Turn slowly
        geometry_msgs::Twist twist{};
        twist.angular.z = 1.;
        cmd_vel_pub_.publish(twist);

        if (localised_)
        {
            ROS_INFO("Localised");
            break;
        }

        ros::Duration(0.1).sleep();
    }

    // Stop turning
    geometry_msgs::Twist twist{};
    twist.angular.z = 0.;
    cmd_vel_pub_.publish(twist);

    // The map is stored in "map_"
    // You will probably need the data stored in "map_.info"
    // You can also access the map data as an OpenCV image with "map_image_"

    // Here's an example of getting the current pose and sending a goal to "move_base":
    // geometry_msgs::Pose2D pose_2d = getPose2d();

    // ROS_INFO_STREAM("Current pose: " << pose_2d);

    // // Move forward 0.5 m
    // pose_2d.x += 0.5 * std::cos(pose_2d.theta);
    // pose_2d.y += 0.5 * std::sin(pose_2d.theta);

    // ROS_INFO_STREAM("Target pose: " << pose_2d);

    // // Send a goal to "move_base" with "move_base_action_client_"
    // move_base_msgs::MoveBaseActionGoal action_goal{};

    // action_goal.goal.target_pose.header.frame_id = "map";
    // action_goal.goal.target_pose.pose = pose2dToPose(pose_2d);

    // ROS_INFO("Sending goal...");
    // move_base_action_client_.sendGoal(action_goal.goal);

    int i = 0;
    // This loop repeats until ROS shuts down, you probably want to put all your code in here
    while (ros::ok())
    {
        ROS_INFO("mainLoop");

        // Delay so the loop doesn't run too fast
        ros::Duration(0.2).sleep();

        BrickSearch::pathPlanning(1.5, 3);

        // move_base_action_client_.sendGoal(action_goal.goal);

        // Desired seach x,y position
        // Covert into required units (x,y or meters)

        // path_planning_algorithm(desired position) - From package or custom?

        // pure_pursuit_algorithm(planned path) - From package or custom? (package probably better?)

        // If box found? - Inside pure_pursuit_algorithm? Or here is ok if pure_pursuit is seperate thread or ROS package
        // Use open CV to move straight to the box or path planning to recalculate a path to the box position (calulcated using lidar or depth camera with current robot position)
    }
}

// Constructor
BrickSearch::BrickSearch(ros::NodeHandle &nh) : it_{nh}
{

    // Wait for "static_map" service to be available
    ROS_INFO("Waiting for \"static_map\" service...");
    ros::service::waitForService("static_map");

    // Get the map
    nav_msgs::GetMap get_map{};

    if (!ros::service::call("static_map", get_map))
    {
        ROS_ERROR("Unable to get map");
        ros::shutdown();
    }
    else
    {
        map_ = get_map.response.map;
        ROS_INFO("Map received");
    }

    // This allows you to access the map data as an OpenCV image
    map_image_ = cv::Mat(map_.info.height, map_.info.width, CV_8U, &map_.data.front());

    // Wait for the transform to be become available
    ROS_INFO("Waiting for transform from \"map\" to \"base_link\"");
    while (ros::ok() && !transform_buffer_.canTransform("map", "base_link", ros::Time(0.)))
    {
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Transform available");

    // Subscribe to "amcl_pose" to get pose covariance
    amcl_pose_sub_ = nh.subscribe("amcl_pose", 1, &BrickSearch::amclPoseCallback, this);

    // Subscribe to the camera
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &BrickSearch::imageCallback, this);

    // Subscribe to the goal status
    move_base_status_sub_ = nh.subscribe("/move_base/status", 1, &BrickSearch::moveBaseStatusCallback, this);

    // Advertise "cmd_vel" publisher to control TurtleBot manually
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

    // Advertise "move_base_simple_goal" publisher for robot path planning goal
    move_base_simple_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, false);

    // Action client for "move_base"
    ROS_INFO("Waiting for \"move_base\" action...");
    move_base_action_client_.waitForServer();
    ROS_INFO("\"move_base\" action available");

    // Reinitialise AMCL
    ros::ServiceClient global_localization_service_client = nh.serviceClient<std_srvs::Empty>("global_localization");
    std_srvs::Empty srv{};
    global_localization_service_client.call(srv);
}

geometry_msgs::Pose2D BrickSearch::getPose2d()
{
    // Lookup latest transform
    geometry_msgs::TransformStamped transform_stamped =
        transform_buffer_.lookupTransform("map", "base_link", ros::Time(0.), ros::Duration(0.2));

    // Return a Pose2D message
    geometry_msgs::Pose2D pose{};
    pose.x = transform_stamped.transform.translation.x;
    pose.y = transform_stamped.transform.translation.y;

    double qw = transform_stamped.transform.rotation.w;
    double qz = transform_stamped.transform.rotation.z;

    pose.theta = qz >= 0. ? wrapAngle(2. * std::acos(qw)) : wrapAngle(-2. * std::acos(qw));

    return pose;
}

void BrickSearch::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg)
{

    // Check the covariance
    double frobenius_norm = 0.;

    for (const auto e : pose_msg.pose.covariance)
    {
        frobenius_norm += std::pow(e, 2.);
    }

    frobenius_norm = std::sqrt(frobenius_norm);

    if (frobenius_norm < 0.05)
    {
        localised_ = true;

        // Unsubscribe from "amcl_pose" because we should only need to localise once at start up
        amcl_pose_sub_.shutdown();
    }
    // else
    // {
    //   localised_ = false;
    // }
}

void BrickSearch::imageCallback(const sensor_msgs::ImageConstPtr &image_msg_ptr)
{
    // Use this method to identify when the brick is visible

    // The camera publishes at 30 fps, it's probably a good idea to analyse images at a lower rate than that
    if (image_msg_count_ < 15)
    {
        image_msg_count_++;
        return;
    }
    else
    {
        image_msg_count_ = 0;
    }

    // Copy the image message to a cv_bridge image pointer
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_msg_ptr);

    // This is the OpenCV image
    cv::Mat &image = image_ptr->image;

    // You can set "brick_found_" to true to signal to "mainLoop" that you have found a brick
    // You may want to communicate more information
    // Since the "imageCallback" and "mainLoop" methods can run at the same time you should protect any shared variables
    // with a mutex
    // "brick_found_" doesn't need a mutex because it's an atomic

    ROS_INFO("imageCallback");
    ROS_INFO_STREAM("brick_found_: " << brick_found_);
}

void BrickSearch::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray &moveBaseStatus_msg_ptr)
{

    if (moveBaseStatus_msg_ptr.status_list.size() > 0)
    {
        setGoalReachedStatus(moveBaseStatus_msg_ptr.status_list.back().status);
        // std::cout << "front: " << moveBaseStatus_msg_ptr.status_list.front().status << std::endl;
        // std::cout << "back: " << moveBaseStatus_msg_ptr.status_list.back().status << std::endl;
    }
}

double BrickSearch::wrapAngle(double angle)
{
    // Function to wrap an angle between 0 and 2*Pi
    while (angle < 0.)
    {
        angle += 2 * M_PI;
    }

    while (angle > (2 * M_PI))
    {
        angle -= 2 * M_PI;
    }

    return angle;
}

geometry_msgs::Pose BrickSearch::pose2dToPose(const geometry_msgs::Pose2D &pose_2d)
{
    geometry_msgs::Pose pose{};

    pose.position.x = pose_2d.x;
    pose.position.y = pose_2d.y;

    pose.orientation.w = std::cos(pose_2d.theta);
    pose.orientation.z = std::sin(pose_2d.theta / 2.);

    return pose;
}

void BrickSearch::detection(void)
{
    //Convert image
    //BGR2HSV
    cv::Mat image_ = cv::imread("/home/ros/catkin_ws/src/maze_navigating_robot/imageTuning/image2.jpg"); //Remove and replace with topic/imagecallback
    // cv::Mat image_ = &image; How do I access &image?-----------------------------------------------------------------------------
    cv::Mat hsv; //Make class variable?/Only use one vartaiable to save time for all hsv redMask edges etc--------------------------
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

    //Find contours: https://docs.opencv.org/master/d4/d73/tutorial_py_contours_begin.html
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    //Draw contours
    cv::drawContours(image_, contours, 0, cv::Scalar(0, 255, 0), 2);

    //Get centre position of blob
    //Get moments of contours
    cv::Moments m = cv::moments(contours[0], true);
    //centre of blob: https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/
    int cx = m.m10 / m.m00;
    int cy = m.m01 / m.m00;
    cv::Point pt(cx, cy);
    cv::circle(image_, pt, 3, CV_RGB(0, 255, 0), 1);

    double area = cv::contourArea(contours[0]);

    cv::Size size = image_.size();
    double frameArea = size.width * size.height;
    std::cout << "Contour Area: " << area << std::endl;    //--------------------------------------------------------
    std::cout << "Frame Area: " << frameArea << std::endl; //-----------------------------------------------------
    //Do ratio comparison then initiate takeover?
    double ratio = area / frameArea;
    int cutoff = 0; //Adjust------------------------------
    if (ratio > cutoff)
    {
        //Take over control till ratio is certain amount. - This might need to be in higher loop so that values can be recalculated or not

        //Set linear and angular velocity override
        geometry_msgs::Twist twist{};
        twist.angular.z = 0.;
        twist.linear.x = 0;
        // cmd_vel_pub_.publish(twist); //Needs to be redefined?------------------------------
    }

    cv::imshow("Finalimage", image_); //Delete -----------------------------------------
    cv::waitKey(0);                   //Delete------------------------------------------------------------

    //Publish image to topic for debugging and report--------------------
}

int BrickSearch::getGoalReachedStatus(void)
{
    return goal_reached_status;
}

void BrickSearch::setGoalReachedStatus(int status)
{
    goal_reached_status = status;
}
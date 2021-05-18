#include "brick_search.h"

// Constructor
BrickSearch::BrickSearch(ros::NodeHandle &nh) : it_{nh}, ratio(0), override_(false), centred_(false), finished_(false)
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

    // Subscribe to lidar
    laser_sub_ = nh.subscribe("/scan", 10, &BrickSearch::laserCallback, this);

    // Subscribe to odom
    odometry_sub_ = nh.subscribe("/odom", 10, &BrickSearch::odomCallback, this);

    // // Subscribe to global costmap_sub_
    // global_costmap_sub_ = nh.subscribe("/move_base/global_costmap/costmap", 10, &BrickSearch::globalCostmapCallback, this);

    // // Subscribe to local costmap_sub_
    // local_costmap_sub_ = nh.subscribe("/move_base/local_costmap/costmap", 10, &BrickSearch::localCostmapCallback, this);

    // Publish the processed camera image
    detection_pub_ = it_.advertise("/detection_image", 1);

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
    // if (image_msg_count_ < 15) //Delete-----------------------------------------
    // {
    //     image_msg_count_++;
    //     return;
    // }
    // else
    // {
    //     image_msg_count_ = 0;
    // }

    // Copy the image message to a cv_bridge image pointer
    // cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_msg_ptr);

    // // This is the OpenCV image
    // image_ = image_ptr->image;

    try
    {
        image_ = cv_bridge::toCvShare(image_msg_ptr, "bgr8")->image;
        // cv::imshow("view", image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg_ptr->encoding.c_str());
    }

    // You can set "brick_found_" to true to signal to "mainLoop" that you have found a brick
    // You may want to communicate more information
    // Since the "imageCallback" and "mainLoop" methods can run at the same time you should protect any shared variables
    // with a mutex
    // "brick_found_" doesn't need a mutex because it's an atomic

    ROS_INFO("imageCallback");
    ROS_INFO_STREAM("brick_found_: " << brick_found_);
}

void BrickSearch::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    ranges_ = msg->ranges;
}

void BrickSearch::odomCallback(const nav_msgs::OdometryConstPtr &msg) // ---------------------------------------------------------------------------------------------------------------------
{
    robot_pose = msg->pose.pose;
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
// void BrickSearch::globalCostmapCallback(const nav_msgs::OccupancyGridPtr &msg)
// {
//     // global_costmap_ = msg->data.size;
//     for (int i = 0; i < msg->data.size(); i++)
//     {
//         for (int j = 0; j < msg->data.at(i).size(); j++)
//         {
//             global_costmap_.at(i).at(j).push_back(msg->data.at(i).(j));
//         }
//     }
// }

// void BrickSearch::localCostmapCallback(const nav_msgs::OccupancyGridPtr &msg)
// {
//     // local_costmap_ = msg->data.front();
// }

void BrickSearch::pathPlanning(double x, double y)
{

    // goal = {};

    // i++;
    goal.pose.position.x = x;
    goal.pose.position.y = y;

    // robot_pose_2d = getPose2d();
    quaternion.setRPY(0, 0, getPose2d().theta);

    goal.pose.orientation.w = quaternion.getW();
    goal.pose.orientation.x = quaternion.getX();
    goal.pose.orientation.y = quaternion.getY();
    goal.pose.orientation.z = quaternion.getZ();

    goal.header.frame_id = "map";
    // ros::Duration(0.1).sleep();
    move_base_simple_goal_.publish(goal);
    // ros::Duration(0.1).sleep();
    ROS_INFO("Sending goal...");
}

int BrickSearch::meterY2grid(double y)
{
    int gy = round((map_.info.height / 2 - y) / map_.info.resolution);
    if (gy > map_.info.height - 1)
        gy = map_.info.height - 1;
    if (gy < 0)
        gy = 0;
    return gy;
}

int BrickSearch::meterX2grid(double x)
{
    std::cout << "resolution" << map_.info.resolution << std::endl;
    int gx = round((x + map_.info.width / 2) / map_.info.resolution);
    if (gx > map_.info.width - 1)
        gx = map_.info.width - 1;
    if (gx < 0)
        gx = 0;
    return gx;
}

double BrickSearch::meterY2pixel(double y)
{

    int gy = round(((image_size_meters / 2) - y) / 0.05);

    if (gy > (image_size_meters / 0.05) - 1)
        gy = (image_size_meters / 0.05) - 1;
    if (gy < 0)
        gy = 0;
    return gy;
}

double BrickSearch::meterX2pixel(double x)
{
    int gx = round((x + (image_size_meters / 2)) / 0.05);
    if (gx > (image_size_meters / 0.05) - 1)
        gx = (image_size_meters / 0.05) - 1;
    if (gx < 0)
        gx = 0;
    return gx;
}

double BrickSearch::grid2meterX(int x)
{
    double nx = x * map_.info.resolution - map_.info.width / 2 + map_.info.resolution / 2;
    return nx;
}

double BrickSearch::grid2meterY(int y)
{
    double ny = y * map_.info.resolution - map_.info.width / 2 - map_.info.resolution / 2;
    return ny;
}

void BrickSearch::wallBuffer(void) //Delete in not used-------------------------------------------------------
{
    for (int i = 0; i < image_size_pixel; i++)
    {
        for (int j = 0; j < image_size_pixel; j++)
        {

            uchar colour = track_map_.at<uchar>(j, i);

            if (int(colour) == 100)
            {
                int buffer_size = 5;

                cv::Point point1(i, j); // check in case should be (j, i)------------------------------------------

                cv::circle(track_map_, point1, 10, cv::Scalar(255, 255, 255), CV_FILLED);
            }
        }
    }
}
std::vector<double> BrickSearch::randomExploration(void)
{
    double waypoint_x_pixel;
    double waypoint_y_pixel;

    while (1)
    {
        std::random_device rd;                           // obtain a random number from hardware
        std::mt19937 gen(rd());                          // seed the generator
        std::uniform_int_distribution<> distr(192, 288); // define the range

        int i = distr(gen); // generate numbers
        int j = distr(gen); // generate numbers

        // if (int(colour) == 0)
        if ((i < 288 && i > 192 && j > 192 && j < 240) || (i < 288 && i > 240 && j > 240 && j < 288))
        {
            waypoint_x_pixel = (waypoint_x / meters_to_pixel_conversion) + (image_size_pixel / 2);
            waypoint_y_pixel = (waypoint_y / meters_to_pixel_conversion) + (image_size_pixel / 2);

            if (((i - waypoint_x_pixel) > 30 && (j - waypoint_y_pixel) > 30) || lock2 == false)
            {

                waypoint_x = (i - (image_size_pixel / 2)) * meters_to_pixel_conversion;
                waypoint_y = (j - (image_size_pixel / 2)) * meters_to_pixel_conversion;
                waypoints = {waypoint_x, waypoint_y};

                return waypoints;

                break;
            }
        }
    }
}

std::vector<double> BrickSearch::exploration(void)
{
    // map_image_;'';

    std::vector<std::vector<int>> grid_cost;

    grid_cost.resize(image_size_pixel);
    for (int i = 0; i < image_size_pixel; ++i)
    {
        grid_cost.at(i).resize(image_size_pixel);
    }

    std::vector<int> min_cost_grid = {385, 385};
    std::vector<double> min_cost_waypoint = {0, 0};

    double robot_x_pixel = ((getPose2d().x) / meters_to_pixel_conversion) + image_size_pixel / 2;
    double robot_y_pixel = /*-*/ ((getPose2d().y) / meters_to_pixel_conversion) + image_size_pixel / 2;

    // lock2 = false;

    // calculate heuristic of all grids from current robot position
    std::cout << std::endl
              << "Initialising heuristic cost grid" << std::endl;

    // Looping through each node/grid in the gripmap_
    // std::cout << "map: ";
    cv::Mat test;
    cv::cvtColor(track_map_, test, cv::COLOR_GRAY2BGR);
    // auto color = track_map_.at<uchar>(i, j);

    for (int i = 0; i < image_size_pixel; i++)
    {
        for (int j = 0; j < image_size_pixel; j++)
        {

            uchar colour = track_map_.at<uchar>(j, i);

            if (int(colour) == 100)
            {
                int buffer_size = 5;

                cv::Point point1(i, j); // check in case should be (j, i)

                cv::circle(track_map_, point1, 10, cv::Scalar(255, 255, 255), CV_FILLED);
            }
        }
    }

    for (int i = 0; i < image_size_pixel; i++)
    {
        for (int j = 0; j < image_size_pixel; j++)
        {

            uchar colour = track_map_.at<uchar>(j, i);

            if (int(colour) == 0)
            {
                if (((std::abs(i - (robot_x_pixel)-9.6) > 10 && std::abs(j - (robot_y_pixel)-9.6) > 10)))
                {
                    // if (global_costmap_.at(i).at(j) == 0)
                    // {
                    // std::cout << "val" << colour.val[0] << std::endl;
                    // Change in x distance between the current node/grid and the goal_node position
                    int delta_x = i - robot_x_pixel;

                    // Change in x distance between the current node/grid and the goal_node position
                    int delta_y = j - robot_y_pixel; //Might need to remove the negative if not working

                    // Heuristic for each grid/node equals combined x and y distance, which is equivelant to the number of moves to reach the goal node
                    grid_cost.at(i).at(j) = std::abs(delta_x) + std::abs(delta_y);

                    if (grid_cost.at(i).at(j) < (std::abs(min_cost_grid.at(0) - robot_x_pixel) + std::abs(min_cost_grid.at(1) - robot_y_pixel)))
                    {
                        min_cost_grid = {i, j};
                        std::cout << "min_cost_grid" << min_cost_grid.at(0) << "," << min_cost_grid.at(1) << std::endl;
                        std::cout << "delta" << delta_x << "," << delta_y << std::endl;
                    }
                    // }
                }
            }
        }
    }

    // std::cout << std::type_info(track_map_ << std::endl;

    waypoint_x = (min_cost_grid.at(0) - (image_size_pixel / 2)) * meters_to_pixel_conversion;
    waypoint_y = (min_cost_grid.at(1) - (image_size_pixel / 2)) * meters_to_pixel_conversion;
    // cv::Point pointxy(min_cost_grid.at(0),min_cost_grid.at(1));
    // cv::circle(track_map_, pointxy, 2,cv::Scalar(100,100,100), CV_FILLED);
    // cv::imshow("track_map", track_map_);
    // cv::waitKey(0);
    // cv::destroyWindow("track_map");

    std::cout << "waypoint (" << waypoint_x << ", " << waypoint_y << ")" << std::endl;

    min_cost_waypoint = {waypoint_x - 0.5, waypoint_y - 0.5};

    return min_cost_waypoint;

    // set waypoint for lowest cost huesrisitci (closest to robot) that is also black (unknown)

    // return waypoint from function. Use returned waypoint as input fro path planning function.
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
    ratio = 0;
    override_ = false;
    ros::Rate rate(10);
    if (!image_.empty())
    {
        publish_image_ = image_;
        //BGR2HSV conversion
        cv::Mat hsv;
        cv::cvtColor(publish_image_, hsv, cv::COLOR_BGR2HSV);

        //https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
        //Threshold to isolate Red
        cv::Mat redMask;
        //Real Robot
        // cv::inRange(hsv, cv::Scalar(0, 127, 50), cv::Scalar(6, 255, 255), redMask);
        //Sim
        cv::inRange(hsv, cv::Scalar(0, 127, 50), cv::Scalar(6, 255, 255), redMask);

        //Draw box around red blob
        cv::Mat edges;
        cv::Canny(redMask, edges, 400, 1400, 3);

        //Find contours: https://docs.opencv.org/master/d4/d73/tutorial_py_contours_begin.html
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        //Draw contours
        cv::drawContours(publish_image_, contours, 0, cv::Scalar(0, 255, 0), 2);
        //Get centre position of blob
        if (contours.size() > 0)
        {
            //Get moments of contours
            cv::Moments m = cv::moments(contours[0], true);
            //centre of blob: https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/
            cx = m.m10 / m.m00;
            cy = m.m01 / m.m00;
            cv::Point pt(cx, cy);
            cv::circle(publish_image_, pt, 3, CV_RGB(0, 255, 0), 1);

            double area = cv::contourArea(contours[0]);

            size_ = publish_image_.size();
            double frameArea = size_.width * size_.height;
            //Do ratio comparison then initiate takeover?
            ratio = area / frameArea;
            std::cout << "Contour Area: " << area << std::endl;    //--------Delete------------------------------------------------
            std::cout << "Frame Area: " << frameArea << std::endl; //--------Delete---------------------------------------------
            std::cout << "Ratio: " << ratio << std::endl;          //--------Delete---------------------------------------------
            brick_found_ = true;
        }
        else
        {
            brick_found_ = false;
        }
        if (ratio > cutoff && brick_found_) //Might need to adjust this
        {
            // override_ = true;
            //Take over control till ratio is certain amount. - This might need to be in higher loop so that values can be recalculated or not

            //Set linear and angular velocity override
            geometry_msgs::Twist twist{};

            if (ratio < final) //Check condition //This needs to override other commands
            {
                //Publish waypoint ontop of itself
                if (lock2 == false)
                {
                    double x_waypoint = getPose2d().x;
                    double y_waypoint = getPose2d().y;
                    pathPlanning(x_waypoint, y_waypoint);
                    override_ = true;
                    lock2 = true;
                }
                std::cout << "Driving to brick" << std::endl;

                if (cx >= (size_.width / 2 - pixel_tolerance) && cx <= (size_.width / 2 + pixel_tolerance))
                {
                    // centred_ = true; // Delete from here and main
                    twist.angular.z = 0;
                    twist.linear.x = 0.1;
                    cmd_vel_pub_.publish(twist);
                }
                else if (cx > size_.width / 2 + pixel_tolerance)
                {
                    twist.angular.z = -0.2; //Tune value---------------------------------------
                    twist.linear.x = 0.05;
                    cmd_vel_pub_.publish(twist);
                }
                else if (cx < size_.width / 2 - pixel_tolerance)
                {
                    twist.angular.z = 0.2; //Tune value------------------------------------------
                    twist.linear.x = 0.05;
                    cmd_vel_pub_.publish(twist); //Needs to be redefined?------------------------------
                }
            }
            else if (ratio >= final)
            {
                finished_ = true;
                twist.angular.z = 0;
                twist.linear.x = 0;
                cmd_vel_pub_.publish(twist);

                //Check scan straight ahead
                float scan_ahead = ranges_.at(0);
                int i = 0;
                while (scan_ahead <= 0)
                {
                    i++;
                    scan_ahead = ranges_.at(i);
                }
                //Get robot x and y
                double robot_x = getPose2d().x + (image_size_meters / 2);
                double robot_y = getPose2d().y + (image_size_meters / 2);
                double robot_theta = getPose2d().theta;
                // Calculate angle of current lidar ray
                double map_angle = wrapAngle(robot_theta + (i * M_PI) / 180);
                // calculate x position where current lidar ray ends
                int ray_x = ranges_.at(i) * cos(map_angle); //Need to add metre to grid------
                ray_x = (ray_x + robot_x) / meters_to_pixel_conversion;
                // calculate y position where current lidar ray ends
                int ray_y = ranges_.at(i) * sin(map_angle); //Need to add metre to grid-----
                ray_y = (ray_y + robot_y) / meters_to_pixel_conversion;
                cv::Point brick(ray_x, ray_y); // Adjust---------------------------------------------------
                cv::circle(map_image_, brick, 3, CV_RGB(255, 255, 255), 1);
                imshow("map", map_image_);
                cv::waitKey(0);
                //Need to publish image here, could fix by publishing to original map_image_
            }
        }
        // cv::imshow("view", publish_image_); //Delete -----------------------------------------
        // cv::waitKey(0);

        //Publish image
        detection_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", publish_image_).toImageMsg());
        rate.sleep();
    }
}

void BrickSearch::searchedArea(void)
{
    //70 degree FOV - 35 on each side

    double robot_x = getPose2d().x + (image_size_meters / 2);
    double robot_y = getPose2d().y + (image_size_meters / 2);
    double robot_theta = getPose2d().theta;
    cv::Point robot(robot_x, robot_y);

    double ray_x;
    double ray_y;

    // std::vector<float> rangesInFOV;
    // Testing
    // cv::Size test = track_map_.size();
    // for (int i = 0; i < (test.height); i++)
    // {
    //     cv::Point left(0, i);
    //     cv::Point right(test.width, i);
    //     cv::line(track_map_, left, right, cv::Scalar(0, 0, 0), 1);
    // }
    // std::cout << "Ranges: "; //------------------------------------------
    for (int i = 0; i < ranges_.size(); i++)
    {
        if (ranges_.at(i) > 0 && ranges_.at(i) <= 3)
        {
            // Using lidar within FOV of camera which is 70 degrees - 35 on each side of robot front
            if (((i >= (ranges_.size() - 35)) && i <= ranges_.size()) || (i >= 0 && i <= 35))
            {
                // std::cout << ranges_.at(i) << ", "; //---------------------------------------------------
                // Calculate angle of current lidar ray
                double map_angle = wrapAngle(robot_theta + (i * M_PI) / 180);
                // calculate x position where current lidar ray ends
                ray_x = ranges_.at(i) * cos(map_angle);
                ray_x = (ray_x + robot_x) / meters_to_pixel_conversion;
                // std::cout << "ray_x: " << ray_x << std::endl;

                // calculate y position where current lidar ray ends
                ray_y = ranges_.at(i) * sin(map_angle);
                ray_y = (ray_y + robot_y) / meters_to_pixel_conversion;

                cv::Point scan(ray_x, ray_y);
                // cv::circle(track_map_, scan, 3, CV_RGB(0, 255, 0), 1); //green
                robot.x = robot_x / meters_to_pixel_conversion;
                robot.y = robot_y / meters_to_pixel_conversion;
                cv::line(track_map_, robot, scan, cv::Scalar(255, 255, 255), 1);
                // std::cout << "ray: " << ray_x << "," << ray_y << std::endl;
            }
        }
    }
    // std::cout << std::endl;                                 //---------------------------------------------------------
    // cv::circle(track_map_, robot, 3, CV_RGB(255, 0, 0), 1); //Red
    // cv::Size bob = track_map_.size();
    // std::cout << "robot: " << robot.x << "," << robot.y << std::endl;
    // std::cout << "image size: " << bob.width << "," << bob.height << std::endl;
    // ray: 766,766
    // robot: 383,383
    // image size: 384,384

    cv::imshow("track_map", track_map_);
    cv::waitKey(30);
    // cv::waitKey(0);
    // cv::destroyWindow("track map");

    //Do check for brick
    //If brick check depth image for location
    //Add distance to the turtlbot's coords
    //Publish pose to RVIZ/map?
    //else
    // map_image_;

    //Need to get triangle of focus

    //Get each ray of focus?

    //Compare to lidar values/Depth image in cone
    ///camera/depth/points for depth values
    //If using depth image only need 1 strip of data
    //Get the equation of each ray
    //Get limits of each ray from depth image
    //Change map colour pixels to show searched

    //Save map - Might need mutexes in this
    //Or could return the map at the end of the function
}

int BrickSearch::getGoalReachedStatus(void)
{
    return goal_reached_status;
}

void BrickSearch::setGoalReachedStatus(int status)
{
    goal_reached_status = status;
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

    // int i = 0;
    // This loop repeats until ROS shuts down, you probably want to put all your code in here
    cv::namedWindow("track_map");
    track_map_ = map_image_.clone();
    cv::Size test = track_map_.size(); //rename------------------------------------------

    /////////////////////////////////////////////////////////////////////
    // cv::Size test = track_map_.size(); //rename------------------------------------------
    // std::cout << "image[ ";
    // for (int i = 0; i < test.height; i++)
    // {
    //     for (int j = 0; j < test.width; j++)
    //     {
    //         auto colour = track_map_.at<uchar>(i, j);
    //         if (int(colour) != 255)
    //         {
    //         std::cout << int(colour)  << ",";
    //         }
    //     }
    // }
    // std::cout << " ]" << std::endl;
    // ros::shutdown();
    /////////////////////////////////////////////////////////////////////

    ROS_INFO("track_map_ saved");
    while (ros::ok())
    {
        ROS_INFO("mainLoop");
        //If block detected override waypoint onto itself-----------------------------------------------------------------------

        // Delay so the loop doesn't run too fast
        // ros::Duration(0.2).sleep(); // ADD back in ----------------------------------------------------
        // std::cout << "map image: " << map_image_ << std::endl;

        // if (lock == false)
        // {
        //     cv::imshow("map image", track_map_);
        //     cv::waitKey(0);
        //     // cv::imwrite("/home/user/catkin_ws/src/maze_navigating_robot/map.jpg", map_image_);

        //     cv::destroyWindow("map image");
        // }

        if (!finished_)
        {
            detection(); //Might be able to remove----------------------
            searchedArea();
            if (override_ == false)
            {
                double robot_theta = getPose2d().theta;
                if (((localised_ == true && getGoalReachedStatus() == 3) || getGoalReachedStatus() == 4 || (localised_ == true && lock == false)) && lock2 == false) // Only navigate to new goal if the current goal has been reached (goal reached status == 3, goal not reached status == 1) and robot is localised
                {
                    // searchedArea(); //Should probs go here

                    //For 360 turn
                    // geometry_msgs::Twist twist{};
                    // twist.angular.z = 1.;
                    // cmd_vel_pub_.publish(twist);
                    // ros::Duration(0.1).sleep();
                    // while (getPose2d().theta != robot_theta)
                    // {
                    //     detection(); //Hopefully this won't cause an error////////
                    //     searchedArea();
                    //     if (override_)
                    //     {
                    //         break;
                    //     }
                    // ros::Duration(0.1).sleep();
                    // }
                    // twist.angular.z = 0;
                    // cmd_vel_pub_.publish(twist);
                    // if (override_)
                    // {
                    //     break;
                    // }

                    double pose = getPose2d().theta;
                    geometry_msgs::Twist twist{};
                    twist.angular.z = 1.;
                    cmd_vel_pub_.publish(twist);
                    ros::Duration(0.5).sleep();
                    while (lock2 == false)
                    {
                        ros::Duration(0.05).sleep();

                        std::cout << "SCANNING" << std::endl;
                        detection(); //Might be able to remove----------------------
                        searchedArea();

                        if ((pose >= getPose2d().theta - (3 * M_PI / 180) && pose <= getPose2d().theta + (3 * M_PI / 180)) || override_ == true)
                        {
                            twist.angular.z = 0.0;
                            cmd_vel_pub_.publish(twist);
                            break;
                        }
                    }

                    if (override_ == false)
                    {

                        lock = true;
                        // move_base_action_client_.waitForResult();
                        // std::cout << "Status: " << (move_base_action_client_.getState() << std::endl;
                        std::vector<double> goalWaypoint;
                        goalWaypoint.resize(2);
                        goalWaypoint = exploration();
                        // BrickSearch::pathPlanning(1.5, 3);
                        // ros::Duration(0.5).sleep();
                        pathPlanning(goalWaypoint.at(0), goalWaypoint.at(1));
                        // ros::Duration(1).sleep();
                        // move_base_action_client_.waitForResult();
                    }
                }
            }
        }
        else
        {
            std::cout << "Brick has been located" << std::endl;
        }
    }
    cv::destroyWindow("track_map");
}

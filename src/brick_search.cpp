#include "brick_search.h"

// Constructor
BrickSearch::BrickSearch(ros::NodeHandle &nh) : it_{nh}, ratio(0), override(false), centred(false)
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

    // Publish the processed camera image
    detection_pub_ = it_.advertise("/detection_image", 1);

    // Publish searched area
    searched_area_pub_ = it_.advertise("/searched_area", 1);

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

void BrickSearch::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    ranges_ = msg->ranges;
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

void BrickSearch::pathPlanning(double x, double y)
{

    goal = {};

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

    move_base_simple_goal_.publish(goal);

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

int BrickSearch::meterY2pixel(double y)
{

    int gy = round((y+image_size_meters / 2) / 0.05);

    if (gy > image_size_pixel - 1)
        gy = image_size_pixel - 1;
    if (gy < 0)
        gy = 0;
    return gy;
}

int BrickSearch::meterX2pixel(double x)
{
    int gx = round((x+image_size_meters / 2-x) / 0.05);
    if (gx > image_size_pixel - 1)
        gx = image_size_pixel - 1;
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

std::vector<double> BrickSearch::exploration(void)
{
    // map_image_;'';

    std::vector<std::vector<int>> grid_cost;

    grid_cost.resize(map_.info.width);
    for (int i = 0; i < map_.info.width; ++i)
    {
        grid_cost.at(i).resize(map_.info.height);
    }

    std::vector<int> min_cost_grid = {0, 0};
    std::vector<double> min_cost_waypoint = {0, 0};

    // calculate heuristic of all grids from current robot position
    std::cout << std::endl
              << "Initialising heuristic cost grid" << std::endl;

    // Looping through each node/grid in the gripmap_
    for (int i = 0; i < map_.info.height; i++)
    {
        for (int j = 0; j < map_.info.width; j++)
        {
            // Change in x distance between the current node/grid and the goal_node position
            int x = std::abs(i - meterX2grid(getPose2d().x));

            // Change in x distance between the current node/grid and the goal_node position
            int y = std::abs(j - meterY2grid(getPose2d().y));

            // Heuristic for each grid/node equals combined x and y distance, which is equivelant to the number of moves to reach the goal node
            grid_cost.at(i).at(j) = x + y;

            if (grid_cost.at(i).at(j) < grid_cost.at(min_cost_grid.at(0)).at(min_cost_grid.at(1)) && (map_image_.at<int>(i, j) == 0))
            {
                min_cost_grid = {i, j};
            }
        }
    }

    double waypoint_x = grid2meterX(min_cost_grid.at(0));
    double waypoint_y = grid2meterY(min_cost_grid.at(1));

    min_cost_waypoint = {waypoint_x, waypoint_y};

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

            size = publish_image_.size();
            double frameArea = size.width * size.height;
            //Do ratio comparison then initiate takeover?
            ratio = area / frameArea;
            std::cout << "Contour Area: " << area << std::endl;    //--------Delete------------------------------------------------
            std::cout << "Frame Area: " << frameArea << std::endl; //--------Delete---------------------------------------------
            std::cout << "Ratio: " << ratio << std::endl;          //--------Delete---------------------------------------------
            brick_found_ = true;                                   //Check this might cause error because of tolerance------------------------------
        }
        else
        {
            brick_found_ = false;
        }
        if (ratio > cutoff && brick_found_) //Might need to adjust this
        {
            override = true;
            //Take over control till ratio is certain amount. - This might need to be in higher loop so that values can be recalculated or not

            //Set linear and angular velocity override
            geometry_msgs::Twist twist{};

            if (!centred && ratio < final) //Check condition //This needs to override other commands
            {
                if (cx >= (size.width / 2 - pixel_tolerance) && cx <= (size.width / 2 + pixel_tolerance))
                {
                    centred = true;
                }
                else if (cx > size.width / 2)
                {
                    twist.angular.z = -0.5; //Tune value
                    twist.linear.x = 0;
                    cmd_vel_pub_.publish(twist);
                }
                else if (cx < size.width / 2)
                {
                    twist.angular.z = 0.5; //Tune value
                    twist.linear.x = 0;
                    cmd_vel_pub_.publish(twist); //Needs to be redefined?------------------------------
                }
            }
            else if (centred && ratio < final) //Centred and found// need to check ratio too
            {
                twist.angular.z = 0;
                twist.linear.x = 0.1;
                cmd_vel_pub_.publish(twist);
            }
            else if (centred && ratio >= final)
            {
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
                double robot_x = getPose2d().x; //Make struct?---------------------------------------------------------------
                double robot_y = getPose2d().y; //Make struct?---------------------------------------------------------------
                double robot_theta = getPose2d().theta;      //Make struct?---------------------------------------------------------------
                // Calculate angle of current lidar ray
                double map_angle = wrapAngle(robot_theta + (i * M_PI) / 180);

                // calculate x position where current lidar ray ends
                int ray_x = ranges_.at(i) * cos(map_angle); //Need to add metre to grid------
                ray_x = meterX2pixel(ray_x + robot_x);

                // calculate y position where current lidar ray ends
                int ray_y = ranges_.at(i) * sin(map_angle); //Need to add metre to grid-----
                ray_y = meterY2pixel(ray_y + robot_y);
                cv::Point brick(ray_x, ray_y);
                cv::circle(track_map_, brick, 3, CV_RGB(255, 0, 0), 1); //There is gonna be a overriding problem here
                //Need to publish image here, could fix by publishing to original map_image_
            }
        }
        rate.sleep();
        //Publish image
        detection_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", publish_image_).toImageMsg());
    }
}

void BrickSearch::searchedArea(void)
{
    //70 degree FOV - 35 on each side

    double robot_x = getPose2d().x; //Make struct?---------------------------------------------------------------
    double robot_y = getPose2d().y; //Make struct?---------------------------------------------------------------
    double robot_theta = getPose2d().theta;
    cv::Point robot(robot_x, robot_y);

    double ray_x;
    double ray_y;

    std::vector<float> rangesInFOV;
    // Testing
    cv::Size test = track_map_.size();
    for (int i = 0; i < (test.height); i++)
    {
        cv::Point left(0, i);
        cv::Point right(test.width, i);
        cv::line(track_map_, left, right, cv::Scalar(0, 0, 0), 1);
    }
    std::cout << "Ranges: "; //------------------------------------------
    for (int i = 0; i < ranges_.size(); i++)
    {
        // Using lidar within FOV of camera which is 70 degrees - 35 on each side of robot front
        if (((i >= (ranges_.size() - 35)) && i <= ranges_.size()) || (i >= 0 && i <= 35))
        {
            std::cout << ranges_.at(i) << ", "; //---------------------------------------------------
            // Calculate angle of current lidar ray
            double map_angle = wrapAngle(robot_theta + (i * M_PI) / 180);
            // calculate x position where current lidar ray ends
            ray_x = ranges_.at(i) * cos(map_angle);
            ray_x = meterX2pixel(ray_x + robot_x);
            // std::cout << "ray_x: " << ray_x << std::endl;

            // calculate y position where current lidar ray ends
            ray_y = ranges_.at(i) * sin(map_angle);
            ray_y = meterY2pixel(ray_y + robot_y);

            cv::Point scan(ray_x, ray_y);
            cv::circle(track_map_, scan, 3, CV_RGB(0, 255, 0), 1); //green
            cv::line(track_map_, robot, scan, cv::Scalar(255, 255, 255), 1);
            std::cout << "ray: " << ray_x << "," << ray_y << std::endl;
        }
    }
    std::cout << std::endl;                                 //---------------------------------------------------------
    cv::circle(track_map_, robot, 3, CV_RGB(255, 0, 0), 1); //Red
    cv::Size bob = track_map_.size();
    std::cout << "robot: " << meterX2pixel(robot_x) << "," << meterY2pixel(robot_y) << std::endl;
    std::cout << "image size: " << bob.width << "," << bob.height << std::endl;
    // ray: 766,766
    // robot: 383,383
    // image size: 384,384

    //Publish image
    searched_area_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", track_map_).toImageMsg());

    cv::imshow("track map", track_map_);
    cv::waitKey(0);
    cv::destroyWindow("track map");

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

    // int i = 0;
    // This loop repeats until ROS shuts down, you probably want to put all your code in here

    track_map_ = map_image_;
    ROS_INFO("trackmap_ saved");
    while (ros::ok())
    {
        ROS_INFO("mainLoop");

        // Delay so the loop doesn't run too fast
        ros::Duration(0.2).sleep();
        // std::cout << "map image: " << map_image_ << std::endl;

        // if (lock == false)
        // {
        //     cv::imshow("map image", track_map_);
        //     cv::waitKey(0);
        //     // cv::imwrite("/home/user/catkin_ws/src/maze_navigating_robot/map.jpg", map_image_);

        //     cv::destroyWindow("map image");
        // }

        searchedArea();

        if (getGoalReachedStatus() == 3 || lock == false) // Only navigate to new goal if the current goal has been reached (goal reached status == 3, goal not reached status == 1) and robot is localised
        {
            lock = true;

            std::vector<double> goalWaypoint = exploration();
            // BrickSearch::pathPlanning(1.5, 3);
            pathPlanning(goalWaypoint.at(0), goalWaypoint.at(1));
        }

        // move_base_action_client_.sendGoal(action_goal.goal);

        // Desired seach x,y position
        // Covert into required units (x,y or meters)

        // path_planning_algorithm(desired position) - From package or custom?

        // pure_pursuit_algorithm(planned path) - From package or custom? (package probably better?)

        // If box found? - Inside pure_pursuit_algorithm? Or here is ok if pure_pursuit is seperate thread or ROS package
        // Use open CV to move straight to the box or path planning to recalculate a path to the box position (calulcated using lidar or depth camera with current robot position)
    }
}

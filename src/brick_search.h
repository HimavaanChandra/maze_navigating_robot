#include <atomic>
#include <cmath>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp> // can remove probs -----------------------------------------

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>

class BrickSearch
{
public:
    // Constructor
    explicit BrickSearch(ros::NodeHandle &nh);

    // Publich methods
    void mainLoop(void);
    // void detection(void); //Move to private?----------

private:
    // Variables
    nav_msgs::OccupancyGrid map_{};
    cv::Mat map_image_{};
    std::atomic<bool> localised_{false};
    std::atomic<bool> brick_found_{false};
    int image_msg_count_ = 0;
    int goal_reached_status = 3; // 0 = goal not reached 3 = goal reached 4 = Failed to find a valid plan. Even after executing recovery behaviors.
    bool lock = false;
    tf2::Quaternion quaternion;
    geometry_msgs::PoseStamped goal;
    cv::Mat image_;
    cv::Mat publish_image_;
    cv::Mat track_map_;
    cv::Size size;
    int cx;
    int cy;
    const double cutoff = 0.6; //Adjust-----------------------
    const double final = 0.8;  //Adjust
    double ratio;
    bool override;
    const int pixel_tolerance = 3; //Adjust-------------------
    bool centred;
    const int image_size_meters = 20;
    const int image_size_pixel = 384;

    // // Structures
    // struct robot_pose_2d
    // {
    //     double x;
    //     double y;
    //     double theta;
    // };

    // Transform listener
    tf2_ros::Buffer transform_buffer_{};
    tf2_ros::TransformListener transform_listener_{transform_buffer_};

    // Subscribe to the AMCL pose to get covariance
    ros::Subscriber amcl_pose_sub_{};
    ros::Subscriber move_base_status_sub_{};

    // Velocity command publisher
    ros::Publisher cmd_vel_pub_{};

    // Movement goal command publisher
    ros::Publisher move_base_simple_goal_{};

    // Image transport, Publishers and subscriber
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_{};
    image_transport::Publisher detection_pub_;
    image_transport::Publisher searched_area_pub_;

    // Lidar Subscriber and data
    ros::Subscriber laser_sub_;
    std::vector<float> ranges_;

    // Action client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_{"move_base", true};

    // Private methods
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg);
    void imageCallback(const sensor_msgs::ImageConstPtr &image_msg_ptr);
    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray &moveBaseStatus_msg_ptr);
    void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);

    int getGoalReachedStatus(void);
    void setGoalReachedStatus(int status);

    geometry_msgs::Pose2D getPose2d();
    geometry_msgs::Pose pose2dToPose(const geometry_msgs::Pose2D &pose_2d);
    double wrapAngle(double angle);
    void pathPlanning(double x, double y);
    void detection(void);
    std::vector<double> exploration(void);
    void searchedArea(void);

    int meterY2grid(double y);
    int meterX2grid(double X);
    double grid2meterX(int x);
    double grid2meterY(int y);
    int meterY2pixel(double y);
    int meterX2pixel(double X);
};
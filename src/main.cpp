#include <ros/ros.h>
#include "brick_search.h"

int main(int argc, char **argv)
{
  // detection(); //Remove---------------------------

  ros::init(argc, argv, "main");

  ros::NodeHandle nh{};

  // brick_search::BrickSearch bs(nh);
  std::shared_ptr<BrickSearch> brickSearch(new BrickSearch(nh));
  std::thread mazeNavigationThread(&BrickSearch::mainLoop, brickSearch);

  ros::spin();

  // // Asynchronous spinner doesn't block
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  ros::shutdown();

  mazeNavigationThread.join();
}

#include <ros/ros.h>
#include "brick_search.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "main");

  ros::NodeHandle nh{};

  std::shared_ptr<BrickSearch> brickSearch(new BrickSearch(nh));
  std::thread mazeNavigationThread(&BrickSearch::mainLoop, brickSearch);

  ros::spin();

  ros::shutdown();

  mazeNavigationThread.join();
}

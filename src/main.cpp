#include "ros/ros.h"
#include "turtle_follow.h"
//Check message types and packages in cmake and package.xml---------------------------




//For turtlebot ar tags camera will have to be on usb_cam unless can change ar topic or echo topic

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_follow");

  ros::NodeHandle nh;

  std::shared_ptr<TurtleFollow> turtleFollow(new TurtleFollow(nh));
  //Might need to change the function name (robotControl)-------------------------------------
  std::thread t(&TurtleFollow::robotControl, turtleFollow);


//   std::shared_ptr<Pursuit> gc(new Pursuit(nh));
// //   std::thread t(&Pursuit::imagePublish, gc);



  ros::spin();

  ros::shutdown();
//   t.join();



  return 0;
}


#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "zed_uvc_node");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  
  nodelet.load(ros::this_node::getName(), "zed_cam/ZedNodelet", remap, nargv);
  
  ros::spin();
  return 0;
  }
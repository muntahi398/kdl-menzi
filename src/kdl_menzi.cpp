#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sstream>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "body_joints");
    ros::NodeHandle nh;
   KDL::Tree my_tree;
   
   std::string urdf_file = "model.urdf";
//   if (!kdl_parser::treeFromFile("excavator_truck.urdf", my_tree)){
      if (!kdl_parser::treeFromFile(urdf_file, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
}

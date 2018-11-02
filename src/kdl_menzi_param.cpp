#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <urdf/model.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "body_joints");
    ros::NodeHandle nh;
   KDL::Tree my_tree;
   
   //std::string urdf_file = "test_robot.urdf";
  // KDL::Tree my_tree;
   ros::NodeHandle node;
   std::string robot_desc_string;
   node.param("robot_description", robot_desc_string, std::string());
   if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }

}

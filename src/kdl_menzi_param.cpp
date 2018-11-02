#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <urdf/model.h>
//#include <tree.hpp>



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

unsigned int nj = my_tree.getNrOfJoints();
unsigned int js = my_tree.getNrOfSegments();

ROS_INFO("Failed to construct kdl tree");
ROS_INFO("%d   ||  %d \n", nj,js);


   KDL::Chain chain;
//chain= 
 if (!my_tree.getChain("base_link", "bucket", chain)){
      ROS_ERROR("Failed to construct kdl chain");
      return false;
   }

//robot_kin.getChain("base_link", "bucket", chain);
 //  KDL::JntArray joint_pos(chain.getNrOfJoints());
//  KDL::Frame cart_pos;
//  KDL::ChainFkSolverPos_recursive fk_solver(chain);
//  fk_solver.JntToCart(joint_pos, cart_pos);



}

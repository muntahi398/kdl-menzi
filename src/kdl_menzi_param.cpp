#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <urdf/model.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <sensor_msgs/JointState.h>

//#include <tree.hpp>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "body_joints");
    ros::NodeHandle nh;
    KDL::Tree my_tree;
   
   //std::string urdf_file = "test_robot.urdf";

   ros::NodeHandle node;
   std::string robot_desc_string;
   node.param("robot_description", robot_desc_string, std::string());
   if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }

   unsigned int nj = my_tree.getNrOfJoints();
   unsigned int js = my_tree.getNrOfSegments();

   //ROS_INFO("Failed to construct kdl tree");
   ROS_INFO("NrOfJoints =%d   ||  NrOfSegments= %d \n", nj,js);


   KDL::Chain chain;
   //chain= 
   if (!my_tree.getChain("base_link", "bucket", chain)){
      ROS_ERROR("Failed to construct kdl chain");
      return false;
   }

   //fk solver= 
   KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

   // ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);


   //subscribing robot joint angles from gazebo= 
   const sensor_msgs::JointStateConstPtr initJoints = ros::topic::waitForMessage<sensor_msgs::JointState>("/walking_excavator/joint_states", nh);

   ROS_INFO("Joint States published");
   ROS_INFO("length of joint states %d", initJoints->name.size()); 


}

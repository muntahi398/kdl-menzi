#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <urdf/model.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

//#include <kdl/chainjnttojacdotsolver.hh>

#include <sensor_msgs/JointState.h>

//#include <tree.hpp>

void update_message(sensor_msgs::JointState &msg,
                    const KDL::JntArray &joints) {
	msg.header.stamp = ros::Time::now();
	Eigen::Map<Eigen::VectorXd>(msg.position.data(), msg.position.size()) = joints.data;
}

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


   KDL::Chain kdl_chain;
   //chain= 
   if (!my_tree.getChain("base_link", "bucket", kdl_chain)){
      ROS_ERROR("Failed to construct kdl chain");
      return false;
   }
   ROS_INFO(" chain constructed \n");
   //fk solver= 
   KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(kdl_chain);

   // ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	KDL::JntArray kdl_joints = KDL::JntArray(kdl_chain.getNrOfJoints());
	KDL::Frame kdl_pose;
	fksolver.JntToCart(kdl_joints, kdl_pose);

      ROS_INFO(" fksolver constructed \n");

   KDL::Chain kdl_chain_wheel_fl;
   KDL::Chain kdl_chain_wheel_fr;
   KDL::Chain kdl_chain_wheel_rl;
   KDL::Chain kdl_chain_wheel_rr;
   if (!my_tree.getChain("base_link", "wheel_front_left", kdl_chain_wheel_fl)){
      ROS_ERROR("Failed to construct kdl_chain_wheel_fl");
      return false;
   }
   ROS_INFO(" chain constructed \n");
   if (!my_tree.getChain("base_link", "wheel_front_right", kdl_chain_wheel_fr)){
      ROS_ERROR("Failed to construct kdl_chain_wheel_fr ");
      return false;
   }
   ROS_INFO(" chain constructed \n");
   if (!my_tree.getChain("base_link", "wheel_rear_right", kdl_chain_wheel_rl)){
      ROS_ERROR("Failed to construct kdl_chain_wheel_rl");
      return false;
   }
   ROS_INFO(" chain constructed \n");
   if (!my_tree.getChain("base_link", "wheel_rear_right", kdl_chain_wheel_rr)){
      ROS_ERROR("Failed to construct kdl_chain_wheel_rr");
      return false;
   }
   ROS_INFO(" chain constructed \n");

   //subscribing robot joint angles from gazebo= 
   const sensor_msgs::JointStateConstPtr initJoints = ros::topic::waitForMessage<sensor_msgs::JointState>("/walking_excavator/joint_states", nh);

   ROS_INFO("Joint States published");
   ROS_INFO("length of joint states %d", initJoints->name.size()); 

   

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
    KDL::ChainIkSolverVel_pinv ik_solver_vel(kdl_chain);
    KDL::ChainIkSolverPos_NR ik_solver(kdl_chain, fk_solver,
            ik_solver_vel, 1000, 100); //max 100 iterations and stop by an accuracy of 1e-6

    KDL::JntArray q_init(kdl_chain.getNrOfJoints());
    KDL::JntArray q_out(kdl_chain.getNrOfJoints());

    //q_init(1,0) = 0.0;
    //q_init(1,1) = 0.0;
    //q_init(1,2) = 0.0;

    KDL::Frame dest_frame(KDL::Vector(0.0, 2.0, 0.0));

    if (ik_solver.CartToJnt(q_init, dest_frame, q_out) < 0) {
        ROS_ERROR( "Something really bad happened. You are in trouble now");
        return -1;
    } else {
        // parse output of ik_solver to the robot

        for (unsigned int j = 0; j < q_out.rows(); j++) {
            std::cout << q_out(1, j) * 180 / 3.14 << "  "<<j<< "|";
        }
        std::cout << std::endl;
    }
    KDL::ChainJntToJacSolver jac_menzi_manipulator = KDL::ChainJntToJacSolver(kdl_chain); //works jacobian -- but how to use
        KDL::ChainJntToJacSolver jac_menzi_wheel_fl = KDL::ChainJntToJacSolver(kdl_chain_wheel_fl); 
        KDL::ChainJntToJacSolver jac_menzi_wheel_fr = KDL::ChainJntToJacSolver(kdl_chain_wheel_fr); 
        KDL::ChainJntToJacSolver jac_menzi_wheel_rl = KDL::ChainJntToJacSolver(kdl_chain_wheel_rl); 
        KDL::ChainJntToJacSolver jac_menzi_wheel_rr = KDL::ChainJntToJacSolver(kdl_chain_wheel_rr); 


}

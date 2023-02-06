#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <ros/ros.h>
#include <ros/master.h>
#include <eigen3/Eigen/Core>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <sstream>
#include "uvs_bridge/misc_utilities.h"
#include <std_srvs/Empty.h>
#include "wam_srvs/GravityComp.h"
#include "wam_srvs/Hold.h"
#include "wam_srvs/HapticSphere.h"
#include "wam_srvs/JointMove.h"
#include "wam_srvs/JointMoveBlock.h"
#include "wam_srvs/PoseMove.h"
#include "wam_srvs/CartPosMove.h"
#include "wam_srvs/CartVel.h"
#include "wam_srvs/OrtnMove.h"
#include "wam_srvs/OrtnSplitMove.h"
#include "wam_srvs/ForceTorqueTool.h"
#include "wam_srvs/ForceTorqueToolTime.h"
#include "wam_srvs/Teach.h"
#include "wam_srvs/Play.h"
#include "wam_srvs/Link.h"
#include "wam_srvs/FollowPath.h"
#include "wam_srvs/GetToolPose.h"
#include "wam_msgs/RTCartVel.h"
#include "wam_msgs/RTJointVel.h"
#include "wam_msgs/MatrixMN.h"

typedef Eigen::Matrix< double, 7, 1 > Vector7d;
class ArmControl {
    public:
        // global variables
        bool connected_to_zeus;
        bool connected_to_slax;
        int dof;
        std::string wam_namespace;
        // callback variables
        geometry_msgs::PoseStamped previous_pose;
        sensor_msgs::JointState previous_joint_state;
        wam_msgs::MatrixMN previous_jacobian;
        // setup ROS subscribers
        ros::Subscriber pose_sub;
        ros::Subscriber joint_sub;
        ros::Subscriber jacobian_sub;
        // initialization and destructor methods
        ArmControl(ros::NodeHandle nh);
        ~ArmControl();
        void initialize_wam_connection();
        // service call methods
        bool call_haptic_sphere();
        bool set_gravity_compensation(); 
        bool set_gravity_compensation(bool gravity); 
        bool lock_joint_position(); 
        bool lock_joint_position(bool lock); 
        bool lock_cartesian_position(); 
        bool lock_cartesian_position(bool lock); 
        bool lock_eef_orientation(); 
        bool lock_eef_orientation(bool lock); 
        bool lock_eef_orientation2(); 
        bool lock_eef_orientation2(bool lock); 
        bool call_move_joints(const Eigen::VectorXd& target_joint_state, bool block_control);
        bool call_move_orientation(const Eigen::VectorXd& target_orientation);
        bool call_move_orientation_split(const Eigen::VectorXd& target_orientation, const Eigen::VectorXd& gains);
        bool call_move_cartesian(const Eigen::VectorXd& target_position);
        bool call_move_cartesian_velocity(const Eigen::VectorXd& target);
        bool call_move_force_torque(const Eigen::VectorXd& force, const Eigen::VectorXd& torque, bool base);
        bool move_one_joint();
        bool move_one_joint_by_delta();
        bool move_all_joints();
        bool move_to_zero();
        bool move_to_home_position();
        bool move_to_initial_position();
        bool move_to_previous_position();
        bool move_orientation();
        bool move_orientation_split();
        bool move_cartesian();
        bool move_cartesian_velocity();
        bool move_force_torque_base();
        bool move_force_torque_tool();
        bool teach_motion(std::string path_name);
        bool follow_motion(std::string path_name);
        bool follow_path();
        bool link_arms();
        bool unlink_arms();
        bool start_visual_fix();
        bool stop_visual_fix();
        bool pose_move(const Eigen::VectorXd& target_pose);
        Vector7d get_tool_pose(const Eigen::VectorXd&  target_joint_state);
        // information debugging methods
        void print_information_to_terminal();
        void print_joint_positions_to_terminal();
        void print_orientation_to_terminal();
        void print_cartesian_position_to_terminal();
        // information access methods
        int get_dof()  { return previous_joint_state.position.size(); }
        // int get_dof()  { return dof; }
        int get_sequence_number() { return previous_joint_state.header.seq; }

        geometry_msgs::PoseStamped get_pose() { return previous_pose; }
        sensor_msgs::JointState get_state() { return previous_joint_state; }
        wam_msgs::MatrixMN get_jacobian() { return previous_jacobian; }

        Eigen::MatrixXd get_lin_tool_jacobian()         
        {
            Eigen::MatrixXd jac(3, dof);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < dof; ++j) {   
                    jac(i, j) = previous_jacobian.data[j*6 + i];
                }
            }
            return jac;
        }
        Eigen::MatrixXd get_ang_tool_jacobian()         
        {
            Eigen::MatrixXd jac(3, dof);
            for (int i = 3; i < 6; ++i)
            {
                for (int j = 0; j < dof; ++j)
                {   
                    jac(i, j) = previous_jacobian.data[j*6 + i];
                }
            }
            return jac;
        }
        Eigen::VectorXd get_positions() 
        {
            Eigen::VectorXd jp(dof);
            for(int i = 0; i < dof; ++i) {
                jp[i] = previous_joint_state.position[i];
            }
            return jp; 
        }

        Eigen::Vector3d get_tool_position() 
        {
            Eigen::Vector3d cp;
            cp[0] = previous_pose.pose.position.x;
            cp[1] = previous_pose.pose.position.y;
            cp[2] = previous_pose.pose.position.z;
            return cp; 
        }

        Eigen::VectorXd get_velocities() 
        {
            Eigen::VectorXd v(dof);
            for(int i = 0; i < dof; ++i) {
                v[i] = previous_joint_state.velocity[i];
            }
            return v; 
        }

        Eigen::Vector3d get_tool_velocities() 
        {
            Eigen::Vector3d v;
            for(int i = 0; i < 3; ++i) {
                v[i] = previous_joint_state.velocity[i];
            }
            return v; 
        }

    private:
        // arm service clients
        ros::ServiceClient gravity_comp_srvs; 
        ros::ServiceClient haptic_sphere_srvs;
        ros::ServiceClient go_home_srvs;
        ros::ServiceClient hold_jnt_srvs;
        ros::ServiceClient hold_cart_pos_srvs;
        ros::ServiceClient hold_ortn_srvs;
        ros::ServiceClient hold_ortn2_srvs;
        ros::ServiceClient joint_move_srvs;
        ros::ServiceClient joint_move_block_srvs;
        ros::ServiceClient pose_move_srvs;
        ros::ServiceClient cart_move_srvs;
        ros::ServiceClient cart_vel_srvs;
        ros::ServiceClient ortn_move_srvs;
        ros::ServiceClient ortn_split_move_srvs;
        ros::ServiceClient force_torque_base_srvs;
        ros::ServiceClient force_torque_tool_srvs;
        ros::ServiceClient teach_motion_srvs;
        ros::ServiceClient play_srvs;
        ros::ServiceClient link_arm_srvs;
        ros::ServiceClient unlink_arm_srvs;
        ros::ServiceClient start_visual_fix_srvs;
        ros::ServiceClient stop_visual_fix_srvs;
        ros::ServiceClient follow_path_srvs;
        ros::ServiceClient get_tool_pose_srvs;
        ros::ServiceClient force_torque_base_time_srvs;

        // ROS callbacks
        void pose_cb(geometry_msgs::PoseStampedConstPtr pose) { previous_pose = *pose; }
        void joint_state_cb(sensor_msgs::JointStateConstPtr state) { previous_joint_state = *state; }
        void jacobian_cb(wam_msgs::MatrixMNConstPtr jac) { previous_jacobian = *jac; }
};

#endif // ARM_CONTROL_H

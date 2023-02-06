#include "uvs_bridge/arm_control.h"

ArmControl::ArmControl(ros::NodeHandle nh) {
	// initialize global variables
	connected_to_zeus = false;
	connected_to_slax = false;
	dof = 0;
	std::string t = current_time();
    wam_namespace = "";
	initialize_wam_connection();
	ROS_INFO_STREAM("ArmControl: Connected to " << wam_namespace << " with " << dof << " DOF");
	// Arm Services
	gravity_comp_srvs = nh.serviceClient<wam_srvs::GravityComp>(wam_namespace + "/wam/gravity_comp");
	haptic_sphere_srvs = nh.serviceClient<wam_srvs::HapticSphere>(wam_namespace + "/wam/haptic_sphere");
	go_home_srvs = nh.serviceClient<std_srvs::Empty>(wam_namespace + "/wam/go_home");
	hold_jnt_srvs = nh.serviceClient<wam_srvs::Hold>(wam_namespace + "/wam/hold_joint_pos");
	hold_cart_pos_srvs = nh.serviceClient<wam_srvs::Hold>(wam_namespace + "/wam/hold_cart_pos");
	hold_ortn_srvs = nh.serviceClient<wam_srvs::Hold>(wam_namespace + "/wam/hold_ortn");
	hold_ortn2_srvs = nh.serviceClient<wam_srvs::Hold>(wam_namespace + "/wam/hold_ortn2");
	joint_move_srvs = nh.serviceClient<wam_srvs::JointMove>(wam_namespace + "/wam/joint_move");
	joint_move_block_srvs = nh.serviceClient<wam_srvs::JointMoveBlock>(wam_namespace + "/wam/joint_move_block");
	pose_move_srvs = nh.serviceClient<wam_srvs::PoseMove>(wam_namespace + "/wam/pose_move");
	cart_move_srvs = nh.serviceClient<wam_srvs::CartPosMove>(wam_namespace + "/wam/cart_move");
	cart_vel_srvs = nh.serviceClient<wam_srvs::CartVel>(wam_namespace + "/wam/cart_vel");
	ortn_move_srvs = nh.serviceClient<wam_srvs::OrtnMove>(wam_namespace + "/wam/ortn_move");
	ortn_split_move_srvs = nh.serviceClient<wam_srvs::OrtnSplitMove>(wam_namespace + "/wam/ortn_split_move");
	force_torque_base_srvs = nh.serviceClient<wam_srvs::ForceTorqueTool>(wam_namespace + "/wam/force_torque_base");
	force_torque_tool_srvs = nh.serviceClient<wam_srvs::ForceTorqueTool>(wam_namespace + "/wam/force_torque_tool");
	force_torque_base_time_srvs = nh.serviceClient<wam_srvs::ForceTorqueToolTime>(wam_namespace + "/wam/force_torque_base_time");
	teach_motion_srvs = nh.serviceClient<wam_srvs::Teach>(wam_namespace + "/wam/teach_motion");
	play_srvs = nh.serviceClient<wam_srvs::Play>(wam_namespace + "/wam/play_motion", true); // true to mark persistent connection -- meaning it may not always be valid
	link_arm_srvs = nh.serviceClient<wam_srvs::Link>(wam_namespace + "/wam/link_arm");
	unlink_arm_srvs = nh.serviceClient<std_srvs::Empty>(wam_namespace + "/wam/unlink_arm");
	start_visual_fix_srvs = nh.serviceClient<std_srvs::Empty>(wam_namespace + "/wam/start_visual_fix");
	stop_visual_fix_srvs = nh.serviceClient<std_srvs::Empty>(wam_namespace + "/wam/stop_visual_fix");
	follow_path_srvs = nh.serviceClient<wam_srvs::FollowPath>(wam_namespace + "/wam/follow_path");
	get_tool_pose_srvs = nh.serviceClient<wam_srvs::GetToolPose>(wam_namespace + "/wam/get_tool_pose");

	// ROS Subscribers
	pose_sub = nh.subscribe(wam_namespace + "/wam/pose", 1, &ArmControl::pose_cb, this);
	joint_sub = nh.subscribe(wam_namespace + "/wam/joint_states", 1, &ArmControl::joint_state_cb, this);
	jacobian_sub = nh.subscribe(wam_namespace + "/wam/jacobian", 1, &ArmControl::jacobian_cb, this);
	// ros::spinOnce();
}

ArmControl::~ArmControl() 
{
	gravity_comp_srvs.shutdown();
	haptic_sphere_srvs.shutdown();
	go_home_srvs.shutdown();
	hold_jnt_srvs.shutdown();
	hold_cart_pos_srvs.shutdown();
	hold_ortn_srvs.shutdown();
	hold_ortn2_srvs.shutdown();
	joint_move_srvs.shutdown();
	joint_move_block_srvs.shutdown();
	pose_move_srvs.shutdown();
	cart_move_srvs.shutdown();
	cart_vel_srvs.shutdown();
	ortn_move_srvs.shutdown();
	ortn_split_move_srvs.shutdown();
	force_torque_base_srvs.shutdown();
	force_torque_tool_srvs.shutdown();
	teach_motion_srvs.shutdown();
	play_srvs.shutdown();
	link_arm_srvs.shutdown();
	unlink_arm_srvs.shutdown();
	start_visual_fix_srvs.shutdown();
	stop_visual_fix_srvs.shutdown();
	follow_path_srvs.shutdown();
	pose_sub.shutdown();
	joint_sub.shutdown();
	jacobian_sub.shutdown();
}

void ArmControl::initialize_wam_connection()
{
	bool connected = false;
	std::string z = "/zeus";
	std::string s = "/slax";
	while (!connected) {
		ros::master::V_TopicInfo topics;
		ros::master::V_TopicInfo::iterator it;
		ros::master::getTopics(topics);
		for (it = topics.begin(); it != topics.end(); it++) {
			const ros::master::TopicInfo& info = *it;
			std::string topic_name = info.name;
			if (topic_name.substr(0, z.size()) == z) {
				connected_to_zeus = true;
			}
			if (topic_name.substr(0, s.size()) == s) {
				connected_to_slax = true;
			}	
		}
		if (!connected_to_zeus && !connected_to_slax) { 
			std::cout << "waiting to connect to wam..." << std::endl;
			ros::Duration(1.0).sleep();
			continue; 
		} else {
			connected = true;
			if (connected_to_zeus && (!connected_to_slax)) {
				wam_namespace = z;
				dof = 7;
			} else if ((!connected_to_zeus) && connected_to_slax) {
				wam_namespace = s;
				dof = 4;
			} else {
				while (true) {
					std::string reply = string_input("Would you like to connect to zeus or slax?");
					if (reply[0] == 'z' || reply[0] == 'Z') {
						wam_namespace = z;
						dof = 7;
						break;
					} else if (reply[0] == 's' || reply[0] == 'S') {
						wam_namespace = s;
						dof = 4;
						break;
					}
				}
			}
		}
	}
}

/*************************************************************************
 * ARM SERVICE CALL METHODS
 *************************************************************************/
bool ArmControl::call_haptic_sphere()
{ // set a haptic sphere
    bool t = boolean_input("Would you like to set haptic sphere to true or false? >> "); 
	wam_srvs::HapticSphere req;
	req.request.radius = 0.05;
	req.request.kp = 4e3;
	req.request.kd = 3e1;
	req.request.trigger = t;
	return haptic_sphere_srvs.call(req);  
}


bool ArmControl::set_gravity_compensation()
{ // set gravity compensation to users input
    bool compensation = boolean_input("Would you like to set gravity compensation to true or false? >> "); 
	wam_srvs::GravityComp req;
	req.request.gravity = compensation;
	return gravity_comp_srvs.call(req);  
}

bool ArmControl::set_gravity_compensation(bool compensation)
{ // set gravity compensation to parameter value
	wam_srvs::GravityComp req;
	req.request.gravity = compensation;
	return gravity_comp_srvs.call(req);  
}

bool ArmControl::lock_joint_position()
{ // set joint position lock to users input
    bool lock = boolean_input("Would you like to lock joint positions? >> "); 
	wam_srvs::Hold req;
	req.request.hold = lock;
	return hold_jnt_srvs.call(req);  
}

bool ArmControl::lock_joint_position(bool lock)
{ // set joint position lock to parameter value
	wam_srvs::Hold req;
	req.request.hold = lock;
	return hold_jnt_srvs.call(req);  
}

bool ArmControl::lock_cartesian_position()
{  // set cartesian position lock to users input
    bool lock = boolean_input("Would you like to lock end effector cartesian position? >> "); 
	wam_srvs::Hold req;
	req.request.hold = lock;
	return hold_cart_pos_srvs.call(req);  
}

bool ArmControl::lock_cartesian_position(bool lock)
{  // set cartesian position lock to parameter value
	wam_srvs::Hold req;
	req.request.hold = lock;
	return hold_cart_pos_srvs.call(req);  
}

bool ArmControl::lock_eef_orientation()
{  // set eef orientation lock to users input
    bool lock = boolean_input("Would you like to lock end effector orientation? >> "); 
	wam_srvs::Hold req;
	req.request.hold = lock;
	return hold_ortn_srvs.call(req);  
}

bool ArmControl::lock_eef_orientation(bool lock)
{  // set eef orientation lock to parameter value
	wam_srvs::Hold req;
	req.request.hold = lock;
	return hold_ortn_srvs.call(req);  
}

bool ArmControl::lock_eef_orientation2()
{  // set eef orientation lock to users input
    bool lock = boolean_input("Would you like to lock end effector orientation? >> "); 
	wam_srvs::Hold req;
	req.request.hold = lock;
	return hold_ortn2_srvs.call(req);  
}

bool ArmControl::lock_eef_orientation2(bool lock)
{  // set eef orientation lock to parameter value
	wam_srvs::Hold req;
	req.request.hold = lock;
	return hold_ortn2_srvs.call(req);  
}

bool ArmControl::call_move_joints(const Eigen::VectorXd& target_joint_state, bool block_control) 
{ // move joints to desired position in joint space
	wam_srvs::JointMoveBlock req;
	for(long i = 0; i < target_joint_state.size(); ++i) {
		req.request.joints.push_back(target_joint_state[i]);
	}
	req.request.blocking = block_control;
	bool res = joint_move_block_srvs.call(req);
	return res;
}

bool ArmControl::move_one_joint()
{ // move single joint according to users input in joint space and move arm
	print_joint_positions_to_terminal();
	Eigen::VectorXd target = get_positions();
    single_joint_position_input(dof, target);
	bool block_control = boolean_input("Would you like to set blocking to true or false? >> ");
	return call_move_joints(target, block_control);;
}

Vector7d ArmControl::get_tool_pose(const Eigen::VectorXd& target_joint_state)
{
	//get tool pose given a joint coordinate set
	wam_srvs::GetToolPose req;
	Vector7d tool_pose; // cart_position then quaternion for 7 components

	for (int i = 0; i < dof; ++i)
	{
		req.request.joint_position.push_back(target_joint_state[i]);
	}
	get_tool_pose_srvs.call(req);
	tool_pose[0] = req.response.cart_position[0];
	tool_pose[1] = req.response.cart_position[1];
	tool_pose[2] = req.response.cart_position[2];
	tool_pose[3] = req.response.quaternion[0];
	tool_pose[4] = req.response.quaternion[1];
	tool_pose[5] = req.response.quaternion[2];
	tool_pose[6] = req.response.quaternion[3];

	return tool_pose;
}

bool ArmControl::move_one_joint_by_delta()
{ // perturb single joint by radian of users choosing and move arm
	print_joint_positions_to_terminal();
	Eigen::VectorXd target = get_positions();
    single_joint_delta_input(dof, target);
	bool block_control = boolean_input("Would you like to set blocking to true or false? >> ");
	return call_move_joints(target, block_control);;
}

bool ArmControl::move_all_joints() 
{ // get users input to set joint positions and move arm
	Eigen::VectorXd target = all_joint_positions_input(dof);
	bool block_control = boolean_input("Would you like to set blocking to true or false? >> ");
	return call_move_joints(target, block_control);
}

bool ArmControl::move_to_zero() 
{ // move arm to zero position
	if (dof == 4) {
		Eigen::VectorXd zero_pos(4);
		zero_pos << 0, 0, 0, 0;
		return call_move_joints(zero_pos, true);
	} else if (dof == 7) {
		Eigen::VectorXd zero_pos(7);
		zero_pos << 0, 0, 0, 0, 0, 0, 0;
		return call_move_joints(zero_pos, true);
	}
	return false;
}

bool ArmControl::move_to_home_position()
{ // move to home position
	std_srvs::Empty e;
	bool res = go_home_srvs.call(e);
	return res;
}

bool ArmControl::move_to_initial_position() 
{ // move arm to initial postition
	if (dof == 4) {
		Eigen::VectorXd initial_pos(4);
		initial_pos << 0, 0, 0, 1.57;
		return call_move_joints(initial_pos, true);
	} else if (dof == 7) {
		Eigen::VectorXd initial_pos(7);
		initial_pos << 0, 0, 0, 1.57, 0, 0, 0;
		return call_move_joints(initial_pos, true);
	}
	return false;
}

bool ArmControl::move_to_previous_position() 
{ // move robot to desired pose and press enter, then move to arbitrary position and press enter
    wam_srvs::PoseMove req;
    // // req.request.hold = hold_joints;
    // return pose_move_srvs.call(req);
    ROS_WARN_STREAM("TODO");
    return true;
}

bool ArmControl::pose_move(const Eigen::VectorXd& target_pose) 
{ // move robot to desired pose and press enter, then move to arbitrary position and press enter
    wam_srvs::PoseMove srv;
	// Eigen::VectorXd pose = getToolPose(ArmControl::get_positions(), 7);
	srv.request.pose.position.x = target_pose[0];
	srv.request.pose.position.y = target_pose[1];
	srv.request.pose.position.z = target_pose[2];
	srv.request.pose.orientation.x = target_pose[3];
	srv.request.pose.orientation.y = target_pose[4];
	srv.request.pose.orientation.z = target_pose[5];
	srv.request.pose.orientation.w = target_pose[6];
    // // req.request.hold = hold_joints;
    return pose_move_srvs.call(srv);
    // ROS_WARN_STREAM("TODO");
    // return true;
}


bool ArmControl::call_move_orientation(const Eigen::VectorXd& target_orientation)
{ // move to target orientation 
	wam_srvs::OrtnMove req;
	req.request.orientation[0] = target_orientation[0];
	req.request.orientation[1] = target_orientation[1];
	req.request.orientation[2] = target_orientation[2];
	req.request.orientation[3] = target_orientation[3];
	bool res = ortn_move_srvs.call(req);
	return res;
}

bool ArmControl::move_orientation()
{ // set orientation with users input and move arm
	print_orientation_to_terminal();
    Eigen::VectorXd target_orientation = orientation_input(0, 100); 
	return call_move_orientation(target_orientation);
}

bool ArmControl::call_move_orientation_split(const Eigen::VectorXd& target_orientation, const Eigen::VectorXd& gains) 
{ // move arm to set orientation with gains
	wam_srvs::OrtnSplitMove req;
	req.request.orientation[0] = target_orientation[0];
	req.request.orientation[1] = target_orientation[1];
	req.request.orientation[2] = target_orientation[2];
	req.request.orientation[3] = target_orientation[3];
	req.request.kp_gain[0] = gains[0];
	req.request.kp_gain[1] = gains[1];
	req.request.kp_gain[2] = gains[2];
	req.request.kd_gain[0] = gains[3];
	req.request.kd_gain[1] = gains[4];
	req.request.kd_gain[2] = gains[5];
	bool res = ortn_split_move_srvs.call(req);
	return res;
}

bool ArmControl::move_orientation_split() 
{ // set orientation and gains with users input and move arm
	print_orientation_to_terminal();
    Eigen::VectorXd target_orientation = orientation_input(0, 100); 
    Eigen::VectorXd gains = gains_input(0, 1);
	return call_move_orientation_split(target_orientation, gains);
}

bool ArmControl::call_move_cartesian(const Eigen::VectorXd& target_position) 
{ // move arm to target cartesian position
	wam_srvs::CartPosMove req;
	req.request.position[0] = target_position[0];
	req.request.position[1] = target_position[1];
	req.request.position[2] = target_position[2];
	bool res = cart_move_srvs.call(req);
	return res;
}

bool ArmControl::move_cartesian() 
{ // set cartesian position and move arm
	print_cartesian_position_to_terminal();
	Eigen::VectorXd target_position = cartesian_input(0, 1); // TODO get real min and max values
	return call_move_cartesian(target_position);
}

bool ArmControl::call_move_cartesian_velocity(const Eigen::VectorXd& target) 
{ // move arm with target cartesian velocity
	wam_srvs::CartVel req;
	req.request.v_direction[0] = target[0];
	req.request.v_direction[1] = target[1];
	req.request.v_direction[2] = target[2];
	req.request.v_magnitude = 0.1;
	req.request.kp = 75;
	req.request.visual_system = false;
	return cart_vel_srvs.call(req);
}

bool ArmControl::move_cartesian_velocity() 
{ // set cartesian velocity and move arm
	print_cartesian_position_to_terminal();
	Eigen::VectorXd target = cartesian_input(0, 10); // TODO get real min and max values
	return call_move_cartesian_velocity(target);
}

bool ArmControl::call_move_force_torque(const Eigen::VectorXd& f, const Eigen::VectorXd& t, bool base) 
{ // move arm with target force and torque with respect to either base or tool
	wam_srvs::ForceTorqueTool req;
	req.request.force.push_back(f[0]);
	req.request.force.push_back(f[1]);
	req.request.force.push_back(f[2]);
	req.request.torque.push_back(t[0]);
	req.request.torque.push_back(t[1]);
	req.request.torque.push_back(t[2]);
	if (base) { return force_torque_base_srvs.call(req); }
	else { return force_torque_tool_srvs.call(req); }
}

bool ArmControl::move_force_torque_base() 
{ // set force and torque then move arm
	std::cout << "Setting force..." << std::endl;
	Eigen::VectorXd force = cartesian_input(0, 10); // TODO get real min and max values
	std::cout << "Setting torque..." << std::endl;
	Eigen::VectorXd torque = cartesian_input(0, 10);
	return call_move_force_torque(force, torque, true);
}

bool ArmControl::move_force_torque_tool() 
{ // set force and torque then move arm
	std::cout << "Setting force..." << std::endl;
	Eigen::VectorXd force = cartesian_input(0, 10);
	std::cout << "Setting torque..." << std::endl;
	Eigen::VectorXd torque = cartesian_input(0, 10);
	return call_move_force_torque(force, torque, false);
}

bool ArmControl::teach_motion(std::string path_name) 
{
	std::cout << "Saving to: " << path_name << std::endl;
	wam_srvs::Teach req;
	req.request.path = path_name;
	return teach_motion_srvs.call(req);
}

bool ArmControl::follow_motion(std::string path_name) 
{
	wam_srvs::Play req;
	req.request.path = "motions/" + path_name;
	return play_srvs.call(req);
}

bool ArmControl::link_arms()
{
	wam_srvs::Link req;
	if (dof == 4) { req.request.remote_ip = "192.168.1.10"; }
	else { req.request.remote_ip = "192.168.1.11"; }
	return link_arm_srvs.call(req);
}

bool ArmControl::unlink_arms() 
{
	std_srvs::Empty e;
	return unlink_arm_srvs.call(e);
}

bool ArmControl::start_visual_fix() 
{
	std_srvs::Empty e;
	return start_visual_fix_srvs.call(e);
}

bool ArmControl::stop_visual_fix() 
{
	std_srvs::Empty e;
	return stop_visual_fix_srvs.call(e);
}

bool ArmControl::follow_path() 
{
	// wam_srvs::FollowPath req;
	// req.request.hold = hold_joints;
	// return follow_path_srvs.call(req);
	ROS_WARN_STREAM("TODO");
	return true;
}

/*************************************************************************
 * INFORMATION DEBUGGING METHODS
 *************************************************************************/
void ArmControl::print_information_to_terminal() 
{ // print all information to the terminal
	std::cout << "\n****************************************************************************" <<
		"\nwam namespace: " << wam_namespace <<
		"\ndegrees of freedom: " << dof <<
		"\nend effector position (x, y, z): (" << previous_pose.pose.position.x <<
		", " << previous_pose.pose.position.y <<
		", " << previous_pose.pose.position.z << ")" <<
		"\nend effector orientation (x, y, z, w): (" << previous_pose.pose.orientation.x <<
		", " << previous_pose.pose.orientation.y <<
		", " << previous_pose.pose.orientation.z <<
		", " << previous_pose.pose.orientation.w << ")" <<
				std::endl;	
	for (int i = 0; i < dof; ++i) {
		std::cout << "\njoint " << i + 1 << ":" <<
			"\n\tposition: " << previous_joint_state.position[i] <<
			"\n\tvelocity: " << previous_joint_state.velocity[i] <<
			"\n\ttorque: " << previous_joint_state.effort[i];	}
	std::cout << "\n****************************************************************************" << std::endl;
}

void ArmControl::print_joint_positions_to_terminal() 
{ // prints current joint positions to the terminal
	std::cout << "****************************************************************************" << std::endl;
	std::cout << "current joint positions: [";
	for (int i = 0; i < dof-1; ++i) {
		std::cout << previous_joint_state.position[i] << ", ";
	}
	std::cout << previous_joint_state.position[dof-1] << "]" << std::endl;
	std::cout << "****************************************************************************" << std::endl;
}

void ArmControl::print_orientation_to_terminal() 
{ // prints current end effector orientation to the terminal
	std::cout << "****************************************************************************" << std::endl;
	std::cout << "current end effector orientation (x, y, z, w): (" << previous_pose.pose.orientation.x <<
		", " << previous_pose.pose.orientation.y <<
		", " << previous_pose.pose.orientation.z <<
		", " << previous_pose.pose.orientation.w << ")" << std::endl;	
	std::cout << "****************************************************************************" << std::endl;
}

void ArmControl::print_cartesian_position_to_terminal() 
{ // prints current cartesian position to the terminal
	std::cout << "****************************************************************************" << std::endl;
	std::cout << "current end effector position (x, y, z): (" << previous_pose.pose.position.x <<
		", " << previous_pose.pose.position.y <<
		", " << previous_pose.pose.position.z << ")" << std::endl;	
	std::cout << "****************************************************************************" << std::endl;
}

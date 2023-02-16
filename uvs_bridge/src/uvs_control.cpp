#include <fstream>
#include "uvs_bridge/uvs_control.h"
using namespace std;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

UVSControl::UVSControl(ros::NodeHandle nh_)
{
	image_tol = 50.0;
	dof = 4;
	arm = new ArmControl(nh_, "/wam", 4);
	total_joints = 4;
	dof = 3;
	ROS_INFO_STREAM("Robot has " << dof << " DOF");
	error_sub = nh_.subscribe("/tracker/image_error", 1, &UVSControl::error_cb, this);
	eef_sub = nh_.subscribe("/tracker/end_effector", 1, &UVSControl::eef_cb, this);
}

UVSControl::~UVSControl()
{ // shutdown ROS subscribers properly
	error_sub.shutdown();
	eef_sub.shutdown();
	delete arm;
}

Eigen::VectorXd UVSControl::calculate_delta_q()
{ // calculates the actual motion change in joint space to use in Broyden's update
	Eigen::VectorXd total_dq;
	Eigen::VectorXd dq(dof);
	Eigen::VectorXd current_joint_positions = arm->get_positions();
	total_dq = current_joint_positions - previous_joint_positions;
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			dq[j] = total_dq[i];
			j++;
		}
	}
	return dq;
}

Eigen::VectorXd UVSControl::calculate_target(const Eigen::VectorXd &current_state, const Eigen::VectorXd &delta)
{ // calculates target vector in joint space to move to with given delta added to active joints
	Eigen::VectorXd target_state(total_joints);
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			target_state[i] = (current_state[i] + delta[j]);
			j++;
		} else {
			target_state[i] = current_state[i];
		}
	}
	return target_state;
}

Eigen::VectorXd UVSControl::calculate_step(const Eigen::VectorXd &current_error_value)
{ // calculates new motion step to take with the control law: step = −λJ+e
	Eigen::VectorXd step;
	step = jacobian_inverse * current_error_value;
	step *= 0.1;
	return step;
}

bool UVSControl::convergence_check(const Eigen::VectorXd &current_error)
{ 
	double n = current_error.norm();
	if (n < image_tol) {
		std::cout << "current error norm is less than image tolerance -- we have arrived at our destination" << std::endl;
		return true;
	}
	std::cout << "current error norm: " << n << std::endl;
	return false;
}

bool UVSControl::broyden_update(double alpha)
{ // update jacobian
	Eigen::MatrixXd update(jacobian.rows(), jacobian.cols());
	Eigen::VectorXd current_eef_position;
	Eigen::VectorXd dy;
	Eigen::VectorXd dq;
	double dq_norm;

	dq = calculate_delta_q();
	dq_norm = dq.norm();
	if (dq_norm < 1e-3) {
		cout << "Small dq - no update" << endl;
		std::cout << "dq: \n" << dq.format(CleanFmt) << std::endl;
		return true;
	}
	current_eef_position = get_eef_position();
	dy = current_eef_position - previous_eef_position;
	if (dy.norm() < 1) {
		cout << "Small dy - no update" << endl;
		std::cout << "dy: \n" << dy.format(CleanFmt) << std::endl;
		return true;
	}
	update = ((dy - jacobian * dq) * dq.transpose()) / ((dq.transpose() * dq) + 0.001);
	previous_jacobian = jacobian;
	jacobian = jacobian + (alpha * update);
	pseudoInverse(jacobian, jacobian_inverse);

	std::cout << "current_eef_position: \n" << current_eef_position.format(CleanFmt) << std::endl;
	std::cout << "dq: \n" << dq.format(CleanFmt) << std::endl;
	std::cout << "dy: \n" << dy.format(CleanFmt) << std::endl;
	std::cout << "update: \n" << update.format(CleanFmt) << std::endl;
	std::cout << "jacobian: \n" << jacobian.format(CleanFmt) << std::endl;
	std::cout << "jacobian_inverse: \n" << jacobian_inverse.format(CleanFmt) << std::endl;
	return true;
}

int UVSControl::move_step()
{ // step through one loop of VS
	Eigen::VectorXd current_error;
	Eigen::VectorXd current_joint_positions;
	Eigen::VectorXd step_delta;
	Eigen::VectorXd target_position;
	double sleep_time = 0.2;
	// grab and use current error, check for convergence
	current_error = get_error();
	if (convergence_check(current_error)) {
		return 0;
	}
	step_delta = calculate_step(current_error);
	// grab and use current joint positions, check if valid
	current_joint_positions = arm->get_positions();
	target_position = calculate_target(current_joint_positions, step_delta);

	if (!limit_check(target_position, total_joints)) {
		return 1;
	}

	previous_joint_positions = current_joint_positions;
	previous_eef_position = get_eef_position();

	if (confirm_movement) {
		// write to screen for debugging
		std::cout << "current_error: \n" << current_error.format(CleanFmt) << std::endl;
		std::cout << "previous_joint_positions: \n" << previous_joint_positions.format(CleanFmt) << std::endl;
		std::cout << "current_joint_positions: \n" << current_joint_positions.format(CleanFmt) << std::endl;
		std::cout << "previous_eef_position: \n" << previous_eef_position.format(CleanFmt) << std::endl;
		std::cout << "step_delta: \n" << step_delta.format(CleanFmt) << std::endl;
		std::cout << "target_position: \n" << target_position.format(CleanFmt) << std::endl;
		bool response = boolean_input("continue [y/n]");
		if (response) {
			arm->call_move_joints(target_position, false);
		} else {
			return 0;
		}
	} else {
		arm->call_move_joints(target_position, false);
	}

	ros::Duration(sleep_time).sleep();
	return 2;
}


void UVSControl::converge(double alpha, int max_iterations)
{
	int c;
	std::cout << "\n**************************************" << std::endl;
	for (int i = 0; i < max_iterations; ++i) {
		std::cout << "iteration: " << i << std::endl;
		ros::Time begin = ros::Time::now();
		c = move_step();
		switch (c) {
			case 0: // convergence - return early
				return;
			case 1: // joints out of limit
				std::cout << "target not within joint limits, stopping \n" << std::endl;
				return;
			case 2: // step completed successfully
				broyden_update(alpha);
				break;
		}
		std::cout << "loop duration: " << ros::Time::now() - begin << "\n**************************************" << std::endl;
	}
	return;
}


bool UVSControl::jacobian_estimate(double perturbation_delta)
{ // perturb each active joint for the initial jacobian estimation
	Eigen::VectorXd e1;
	Eigen::VectorXd e2;
	Eigen::VectorXd target;
	Eigen::VectorXd position;
	jacobian.resize(get_error().size(), dof);
	initial_jacobian.resize(get_error().size(), dof);
	jacobian_inverse.resize(dof, get_error().size());
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			ros::Duration(0.2).sleep();
			position = arm->get_positions();
			target = vector_target(position, i, -perturbation_delta);
			arm->call_move_joints(target, true);
			ros::Duration(0.2).sleep();
			e1 = get_eef_position();
			target = vector_target(position, i, perturbation_delta);
			arm->call_move_joints(target, true);
			ros::Duration(0.2).sleep();
			e2 = get_eef_position();
			ros::Duration(0.2).sleep();
			arm->call_move_joints(position, true);
			jacobian.col(j) = (e2 - e1) / (2 * perturbation_delta);
			j++;
		}
	}
	initial_jacobian = jacobian;
	previous_joint_positions = arm->get_positions();
	previous_eef_position = get_eef_position();
	if (!pseudoInverse(jacobian, jacobian_inverse)) {
		std::cout << "Initial jacobian estimate failed -- condition number too large" << std::endl;
		return false;
	}
	std::cout << "initial jacobian: \n" << initial_jacobian.format(CleanFmt) << std::endl;
	std::cout << "inverse jacobian: \n" << jacobian_inverse.format(CleanFmt) << std::endl;

	return true;
}

void UVSControl::loop()
{ // main loop for user interaction
	bool jacobian_initialized = false;
	bool exit_loop = false;
	double perturbation_delta = 0.15;
	double alpha = 0.5; // update rate
	int max_iterations = 25;
	Eigen::VectorXd pose;
	std::string line;
	std::string s;
	while (ros::ok() && !exit_loop) {
		std::cout << "**********************************************************"
				  << "\nSelect option:"
				  << "\n\tm: Move one joint"
				  << "\n\tj: Initialize Jacobian (Central Diff)"
				  << "\n\tv: Complete VS convergence with set max iterations"
				  << "\n\ts: Compute and move one step"
				  << "\n\ti: Move to initial position"
				  << "\n\th: Move to home position"
				  << "\n\tp: Print info"
				  << "\n\tq: quit"
				  << "\n\t>> " << std::endl;
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'm':
			arm->move_one_joint();
			break;
		case 'j':
			if (ready()) {
				jacobian_initialized = jacobian_estimate(perturbation_delta);
			}
			break;
		case 'i':
			{
				bool res = arm->move_to_initial_position();
				if (!res) {
					ROS_WARN_STREAM("unsuccessful move");
				}
				break;
			}
		case 'v':
			if (ready() && jacobian_initialized) {
				converge(alpha, max_iterations - 1);
			} else {
				ROS_WARN_STREAM("Jacobian is not initialized");
			}
			break;
		case 's':
			if (ready() && jacobian_initialized) {
				converge(alpha, 1);
			} else {
				ROS_WARN_STREAM("Jacobian is not initialized");
			}
			break;
		case 'h':
			arm->move_to_home_position();
			break;
		case 'q':
			exit_loop = true;
			break;
		case 'p':
			arm->print_information_to_terminal();
			break;
		default:
			ROS_WARN_STREAM("Unknown option");
		}
	}
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "UVSControl");
	ros::NodeHandle nh_("~");
	ros::AsyncSpinner spinner(0);
	spinner.start();
	UVSControl uvs(nh_);
	uvs.loop();
	return 0;
}

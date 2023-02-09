#ifndef MISC_UTILITIES_H
#define MISC_UTILITIES_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdarg>
#include <fstream>
#include <string>
#include <sstream>
#include <ctime>
#include <boost/filesystem.hpp>
#include <boost/range.hpp>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

Eigen::Vector2d J1(-2.46, 2.46);
Eigen::Vector2d J2(-1.91, 1.91);
Eigen::Vector2d J3(-2.74, 2.74);
Eigen::Vector2d J4(-0.88, 3.13);
Eigen::Vector2d J5(-4.73, 1.35);
Eigen::Vector2d J6(-1.52, 1.52);
Eigen::Vector2d J7(-2.34, 2.34);
std::array<Eigen::Vector2d, 7> JOINT_RANGES = {J1, J2, J3, J4, J5, J6, J7};

float sign(double num)
{
    return num / std::abs(num);
}

Eigen::VectorXd normalize(const Eigen::VectorXd& vec)
{
    return vec/vec.norm();
}

Eigen::VectorXd getToolPose(const Eigen::VectorXd& joint_positions, int dof)
{   
    double d3 = 4.76e-2;
    double L3 = 55.8e-2;
    double L4 = 29.1e-2;
    double L7 = 6.5e-2;
    Eigen::VectorXd denavit_a(7);
    denavit_a << 0.0, 0.0, 0.0, d3, -d3, 0.0, 0.0;
    Eigen::VectorXd denavit_alpha(7);
    denavit_alpha << 0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0;
    Eigen::VectorXd denavit_d(7);
    denavit_d << 0, 0, L3, 0, L4, 0, L7;
    Eigen::VectorXd tool_pose(7);
    Eigen::Vector3d tool_pos;
    Eigen::Matrix4d temp_matrix;
    Eigen::Matrix4d hom_mat = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix3d rot_mat;
    Eigen::VectorXd cos_theta = joint_positions.array().cos();
    Eigen::VectorXd sin_theta = joint_positions.array().sin();
    Eigen::VectorXd cos_alpha = denavit_alpha.array().cos();
    Eigen::VectorXd sin_alpha = denavit_alpha.array().sin();
    for (int i = 0; i < dof; ++i)
    {   
        temp_matrix(0, 0) = cos_theta[i];
        temp_matrix(0, 1) = -sin_theta[i];
        temp_matrix(0, 2) = 0.0;
        temp_matrix(0, 3) = denavit_a[i];
        //
        temp_matrix(1, 0) = sin_theta[i]*cos_alpha[i];
        temp_matrix(1, 1) = cos_theta[i]*cos_alpha[i];
        temp_matrix(1, 2) = -sin_alpha[i];
        temp_matrix(1, 3) = -sin_alpha[i]*denavit_d[i];
        //
        temp_matrix(2, 0) = sin_theta[i]*sin_alpha[i];
        temp_matrix(2, 1) = cos_theta[i]*sin_alpha[i];
        temp_matrix(2, 2) = cos_alpha[i];
        temp_matrix(2, 3) = cos_alpha[i]*denavit_d[i];
        //
        temp_matrix(3, 0) = 0.0;
        temp_matrix(3, 1) = 0.0;
        temp_matrix(3, 2) = 0.0;
        temp_matrix(3, 3) = 1.0;
        hom_mat = hom_mat*temp_matrix;
    }
    for (int j = 0; j < 3; ++j)
    {
        tool_pos[j] = hom_mat(j, 3);
        for (int i = 0; i < 3; ++i)
        {
            rot_mat(i, j) = hom_mat(i, j);
        }
    }
    Eigen::Quaterniond tool_ortn(rot_mat.transpose());
    // tool_ortn = tool_ortn.conjugate();
    tool_pose << tool_pos[0], tool_pos[1], tool_pos[2], tool_ortn.x(), tool_ortn.y(), tool_ortn.z(), tool_ortn.w();
    return tool_pose;
}

Eigen::Vector3d getToolPosition(const Eigen::VectorXd& joint_positions, int dof)
{   
    double d3 = 4.76e-2;
    double L3 = 55.8e-2;
    double L4 = 29.1e-2;
    double L7 = 6.5e-2;
    Eigen::VectorXd denavit_a(7);
    denavit_a << 0.0, 0.0, 0.0, d3, -d3, 0.0, 0.0;
    Eigen::VectorXd denavit_alpha(7);
    denavit_alpha << 0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0;
    Eigen::VectorXd denavit_d(7);
    denavit_d << 0, 0, L3, 0, L4, 0, L7;
    Eigen::Vector3d tool_pos;
    Eigen::Matrix4d temp_matrix;
    Eigen::Matrix4d hom_mat = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix3d rot_mat;
    Eigen::VectorXd cos_theta = joint_positions.array().cos();
    Eigen::VectorXd sin_theta = joint_positions.array().sin();
    Eigen::VectorXd cos_alpha = denavit_alpha.array().cos();
    Eigen::VectorXd sin_alpha = denavit_alpha.array().sin();
    for (int i = 0; i < dof; ++i)
    {   
        temp_matrix(0, 0) = cos_theta[i];
        temp_matrix(0, 1) = -sin_theta[i];
        temp_matrix(0, 2) = 0.0;
        temp_matrix(0, 3) = denavit_a[i];
        //
        temp_matrix(1, 0) = sin_theta[i]*cos_alpha[i];
        temp_matrix(1, 1) = cos_theta[i]*cos_alpha[i];
        temp_matrix(1, 2) = -sin_alpha[i];
        temp_matrix(1, 3) = -sin_alpha[i]*denavit_d[i];
        //
        temp_matrix(2, 0) = sin_theta[i]*sin_alpha[i];
        temp_matrix(2, 1) = cos_theta[i]*sin_alpha[i];
        temp_matrix(2, 2) = cos_alpha[i];
        temp_matrix(2, 3) = cos_alpha[i]*denavit_d[i];
        //
        temp_matrix(3, 0) = 0.0;
        temp_matrix(3, 1) = 0.0;
        temp_matrix(3, 2) = 0.0;
        temp_matrix(3, 3) = 1.0;
        hom_mat = hom_mat*temp_matrix;
    }
    for (int j = 0; j < 3; ++j)
    {
        tool_pos[j] = hom_mat(j, 3);
    }
    return tool_pos;
}

bool limit_check(const Eigen::VectorXd& target, int dof) 
{
    for (int i = 0; i < dof; ++i) {
        if (JOINT_RANGES[i][1] < target[i] || target[i] < JOINT_RANGES[i][0]) {
            return false;
        }
    }
    return true;
}

void wait_for_enter() 
{
    std::cin.get();
}

std::string current_time()
{
    time_t curr_time;
    tm * curr_tm;
    char date_string[100];
    time(&curr_time);
    curr_tm = localtime(&curr_time);
    // strftime(date_string, 50, "_%m%d%H%M", curr_tm);
    strftime(date_string, 50, "%m%d%H%M", curr_tm);
    return date_string;
}

/*************************************************************************
 * INPUT UTILITIES
 *************************************************************************/
bool boolean_input(std::string msg)
{ // asks for boolean answer from user, accepts both true/false and yes/no
    std::string s;
    std::stringstream ss;
    ss.clear();
    ss.str("");
    while (true) {
        std::cout << msg << std::endl;
        while (!std::getline(std::cin, s)) { continue; } // wait for users input
        if (s[0] == 't' || s[0] == 'T' || s[0] == 'y' || s[0] == 'Y') { return true; }
        else if (s[0] == 'f' || s[0] == 'F' || s[0] == 'n' || s[0] == 'N') { return false; }
        ROS_WARN_STREAM("Invalid boolean input");
        ss.clear();
        ss.str("");
        s.clear();
    }
}

std::string string_input(std::string msg)
{ // gets string input from user
    std::string input = "";
    std::cout << msg;
    while (!std::getline(std::cin, input)) { continue; } // wait for users input
    return input;
}

double double_input(double min, double max)
{ // gets double input from user
    double d;
    std::stringstream ss;
    std::string s = "";
    std::cout << "please input a number in range [" << min << ", " << max << "] >> ";
    while (true) {
        while (!std::getline(std::cin, s)) { continue; }
        ss.str(s);
        if (ss >> d && min <= d && d <= max) { return d; }
        ROS_WARN_STREAM("invalid input\n\t>>");
        ss.clear();
        ss.str("");
        s.clear();
    }
}

Eigen::VectorXd vectorxd_input(double min, double max, std::vector<std::string> terminal_output)
{ // gets users input to store in return vector
    Eigen::VectorXd return_vector(terminal_output.size());
    for (size_t i = 0; i < terminal_output.size(); ++i) {
        std::cout << terminal_output[i] << std::endl;
        return_vector[i] = double_input(min, max);
    }
    return return_vector;
}

Eigen::VectorXd cartesian_input(double min, double max)
{ // asks user to cartesian values
    const char *args[] = {"setting x value", "setting y value", "setting z value"};
    std::vector<std::string> terminal_output(args, std::end(args));
    return vectorxd_input(min, max, terminal_output);
}

Eigen::VectorXd gains_input(double min, double max)
{ // asks user to set kp and kd gains
    const char *args[] = {"setting kp x value", "setting kp y value", "setting kp z value", "setting kd x value", "setting kd y value", "setting kd z value"};
    std::vector<std::string> terminal_output(args, std::end(args));
    return vectorxd_input(min, max, terminal_output);
}

Eigen::VectorXd orientation_input(double min, double max)
{ // asks user to set end effector orientation (x, y, z, w)
    const char *args[] = {"setting x value", "setting y value", "setting z value", "setting w value"};
    std::vector<std::string> terminal_output(args, std::end(args));
    return vectorxd_input(min, max, terminal_output);
}

Eigen::VectorXd all_joint_positions_input(int dof) 
{ // asks for users input to set all joint positions
    Eigen::VectorXd return_vector(dof);
    for (int i = 0; i < dof; ++i) {
        std::cout << "setting value for joint " << i+1 << std::endl;
        return_vector[i] = double_input(JOINT_RANGES[i][0], JOINT_RANGES[i][1]);
    }
    return return_vector;
}

void single_joint_position_input(Eigen::VectorXd& target) 
{ // asks user to set position of one joint and sets joint in current_position vector
    std::cout << "which joint would you like to move?" << std::endl;
    int idx = (int)double_input(0, 7);
    std::cout << "setting value for joint " << idx << std::endl;
    target[idx-1] = double_input(JOINT_RANGES[idx-1][0], JOINT_RANGES[idx-1][1]);
}

void single_joint_delta_input(Eigen::VectorXd& target) 
{ // asks user to set perturbation delta for one joint and sets position in target vector
    std::cout << "which joint would you like to move?" << std::endl;
    int idx = (int)double_input(0, 7); 
    double deg;
    std::string s = "";
    std::stringstream ss;
    ss.clear();
    ss.str("");
    while (true) {
        std::cout << "please enter a perturbation delta for joint " << idx << " >> ";
        while (!std::getline(std::cin, s)) { continue; }
        ss.str(s);
        if (ss >> deg) {
            double r = target[idx-1] + degreesToRadians(deg);
            if (JOINT_RANGES[idx-1][0] <= r && r <= JOINT_RANGES[idx-1][1]) {
                std::cout << "\n";
                target[idx-1] = r;
                return; 
            }
        }
        ROS_WARN_STREAM("invalid input, perturbation delta puts target out of range");
        ss.clear();
        ss.str("");
        s.clear();
    }
}


#endif // MISC_UTILITIES_H

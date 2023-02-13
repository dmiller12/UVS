/* 
 * lpetrich 01/07/18
 */

#ifndef UTILITIES_H
#define UTILITIES_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/QR>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

Eigen::Vector3d spherical_to_cartesian(const Eigen::Vector3d &spherical_cooords)
{
    Eigen::Vector3d cartesian_coords;
    double r = spherical_cooords[0];
    double theta = spherical_cooords[1];
    double phi = spherical_cooords[2];
    double c_theta = std::cos(theta);
    double s_theta = std::sin(theta);
    double c_phi = std::cos(phi);
    double s_phi = std::sin(phi);
    cartesian_coords[0] = r * s_theta * c_phi;
    cartesian_coords[1] = r * s_theta * s_phi;
    cartesian_coords[2] = r * c_theta;
    return cartesian_coords;
}

Eigen::Vector3d cartesian_to_spherical(const Eigen::Vector3d &cartesian_coords)
{
    Eigen::Vector3d spherical_coords;
    double x = cartesian_coords[0];
    double y = cartesian_coords[1];
    double z = cartesian_coords[2];
    spherical_coords[0] = std::sqrt(x * x + y * y + z * z);
    spherical_coords[1] = std::atan2(std::sqrt(x * x + y * y), z);
    spherical_coords[2] = std::atan2(y, x);
    return spherical_coords;
}

Eigen::Quaterniond toQuaternion(const Eigen::Vector3d &RPY)
{
    Eigen::Quaterniond q;
    double roll = RPY[0];
    double pitch = RPY[1];
    double yaw = RPY[2];
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    q.w() = cy * cr * cp + sy * sr * sp;
    q.x() = cy * sr * cp - sy * cr * sp;
    q.y() = cy * cr * sp + sy * sr * cp;
    q.z() = sy * cr * cp - cy * sr * sp;
    return q;
}

Eigen::Quaterniond inwards_normal_to_quaternion(const Eigen::Vector3d &spherical_coords)
{ // returns the quaternion that gives the orientation of the inward normal vector of a sphere, at a given phi, theta in spherical coords.
    Eigen::Quaterniond quaternion;
    Eigen::Quaterniond quaternion_2;
    Eigen::Vector3d rpy;
    Eigen::Vector3d u, v, x, y;
    Eigen::Matrix3d SO_3;
    double theta = spherical_coords[1];
    double phi = spherical_coords[2];
    double c_phi = std::cos(phi);
    double c_theta = std::cos(theta);
    double s_phi = std::sin(phi);
    double s_theta = std::sin(theta);

    Eigen::Vector3d vertical(0.0, 0.0, 1.0);
    Eigen::Vector3d inwards_normal((-c_phi * s_theta), (-s_phi * s_theta), (-c_theta));
    x << 1, 0, 0;
    y << 0, 1, 0;
    u = inwards_normal.cross(-y);
    if (u.norm() < 1e-10)
    {
        u << -1, 0, 0;
    }
    v = u.cross(-inwards_normal);
    for (int i = 0; i < 3; ++i)
    {
        SO_3(i, 0) = u[i];
        SO_3(i, 1) = v[i];
        SO_3(i, 2) = inwards_normal[i];
    }
    quaternion = Eigen::Quaterniond(SO_3);
    // if (inwards_normal.dot(vertical) < -0.95)
    // {
    //     quaternion.x() = 0.0;
    //     quaternion.y() = -1.0;
    //     quaternion.z() = 0.0;
    //     quaternion.w() = 0.0;
    // }
    // else
    // {
    //     std::cout << "Inwards normal: \n*****\n" << inwards_normal << "\n*******" << std::endl;
    //     quaternion.setFromTwoVectors(inwards_normal, vertical);
    //     quaternion_2 = toQuaternion(rpy);
    //     std::cout << "Quaternions: old --> " << quaternion.w() << quaternion.x() <<
    //     quaternion.y() << quaternion.z() << std::endl;
    //     std::cout << "Quaternions: new --> " << quaternion_2.w() << quaternion_2.x() <<
    //     quaternion_2.y() << quaternion_2.z() << std::endl;
    // }
    return quaternion;
}

geometry_msgs::Pose get_pose(const Eigen::Vector3d &object_position,const Eigen::Vector3d &tool_position)
{
    geometry_msgs::Pose pose_msg;
    Eigen::Vector3d rel_position = tool_position - object_position;
    Eigen::Vector3d local_spherical_position = cartesian_to_spherical(rel_position);
    Eigen::Quaterniond quaternion = inwards_normal_to_quaternion(local_spherical_position);
    pose_msg.position.x = tool_position[0];
    pose_msg.position.y = tool_position[1];
    pose_msg.position.z = tool_position[2];
    pose_msg.orientation.x = quaternion.x();
    pose_msg.orientation.y = quaternion.y();
    pose_msg.orientation.z = quaternion.z();
    pose_msg.orientation.w = quaternion.w();
    return pose_msg;
}

Eigen::VectorXd concatenate_vectorxd(const Eigen::VectorXd &v1, const Eigen::VectorXd &v2)
{
    Eigen::VectorXd v(v1.rows() + v2.rows());
    v << v1, v2;
    return v;
}

std::vector<Eigen::Vector2d> slice(const std::vector<Eigen::Vector2d> &v, int start, int end)
{
    // auto first = v.cbegin() + start;
    // auto last = v.cbegin() + end + 1;

    std::vector<Eigen::Vector2d> subv(v.begin() + start, v.begin() + end + 1);
    return subv;
}

template <typename _Matrix_Type_>
bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result)
{
    result = a.completeOrthogonalDecomposition().pseudoInverse();
    double cond = result.norm() * a.norm();
    std::cout << "Condition number: " << cond << std::endl;

    return 1; // need additional checks?
}

Eigen::Vector3d toEulerAngle(const Eigen::VectorXd &q)
{
    Eigen::Vector3d euler;
    // Eigen::VectorXd q(4);
    // q[0] = quat[3];
    // q[1] = quat[1];
    // q[2] = quat[2];
    // q[3] = quat[0];

    double roll;
    double pitch;
    double yaw;
    // roll (x-axis rotation)
    double sinr = +2.0 * (q[3] * q[0] + q[1] * q[2]);
    double cosr = +1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q[3] * q[1] - q[2] * q[0]);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q[3] * q[2] + q[0] * q[1]);
    double cosy = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    yaw = atan2(siny, cosy);

    euler << roll, pitch, yaw;
    return euler;
}

Eigen::VectorXd state_to_vector(sensor_msgs::JointState js, int dof)
{
    Eigen::VectorXd joints(dof);
    for (int i = 0; i < dof; ++i)
    {
        joints[i] = js.position[i];
    }
    return joints;
}

std::string int_to_string(int i)
{
    std::string data;
    std::stringstream ss;
    ss << i << "\n";
    return data = ss.str();
}

std::string double_to_string(double d)
{
    std::string data;
    std::stringstream ss;
    ss << d << "\n";
    return data = ss.str();
}

std::string vector_to_string(Eigen::VectorXd v)
{
    std::string data;
    std::stringstream ss;
    for (int i = 0; i < v.size(); ++i)
    {
        ss << v[i] << " ";
    }
    ss << "\n";
    return data = ss.str();
}

std::string matrix_to_string(Eigen::MatrixXd m)
{
    std::string data;
    std::stringstream ss;
    for (int i = 0; i < m.rows(); ++i)
    {
        for (int j = 0; j < m.cols(); ++j)
        {
            ss << m(i, j) << " ";
        }
    }
    ss << "\n";
    return data = ss.str();
}

void print_joint_position(sensor_msgs::JointState joints, std::string msg, int dof)
{
    std::cout << msg << std::endl;
    for (int i = 0; i < dof; ++i)
    {
        std::cout << "Joint " << i + 1 << " position: " << joints.position[i] << std::endl;
    }
}

// Eigen::VectorXd get_xy_error(ArmControl *arm)
// {
//     // return error between current and target xy position
//     geometry_msgs::PoseStamped p = arm->get_pose();
//     double x = p.pose.position.x;
//     double y = p.pose.position.y;
//     double z = p.pose.position.z;
//     Eigen::VectorXd xy(2);
//     xy << x / z, y / z;
//     std::cout << "xy: (" << xy[0] << ", " << xy[1] << ")" << std::endl;
//     return xy;
// }

// Eigen::VectorXd get_xyz_error(ArmControl *arm)
// {
//     // return error between current and target xyz position
//     geometry_msgs::PoseStamped p = arm->get_pose();
//     Eigen::VectorXd xyz(3);
//     xyz << p.pose.position.x, p.pose.position.y, p.pose.position.z;
//     std::cout << "xyz: (" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
//     return xyz;
// }

// Eigen::VectorXd get_pose_error(ArmControl *arm)
// {
//     // return error between current and target pose
//     geometry_msgs::PoseStamped p = arm->get_pose();
//     Eigen::VectorXd fp(7);
//     fp << p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w;
//     // std::cout << "xyz: (" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
//     return fp;
// }

// Eigen::VectorXd get_image_error(ArmControl *arm)
// {
//     // return error between current and target pose
//     geometry_msgs::PoseStamped p = arm->get_pose();
//     Eigen::VectorXd fp(7);
//     fp << p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w;
//     // std::cout << "xyz: (" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
//     return fp;
// }

Eigen::VectorXd vector_target(Eigen::VectorXd current_state, size_t idx, double delta)
{
    Eigen::VectorXd target_state(current_state.size());
    for (size_t i = 0; i < (size_t)current_state.size(); ++i)
    {
        if (i != idx)
        {
            target_state[i] = current_state[i];
        }
        else
        {
            target_state[i] = (current_state[i] + delta);
        }
    }
    return target_state;
}


#endif // UTILITIES_H
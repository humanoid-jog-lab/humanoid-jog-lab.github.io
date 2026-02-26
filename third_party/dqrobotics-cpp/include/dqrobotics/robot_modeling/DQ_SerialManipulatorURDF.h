#pragma once
/**
(C) Copyright 2011-2025 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

URDF support: serial manipulator built from URDF-style joint chain
(origin + axis + type per joint). Chain can be built from URDF parsing
or passed from external code (e.g. JavaScript).
*/

#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <dqrobotics/robot_modeling/DQ_JointType.h>
#include <dqrobotics/DQ.h>
#include <Eigen/Dense>
#include <vector>

namespace DQ_robotics
{

/**
 * @brief One joint in a URDF-style chain: fixed transform (origin) from parent to joint frame,
 *        joint axis (unit vector in that frame), and joint type (revolute/prismatic).
 */
struct URDFJointData
{
    DQ_JointType joint_type;
    Eigen::Vector3d axis;   ///< unit vector, joint axis in joint frame
    DQ origin;             ///< rigid transform from parent link to child link frame (when q=0)
};

/**
 * @brief Serial manipulator described by a URDF-style joint chain.
 *        Each joint has an origin (fixed transform) and axis + type (revolute/prismatic).
 *        No XML parsing; chain is built externally (e.g. from URDF parser in JS or C++).
 */
class DQ_SerialManipulatorURDF : public DQ_SerialManipulator
{
protected:
    std::vector<URDFJointData> chain_;
    DQ effector_frame_;  ///< Fixed transform from last joint to effector (for trailing fixed joints)

    DQ _joint_to_dq(const double& q, const int& ith) const;
    DQ _get_w(const int& ith) const;

public:
    DQ_SerialManipulatorURDF() = delete;

    /**
     * @brief Build manipulator from a URDF-style joint chain.
     * @param chain Vector of URDFJointData: for each joint, origin (parent->child when q=0),
     *              axis (unit vector), and joint type (REVOLUTE or PRISMATIC).
     * @param effector_frame Fixed transform from last joint frame to effector frame (default: identity)
     */
    explicit DQ_SerialManipulatorURDF(const std::vector<URDFJointData>& chain, const DQ& effector_frame = DQ(1));

    /**
     * @brief Set the effector frame (fixed transform from last joint to effector)
     */
    void set_effector_frame(const DQ& effector_frame);

    /**
     * @brief Get the effector frame
     */
    DQ get_effector_frame() const;

    std::vector<DQ_JointType> get_supported_joint_types() const override;

    using DQ_SerialManipulator::raw_pose_jacobian;
    using DQ_SerialManipulator::raw_pose_jacobian_derivative;
    using DQ_SerialManipulator::raw_fkm;

    MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    MatrixXd raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const override;
    DQ raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const override;
};

}

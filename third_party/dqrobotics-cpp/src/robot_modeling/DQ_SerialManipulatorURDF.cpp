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
*/

#include <dqrobotics/robot_modeling/DQ_SerialManipulatorURDF.h>
#include <cmath>
#include <stdexcept>

namespace DQ_robotics
{

namespace
{
constexpr int JOINT_ROTATIONAL = 0;
constexpr int JOINT_PRISMATIC = 1;
}

DQ_SerialManipulatorURDF::DQ_SerialManipulatorURDF(const std::vector<URDFJointData>& chain, const DQ& effector_frame)
    : DQ_SerialManipulator(static_cast<int>(chain.size()))
    , chain_(chain)
    , effector_frame_(effector_frame.normalize())
{
    if (chain_.empty())
        throw std::range_error("DQ_SerialManipulatorURDF: chain cannot be empty");
    VectorXd types(chain_.size());
    for (size_t i = 0; i < chain_.size(); ++i)
        types(static_cast<Eigen::Index>(i)) = static_cast<int>(chain_[i].joint_type);
    set_joint_types(types);
}

void DQ_SerialManipulatorURDF::set_effector_frame(const DQ& effector_frame)
{
    effector_frame_ = effector_frame.normalize();
}

DQ DQ_SerialManipulatorURDF::get_effector_frame() const
{
    return effector_frame_;
}

DQ DQ_SerialManipulatorURDF::_joint_to_dq(const double& q, const int& ith) const
{
    const URDFJointData& j = chain_[static_cast<size_t>(ith)];
    const double ax = j.axis(0), ay = j.axis(1), az = j.axis(2);

    DQ motion;
    if (static_cast<int>(j.joint_type) == JOINT_ROTATIONAL)
    {
        double c = std::cos(q / 2.0);
        double s = std::sin(q / 2.0);
        motion = DQ(c, s * ax, s * ay, s * az, 0, 0, 0, 0);
    }
    else
    {
        motion = DQ(1, 0, 0, 0, 0, 0.5 * ax * q, 0.5 * ay * q, 0.5 * az * q);
    }
    return j.origin * motion;
}

DQ DQ_SerialManipulatorURDF::_get_w(const int& ith) const
{
    const URDFJointData& j = chain_[static_cast<size_t>(ith)];
    const double ax = j.axis(0), ay = j.axis(1), az = j.axis(2);
    DQ w_local;
    if (static_cast<int>(j.joint_type) == JOINT_ROTATIONAL)
        w_local = DQ(0, ax, ay, az, 0, 0, 0, 0);
    else
        w_local = DQ(0, 0, 0, 0, 0, ax, ay, az);
    return Ad(j.origin, w_local);
}

std::vector<DQ_JointType> DQ_SerialManipulatorURDF::get_supported_joint_types() const
{
    return { DQ_JointType::REVOLUTE, DQ_JointType::PRISMATIC };
}

DQ DQ_SerialManipulatorURDF::raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);
    DQ x(1);
    for (int i = 0; i <= to_ith_link; ++i)
        x = x * _joint_to_dq(q_vec(i), i);
    // Apply effector frame only when computing to the last link
    if (to_ith_link == static_cast<int>(chain_.size()) - 1)
        x = x * effector_frame_;
    return x;
}

MatrixXd DQ_SerialManipulatorURDF::raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);
    MatrixXd J = MatrixXd::Zero(8, to_ith_link + 1);
    DQ x_effector = raw_fkm(q_vec, to_ith_link);
    DQ x(1);
    for (int i = 0; i <= to_ith_link; ++i)
    {
        DQ w = _get_w(i);
        DQ z = 0.5 * Ad(x, w);
        x = x * _joint_to_dq(q_vec(i), i);
        J.col(i) = vec8(z * x_effector);
    }
    return J;
}

MatrixXd DQ_SerialManipulatorURDF::raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const
{
    _check_q_vec(q);
    _check_q_vec(q_dot);
    _check_to_ith_link(to_ith_link);
    int n = to_ith_link + 1;
    DQ x_effector = raw_fkm(q, to_ith_link);
    MatrixXd J = raw_pose_jacobian(q, to_ith_link);
    VectorXd vec_x_effector_dot = J * q_dot.head(n);
    DQ x(1);
    MatrixXd J_dot = MatrixXd::Zero(8, n);
    for (int i = 0; i < n; ++i)
    {
        DQ w = _get_w(i);
        DQ z = 0.5 * Ad(x, w);
        VectorXd vec_zdot;
        if (i == 0)
            vec_zdot = VectorXd::Zero(8);
        else
            vec_zdot = 0.5 * (haminus8(w * conj(x)) + hamiplus8(x * w) * C8()) * raw_pose_jacobian(q, i - 1) * q_dot.head(i);
        J_dot.col(i) = haminus8(x_effector) * vec_zdot + hamiplus8(z) * vec_x_effector_dot;
        x = x * _joint_to_dq(q(i), i);
    }
    return J_dot;
}

}

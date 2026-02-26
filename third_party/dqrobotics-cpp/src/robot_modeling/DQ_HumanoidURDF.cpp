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

#include <dqrobotics/robot_modeling/DQ_HumanoidURDF.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/DQ.h>
#include <stdexcept>
#include <cmath>

namespace DQ_robotics
{

DQ_HumanoidURDF::DQ_HumanoidURDF(
    const DQ& base_frame,
    const std::vector<URDFJointData>& waist_chain,
    const std::vector<URDFJointData>& left_arm_chain,
    const std::vector<URDFJointData>& right_arm_chain,
    const std::vector<URDFJointData>& head_chain,
    const DQ& waist_effector,
    const DQ& left_effector,
    const DQ& right_effector,
    const DQ& head_effector
)
    : DQ_Kinematics()
    , waist_(waist_chain, waist_effector)
    , left_arm_(left_arm_chain, left_effector)
    , right_arm_(right_arm_chain, right_effector)
    , head_(head_chain, head_effector)
{
    set_reference_frame(base_frame);
    n_waist_  = waist_.get_dim_configuration_space();
    n_left_   = left_arm_.get_dim_configuration_space();
    n_right_  = right_arm_.get_dim_configuration_space();
    n_head_   = head_.get_dim_configuration_space();
    dim_configuration_space_ = n_waist_ + n_left_ + n_right_ + n_head_;
}

void DQ_HumanoidURDF::_check_q_vec(const VectorXd& q) const
{
    if (q.size() != dim_configuration_space_)
        throw std::runtime_error(
            "DQ_HumanoidURDF: q size " + std::to_string(static_cast<int>(q.size())) +
            " != dim " + std::to_string(dim_configuration_space_));
}

void DQ_HumanoidURDF::_check_effector_index(int effector_index) const
{
    if (effector_index < 0 || effector_index > 3)
        throw std::runtime_error("DQ_HumanoidURDF: effector_index must be 0..3");
}

void DQ_HumanoidURDF::_q_segments(const VectorXd& q,
                                   VectorXd& q_waist, VectorXd& q_left,
                                   VectorXd& q_right, VectorXd& q_head) const
{
    int i = 0;
    q_waist = q.segment(i, n_waist_);  i += n_waist_;
    q_left  = q.segment(i, n_left_);   i += n_left_;
    q_right = q.segment(i, n_right_);  i += n_right_;
    q_head  = q.segment(i, n_head_);
}

int DQ_HumanoidURDF::get_dim_configuration_space() const
{
    return dim_configuration_space_;
}

DQ DQ_HumanoidURDF::raw_fkm_effector(const VectorXd& q, int effector_index) const
{
    _check_q_vec(q);
    _check_effector_index(effector_index);

    VectorXd q_waist, q_left, q_right, q_head;
    _q_segments(q, q_waist, q_left, q_right, q_head);

    DQ T_waist = waist_.fkm(q_waist);

    switch (effector_index)
    {
    case EffectorWaist:
        return T_waist;
    case EffectorLeftArm:
        return T_waist * left_arm_.fkm(q_left);
    case EffectorRightArm:
        return T_waist * right_arm_.fkm(q_right);
    case EffectorHead:
        return T_waist * head_.fkm(q_head);
    default:
        return T_waist * head_.fkm(q_head);
    }
}

DQ DQ_HumanoidURDF::fkm(const VectorXd& q) const
{
    return fkm(q, EffectorHead);
}

DQ DQ_HumanoidURDF::fkm(const VectorXd& q, const int& to_ith_link) const
{
    _check_q_vec(q);
    if (to_ith_link < 0 || to_ith_link > 3)
        throw std::runtime_error("DQ_HumanoidURDF::fkm to_ith_link must be 0..3 (effector index)");
    return raw_fkm_effector(q, to_ith_link);
}

MatrixXd DQ_HumanoidURDF::raw_pose_jacobian_effector(const VectorXd& q, int effector_index) const
{
    _check_q_vec(q);
    _check_effector_index(effector_index);

    VectorXd q_waist, q_left, q_right, q_head;
    _q_segments(q, q_waist, q_left, q_right, q_head);

    const int n = dim_configuration_space_;
    MatrixXd J = MatrixXd::Zero(8, n);

    DQ T_waist = waist_.fkm(q_waist);

    MatrixXd J_waist  = waist_.pose_jacobian(q_waist);
    MatrixXd J_left   = left_arm_.pose_jacobian(q_left);
    MatrixXd J_right  = right_arm_.pose_jacobian(q_right);
    MatrixXd J_head   = head_.pose_jacobian(q_head);

    int col = 0;

    switch (effector_index)
    {
    case EffectorWaist:
        J.block(0, col, 8, n_waist_) = J_waist;
        return J;

    case EffectorLeftArm:
    {
        DQ T_left = left_arm_.fkm(q_left);
        J.block(0, col, 8, n_waist_) = haminus8(T_left) * J_waist;
        col += n_waist_;
        J.block(0, col, 8, n_left_) = hamiplus8(T_waist) * J_left;
        return J;
    }
    case EffectorRightArm:
    {
        DQ T_right = right_arm_.fkm(q_right);
        J.block(0, col, 8, n_waist_) = haminus8(T_right) * J_waist;
        col += n_waist_;
        // 跳过左臂关节列（右臂关节在左臂之后）
        col += n_left_;
        J.block(0, col, 8, n_right_) = hamiplus8(T_waist) * J_right;
        return J;
    }
    case EffectorHead:
    {
        DQ T_head = head_.fkm(q_head);
        J.block(0, col, 8, n_waist_) = haminus8(T_head) * J_waist;
        col += n_waist_;
        J.block(0, col, 8, n_left_) = MatrixXd::Zero(8, n_left_);
        col += n_left_;
        J.block(0, col, 8, n_right_) = MatrixXd::Zero(8, n_right_);
        col += n_right_;
        J.block(0, col, 8, n_head_) = hamiplus8(T_waist) * J_head;
        return J;
    }
    default:
        return J;
    }
}

MatrixXd DQ_HumanoidURDF::pose_jacobian(const VectorXd& q, const int& to_ith_link) const
{
    if (to_ith_link < 0 || to_ith_link > 3)
        throw std::runtime_error("DQ_HumanoidURDF::pose_jacobian to_ith_link must be 0..3");
    return hamiplus8(get_reference_frame()) * raw_pose_jacobian_effector(q, to_ith_link);
}

MatrixXd DQ_HumanoidURDF::pose_jacobian(const VectorXd& q) const
{
    return pose_jacobian(q, EffectorHead);
}

MatrixXd DQ_HumanoidURDF::pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const
{
    (void)q;
    (void)q_dot;
    (void)to_ith_link;
    throw std::runtime_error("DQ_HumanoidURDF::pose_jacobian_derivative not implemented");
}

/**
 * 确保两个 DQ 的主部四元数在同一半球（避免 q 和 -q 双重覆盖问题）。
 * 如果点积 < 0，则翻转 x1 的符号。
 */
static DQ ensure_same_hemisphere(const DQ& x1, const DQ& x2)
{
    // 提取主部四元数 (前4个元素)
    const Vector4d r1 = x1.q.head<4>();
    const Vector4d r2 = x2.q.head<4>();
    // 点积 < 0 说明在相反半球
    if (r1.dot(r2) < 0.0)
        return -x1;
    return x1;
}

DQ DQ_HumanoidURDF::relativePoseDualArm(const VectorXd& q) const
{
    DQ x1 = raw_fkm_effector(q, EffectorLeftArm);
    const DQ x2 = raw_fkm_effector(q, EffectorRightArm);
    // 确保四元数在同一半球
    x1 = ensure_same_hemisphere(x1, x2);
    return conj(x2) * x1;
}

DQ DQ_HumanoidURDF::absolutePoseDualArm(const VectorXd& q) const
{
    DQ x1 = raw_fkm_effector(q, EffectorLeftArm);
    DQ x2 = raw_fkm_effector(q, EffectorRightArm);
    // 确保四元数在同一半球
    x1 = ensure_same_hemisphere(x1, x2);
    const DQ xr = conj(x2) * x1;
    return x2 * pow(xr, 0.5);
}

MatrixXd DQ_HumanoidURDF::relativePoseJacobianDualArm(const VectorXd& q) const
{
    MatrixXd Jx1 = raw_pose_jacobian_effector(q, EffectorLeftArm);
    const MatrixXd Jx2 = raw_pose_jacobian_effector(q, EffectorRightArm);
    DQ x1 = raw_fkm_effector(q, EffectorLeftArm);
    const DQ x2 = raw_fkm_effector(q, EffectorRightArm);
    // 确保四元数在同一半球
    const Vector4d r1 = x1.q.head<4>();
    const Vector4d r2 = x2.q.head<4>();
    if (r1.dot(r2) < 0.0) {
        x1 = -x1;
        Jx1 = -Jx1;
    }
    return hamiplus8(conj(x2)) * Jx1 + haminus8(x1) * C8() * Jx2;
}

MatrixXd DQ_HumanoidURDF::absolutePoseJacobianDualArm(const VectorXd& q) const
{
    const int n = dim_configuration_space_;
    MatrixXd Jx1 = raw_pose_jacobian_effector(q, EffectorLeftArm);
    const MatrixXd Jx2 = raw_pose_jacobian_effector(q, EffectorRightArm);
    DQ x1 = raw_fkm_effector(q, EffectorLeftArm);
    const DQ x2 = raw_fkm_effector(q, EffectorRightArm);
    // 确保四元数在同一半球
    const Vector4d r1 = x1.q.head<4>();
    const Vector4d r2 = x2.q.head<4>();
    if (r1.dot(r2) < 0.0) {
        x1 = -x1;
        Jx1 = -Jx1;
    }
    
    // 计算相对位姿及其 Jacobian
    const DQ xr = conj(x2) * x1;
    const MatrixXd Jxr = hamiplus8(conj(x2)) * Jx1 + haminus8(x1) * C8() * Jx2;
    const MatrixXd Jtr = DQ_Kinematics::translation_jacobian(Jxr, xr);

    const MatrixXd Jrr2 = 0.5 * haminus4(conj(xr.P()) * pow(xr.P(), 0.5)) * Jxr.block(0, 0, 4, n);
    MatrixXd Jxr2(8, n);
    Jxr2 << Jrr2, 0.25 * (haminus4(pow(xr.P(), 0.5)) * Jtr + hamiplus4(translation(xr)) * Jrr2);

    return haminus8(pow(xr, 0.5)) * Jx2 + hamiplus8(x2) * Jxr2;
}

}

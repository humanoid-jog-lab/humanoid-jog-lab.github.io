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

#pragma once

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorURDF.h>
#include <vector>

namespace DQ_robotics
{

/** Link names for loading humanoid chains from URDF (used by examples/loaders). */
struct HumanoidURDFLinkNames
{
    std::string base_link;
    std::string waist_start;
    std::string waist_end;
    std::string left_arm_start;
    std::string left_arm_end;
    std::string right_arm_start;
    std::string right_arm_end;
    std::string head_start;
    std::string head_end;
};

/**
 * Humanoid robot as tree: waist + left arm + right arm + head.
 * Effector indices: 0=waist, 1=left arm, 2=right arm, 3=head.
 * Dual-arm methods (relativePoseDualArm, absolutePoseDualArm, etc.) use
 * DQ_CooperativeDualTaskSpace formulas: relative = conj(x2)*x1, absolute = x2*pow(relative, 0.5).
 */
class DQ_HumanoidURDF : public DQ_Kinematics
{
public:
    enum { EffectorWaist = 0, EffectorLeftArm = 1, EffectorRightArm = 2, EffectorHead = 3 };

    DQ_HumanoidURDF() = delete;

    DQ_HumanoidURDF(const DQ& base_frame,
                    const std::vector<URDFJointData>& waist_chain,
                    const std::vector<URDFJointData>& left_arm_chain,
                    const std::vector<URDFJointData>& right_arm_chain,
                    const std::vector<URDFJointData>& head_chain,
                    const DQ& waist_effector = DQ(1),
                    const DQ& left_effector = DQ(1),
                    const DQ& right_effector = DQ(1),
                    const DQ& head_effector = DQ(1));

    DQ fkm(const VectorXd& q) const override;
    DQ fkm(const VectorXd& q, const int& to_ith_link) const override;
    MatrixXd pose_jacobian(const VectorXd& q, const int& to_ith_link) const override;
    MatrixXd pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const override;

    MatrixXd pose_jacobian(const VectorXd& q) const override;

    int get_dim_configuration_space() const override;

    /** Left arm relative to right arm: conj(x2)*x1 (same convention as DQ_CooperativeDualTaskSpace). */
    DQ relativePoseDualArm(const VectorXd& q) const;
    /** Geodesic midpoint pose: x2 * pow(relative, 0.5). */
    DQ absolutePoseDualArm(const VectorXd& q) const;
    MatrixXd relativePoseJacobianDualArm(const VectorXd& q) const;
    MatrixXd absolutePoseJacobianDualArm(const VectorXd& q) const;

protected:
    DQ_SerialManipulatorURDF waist_;
    DQ_SerialManipulatorURDF left_arm_;
    DQ_SerialManipulatorURDF right_arm_;
    DQ_SerialManipulatorURDF head_;
    int n_waist_;
    int n_left_;
    int n_right_;
    int n_head_;

    void _check_q_vec(const VectorXd& q) const;
    void _check_effector_index(int effector_index) const;
    void _q_segments(const VectorXd& q, VectorXd& q_waist, VectorXd& q_left, VectorXd& q_right, VectorXd& q_head) const;

    DQ raw_fkm_effector(const VectorXd& q, int effector_index) const;
    MatrixXd raw_pose_jacobian_effector(const VectorXd& q, int effector_index) const;
};

}  // namespace DQ_robotics

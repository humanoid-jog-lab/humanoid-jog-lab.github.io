// Whole-body velocity control example:
// Differential-drive mobile base (wheel velocities) + humanoid URDF joints.
//
// Usage:
//   sailor_r1_pro_wholebody_diffdrive [urdf_path]
//
// Default urdf_path (when run from build/):
//   ../../public/robots/sailor_r1_pro_description/urdf/sailor_r1_pro_description.urdf

#include "urdf_humanoid_chain_loader.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h>
#include <dqrobotics/robot_modeling/DQ_HumanoidURDF.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace DQ_robotics;
using namespace Eigen;

int main(int argc, char** argv)
{
    std::string urdf_path = (argc > 1)
                                ? argv[1]
                                : "../../public/robots/sailor_r1_pro_description/urdf/sailor_r1_pro_description.urdf";

    HumanoidURDFLinkNames names;
    names.base_link = "base_link";
    names.waist_start = "base_link";
    names.waist_end = "body_link";
    names.left_arm_start = "body_link";
    names.left_arm_end = "lee_link";
    names.right_arm_start = "body_link";
    names.right_arm_end = "ree_link";
    names.head_start = "body_link";
    names.head_end = "head_pitch_link";

    std::vector<URDFJointData> waist_chain, left_arm_chain, right_arm_chain, head_chain;
    DQ waist_effector(1), left_effector(1), right_effector(1), head_effector(1);
    std::string err_msg;
    if (!load_humanoid_chains_from_urdf(urdf_path,
                                        names,
                                        &waist_chain,
                                        &left_arm_chain,
                                        &right_arm_chain,
                                        &head_chain,
                                        &waist_effector,
                                        &left_effector,
                                        &right_effector,
                                        &head_effector,
                                        &err_msg))
    {
        std::cerr << "Failed to load URDF chains: " << err_msg << "\n";
        return 1;
    }

    DQ_HumanoidURDF humanoid(DQ(1),
                             waist_chain,
                             left_arm_chain,
                             right_arm_chain,
                             head_chain,
                             waist_effector,
                             left_effector,
                             right_effector,
                             head_effector);

    // Replace by your real parameters.
    const double wheel_radius = 0.08;
    const double wheel_distance = 0.45;
    DQ_DifferentialDriveRobot base(wheel_radius, wheel_distance);

    // If the wheel axle frame is not the same as URDF base_link, set a fixed displacement here.
    base.set_frame_displacement(DQ(1));

    VectorXd q_base(3);
    q_base << 0.0, 0.0, 0.0; // [x, y, yaw]

    VectorXd q_humanoid = VectorXd::Zero(humanoid.get_dim_configuration_space());

    // Control right arm end-effector in world frame.
    const int effector_index = DQ_HumanoidURDF::EffectorRightArm;
    const int n_h = static_cast<int>(q_humanoid.size());

    const double dt = 1e-3;
    const double gain = 80.0;
    const int max_steps = 20000;
    const double tol = 1e-2;

    DQ x_base = base.fkm(q_base);
    DQ x_h = humanoid.fkm(q_humanoid, effector_index);
    DQ x = x_base * x_h;

    // Desired pose: 20 cm x-shift + 10 cm y-shift from current pose.
    const DQ p = 0.2 * i_ + 0.1 * j_;
    const DQ xd = (DQ(1) + DQ::E * 0.5 * p) * x;

    std::cout << "Whole-body DOF: wheel velocities=2, humanoid joints=" << n_h
              << ", total controls=" << (n_h + 2) << "\n";

    for (int k = 0; k < max_steps; ++k)
    {
        x_base = base.fkm(q_base);
        x_h = humanoid.fkm(q_humanoid, effector_index);
        x = x_base * x_h;

        const MatrixXd J_base = base.pose_jacobian(q_base); // 8x2, wrt wheel velocities
        const MatrixXd J_h = humanoid.pose_jacobian(q_humanoid, effector_index); // 8xn_h

        MatrixXd J(8, n_h + 2);
        J.block(0, 0, 8, 2) = haminus8(x_h) * J_base;
        J.block(0, 2, 8, n_h) = hamiplus8(x_base) * J_h;

        const MatrixXd N = haminus8(xd) * C8() * J;
        const VectorXd e = vec8(conj(x) * xd - DQ(1));
        const VectorXd u = -pinv(N) * (gain * e);

        const Vector2d u_wheels = u.head<2>();
        const VectorXd qh_dot = u.tail(n_h);

        const Vector3d q_base_dot = base.constraint_jacobian(q_base(2)) * u_wheels;
        q_base += dt * q_base_dot;
        q_humanoid += dt * qh_dot;

        if (k % 200 == 0)
        {
            std::cout << "step=" << k << " error=" << e.norm() << "\n";
        }

        if (e.norm() < tol)
        {
            std::cout << "Converged at step " << k << ", error=" << e.norm() << "\n";
            break;
        }
    }

    std::cout << "Final base q = [" << q_base.transpose() << "]\n";
    std::cout << "Done.\n";
    return 0;
}

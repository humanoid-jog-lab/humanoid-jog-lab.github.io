#pragma once

#include <limits>
#include <string>
#include <vector>

namespace DQ_robotics {
class DQ_HumanoidURDF;
}

struct HumanoidAbsoluteNloptRequest {
    std::vector<double> q0;
    std::vector<double> absolute_target;
    std::vector<double> relative_reference;
    std::vector<int> active_indices;
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    std::vector<double> step_bounds;
    double wvfi_min = 5e-4;
    double weight_absolute = 1.0;
    double weight_delta = 1e-3;
    double weight_wvfi_reward = 1e-3;
    bool enforce_relative_pose_constraint = true;
    double relative_equality_tolerance = 1e-8;
    double weight_relative_penalty = 200.0;
    bool keep_body_z_vertical = false;
    double body_vertical_tolerance = 1e-8;
    double weight_body_vertical_penalty = 200.0;
    bool enforce_waist_x_corridor = false;
    double waist_x_min = -std::numeric_limits<double>::infinity();
    double waist_x_max = std::numeric_limits<double>::infinity();
    // Near-active joint limits attenuate w_vfi when the remaining margin drops below this threshold.
    double joint_limit_activation_threshold = 0.05;
    int repair_max_iter = 6;
    double repair_damping = 1e-4;
    int max_eval = 80;
    double xtol_rel = 1e-6;
    double ftol_rel = 1e-8;
};

struct HumanoidAbsoluteNloptResult {
    bool converged = false;
    std::string reason;
    int iterations = 0;
    double final_cost = std::numeric_limits<double>::infinity();
    double final_absolute_residual = std::numeric_limits<double>::infinity();
    double final_relative_residual = std::numeric_limits<double>::infinity();
    double final_body_vertical_residual = 0.0;
    int repair_iterations = 0;
    bool repair_converged = false;
    double final_wvfi = 0.0;
    std::vector<double> q_next;
};

HumanoidAbsoluteNloptResult solveHumanoidAbsoluteTargetNlopt(
    const DQ_robotics::DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteNloptRequest& request
);

#pragma once

#include <limits>
#include <string>
#include <vector>

namespace DQ_robotics {
class DQ_HumanoidURDF;
}

struct HumanoidMJRestoreReferenceAxis {
    double stiffness = 1.0;
    double damping = 0.0;
};

struct HumanoidMJRestoreWaistXCorridorRequest {
    bool enabled = false;
    double min = -std::numeric_limits<double>::infinity();
    double max = std::numeric_limits<double>::infinity();
    double tolerance = 0.0;
    std::vector<std::vector<double>> rows;
    std::vector<double> rhs;
};

struct HumanoidAbsoluteMJRestoreRequest {
    std::vector<double> q_current;
    int active_start = 0;
    int active_count = 0;
    std::vector<double> absolute_target;
    std::vector<double> relative_target;
    bool keep_body_z_vertical = false;
    double body_equality_tolerance = 0.0;
    double absolute_weight = 1.0;
    double relative_weight = 1.0;
    double body_weight = 1.0;
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    std::vector<double> step_bounds;
    double wvfi_lower_bound = 0.0;
    double joint_limit_activation_threshold = 0.05;
    HumanoidMJRestoreWaistXCorridorRequest waist_x_corridor;
    HumanoidMJRestoreReferenceAxis absolute_reference;
    HumanoidMJRestoreReferenceAxis relative_reference;
    HumanoidMJRestoreReferenceAxis body_reference;
    int max_iterations = 16;
    double damping = 1e-6;
    double initial_scale = 1.0;
    double min_scale = 1e-6;
    double reduction_factor = 0.5;
    double equality_tolerance = 1e-12;
    double inequality_tolerance = 1e-12;
};

struct HumanoidAbsoluteMJRestoreResult {
    bool converged = false;
    std::string reason;
    int iterations = 0;
    int accepted_iterations = 0;
    double last_accepted_scale = 0.0;
    double command_residual_norm = std::numeric_limits<double>::infinity();
    double relative_residual_norm = std::numeric_limits<double>::infinity();
    double body_residual_norm = 0.0;
    double inequality_violation = std::numeric_limits<double>::infinity();
    std::vector<double> reference_residual;
    std::vector<double> constraint_motion;
    std::vector<double> q_applied;
};

HumanoidAbsoluteMJRestoreResult solveHumanoidAbsoluteTargetMJRestore(
    const DQ_robotics::DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
);

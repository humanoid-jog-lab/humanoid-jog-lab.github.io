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

struct HumanoidMJRestoreJointLimitVFIRequest {
    bool enabled = false;
    std::vector<std::vector<double>> rows;
    std::vector<double> rhs;
};

struct HumanoidAbsoluteMJRestoreRequest {
    std::string solve_variant = "absolute_target";
    std::vector<double> q_current;
    int active_start = 0;
    int active_count = 0;
    std::vector<double> absolute_target;
    std::vector<double> relative_target;
    std::vector<double> left_target;
    std::vector<double> right_target;
    bool keep_body_z_vertical = false;
    double body_equality_tolerance = 0.0;
    double absolute_weight = 1.0;
    double relative_weight = 1.0;
    double body_weight = 1.0;
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    std::vector<double> step_bounds;
    std::vector<double> regularization_weights;
    double wvfi_lower_bound = 0.0;
    double joint_limit_activation_threshold = 0.05;
    HumanoidMJRestoreJointLimitVFIRequest joint_limit_vfi;
    HumanoidMJRestoreWaistXCorridorRequest waist_x_corridor;
    HumanoidMJRestoreReferenceAxis absolute_reference;
    HumanoidMJRestoreReferenceAxis relative_reference;
    HumanoidMJRestoreReferenceAxis body_reference;
    int max_iterations = 16;
    double damping = 1e-6;
    int max_damping_retries = 8;
    double damping_growth_factor = 10.0;
    double initial_scale = 1.0;
    double min_scale = 1e-6;
    double reduction_factor = 0.5;
    double initial_trust_radius = 0.2;
    double min_trust_radius = 1e-6;
    double max_trust_radius = 1.0;
    int max_trust_region_retries = 6;
    double trust_region_shrink_factor = 0.5;
    double trust_region_expand_factor = 1.75;
    double rho_acceptance_threshold = 0.05;
    double rho_shrink_threshold = 0.05;
    double rho_expand_threshold = 0.75;
    double equality_tolerance = 1e-12;
    double inequality_tolerance = 1e-12;
};

struct HumanoidAbsoluteMJRestoreResult {
    bool converged = false;
    std::string reason;
    int iterations = 0;
    int accepted_iterations = 0;
    double last_accepted_scale = 0.0;
    double last_accepted_damping = std::numeric_limits<double>::quiet_NaN();
    double command_residual_norm = std::numeric_limits<double>::infinity();
    double relative_residual_norm = std::numeric_limits<double>::infinity();
    double body_residual_norm = 0.0;
    double inequality_violation = std::numeric_limits<double>::infinity();
    double trust_radius = std::numeric_limits<double>::quiet_NaN();
    double last_accepted_rho = std::numeric_limits<double>::quiet_NaN();
    double last_predicted_reduction = std::numeric_limits<double>::quiet_NaN();
    double last_actual_reduction = std::numeric_limits<double>::quiet_NaN();
    std::string accepted_by;
    double last_rejected_rho = std::numeric_limits<double>::quiet_NaN();
    double last_rejected_predicted_reduction = std::numeric_limits<double>::quiet_NaN();
    double last_rejected_actual_reduction = std::numeric_limits<double>::quiet_NaN();
    std::string last_rejected_reason;
    double best_constraint_violation = std::numeric_limits<double>::infinity();
    int filter_size = 0;
    std::vector<double> reference_residual;
    std::vector<double> constraint_motion;
    std::vector<double> q_applied;
};

HumanoidAbsoluteMJRestoreResult solveHumanoidAbsoluteTargetMJRestore(
    const DQ_robotics::DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
);

HumanoidAbsoluteMJRestoreResult solveHumanoidAbsoluteTargetMJRestoreTrustRegion(
    const DQ_robotics::DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
);

HumanoidAbsoluteMJRestoreResult solveHumanoidAbsoluteDQSetpointMJRestore(
    const DQ_robotics::DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
);

HumanoidAbsoluteMJRestoreResult solveHumanoidAbsoluteDQSetpointMJRestoreTrustRegion(
    const DQ_robotics::DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
);

HumanoidAbsoluteMJRestoreResult solveHumanoidCooperativeDQSetpointMJRestore(
    const DQ_robotics::DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
);

HumanoidAbsoluteMJRestoreResult solveHumanoidCooperativeDQSetpointMJRestoreTrustRegion(
    const DQ_robotics::DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
);

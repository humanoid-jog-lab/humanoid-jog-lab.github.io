#include "mj_restore_humanoid_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_HumanoidURDF.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>

namespace {

using DQ_robotics::DQ;
using DQ_robotics::DQ_HumanoidURDF;
using DQ_robotics::DQ_Kinematics;
using DQ_robotics::conj;
using DQ_robotics::hamiplus8;
using DQ_robotics::rotation;
using DQ_robotics::translation;
using Eigen::MatrixXd;
using Eigen::VectorXd;

constexpr double kBoundTolerance = 1e-12;

bool isFiniteValue(double value) {
    return std::isfinite(value);
}

bool isFiniteVector(const std::vector<double>& values) {
    for (double value : values) {
        if (!isFiniteValue(value)) return false;
    }
    return true;
}

bool isFiniteRows(const std::vector<std::vector<double>>& rows) {
    for (const auto& row : rows) {
        if (!isFiniteVector(row)) return false;
    }
    return true;
}

VectorXd toEigenVector(const std::vector<double>& values) {
    VectorXd out(static_cast<Eigen::Index>(values.size()));
    for (Eigen::Index i = 0; i < out.size(); ++i) {
        out(i) = values[static_cast<std::size_t>(i)];
    }
    return out;
}

std::vector<double> toStdVector(const VectorXd& values) {
    std::vector<double> out(static_cast<std::size_t>(values.size()), 0.0);
    for (Eigen::Index i = 0; i < values.size(); ++i) {
        out[static_cast<std::size_t>(i)] = values(i);
    }
    return out;
}

std::vector<double> sliceActiveConfiguration(const HumanoidAbsoluteMJRestoreRequest& request) {
    std::vector<double> q_active(static_cast<std::size_t>(request.active_count), 0.0);
    for (int i = 0; i < request.active_count; ++i) {
        q_active[static_cast<std::size_t>(i)] =
            request.q_current[static_cast<std::size_t>(request.active_start + i)];
    }
    return q_active;
}

VectorXd buildFullConfiguration(
    const HumanoidAbsoluteMJRestoreRequest& request,
    const std::vector<double>& q_active
) {
    VectorXd q_full = toEigenVector(request.q_current);
    for (int i = 0; i < request.active_count; ++i) {
        q_full(request.active_start + i) = q_active[static_cast<std::size_t>(i)];
    }
    return q_full;
}

MatrixXd sliceActiveColumns(const MatrixXd& jacobian, int active_start, int active_count) {
    return jacobian.block(0, active_start, jacobian.rows(), active_count);
}

double signedAlignment(const VectorXd& target, const VectorXd& current) {
    return target.dot(current) < 0.0 ? -1.0 : 1.0;
}

VectorXd signedPoseError(const VectorXd& target, const VectorXd& current) {
    const double sign = signedAlignment(target, current);
    return target - sign * current;
}

VectorXd alignDualCoverTarget(const VectorXd& current, const VectorXd& target) {
    return signedAlignment(target, current) < 0.0 ? -target : target;
}

MatrixXd buildRightMultiplicationMatrix(const VectorXd& right_target) {
    MatrixXd out(8, 8);
    for (Eigen::Index col = 0; col < 8; ++col) {
        VectorXd basis = VectorXd::Zero(8);
        basis(col) = 1.0;
        const DQ basis_dq(basis);
        const DQ right_dq(right_target);
        out.col(col) = (basis_dq * right_dq).q;
    }
    return out;
}

struct DQSetpointErrorModel {
    VectorXd aligned_target;
    VectorXd error;
    MatrixXd jacobian;
};

DQSetpointErrorModel buildDQSetpointErrorModel(
    const DQ& current_pose,
    const MatrixXd& pose_jacobian_active,
    const VectorXd& raw_target
) {
    const VectorXd aligned_target = alignDualCoverTarget(current_pose.q, raw_target);
    const DQ target_dq(aligned_target);
    const DQ current_to_target = conj(current_pose) * target_dq;
    VectorXd error = VectorXd::Zero(8);
    error(0) = 1.0 - current_to_target.q(0);
    for (Eigen::Index i = 1; i < 8; ++i) {
        error(i) = -current_to_target.q(i);
    }

    MatrixXd d8 = MatrixXd::Identity(8, 8);
    d8(1, 1) = -1.0;
    d8(2, 2) = -1.0;
    d8(3, 3) = -1.0;
    d8(5, 5) = -1.0;
    d8(6, 6) = -1.0;
    d8(7, 7) = -1.0;

    const MatrixXd constrained_jacobian =
        -buildRightMultiplicationMatrix(aligned_target) * d8 * pose_jacobian_active;
    return { aligned_target, error, constrained_jacobian };
}

MatrixXd computeTwistJacobian(const MatrixXd& pose_jacobian, const DQ& pose) {
    const MatrixXd transformed = 2.0 * hamiplus8(conj(pose)) * pose_jacobian;
    MatrixXd twist_jacobian(6, pose_jacobian.cols());
    twist_jacobian.row(0) = transformed.row(1);
    twist_jacobian.row(1) = transformed.row(2);
    twist_jacobian.row(2) = transformed.row(3);
    twist_jacobian.row(3) = transformed.row(5);
    twist_jacobian.row(4) = transformed.row(6);
    twist_jacobian.row(5) = transformed.row(7);
    return twist_jacobian;
}

MatrixXd computeRotationMatrixFromPose(const DQ& pose) {
    const DQ rot = rotation(pose);
    const double w = rot.q(0);
    const double x = rot.q(1);
    const double y = rot.q(2);
    const double z = rot.q(3);

    MatrixXd mat(3, 3);
    mat(0, 0) = 1 - 2 * (y * y + z * z);
    mat(0, 1) = 2 * (x * y - w * z);
    mat(0, 2) = 2 * (x * z + w * y);
    mat(1, 0) = 2 * (x * y + w * z);
    mat(1, 1) = 1 - 2 * (x * x + z * z);
    mat(1, 2) = 2 * (y * z - w * x);
    mat(2, 0) = 2 * (x * z - w * y);
    mat(2, 1) = 2 * (y * z + w * x);
    mat(2, 2) = 1 - 2 * (x * x + y * y);
    return mat;
}

double nonNegativeDeterminant(const MatrixXd& matrix) {
    if (matrix.rows() == 0 || matrix.cols() == 0) return 0.0;
    const MatrixXd symmetric = 0.5 * (matrix + matrix.transpose());
    Eigen::SelfAdjointEigenSolver<MatrixXd> solver(symmetric);
    if (solver.info() != Eigen::Success) return 0.0;

    double determinant = 1.0;
    const auto eigenvalues = solver.eigenvalues();
    for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
        determinant *= std::max(0.0, eigenvalues(i));
        if (!std::isfinite(determinant)) return 0.0;
    }
    return determinant;
}

MatrixXd buildVFIAttenuationMatrix(
    const std::vector<double>& q_active,
    const std::vector<double>& lower_bounds,
    const std::vector<double>& upper_bounds,
    double activation_threshold
) {
    const Eigen::Index active_count = static_cast<Eigen::Index>(q_active.size());
    MatrixXd attenuation = MatrixXd::Identity(active_count, active_count);
    if (!(activation_threshold > 0.0)) return attenuation;

    for (Eigen::Index i = 0; i < active_count; ++i) {
        const double qi = q_active[static_cast<std::size_t>(i)];
        const double d_up = std::max(0.0, upper_bounds[static_cast<std::size_t>(i)] - qi);
        const double d_lo = std::max(0.0, qi - lower_bounds[static_cast<std::size_t>(i)]);
        const double d_min = std::min(d_up, d_lo);
        if (!std::isfinite(d_min) || d_min >= activation_threshold) continue;
        const double omega = d_min / activation_threshold;
        if (!(omega >= 0.0 && omega < 1.0)) continue;
        attenuation(i, i) -= (1.0 - omega);
    }

    return attenuation;
}

double computeWaistXPosition(const DQ_HumanoidURDF& robot, const VectorXd& q_full) {
    const DQ waist_pose = robot.fkm(q_full, 0);
    const DQ waist_translation = translation(waist_pose);
    return waist_translation.q(1);
}

double dotStd(const std::vector<double>& a, const std::vector<double>& b) {
    const std::size_t count = std::min(a.size(), b.size());
    double sum = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        sum += a[i] * b[i];
    }
    return sum;
}

double vectorNormStd(const std::vector<double>& values) {
    return std::sqrt(dotStd(values, values));
}

std::vector<double> normalizeRegularizationWeights(
    const std::vector<double>& regularization_weights,
    int width
) {
    if (width <= 0) return {};
    if (regularization_weights.empty()) return {};

    std::vector<double> weights(static_cast<std::size_t>(width), 1.0);
    bool has_custom_weight = false;
    for (int i = 0; i < width; ++i) {
        const double weight = std::max(
            1e-12,
            i < static_cast<int>(regularization_weights.size())
                ? regularization_weights[static_cast<std::size_t>(i)]
                : 1.0
        );
        weights[static_cast<std::size_t>(i)] = weight;
        if (std::abs(weight - 1.0) > 1e-12) has_custom_weight = true;
    }
    return has_custom_weight ? weights : std::vector<double>{};
}

double clipDamping(double residual, double damping_term) {
    if (std::abs(residual) <= 1e-18) return 0.0;
    if ((residual < 0.0) != (damping_term < 0.0)) return damping_term;
    const double max_magnitude = std::abs(residual) * 0.8;
    return std::copysign(std::min(std::abs(damping_term), max_magnitude), damping_term);
}

double computeRestorationMerit(double phi, double theta, double penalty) {
    return phi + penalty * theta;
}

struct TrustRegionStep {
    std::vector<double> dq;
    double step_norm = 0.0;
    bool clipped = false;
};

TrustRegionStep clipStepToTrustRegion(
    const std::vector<double>& step,
    double trust_radius
) {
    const double radius = std::max(1e-12, trust_radius);
    const double step_norm = vectorNormStd(step);
    if (!std::isfinite(radius) || step_norm <= radius) {
        return { step, step_norm, false };
    }
    const double scale = radius / std::max(step_norm, 1e-18);
    std::vector<double> clipped(step.size(), 0.0);
    for (std::size_t i = 0; i < step.size(); ++i) clipped[i] = step[i] * scale;
    return { clipped, radius, true };
}

struct FilterEntry {
    double phi = std::numeric_limits<double>::infinity();
    double theta = std::numeric_limits<double>::infinity();
};

bool dominatesFilterEntry(
    const FilterEntry& lhs,
    const FilterEntry& rhs,
    double phi_tolerance,
    double theta_tolerance
) {
    const bool phi_better = lhs.phi <= rhs.phi + phi_tolerance;
    const bool theta_better = lhs.theta <= rhs.theta + theta_tolerance;
    const bool strictly_better =
        lhs.phi < rhs.phi - phi_tolerance || lhs.theta < rhs.theta - theta_tolerance;
    return phi_better && theta_better && strictly_better;
}

bool isDominatedByFilter(
    const std::vector<FilterEntry>& filter,
    const FilterEntry& candidate,
    double phi_tolerance,
    double theta_tolerance
) {
    for (const auto& entry : filter) {
        if (dominatesFilterEntry(entry, candidate, phi_tolerance, theta_tolerance)) {
            return true;
        }
    }
    return false;
}

std::vector<FilterEntry> insertFilterEntry(
    const std::vector<FilterEntry>& filter,
    const FilterEntry& candidate,
    double phi_tolerance,
    double theta_tolerance
) {
    std::vector<FilterEntry> next;
    next.reserve(filter.size() + 1);
    for (const auto& entry : filter) {
        if (!dominatesFilterEntry(candidate, entry, phi_tolerance, theta_tolerance)) {
            next.push_back(entry);
        }
    }
    next.push_back(candidate);
    return next;
}

struct FilterAcceptance {
    bool accepted = false;
    std::string reason;
    std::vector<FilterEntry> next_filter;
};

FilterAcceptance acceptRestorationCandidateByFilter(
    const std::vector<FilterEntry>& filter,
    double current_phi,
    double current_theta,
    double candidate_phi,
    double candidate_theta,
    double predicted_reduction,
    double actual_reduction,
    double rho,
    double rho_acceptance_threshold,
    double phi_tolerance,
    double theta_tolerance
) {
    const FilterEntry candidate{ candidate_phi, candidate_theta };
    if (isDominatedByFilter(filter, candidate, phi_tolerance, theta_tolerance)) {
        return { false, "dominated", filter };
    }

    const bool phi_improved = candidate_phi + phi_tolerance < current_phi;
    const bool theta_improved = candidate_theta + theta_tolerance < current_theta;
    const bool rho_accepted = predicted_reduction <= 1e-12 || rho >= rho_acceptance_threshold;

    if (!phi_improved && !theta_improved) {
        return { false, "not_improved", filter };
    }
    if (!rho_accepted && !theta_improved) {
        return { false, "poor_ratio", filter };
    }
    return {
        true,
        (theta_improved && !rho_accepted) || (theta_improved && !phi_improved) ? "filter" : "rho",
        insertFilterEntry(filter, candidate, phi_tolerance, theta_tolerance)
    };
}

double updateTrustRegionRadius(
    double radius,
    double rho,
    double step_norm,
    bool clipped,
    double min_trust_radius,
    double max_trust_radius,
    double shrink_factor,
    double expand_factor,
    double rho_shrink_threshold,
    double rho_expand_threshold
) {
    double next_radius = radius;
    if (!std::isfinite(rho) || rho < rho_shrink_threshold) {
        next_radius = std::max(min_trust_radius, radius * shrink_factor);
    } else if (clipped && rho > rho_expand_threshold && step_norm >= radius * 0.99) {
        next_radius = std::min(max_trust_radius, radius * expand_factor);
    }
    return next_radius;
}

struct ActiveLinearRows {
    MatrixXd rows;
    VectorXd rhs;
};

ActiveLinearRows buildLinearStepRows(
    const HumanoidAbsoluteMJRestoreRequest& request
) {
    std::vector<std::vector<double>> combined_rows;
    std::vector<double> combined_rhs;

    if (request.joint_limit_vfi.enabled) {
        const std::size_t row_count = std::min(
            request.joint_limit_vfi.rows.size(),
            request.joint_limit_vfi.rhs.size()
        );
        for (std::size_t i = 0; i < row_count; ++i) {
            combined_rows.push_back(request.joint_limit_vfi.rows[i]);
            combined_rhs.push_back(request.joint_limit_vfi.rhs[i]);
        }
    }

    if (request.waist_x_corridor.enabled) {
        const std::size_t row_count = std::min(
            request.waist_x_corridor.rows.size(),
            request.waist_x_corridor.rhs.size()
        );
        for (std::size_t i = 0; i < row_count; ++i) {
            combined_rows.push_back(request.waist_x_corridor.rows[i]);
            combined_rhs.push_back(request.waist_x_corridor.rhs[i]);
        }
    }

    if (combined_rows.empty()) {
        return { MatrixXd(0, request.active_count), VectorXd(0) };
    }

    MatrixXd rows(static_cast<Eigen::Index>(combined_rows.size()), request.active_count);
    VectorXd rhs(static_cast<Eigen::Index>(combined_rows.size()));
    for (std::size_t i = 0; i < combined_rows.size(); ++i) {
        for (int j = 0; j < request.active_count; ++j) {
            rows(static_cast<Eigen::Index>(i), j) = combined_rows[i][static_cast<std::size_t>(j)];
        }
        rhs(static_cast<Eigen::Index>(i)) = combined_rhs[i];
    }
    return { rows, rhs };
}

ActiveLinearRows selectViolatedLinearRows(
    const ActiveLinearRows& inequalities,
    const VectorXd& step
) {
    std::vector<Eigen::Index> violated;
    for (Eigen::Index i = 0; i < inequalities.rows.rows(); ++i) {
        if (inequalities.rows.row(i).dot(step) > inequalities.rhs(i) + 1e-12) {
            violated.push_back(i);
        }
    }
    if (violated.empty()) {
        return { MatrixXd(0, inequalities.rows.cols()), VectorXd(0) };
    }
    MatrixXd rows(static_cast<Eigen::Index>(violated.size()), inequalities.rows.cols());
    VectorXd rhs(static_cast<Eigen::Index>(violated.size()));
    for (std::size_t i = 0; i < violated.size(); ++i) {
        rows.row(static_cast<Eigen::Index>(i)) = inequalities.rows.row(violated[i]);
        rhs(static_cast<Eigen::Index>(i)) = inequalities.rhs(violated[i]);
    }
    return { rows, rhs };
}

HumanoidAbsoluteMJRestoreResult makeFailure(
    const HumanoidAbsoluteMJRestoreRequest& request,
    const std::string& reason
) {
    HumanoidAbsoluteMJRestoreResult result;
    result.reason = reason;
    result.q_applied = sliceActiveConfiguration(request);
    return result;
}

bool validateRequest(
    const DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request,
    std::string& reason
) {
    const std::size_t dof = request.q_current.size();
    if (dof == 0) {
        reason = "qCurrent must not be empty";
        return false;
    }
    if (static_cast<std::size_t>(robot.get_dim_configuration_space()) != dof) {
        reason = "qCurrent size must match the humanoid robot configuration dimension";
        return false;
    }
    if (request.active_start < 0 || request.active_count <= 0 ||
        static_cast<std::size_t>(request.active_start + request.active_count) > dof) {
        reason = "activeStart/activeCount must define a valid active slice";
        return false;
    }
    const bool cooperative_dq_setpoint = request.solve_variant == "cooperative_dq_setpoint";
    if (cooperative_dq_setpoint) {
        if (request.left_target.size() != 8 || request.right_target.size() != 8) {
            reason = "leftTarget and rightTarget must be 8-element dual quaternions";
            return false;
        }
    } else if (request.absolute_target.size() != 8 || request.relative_target.size() != 8) {
        reason = "absoluteTarget and relativeTarget must be 8-element dual quaternions";
        return false;
    }
    if (static_cast<int>(request.lower_bounds.size()) != request.active_count ||
        static_cast<int>(request.upper_bounds.size()) != request.active_count ||
        static_cast<int>(request.step_bounds.size()) != request.active_count) {
        reason = "lowerBounds, upperBounds, and stepBounds must match activeCount";
        return false;
    }
    if (!request.regularization_weights.empty() &&
        static_cast<int>(request.regularization_weights.size()) != request.active_count) {
        reason = "regularizationWeights must be empty or match activeCount";
        return false;
    }
    if (!isFiniteVector(request.q_current) ||
        !isFiniteVector(request.absolute_target) ||
        !isFiniteVector(request.relative_target) ||
        !isFiniteVector(request.left_target) ||
        !isFiniteVector(request.right_target) ||
        !isFiniteVector(request.lower_bounds) ||
        !isFiniteVector(request.upper_bounds) ||
        !isFiniteVector(request.step_bounds) ||
        !isFiniteVector(request.regularization_weights) ||
        !isFiniteRows(request.joint_limit_vfi.rows) ||
        !isFiniteVector(request.joint_limit_vfi.rhs) ||
        !isFiniteRows(request.waist_x_corridor.rows) ||
        !isFiniteVector(request.waist_x_corridor.rhs)) {
        reason = "request vectors must contain only finite values";
        return false;
    }
    if (!isFiniteValue(request.body_equality_tolerance) ||
        !isFiniteValue(request.absolute_weight) ||
        !isFiniteValue(request.relative_weight) ||
        !isFiniteValue(request.body_weight) ||
        !isFiniteValue(request.wvfi_lower_bound) ||
        !isFiniteValue(request.joint_limit_activation_threshold) ||
        !isFiniteValue(request.absolute_reference.stiffness) ||
        !isFiniteValue(request.absolute_reference.damping) ||
        !isFiniteValue(request.relative_reference.stiffness) ||
        !isFiniteValue(request.relative_reference.damping) ||
        !isFiniteValue(request.body_reference.stiffness) ||
        !isFiniteValue(request.body_reference.damping) ||
        !isFiniteValue(request.damping) ||
        !isFiniteValue(request.damping_growth_factor) ||
        !isFiniteValue(request.initial_scale) ||
        !isFiniteValue(request.min_scale) ||
        !isFiniteValue(request.reduction_factor) ||
        !isFiniteValue(request.initial_trust_radius) ||
        !isFiniteValue(request.min_trust_radius) ||
        !isFiniteValue(request.max_trust_radius) ||
        !isFiniteValue(request.trust_region_shrink_factor) ||
        !isFiniteValue(request.trust_region_expand_factor) ||
        !isFiniteValue(request.rho_acceptance_threshold) ||
        !isFiniteValue(request.rho_shrink_threshold) ||
        !isFiniteValue(request.rho_expand_threshold) ||
        !isFiniteValue(request.equality_tolerance) ||
        !isFiniteValue(request.inequality_tolerance) ||
        !isFiniteValue(request.safety_push.weight) ||
        !isFiniteValue(request.safety_push.activation_residual) ||
        !isFiniteValue(request.safety_push.pose_hold_tolerance) ||
        !isFiniteValue(request.safety_push.target_fraction) ||
        !isFiniteValue(request.safety_push.max_distance_step) ||
        !isFiniteValue(request.waist_x_corridor.activation_threshold) ||
        !isFiniteValue(request.waist_x_corridor.tolerance)) {
        reason = "request scalar fields must be finite";
        return false;
    }
    if (request.absolute_weight < 0.0 || request.relative_weight < 0.0 || request.body_weight < 0.0) {
        reason = "equality weights must be non-negative";
        return false;
    }
    if (request.wvfi_lower_bound < 0.0) {
        reason = "wvfiLowerBound must be non-negative";
        return false;
    }
    if (request.joint_limit_activation_threshold < 0.0) {
        reason = "jointLimitActivationThreshold must be non-negative";
        return false;
    }
    if (request.safety_push.weight < 0.0 ||
        request.safety_push.activation_residual < 0.0 ||
        request.safety_push.pose_hold_tolerance < 0.0 ||
        request.safety_push.target_fraction < 0.0 ||
        request.safety_push.max_distance_step < 0.0 ||
        request.waist_x_corridor.activation_threshold < 0.0) {
        reason = "safety push parameters must be non-negative";
        return false;
    }
    if (request.max_iterations <= 0) {
        reason = "maxIterations must be positive";
        return false;
    }
    if (request.max_damping_retries < 0 || request.max_trust_region_retries <= 0) {
        reason = "retry counts must be non-negative";
        return false;
    }
    if (request.damping < 0.0 || request.initial_scale <= 0.0 || request.min_scale <= 0.0) {
        reason = "damping and line-search scales must be non-negative";
        return false;
    }
    if (request.damping_growth_factor <= 1.0) {
        reason = "dampingGrowthFactor must exceed 1";
        return false;
    }
    if (request.reduction_factor <= 0.0 || request.reduction_factor >= 1.0) {
        reason = "reductionFactor must be between 0 and 1";
        return false;
    }
    if (request.initial_trust_radius <= 0.0 ||
        request.min_trust_radius <= 0.0 ||
        request.max_trust_radius < request.min_trust_radius) {
        reason = "trust-region radii must be positive and ordered";
        return false;
    }
    if (request.trust_region_shrink_factor <= 0.0 || request.trust_region_shrink_factor >= 1.0) {
        reason = "trustRegionShrinkFactor must be between 0 and 1";
        return false;
    }
    if (request.trust_region_expand_factor <= 1.0) {
        reason = "trustRegionExpandFactor must exceed 1";
        return false;
    }
    if (request.waist_x_corridor.enabled &&
        request.waist_x_corridor.rows.size() != request.waist_x_corridor.rhs.size()) {
        reason = "waistXCorridor rows/rhs size mismatch";
        return false;
    }
    if (request.joint_limit_vfi.enabled &&
        request.joint_limit_vfi.rows.size() != request.joint_limit_vfi.rhs.size()) {
        reason = "jointLimitVFI rows/rhs size mismatch";
        return false;
    }
    for (const auto& row : request.joint_limit_vfi.rows) {
        if (static_cast<int>(row.size()) != request.active_count) {
            reason = "jointLimitVFI rows must have activeCount columns";
            return false;
        }
    }
    for (const auto& row : request.waist_x_corridor.rows) {
        if (static_cast<int>(row.size()) != request.active_count) {
            reason = "waistXCorridor rows must have activeCount columns";
            return false;
        }
    }
    for (int i = 0; i < request.active_count; ++i) {
        if (request.lower_bounds[static_cast<std::size_t>(i)] > request.upper_bounds[static_cast<std::size_t>(i)]) {
            reason = "lowerBounds must not exceed upperBounds";
            return false;
        }
        if (request.step_bounds[static_cast<std::size_t>(i)] < 0.0) {
            reason = "stepBounds must be non-negative";
            return false;
        }
    }
    return true;
}

struct ProblemContext {
    const DQ_HumanoidURDF& robot;
    const HumanoidAbsoluteMJRestoreRequest& request;
    std::string solve_variant;
    VectorXd absolute_target;
    VectorXd relative_target;
    VectorXd left_target;
    VectorXd right_target;
    std::vector<double> q_initial_active;
    std::vector<double> effective_lower_bounds;
    std::vector<double> effective_upper_bounds;
};

struct Evaluation {
    VectorXd q_full;
    std::vector<double> q_active;
    VectorXd weighted_residual;
    MatrixXd weighted_jacobian;
    Eigen::Index absolute_row_count = 0;
    Eigen::Index relative_row_count = 0;
    Eigen::Index body_row_count = 0;
    double active_residual_norm = std::numeric_limits<double>::infinity();
    double absolute_residual_norm = std::numeric_limits<double>::infinity();
    double relative_residual_norm = std::numeric_limits<double>::infinity();
    double body_residual_norm = 0.0;
    double wvfi = 0.0;
    double waist_x_position = 0.0;
    double waist_state_violation = 0.0;
    double inequality_violation = std::numeric_limits<double>::infinity();
};

struct SafetyObjective {
    MatrixXd rows;
    VectorXd targets;
    double residual_norm = 0.0;
};

SafetyObjective buildSafetyObjective(
    const HumanoidAbsoluteMJRestoreRequest& request,
    const Evaluation& evaluation
) {
    std::vector<std::vector<double>> rows;
    std::vector<double> targets;
    const int width = request.active_count;
    const double target_fraction = std::max(0.0, request.safety_push.target_fraction);
    const double max_distance_step = std::max(0.0, request.safety_push.max_distance_step);

    const auto add_distance_target = [&](const std::vector<double>& row, double distance, double target_margin) {
        if (static_cast<int>(row.size()) != width || !std::isfinite(distance)) return;
        const double residual = std::max(0.0, target_margin) - std::max(0.0, distance);
        if (!(residual > 1e-12)) return;
        rows.push_back(row);
        targets.push_back(std::min(max_distance_step, residual));
    };

    const double joint_target_margin =
        std::max(0.0, request.joint_limit_activation_threshold) * target_fraction;
    if (joint_target_margin > 0.0 && max_distance_step > 0.0) {
        for (int i = 0; i < width; ++i) {
            const double qi = evaluation.q_active[static_cast<std::size_t>(i)];
            const double lower = request.lower_bounds[static_cast<std::size_t>(i)];
            const double upper = request.upper_bounds[static_cast<std::size_t>(i)];
            if (std::isfinite(upper)) {
                std::vector<double> row(static_cast<std::size_t>(width), 0.0);
                row[static_cast<std::size_t>(i)] = -1.0;
                add_distance_target(row, upper - qi, joint_target_margin);
            }
            if (std::isfinite(lower)) {
                std::vector<double> row(static_cast<std::size_t>(width), 0.0);
                row[static_cast<std::size_t>(i)] = 1.0;
                add_distance_target(row, qi - lower, joint_target_margin);
            }
        }
    }

    if (request.waist_x_corridor.enabled && max_distance_step > 0.0) {
        const double corridor_target_margin =
            std::max(0.0, request.waist_x_corridor.activation_threshold) * target_fraction;
        std::size_t row_index = 0;
        if (std::isfinite(request.waist_x_corridor.max) &&
            row_index < request.waist_x_corridor.rows.size()) {
            std::vector<double> row = request.waist_x_corridor.rows[row_index];
            for (double& value : row) value = -value;
            add_distance_target(row, request.waist_x_corridor.max - evaluation.waist_x_position, corridor_target_margin);
            row_index += 1;
        }
        if (std::isfinite(request.waist_x_corridor.min) &&
            row_index < request.waist_x_corridor.rows.size()) {
            std::vector<double> row = request.waist_x_corridor.rows[row_index];
            for (double& value : row) value = -value;
            add_distance_target(row, evaluation.waist_x_position - request.waist_x_corridor.min, corridor_target_margin);
        }
    }

    SafetyObjective objective;
    objective.rows = MatrixXd(static_cast<Eigen::Index>(rows.size()), width);
    objective.targets = VectorXd(static_cast<Eigen::Index>(targets.size()));
    for (std::size_t i = 0; i < rows.size(); ++i) {
        for (int j = 0; j < width; ++j) {
            objective.rows(static_cast<Eigen::Index>(i), j) = rows[i][static_cast<std::size_t>(j)];
        }
        objective.targets(static_cast<Eigen::Index>(i)) = targets[i];
    }
    objective.residual_norm = objective.targets.norm();
    return objective;
}

double computeModelObjectiveForRows(
    const MatrixXd& model_jacobian,
    const std::vector<double>& dq_step,
    double damping,
    const std::vector<double>& regularization_weights,
    const std::vector<double>& reference_residual
) {
    double residual_cost = 0.0;
    for (Eigen::Index row_index = 0; row_index < model_jacobian.rows(); ++row_index) {
        double predicted = 0.0;
        for (Eigen::Index col_index = 0; col_index < model_jacobian.cols(); ++col_index) {
            predicted += model_jacobian(row_index, col_index) *
                dq_step[static_cast<std::size_t>(col_index)];
        }
        const double target = row_index < static_cast<Eigen::Index>(reference_residual.size())
            ? reference_residual[static_cast<std::size_t>(row_index)]
            : 0.0;
        const double diff = predicted - target;
        residual_cost += diff * diff;
    }

    double regularization_cost = 0.0;
    for (std::size_t i = 0; i < dq_step.size(); ++i) {
        const double weight = i < regularization_weights.size()
            ? std::max(1e-12, regularization_weights[i])
            : 1.0;
        regularization_cost += weight * dq_step[i] * dq_step[i];
    }
    return 0.5 * residual_cost + 0.5 * std::max(1e-12, damping) * regularization_cost;
}

double computeModelObjective(
    const Evaluation& evaluation,
    const std::vector<double>& dq_step,
    double damping,
    const std::vector<double>& regularization_weights,
    const std::vector<double>& reference_residual
) {
    return computeModelObjectiveForRows(
        evaluation.weighted_jacobian,
        dq_step,
        damping,
        regularization_weights,
        reference_residual
    );
}

Evaluation evaluateConfiguration(
    const ProblemContext& context,
    const std::vector<double>& q_active
) {
    Evaluation evaluation;
    evaluation.q_active = q_active;
    evaluation.q_full = buildFullConfiguration(context.request, q_active);

    std::vector<VectorXd> task_weighted_residual_blocks;
    std::vector<MatrixXd> task_weighted_jacobian_blocks;
    if (context.solve_variant == "cooperative_dq_setpoint") {
        const DQ left_pose = context.robot.fkm(evaluation.q_full, 1);
        const DQ right_pose = context.robot.fkm(evaluation.q_full, 2);
        const MatrixXd left_pose_jacobian_active = sliceActiveColumns(
            context.robot.pose_jacobian(evaluation.q_full, 1),
            context.request.active_start,
            context.request.active_count
        );
        const MatrixXd right_pose_jacobian_active = sliceActiveColumns(
            context.robot.pose_jacobian(evaluation.q_full, 2),
            context.request.active_start,
            context.request.active_count
        );
        const DQSetpointErrorModel left_model = buildDQSetpointErrorModel(
            left_pose,
            left_pose_jacobian_active,
            context.left_target
        );
        const DQSetpointErrorModel right_model = buildDQSetpointErrorModel(
            right_pose,
            right_pose_jacobian_active,
            context.right_target
        );

        task_weighted_residual_blocks.push_back((-left_model.error) * context.request.absolute_weight);
        task_weighted_jacobian_blocks.push_back(left_model.jacobian * context.request.absolute_weight);
        task_weighted_residual_blocks.push_back((-right_model.error) * context.request.absolute_weight);
        task_weighted_jacobian_blocks.push_back(right_model.jacobian * context.request.absolute_weight);

        evaluation.absolute_row_count =
            task_weighted_residual_blocks[0].size() + task_weighted_residual_blocks[1].size();
        evaluation.absolute_residual_norm = std::sqrt(
            left_model.error.squaredNorm() + right_model.error.squaredNorm()
        );
        evaluation.relative_residual_norm = 0.0;
    } else if (context.solve_variant == "absolute_dq_setpoint") {
        const DQ absolute_pose = context.robot.absolutePoseDualArm(evaluation.q_full);
        const DQ relative_pose = context.robot.relativePoseDualArm(evaluation.q_full);
        const MatrixXd absolute_pose_jacobian_active = sliceActiveColumns(
            context.robot.absolutePoseJacobianDualArm(evaluation.q_full),
            context.request.active_start,
            context.request.active_count
        );
        const MatrixXd relative_pose_jacobian_active = sliceActiveColumns(
            context.robot.relativePoseJacobianDualArm(evaluation.q_full),
            context.request.active_start,
            context.request.active_count
        );
        const DQSetpointErrorModel absolute_model = buildDQSetpointErrorModel(
            absolute_pose,
            absolute_pose_jacobian_active,
            context.absolute_target
        );
        const DQSetpointErrorModel relative_model = buildDQSetpointErrorModel(
            relative_pose,
            relative_pose_jacobian_active,
            context.relative_target
        );

        evaluation.absolute_residual_norm = absolute_model.error.norm();
        evaluation.relative_residual_norm = relative_model.error.norm();

        task_weighted_residual_blocks.push_back((-absolute_model.error) * context.request.absolute_weight);
        task_weighted_jacobian_blocks.push_back(absolute_model.jacobian * context.request.absolute_weight);
        evaluation.absolute_row_count = task_weighted_residual_blocks.back().size();

        task_weighted_residual_blocks.push_back((-relative_model.error) * context.request.relative_weight);
        task_weighted_jacobian_blocks.push_back(relative_model.jacobian * context.request.relative_weight);
        evaluation.relative_row_count = task_weighted_residual_blocks.back().size();
    } else {
        const DQ absolute_pose = context.robot.absolutePoseDualArm(evaluation.q_full);
        const DQ relative_pose = context.robot.relativePoseDualArm(evaluation.q_full);
        const VectorXd absolute_error = signedPoseError(context.absolute_target, absolute_pose.q);
        const VectorXd relative_error = signedPoseError(context.relative_target, relative_pose.q);

        evaluation.absolute_residual_norm = absolute_error.norm();
        evaluation.relative_residual_norm = relative_error.norm();

        const MatrixXd absolute_pose_jacobian_full =
            context.robot.absolutePoseJacobianDualArm(evaluation.q_full);
        const MatrixXd relative_pose_jacobian_full =
            context.robot.relativePoseJacobianDualArm(evaluation.q_full);
        const MatrixXd absolute_pose_jacobian_active = sliceActiveColumns(
            absolute_pose_jacobian_full,
            context.request.active_start,
            context.request.active_count
        );
        const MatrixXd relative_pose_jacobian_active = sliceActiveColumns(
            relative_pose_jacobian_full,
            context.request.active_start,
            context.request.active_count
        );

        task_weighted_residual_blocks.push_back(absolute_error * context.request.absolute_weight);
        task_weighted_jacobian_blocks.push_back(absolute_pose_jacobian_active * context.request.absolute_weight);
        evaluation.absolute_row_count = task_weighted_residual_blocks.back().size();

        task_weighted_residual_blocks.push_back(relative_error * context.request.relative_weight);
        task_weighted_jacobian_blocks.push_back(relative_pose_jacobian_active * context.request.relative_weight);
        evaluation.relative_row_count = task_weighted_residual_blocks.back().size();
    }

    if (context.request.keep_body_z_vertical) {
        const DQ line_direction(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        const DQ waist_pose = context.robot.fkm(evaluation.q_full, 0);
        const MatrixXd waist_pose_jacobian_full = context.robot.pose_jacobian(evaluation.q_full, 0);
        const MatrixXd line_jacobian_full =
            DQ_Kinematics::line_jacobian(waist_pose_jacobian_full, waist_pose, line_direction);
        const MatrixXd line_jacobian_active = sliceActiveColumns(
            line_jacobian_full,
            context.request.active_start,
            context.request.active_count
        );
        const MatrixXd rot_mat = computeRotationMatrixFromPose(waist_pose);
        const double diff0 = 0.0;
        const double diff1 = -rot_mat(0, 2);
        const double diff2 = -rot_mat(1, 2);
        const double diff3 = 1.0 - rot_mat(2, 2);
        const double mismatch_squared =
            diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
        const VectorXd body_residual = VectorXd::Constant(1, -mismatch_squared);
        MatrixXd body_jacobian(1, context.request.active_count);
        for (Eigen::Index j = 0; j < body_jacobian.cols(); ++j) {
            body_jacobian(0, j) = -2.0 * (
                diff0 * line_jacobian_active(0, j) +
                diff1 * line_jacobian_active(1, j) +
                diff2 * line_jacobian_active(2, j) +
                diff3 * line_jacobian_active(3, j)
            );
        }
        evaluation.body_residual_norm = std::abs(body_residual(0));
        if (evaluation.body_residual_norm > context.request.body_equality_tolerance + 1e-18) {
            task_weighted_residual_blocks.push_back(body_residual * context.request.body_weight);
            task_weighted_jacobian_blocks.push_back(body_jacobian * context.request.body_weight);
            evaluation.body_row_count = body_residual.size();
        }
    }

    const Eigen::Index task_rows = std::accumulate(
        task_weighted_residual_blocks.begin(),
        task_weighted_residual_blocks.end(),
        Eigen::Index(0),
        [](Eigen::Index sum, const VectorXd& block) { return sum + block.size(); }
    );
    const Eigen::Index total_rows = task_rows;

    evaluation.weighted_residual = VectorXd::Zero(total_rows);
    evaluation.weighted_jacobian = MatrixXd::Zero(total_rows, context.request.active_count);
    Eigen::Index row_offset = 0;
    for (std::size_t block_index = 0; block_index < task_weighted_residual_blocks.size(); ++block_index) {
        const VectorXd& residual_block = task_weighted_residual_blocks[block_index];
        const MatrixXd& jacobian_block = task_weighted_jacobian_blocks[block_index];
        evaluation.weighted_residual.segment(row_offset, residual_block.size()) = residual_block;
        evaluation.weighted_jacobian.block(row_offset, 0, residual_block.size(), jacobian_block.cols()) = jacobian_block;
        row_offset += residual_block.size();
    }
    evaluation.active_residual_norm = task_rows > 0
        ? evaluation.weighted_residual.head(task_rows).norm()
        : 0.0;

    const DQ absolute_pose_for_wvfi = context.robot.absolutePoseDualArm(evaluation.q_full);
    const DQ relative_pose_for_wvfi = context.robot.relativePoseDualArm(evaluation.q_full);
    const MatrixXd absolute_pose_jacobian_active_for_wvfi = sliceActiveColumns(
        context.robot.absolutePoseJacobianDualArm(evaluation.q_full),
        context.request.active_start,
        context.request.active_count
    );
    const MatrixXd relative_pose_jacobian_active_for_wvfi = sliceActiveColumns(
        context.robot.relativePoseJacobianDualArm(evaluation.q_full),
        context.request.active_start,
        context.request.active_count
    );
    const MatrixXd absolute_twist_jacobian = computeTwistJacobian(
        absolute_pose_jacobian_active_for_wvfi,
        absolute_pose_for_wvfi
    );
    const MatrixXd relative_twist_jacobian = computeTwistJacobian(
        relative_pose_jacobian_active_for_wvfi,
        relative_pose_for_wvfi
    );
    MatrixXd full_twist_jacobian(
        absolute_twist_jacobian.rows() + relative_twist_jacobian.rows(),
        absolute_twist_jacobian.cols()
    );
    full_twist_jacobian << absolute_twist_jacobian, relative_twist_jacobian;
    const MatrixXd attenuation = buildVFIAttenuationMatrix(
        q_active,
        context.request.lower_bounds,
        context.request.upper_bounds,
        context.request.joint_limit_activation_threshold
    );
    const MatrixXd gram = full_twist_jacobian * attenuation * full_twist_jacobian.transpose();
    evaluation.wvfi = std::sqrt(std::max(0.0, nonNegativeDeterminant(gram)));

    const double wvfi_violation = context.request.wvfi_lower_bound > 0.0
        ? std::max(0.0, context.request.wvfi_lower_bound - evaluation.wvfi)
        : 0.0;

    if (context.request.waist_x_corridor.enabled) {
        evaluation.waist_x_position = computeWaistXPosition(context.robot, evaluation.q_full);
        const double tolerance = std::max(0.0, context.request.waist_x_corridor.tolerance);
        const double low_violation =
            std::isfinite(context.request.waist_x_corridor.min)
                ? std::max(0.0, (context.request.waist_x_corridor.min - tolerance) - evaluation.waist_x_position)
                : 0.0;
        const double high_violation =
            std::isfinite(context.request.waist_x_corridor.max)
                ? std::max(0.0, evaluation.waist_x_position - (context.request.waist_x_corridor.max + tolerance))
                : 0.0;
        evaluation.waist_state_violation = std::max(low_violation, high_violation);
    }

    evaluation.inequality_violation = std::max(wvfi_violation, evaluation.waist_state_violation);
    return evaluation;
}

double stepInequalityViolation(
    const HumanoidAbsoluteMJRestoreRequest& request,
    const std::vector<double>& dq_step
) {
    double max_violation = 0.0;
    const auto accumulate_violation = [&](const std::vector<std::vector<double>>& rows, const std::vector<double>& rhs) {
        const std::size_t row_count = std::min(rows.size(), rhs.size());
        for (std::size_t i = 0; i < row_count; ++i) {
            const double violation = dotStd(rows[i], dq_step) - rhs[i];
            if (violation > max_violation) max_violation = violation;
        }
    };
    if (request.joint_limit_vfi.enabled) {
        accumulate_violation(request.joint_limit_vfi.rows, request.joint_limit_vfi.rhs);
    }
    if (request.waist_x_corridor.enabled) {
        accumulate_violation(request.waist_x_corridor.rows, request.waist_x_corridor.rhs);
    }
    return max_violation;
}

VectorXd buildReferenceResidual(
    const HumanoidAbsoluteMJRestoreRequest& request,
    const Evaluation& evaluation,
    const std::vector<double>& previous_step,
    VectorXd& constraint_motion_out
) {
    constraint_motion_out = evaluation.weighted_jacobian * toEigenVector(previous_step);
    VectorXd reference = VectorXd::Zero(evaluation.weighted_residual.size());

    const Eigen::Index absolute_rows = evaluation.absolute_row_count;
    const Eigen::Index relative_rows = evaluation.relative_row_count;
    const Eigen::Index body_rows = evaluation.body_row_count;

    for (Eigen::Index i = 0; i < evaluation.weighted_residual.size(); ++i) {
        double stiffness = 1.0;
        double damping = 0.0;
        if (i < absolute_rows) {
            stiffness = request.absolute_reference.stiffness;
            damping = request.absolute_reference.damping;
        } else if (i < absolute_rows + relative_rows) {
            stiffness = request.relative_reference.stiffness;
            damping = request.relative_reference.damping;
        } else if (i < absolute_rows + relative_rows + body_rows) {
            stiffness = request.body_reference.stiffness;
            damping = request.body_reference.damping;
        }

        const double base = std::max(0.0, stiffness) * evaluation.weighted_residual(i);
        const double damping_term = std::max(0.0, damping) * constraint_motion_out(i);
        reference(i) = base - clipDamping(base, damping_term);
    }
    return reference;
}

bool solveRestorationStep(
    const HumanoidAbsoluteMJRestoreRequest& request,
    const Evaluation& evaluation,
    const std::vector<double>& previous_step,
    double damping,
    const std::vector<double>& regularization_weights,
    bool safety_active,
    std::vector<double>& dq_out,
    std::vector<double>& reference_residual_out,
    std::vector<double>& constraint_motion_out,
    std::vector<double>& model_reference_residual_out,
    MatrixXd& model_jacobian_out
) {
    if (evaluation.weighted_jacobian.rows() == 0 || evaluation.weighted_jacobian.cols() == 0) {
        dq_out.assign(static_cast<std::size_t>(request.active_count), 0.0);
        reference_residual_out.clear();
        constraint_motion_out.clear();
        model_reference_residual_out.clear();
        model_jacobian_out = MatrixXd(0, request.active_count);
        return true;
    }

    VectorXd constraint_motion;
    const VectorXd reference_residual = buildReferenceResidual(
        request,
        evaluation,
        previous_step,
        constraint_motion
    );
    MatrixXd model_jacobian = evaluation.weighted_jacobian;
    VectorXd model_reference_residual = reference_residual;
    if (safety_active && request.safety_push.weight > 0.0) {
        const SafetyObjective safety = buildSafetyObjective(request, evaluation);
        if (safety.rows.rows() > 0 && safety.targets.size() == safety.rows.rows()) {
            const double safety_scale = std::sqrt(std::max(0.0, request.safety_push.weight));
            MatrixXd augmented_jacobian(
                model_jacobian.rows() + safety.rows.rows(),
                model_jacobian.cols()
            );
            augmented_jacobian.topRows(model_jacobian.rows()) = model_jacobian;
            augmented_jacobian.bottomRows(safety.rows.rows()) = safety_scale * safety.rows;
            VectorXd augmented_reference(model_reference_residual.size() + safety.targets.size());
            augmented_reference.head(model_reference_residual.size()) = model_reference_residual;
            augmented_reference.tail(safety.targets.size()) = safety_scale * safety.targets;
            model_jacobian = augmented_jacobian;
            model_reference_residual = augmented_reference;
        }
    }
    MatrixXd H = model_jacobian.transpose() * model_jacobian;
    const double damping_floor = std::max(1e-12, damping);
    for (Eigen::Index i = 0; i < H.rows(); ++i) {
        const double weight = i < static_cast<Eigen::Index>(regularization_weights.size())
            ? std::max(1e-12, regularization_weights[static_cast<std::size_t>(i)])
            : 1.0;
        H(i, i) += damping_floor * weight;
    }
    const VectorXd g = model_jacobian.transpose() * model_reference_residual;
    const Eigen::LDLT<MatrixXd> factorization(H);
    if (factorization.info() != Eigen::Success) {
        return false;
    }
    VectorXd dq = factorization.solve(g);
    if (factorization.info() != Eigen::Success || !dq.allFinite()) {
        return false;
    }

    const ActiveLinearRows linear_rows = buildLinearStepRows(request);
    const ActiveLinearRows active_linear_rows = selectViolatedLinearRows(linear_rows, dq);
    if (active_linear_rows.rows.rows() > 0) {
        MatrixXd kkt(
            H.rows() + active_linear_rows.rows.rows(),
            H.cols() + active_linear_rows.rows.rows()
        );
        kkt.setZero();
        kkt.block(0, 0, H.rows(), H.cols()) = H;
        kkt.block(0, H.cols(), H.rows(), active_linear_rows.rows.rows()) = active_linear_rows.rows.transpose();
        kkt.block(H.rows(), 0, active_linear_rows.rows.rows(), H.cols()) = active_linear_rows.rows;

        VectorXd rhs(H.rows() + active_linear_rows.rows.rows());
        rhs << g, active_linear_rows.rhs;

        const VectorXd solution = kkt.fullPivLu().solve(rhs);
        if (solution.allFinite()) {
            dq = solution.head(H.rows());
        }
    }
    if (!dq.allFinite()) {
        return false;
    }

    dq_out = toStdVector(dq);
    reference_residual_out = toStdVector(reference_residual);
    constraint_motion_out = toStdVector(constraint_motion);
    model_reference_residual_out = toStdVector(model_reference_residual);
    model_jacobian_out = model_jacobian;
    return true;
}

std::vector<double> clampToEffectiveBounds(
    const std::vector<double>& q_active,
    const ProblemContext& context
) {
    std::vector<double> clamped = q_active;
    for (int i = 0; i < context.request.active_count; ++i) {
        clamped[static_cast<std::size_t>(i)] = std::min(
            context.effective_upper_bounds[static_cast<std::size_t>(i)],
            std::max(
                context.effective_lower_bounds[static_cast<std::size_t>(i)],
                clamped[static_cast<std::size_t>(i)]
            )
        );
    }
    return clamped;
}

std::vector<double> addScaledStep(
    const std::vector<double>& q_active,
    const std::vector<double>& dq,
    double scale
) {
    std::vector<double> out = q_active;
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] += scale * dq[i];
    }
    return out;
}

} // namespace

HumanoidAbsoluteMJRestoreResult solveHumanoidAbsoluteTargetMJRestore(
    const DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
) {
    std::string validation_reason;
    if (!validateRequest(robot, request, validation_reason)) {
        return makeFailure(request, validation_reason);
    }

    ProblemContext context{
        robot,
        request,
        request.solve_variant,
        toEigenVector(request.absolute_target),
        toEigenVector(request.relative_target),
        toEigenVector(request.left_target),
        toEigenVector(request.right_target),
        sliceActiveConfiguration(request),
        std::vector<double>(static_cast<std::size_t>(request.active_count), 0.0),
        std::vector<double>(static_cast<std::size_t>(request.active_count), 0.0)
    };

    for (int i = 0; i < request.active_count; ++i) {
        const double q0 = context.q_initial_active[static_cast<std::size_t>(i)];
        const double step = request.step_bounds[static_cast<std::size_t>(i)];
        const double lower = request.lower_bounds[static_cast<std::size_t>(i)];
        const double upper = request.upper_bounds[static_cast<std::size_t>(i)];
        context.effective_lower_bounds[static_cast<std::size_t>(i)] = std::max(lower, q0 - step);
        context.effective_upper_bounds[static_cast<std::size_t>(i)] = std::min(upper, q0 + step);
        if (context.effective_lower_bounds[static_cast<std::size_t>(i)] >
            context.effective_upper_bounds[static_cast<std::size_t>(i)] + kBoundTolerance) {
            return makeFailure(request, "joint limits and step bounds produce an empty active interval");
        }
    }

    HumanoidAbsoluteMJRestoreResult result;
    result.q_applied = context.q_initial_active;
    result.best_constraint_violation = std::numeric_limits<double>::infinity();

    Evaluation current = evaluateConfiguration(context, context.q_initial_active);
    result.command_residual_norm = current.absolute_residual_norm;
    result.relative_residual_norm = current.relative_residual_norm;
    result.body_residual_norm = current.body_residual_norm;
    result.inequality_violation = current.inequality_violation;
    result.best_constraint_violation = current.inequality_violation;

    if (current.active_residual_norm <= request.equality_tolerance &&
        current.inequality_violation <= request.inequality_tolerance) {
        result.converged = true;
        result.reason = "converged";
        return result;
    }

    std::vector<double> q_current = context.q_initial_active;
    std::vector<double> q_accepted = context.q_initial_active;
    Evaluation accepted = current;
    bool has_accepted = false;
    std::vector<double> previous_accepted_step(static_cast<std::size_t>(request.active_count), 0.0);
    const std::vector<double> normalized_regularization_weights = normalizeRegularizationWeights(
        request.regularization_weights,
        request.active_count
    );
    const double improvement_tolerance = std::max(
        1e-15,
        std::min(1e-12, std::max(0.0, request.equality_tolerance) * 1e-3)
    );

    for (int iter = 0; iter < request.max_iterations; ++iter) {
        result.iterations = iter + 1;

        std::vector<double> dq;
        std::vector<double> reference_residual;
        std::vector<double> constraint_motion;
        std::vector<double> model_reference_residual;
        MatrixXd model_jacobian;
        std::vector<double> last_reference_residual;
        std::vector<double> last_constraint_motion;
        std::vector<double> accepted_reference_residual;
        std::vector<double> accepted_constraint_motion;
        std::vector<double> accepted_dq_trial;
        std::vector<double> accepted_q_trial;
        Evaluation accepted_trial_evaluation;
        double accepted_scale = 0.0;
        double accepted_damping = std::numeric_limits<double>::quiet_NaN();
        bool accepted_trial = false;
        const double scale0 = std::max(request.min_scale, request.initial_scale);

        for (int damping_attempt = 0; damping_attempt <= request.max_damping_retries; ++damping_attempt) {
            const double attempt_damping = request.damping * std::pow(request.damping_growth_factor, damping_attempt);
            if (!solveRestorationStep(
                    request,
                    current,
                    previous_accepted_step,
                    attempt_damping,
                    normalized_regularization_weights,
                    false,
                    dq,
                    reference_residual,
                    constraint_motion,
                    model_reference_residual,
                    model_jacobian
                )) {
                continue;
            }

            last_reference_residual = reference_residual;
            last_constraint_motion = constraint_motion;
            const double dq_norm = std::sqrt(dotStd(dq, dq));
            if (!(dq_norm > 1e-18)) {
                continue;
            }

            for (double scale = scale0; scale >= request.min_scale; scale *= request.reduction_factor) {
                std::vector<double> dq_trial = dq;
                for (double& value : dq_trial) value *= scale;

                if (stepInequalityViolation(request, dq_trial) > request.inequality_tolerance) {
                    if (scale <= request.min_scale) break;
                    continue;
                }

                const std::vector<double> q_trial = clampToEffectiveBounds(
                    addScaledStep(q_current, dq, scale),
                    context
                );
                Evaluation trial = evaluateConfiguration(context, q_trial);
                if (trial.active_residual_norm + improvement_tolerance < current.active_residual_norm &&
                    trial.inequality_violation <= request.inequality_tolerance) {
                    const bool should_replace = !accepted_trial ||
                        trial.active_residual_norm + improvement_tolerance < accepted_trial_evaluation.active_residual_norm;
                    if (should_replace) {
                        accepted_trial = true;
                        accepted_scale = scale;
                        accepted_damping = attempt_damping;
                        accepted_q_trial = q_trial;
                        accepted_dq_trial = dq_trial;
                        accepted_trial_evaluation = trial;
                        accepted_reference_residual = reference_residual;
                        accepted_constraint_motion = constraint_motion;
                    }
                    break;
                }
            }
        }

        result.reference_residual = accepted_trial ? accepted_reference_residual : last_reference_residual;
        result.constraint_motion = accepted_trial ? accepted_constraint_motion : last_constraint_motion;

        if (!accepted_trial) {
            result.reason = has_accepted ? "best_effort" : "line_search_failed";
            result.q_applied = has_accepted ? q_accepted : context.q_initial_active;
            result.command_residual_norm = has_accepted ? accepted.absolute_residual_norm : current.absolute_residual_norm;
            result.relative_residual_norm = has_accepted ? accepted.relative_residual_norm : current.relative_residual_norm;
            result.body_residual_norm = has_accepted ? accepted.body_residual_norm : current.body_residual_norm;
            result.inequality_violation = has_accepted ? accepted.inequality_violation : current.inequality_violation;
            return result;
        }

        q_current = accepted_q_trial;
        current = accepted_trial_evaluation;
        previous_accepted_step = accepted_dq_trial;
        q_accepted = q_current;
        accepted = current;
        has_accepted = true;
        result.accepted_iterations += 1;
        result.last_accepted_scale = accepted_scale;
        result.last_accepted_damping = accepted_damping;
        result.command_residual_norm = current.absolute_residual_norm;
        result.relative_residual_norm = current.relative_residual_norm;
        result.body_residual_norm = current.body_residual_norm;
        result.inequality_violation = current.inequality_violation;
        result.best_constraint_violation = std::min(result.best_constraint_violation, current.inequality_violation);

        if (current.active_residual_norm <= request.equality_tolerance &&
            current.inequality_violation <= request.inequality_tolerance) {
            result.converged = true;
            result.reason = "converged";
            result.q_applied = q_accepted;
            return result;
        }
    }

    result.reason = "max_iter";
    result.q_applied = has_accepted ? q_accepted : context.q_initial_active;
    result.command_residual_norm = has_accepted ? accepted.absolute_residual_norm : current.absolute_residual_norm;
    result.relative_residual_norm = has_accepted ? accepted.relative_residual_norm : current.relative_residual_norm;
    result.body_residual_norm = has_accepted ? accepted.body_residual_norm : current.body_residual_norm;
    result.inequality_violation = has_accepted ? accepted.inequality_violation : current.inequality_violation;
    result.best_constraint_violation = std::min(
        result.best_constraint_violation,
        has_accepted ? accepted.inequality_violation : current.inequality_violation
    );
    return result;
}

HumanoidAbsoluteMJRestoreResult solveHumanoidAbsoluteTargetMJRestoreTrustRegion(
    const DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
) {
    std::string validation_reason;
    if (!validateRequest(robot, request, validation_reason)) {
        return makeFailure(request, validation_reason);
    }

    ProblemContext context{
        robot,
        request,
        request.solve_variant,
        toEigenVector(request.absolute_target),
        toEigenVector(request.relative_target),
        toEigenVector(request.left_target),
        toEigenVector(request.right_target),
        sliceActiveConfiguration(request),
        std::vector<double>(static_cast<std::size_t>(request.active_count), 0.0),
        std::vector<double>(static_cast<std::size_t>(request.active_count), 0.0)
    };

    for (int i = 0; i < request.active_count; ++i) {
        const double q0 = context.q_initial_active[static_cast<std::size_t>(i)];
        const double step = request.step_bounds[static_cast<std::size_t>(i)];
        const double lower = request.lower_bounds[static_cast<std::size_t>(i)];
        const double upper = request.upper_bounds[static_cast<std::size_t>(i)];
        context.effective_lower_bounds[static_cast<std::size_t>(i)] = std::max(lower, q0 - step);
        context.effective_upper_bounds[static_cast<std::size_t>(i)] = std::min(upper, q0 + step);
        if (context.effective_lower_bounds[static_cast<std::size_t>(i)] >
            context.effective_upper_bounds[static_cast<std::size_t>(i)] + kBoundTolerance) {
            return makeFailure(request, "joint limits and step bounds produce an empty active interval");
        }
    }

    HumanoidAbsoluteMJRestoreResult result;
    result.q_applied = context.q_initial_active;
    result.trust_radius = std::max(request.min_trust_radius, request.initial_trust_radius);
    result.filter_size = 1;

    Evaluation current = evaluateConfiguration(context, context.q_initial_active);
    double current_phi = current.active_residual_norm;
    double current_theta = current.inequality_violation;
    double current_safety = buildSafetyObjective(request, current).residual_norm;
    double current_penalty = std::max(1.0, current_phi + 1.0);
    double current_merit = computeRestorationMerit(current_phi, current_theta, current_penalty);
    result.command_residual_norm = current.absolute_residual_norm;
    result.relative_residual_norm = current.relative_residual_norm;
    result.body_residual_norm = current.body_residual_norm;
    result.inequality_violation = current.inequality_violation;
    result.best_constraint_violation = current_theta;
    result.safety_residual_before = current_safety;
    result.safety_residual_after = current_safety;

    const double phi_tolerance = std::max(1e-12, std::max(0.0, request.equality_tolerance) * 1e-3);
    const double theta_tolerance = std::max(1e-12, std::max(0.0, request.inequality_tolerance) * 1e-3);
    const double safety_hold_tolerance = std::max(
        std::max(0.0, request.equality_tolerance),
        std::max(0.0, request.safety_push.pose_hold_tolerance)
    );
    const double safety_residual_tolerance = 1e-12;
    const auto is_primary_held_for_safety = [&](double phi, double theta) {
        return phi <= safety_hold_tolerance &&
            theta <= std::max(0.0, request.inequality_tolerance) + theta_tolerance;
    };
    const auto is_safety_active = [&](const Evaluation& evaluation, double phi, double theta) {
        return request.safety_push.enabled &&
            request.safety_push.weight > 0.0 &&
            phi <= std::max(0.0, request.safety_push.activation_residual) &&
            theta <= std::max(0.0, request.inequality_tolerance) + theta_tolerance &&
            buildSafetyObjective(request, evaluation).residual_norm > safety_residual_tolerance;
    };
    if (is_safety_active(current, current_phi, current_theta)) {
        result.safety_status = "active";
    }

    if (current_phi <= request.equality_tolerance &&
        current_theta <= request.inequality_tolerance &&
        !is_safety_active(current, current_phi, current_theta)) {
        result.converged = true;
        result.reason = "converged";
        return result;
    }

    const std::vector<double> normalized_regularization_weights = normalizeRegularizationWeights(
        request.regularization_weights,
        request.active_count
    );
    const double min_radius = std::max(1e-8, request.min_trust_radius);
    const double max_radius = std::max(min_radius, request.max_trust_radius);
    const double shrink_factor = std::min(0.99, std::max(0.1, request.trust_region_shrink_factor));
    const double expand_factor = std::max(1.01, request.trust_region_expand_factor);

    std::vector<double> q_current = context.q_initial_active;
    std::vector<double> q_accepted = context.q_initial_active;
    Evaluation accepted = current;
    bool has_accepted = false;
    std::vector<double> previous_accepted_step(static_cast<std::size_t>(request.active_count), 0.0);
    const double start_phi = current_phi;
    const double start_theta = current_theta;

    double trust_radius = result.trust_radius;
    std::vector<FilterEntry> filter = { { current_phi, current_theta } };
    const auto finish_safety_limited = [&]() {
        result.converged = true;
        result.reason = "safety_limited";
        result.q_applied = q_current;
        result.command_residual_norm = current.absolute_residual_norm;
        result.relative_residual_norm = current.relative_residual_norm;
        result.body_residual_norm = current.body_residual_norm;
        result.inequality_violation = current.inequality_violation;
        result.best_constraint_violation = std::min(result.best_constraint_violation, current_theta);
        result.trust_radius = trust_radius;
        result.filter_size = static_cast<int>(filter.size());
        result.safety_status = "limited";
        result.safety_residual_after = current_safety;
        return result;
    };

    for (int iter = 0; iter < request.max_iterations; ++iter) {
        result.iterations = iter + 1;

        struct AcceptedTrial {
            std::vector<double> q_trial;
            Evaluation trial_evaluation;
            double phi = std::numeric_limits<double>::infinity();
            double theta = std::numeric_limits<double>::infinity();
            double merit = std::numeric_limits<double>::infinity();
            std::vector<double> dq_trial;
            double damping = std::numeric_limits<double>::quiet_NaN();
            double rho = std::numeric_limits<double>::quiet_NaN();
            double predicted_reduction = std::numeric_limits<double>::quiet_NaN();
            double actual_reduction = std::numeric_limits<double>::quiet_NaN();
            double safety = 0.0;
            std::string accepted_by;
            std::vector<FilterEntry> next_filter;
            double next_radius = std::numeric_limits<double>::quiet_NaN();
            std::vector<double> reference_residual;
            std::vector<double> constraint_motion;
        };

        bool accepted_trial = false;
        AcceptedTrial accepted_trial_state;
        std::vector<double> last_reference_residual;
        std::vector<double> last_constraint_motion;
        double candidate_radius = trust_radius;
        const bool safety_active_this_iteration = is_safety_active(current, current_phi, current_theta);
        if (safety_active_this_iteration) {
            result.safety_status = "active";
        }

        for (int damping_attempt = 0;
             damping_attempt <= request.max_damping_retries && !accepted_trial;
             ++damping_attempt) {
            const double attempt_damping =
                request.damping * std::pow(request.damping_growth_factor, damping_attempt);
            std::vector<double> dq;
            std::vector<double> reference_residual;
            std::vector<double> constraint_motion;
            std::vector<double> model_reference_residual;
            MatrixXd model_jacobian;
            if (!solveRestorationStep(
                    request,
                    current,
                    previous_accepted_step,
                    attempt_damping,
                    normalized_regularization_weights,
                    safety_active_this_iteration,
                    dq,
                    reference_residual,
                    constraint_motion,
                    model_reference_residual,
                    model_jacobian
                )) {
                continue;
            }

            last_reference_residual = reference_residual;
            last_constraint_motion = constraint_motion;
            if (!(vectorNormStd(dq) > 1e-18)) continue;

            for (int radius_attempt = 0; radius_attempt < request.max_trust_region_retries; ++radius_attempt) {
                const TrustRegionStep clipped_step = clipStepToTrustRegion(dq, candidate_radius);
                if (!(clipped_step.step_norm > 1e-18)) break;

                const std::vector<double> q_trial = clampToEffectiveBounds(
                    addScaledStep(q_current, clipped_step.dq, 1.0),
                    context
                );
                Evaluation trial = evaluateConfiguration(context, q_trial);
                const double trial_phi = trial.active_residual_norm;
                const double trial_theta = trial.inequality_violation;
                const double trial_safety = buildSafetyObjective(request, trial).residual_norm;
                const double trial_penalty = std::max(current_penalty, trial_phi + 1.0);
                const double trial_merit = computeRestorationMerit(trial_phi, trial_theta, trial_penalty);
                const double predicted_reduction =
                    computeModelObjectiveForRows(
                        model_jacobian,
                        std::vector<double>(static_cast<std::size_t>(request.active_count), 0.0),
                        attempt_damping,
                        normalized_regularization_weights,
                        model_reference_residual
                    ) -
                    computeModelObjectiveForRows(
                        model_jacobian,
                        clipped_step.dq,
                        attempt_damping,
                        normalized_regularization_weights,
                        model_reference_residual
                    );
                const double actual_reduction = current_merit - trial_merit;
                const double rho = predicted_reduction > 1e-12
                    ? actual_reduction / predicted_reduction
                    : std::numeric_limits<double>::infinity();

                FilterAcceptance acceptance = acceptRestorationCandidateByFilter(
                    filter,
                    current_phi,
                    current_theta,
                    trial_phi,
                    trial_theta,
                    predicted_reduction,
                    actual_reduction,
                    rho,
                    request.rho_acceptance_threshold,
                    phi_tolerance,
                    theta_tolerance
                );
                const bool safety_accepted =
                    safety_active_this_iteration &&
                    is_primary_held_for_safety(current_phi, current_theta) &&
                    is_primary_held_for_safety(trial_phi, trial_theta) &&
                    trial_safety + safety_residual_tolerance < current_safety;
                if (!acceptance.accepted && safety_accepted) {
                    acceptance.accepted = true;
                    acceptance.reason = "safety";
                    acceptance.next_filter = filter;
                }
                const double next_radius = updateTrustRegionRadius(
                    candidate_radius,
                    rho,
                    clipped_step.step_norm,
                    clipped_step.clipped,
                    min_radius,
                    max_radius,
                    shrink_factor,
                    expand_factor,
                    request.rho_shrink_threshold,
                    request.rho_expand_threshold
                );
                if (acceptance.accepted) {
                    accepted_trial = true;
                    accepted_trial_state.q_trial = q_trial;
                    accepted_trial_state.trial_evaluation = trial;
                    accepted_trial_state.phi = trial_phi;
                    accepted_trial_state.theta = trial_theta;
                    accepted_trial_state.merit = trial_merit;
                    accepted_trial_state.dq_trial = clipped_step.dq;
                    accepted_trial_state.damping = attempt_damping;
                    accepted_trial_state.rho = rho;
                    accepted_trial_state.predicted_reduction = predicted_reduction;
                    accepted_trial_state.actual_reduction = actual_reduction;
                    accepted_trial_state.safety = trial_safety;
                    accepted_trial_state.accepted_by = acceptance.reason;
                    accepted_trial_state.next_filter = acceptance.next_filter;
                    accepted_trial_state.next_radius = acceptance.reason == "safety" ? candidate_radius : next_radius;
                    accepted_trial_state.reference_residual = reference_residual;
                    accepted_trial_state.constraint_motion = constraint_motion;
                    break;
                }
                result.last_rejected_rho = rho;
                result.last_rejected_predicted_reduction = predicted_reduction;
                result.last_rejected_actual_reduction = actual_reduction;
                result.last_rejected_reason = acceptance.reason;
                candidate_radius = next_radius;
                if (candidate_radius <= min_radius + 1e-12) break;
            }
        }

        result.reference_residual = accepted_trial
            ? accepted_trial_state.reference_residual
            : last_reference_residual;
        result.constraint_motion = accepted_trial
            ? accepted_trial_state.constraint_motion
            : last_constraint_motion;

        if (!accepted_trial) {
            if (safety_active_this_iteration && is_primary_held_for_safety(current_phi, current_theta)) {
                return finish_safety_limited();
            }
            result.reason = has_accepted ? "best_effort" : "line_search_failed";
            result.q_applied = has_accepted ? q_accepted : context.q_initial_active;
            result.command_residual_norm = has_accepted ? accepted.absolute_residual_norm : current.absolute_residual_norm;
            result.relative_residual_norm = has_accepted ? accepted.relative_residual_norm : current.relative_residual_norm;
            result.body_residual_norm = has_accepted ? accepted.body_residual_norm : current.body_residual_norm;
            result.inequality_violation = has_accepted ? accepted.inequality_violation : current.inequality_violation;
            result.best_constraint_violation = std::min(
                result.best_constraint_violation,
                has_accepted ? accepted.inequality_violation : current.inequality_violation
            );
            result.trust_radius = trust_radius;
            result.filter_size = static_cast<int>(filter.size());
            return result;
        }

        q_current = accepted_trial_state.q_trial;
        current = accepted_trial_state.trial_evaluation;
        current_phi = accepted_trial_state.phi;
        current_theta = accepted_trial_state.theta;
        current_safety = accepted_trial_state.safety;
        current_penalty = std::max(current_penalty, current_phi + 1.0);
        current_merit = accepted_trial_state.merit;
        q_accepted = q_current;
        accepted = current;
        has_accepted = true;
        previous_accepted_step = accepted_trial_state.dq_trial;
        trust_radius = accepted_trial_state.next_radius;
        filter = accepted_trial_state.next_filter;

        result.accepted_iterations += 1;
        result.last_accepted_scale = 1.0;
        result.last_accepted_damping = accepted_trial_state.damping;
        result.command_residual_norm = current.absolute_residual_norm;
        result.relative_residual_norm = current.relative_residual_norm;
        result.body_residual_norm = current.body_residual_norm;
        result.inequality_violation = current.inequality_violation;
        result.trust_radius = trust_radius;
        result.last_accepted_rho = accepted_trial_state.rho;
        result.last_predicted_reduction = accepted_trial_state.predicted_reduction;
        result.last_actual_reduction = accepted_trial_state.actual_reduction;
        result.accepted_by = accepted_trial_state.accepted_by;
        if (accepted_trial_state.accepted_by == "safety") {
            result.safety_accepted_iterations += 1;
        }
        result.safety_residual_after = current_safety;
        result.best_constraint_violation = std::min(result.best_constraint_violation, current_theta);
        result.filter_size = static_cast<int>(filter.size());

        if (current_phi <= request.equality_tolerance &&
            current_theta <= request.inequality_tolerance &&
            !is_safety_active(current, current_phi, current_theta)) {
            result.converged = true;
            result.reason = "converged";
            result.q_applied = q_accepted;
            return result;
        }
    }

    if (is_primary_held_for_safety(current_phi, current_theta) &&
        is_safety_active(current, current_phi, current_theta)) {
        return finish_safety_limited();
    }

    result.reason = (
        has_accepted &&
        (accepted.active_residual_norm + phi_tolerance < start_phi ||
            accepted.inequality_violation + theta_tolerance < start_theta)
    ) ? "best_effort" : "max_iter";
    result.q_applied = has_accepted ? q_accepted : context.q_initial_active;
    result.command_residual_norm = has_accepted ? accepted.absolute_residual_norm : current.absolute_residual_norm;
    result.relative_residual_norm = has_accepted ? accepted.relative_residual_norm : current.relative_residual_norm;
    result.body_residual_norm = has_accepted ? accepted.body_residual_norm : current.body_residual_norm;
    result.inequality_violation = has_accepted ? accepted.inequality_violation : current.inequality_violation;
    result.best_constraint_violation = std::min(
        result.best_constraint_violation,
        has_accepted ? accepted.inequality_violation : current.inequality_violation
    );
    result.trust_radius = trust_radius;
    result.filter_size = static_cast<int>(filter.size());
    return result;
}

HumanoidAbsoluteMJRestoreResult solveHumanoidCooperativeDQSetpointMJRestore(
    const DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
) {
    HumanoidAbsoluteMJRestoreRequest effective_request = request;
    effective_request.solve_variant = "cooperative_dq_setpoint";
    return solveHumanoidAbsoluteTargetMJRestore(robot, effective_request);
}

HumanoidAbsoluteMJRestoreResult solveHumanoidCooperativeDQSetpointMJRestoreTrustRegion(
    const DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
) {
    HumanoidAbsoluteMJRestoreRequest effective_request = request;
    effective_request.solve_variant = "cooperative_dq_setpoint";
    return solveHumanoidAbsoluteTargetMJRestoreTrustRegion(robot, effective_request);
}

HumanoidAbsoluteMJRestoreResult solveHumanoidAbsoluteDQSetpointMJRestore(
    const DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
) {
    HumanoidAbsoluteMJRestoreRequest effective_request = request;
    effective_request.solve_variant = "absolute_dq_setpoint";
    return solveHumanoidAbsoluteTargetMJRestore(robot, effective_request);
}

HumanoidAbsoluteMJRestoreResult solveHumanoidAbsoluteDQSetpointMJRestoreTrustRegion(
    const DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteMJRestoreRequest& request
) {
    HumanoidAbsoluteMJRestoreRequest effective_request = request;
    effective_request.solve_variant = "absolute_dq_setpoint";
    return solveHumanoidAbsoluteTargetMJRestoreTrustRegion(robot, effective_request);
}

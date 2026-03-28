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
constexpr int kMaxDampingRetries = 8;
constexpr double kDampingGrowthFactor = 10.0;

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

double clipDamping(double residual, double damping_term) {
    if (std::abs(residual) <= 1e-18) return 0.0;
    if ((residual < 0.0) != (damping_term < 0.0)) return damping_term;
    const double max_magnitude = std::abs(residual) * 0.8;
    return std::copysign(std::min(std::abs(damping_term), max_magnitude), damping_term);
}

struct ActiveCorridorRows {
    MatrixXd rows;
    VectorXd rhs;
};

ActiveCorridorRows buildActiveCorridorRows(
    const HumanoidAbsoluteMJRestoreRequest& request
) {
    const std::size_t row_count = std::min(
        request.waist_x_corridor.rows.size(),
        request.waist_x_corridor.rhs.size()
    );
    if (!request.waist_x_corridor.enabled || row_count == 0) {
        return { MatrixXd(0, request.active_count), VectorXd(0) };
    }

    MatrixXd rows(static_cast<Eigen::Index>(row_count), request.active_count);
    VectorXd rhs(static_cast<Eigen::Index>(row_count));
    for (std::size_t i = 0; i < row_count; ++i) {
        for (int j = 0; j < request.active_count; ++j) {
            rows(static_cast<Eigen::Index>(i), j) = request.waist_x_corridor.rows[i][static_cast<std::size_t>(j)];
        }
        rhs(static_cast<Eigen::Index>(i)) = request.waist_x_corridor.rhs[i];
    }
    return { rows, rhs };
}

ActiveCorridorRows selectViolatedCorridorRows(
    const ActiveCorridorRows& corridor,
    const VectorXd& step
) {
    std::vector<Eigen::Index> violated;
    for (Eigen::Index i = 0; i < corridor.rows.rows(); ++i) {
        if (corridor.rows.row(i).dot(step) > corridor.rhs(i) + 1e-12) {
            violated.push_back(i);
        }
    }
    if (violated.empty()) {
        return { MatrixXd(0, corridor.rows.cols()), VectorXd(0) };
    }
    MatrixXd rows(static_cast<Eigen::Index>(violated.size()), corridor.rows.cols());
    VectorXd rhs(static_cast<Eigen::Index>(violated.size()));
    for (std::size_t i = 0; i < violated.size(); ++i) {
        rows.row(static_cast<Eigen::Index>(i)) = corridor.rows.row(violated[i]);
        rhs(static_cast<Eigen::Index>(i)) = corridor.rhs(violated[i]);
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
    if (request.absolute_target.size() != 8 || request.relative_target.size() != 8) {
        reason = "absoluteTarget and relativeTarget must be 8-element dual quaternions";
        return false;
    }
    if (static_cast<int>(request.lower_bounds.size()) != request.active_count ||
        static_cast<int>(request.upper_bounds.size()) != request.active_count ||
        static_cast<int>(request.step_bounds.size()) != request.active_count) {
        reason = "lowerBounds, upperBounds, and stepBounds must match activeCount";
        return false;
    }
    if (!isFiniteVector(request.q_current) ||
        !isFiniteVector(request.absolute_target) ||
        !isFiniteVector(request.relative_target) ||
        !isFiniteVector(request.lower_bounds) ||
        !isFiniteVector(request.upper_bounds) ||
        !isFiniteVector(request.step_bounds) ||
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
        !isFiniteValue(request.initial_scale) ||
        !isFiniteValue(request.min_scale) ||
        !isFiniteValue(request.reduction_factor) ||
        !isFiniteValue(request.equality_tolerance) ||
        !isFiniteValue(request.inequality_tolerance) ||
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
    if (request.max_iterations <= 0) {
        reason = "maxIterations must be positive";
        return false;
    }
    if (request.damping < 0.0 || request.initial_scale <= 0.0 || request.min_scale <= 0.0) {
        reason = "damping and line-search scales must be non-negative";
        return false;
    }
    if (request.reduction_factor <= 0.0 || request.reduction_factor >= 1.0) {
        reason = "reductionFactor must be between 0 and 1";
        return false;
    }
    if (request.waist_x_corridor.enabled &&
        request.waist_x_corridor.rows.size() != request.waist_x_corridor.rhs.size()) {
        reason = "waistXCorridor rows/rhs size mismatch";
        return false;
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
    VectorXd absolute_target;
    VectorXd relative_target;
    std::vector<double> q_initial_active;
    std::vector<double> effective_lower_bounds;
    std::vector<double> effective_upper_bounds;
};

struct Evaluation {
    VectorXd q_full;
    std::vector<double> q_active;
    VectorXd weighted_residual;
    MatrixXd weighted_jacobian;
    double active_residual_norm = std::numeric_limits<double>::infinity();
    double absolute_residual_norm = std::numeric_limits<double>::infinity();
    double relative_residual_norm = std::numeric_limits<double>::infinity();
    double body_residual_norm = 0.0;
    double wvfi = 0.0;
    double waist_x_position = 0.0;
    double waist_state_violation = 0.0;
    double inequality_violation = std::numeric_limits<double>::infinity();
};

Evaluation evaluateConfiguration(
    const ProblemContext& context,
    const std::vector<double>& q_active
) {
    Evaluation evaluation;
    evaluation.q_active = q_active;
    evaluation.q_full = buildFullConfiguration(context.request, q_active);

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

    std::vector<VectorXd> weighted_residual_blocks;
    std::vector<MatrixXd> weighted_jacobian_blocks;

    weighted_residual_blocks.push_back(absolute_error * context.request.absolute_weight);
    weighted_jacobian_blocks.push_back(absolute_pose_jacobian_active * context.request.absolute_weight);

    weighted_residual_blocks.push_back(relative_error * context.request.relative_weight);
    weighted_jacobian_blocks.push_back(relative_pose_jacobian_active * context.request.relative_weight);

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
            weighted_residual_blocks.push_back(body_residual * context.request.body_weight);
            weighted_jacobian_blocks.push_back(body_jacobian * context.request.body_weight);
        }
    }

    const Eigen::Index total_rows = std::accumulate(
        weighted_residual_blocks.begin(),
        weighted_residual_blocks.end(),
        Eigen::Index(0),
        [](Eigen::Index sum, const VectorXd& block) { return sum + block.size(); }
    );

    evaluation.weighted_residual = VectorXd::Zero(total_rows);
    evaluation.weighted_jacobian = MatrixXd::Zero(total_rows, context.request.active_count);
    Eigen::Index row_offset = 0;
    for (std::size_t block_index = 0; block_index < weighted_residual_blocks.size(); ++block_index) {
        const VectorXd& residual_block = weighted_residual_blocks[block_index];
        const MatrixXd& jacobian_block = weighted_jacobian_blocks[block_index];
        evaluation.weighted_residual.segment(row_offset, residual_block.size()) = residual_block;
        evaluation.weighted_jacobian.block(row_offset, 0, residual_block.size(), jacobian_block.cols()) = jacobian_block;
        row_offset += residual_block.size();
    }
    evaluation.active_residual_norm = evaluation.weighted_residual.norm();

    const MatrixXd absolute_twist_jacobian = computeTwistJacobian(
        absolute_pose_jacobian_active,
        absolute_pose
    );
    const MatrixXd relative_twist_jacobian = computeTwistJacobian(
        relative_pose_jacobian_active,
        relative_pose
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
    if (!request.waist_x_corridor.enabled) return 0.0;
    double max_violation = 0.0;
    const std::size_t row_count = std::min(
        request.waist_x_corridor.rows.size(),
        request.waist_x_corridor.rhs.size()
    );
    for (std::size_t i = 0; i < row_count; ++i) {
        const double violation = dotStd(request.waist_x_corridor.rows[i], dq_step) -
            request.waist_x_corridor.rhs[i];
        if (violation > max_violation) max_violation = violation;
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

    const Eigen::Index absolute_rows = 8;
    const Eigen::Index relative_rows = 8;
    const Eigen::Index body_rows =
        std::max<Eigen::Index>(0, evaluation.weighted_residual.size() - absolute_rows - relative_rows);

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
    std::vector<double>& dq_out,
    std::vector<double>& reference_residual_out,
    std::vector<double>& constraint_motion_out
) {
    if (evaluation.weighted_jacobian.rows() == 0 || evaluation.weighted_jacobian.cols() == 0) {
        dq_out.assign(static_cast<std::size_t>(request.active_count), 0.0);
        reference_residual_out.clear();
        constraint_motion_out.clear();
        return true;
    }

    VectorXd constraint_motion;
    const VectorXd reference_residual = buildReferenceResidual(
        request,
        evaluation,
        previous_step,
        constraint_motion
    );
    MatrixXd H =
        evaluation.weighted_jacobian.transpose() * evaluation.weighted_jacobian;
    H.diagonal().array() += std::max(1e-12, damping);
    const VectorXd g = evaluation.weighted_jacobian.transpose() * reference_residual;
    const Eigen::LDLT<MatrixXd> factorization(H);
    if (factorization.info() != Eigen::Success) {
        return false;
    }
    VectorXd dq = factorization.solve(g);
    if (factorization.info() != Eigen::Success || !dq.allFinite()) {
        return false;
    }

    const ActiveCorridorRows corridor = buildActiveCorridorRows(request);
    const ActiveCorridorRows active_corridor = selectViolatedCorridorRows(corridor, dq);
    if (active_corridor.rows.rows() > 0) {
        MatrixXd kkt(
            H.rows() + active_corridor.rows.rows(),
            H.cols() + active_corridor.rows.rows()
        );
        kkt.setZero();
        kkt.block(0, 0, H.rows(), H.cols()) = H;
        kkt.block(0, H.cols(), H.rows(), active_corridor.rows.rows()) = active_corridor.rows.transpose();
        kkt.block(H.rows(), 0, active_corridor.rows.rows(), H.cols()) = active_corridor.rows;

        VectorXd rhs(H.rows() + active_corridor.rows.rows());
        rhs << g, active_corridor.rhs;

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
        toEigenVector(request.absolute_target),
        toEigenVector(request.relative_target),
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

    Evaluation current = evaluateConfiguration(context, context.q_initial_active);
    result.command_residual_norm = current.absolute_residual_norm;
    result.relative_residual_norm = current.relative_residual_norm;
    result.body_residual_norm = current.body_residual_norm;
    result.inequality_violation = current.inequality_violation;

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
    const double improvement_tolerance = std::max(
        1e-15,
        std::min(1e-12, std::max(0.0, request.equality_tolerance) * 1e-3)
    );

    for (int iter = 0; iter < request.max_iterations; ++iter) {
        result.iterations = iter + 1;

        std::vector<double> dq;
        std::vector<double> reference_residual;
        std::vector<double> constraint_motion;
        std::vector<double> last_reference_residual;
        std::vector<double> last_constraint_motion;
        std::vector<double> accepted_reference_residual;
        std::vector<double> accepted_constraint_motion;
        std::vector<double> accepted_dq_trial;
        std::vector<double> accepted_q_trial;
        Evaluation accepted_trial_evaluation;
        double accepted_scale = 0.0;
        bool accepted_trial = false;
        const double scale0 = std::max(request.min_scale, request.initial_scale);

        for (int damping_attempt = 0; damping_attempt <= kMaxDampingRetries; ++damping_attempt) {
            const double attempt_damping = request.damping * std::pow(kDampingGrowthFactor, damping_attempt);
            if (!solveRestorationStep(
                    request,
                    current,
                    previous_accepted_step,
                    attempt_damping,
                    dq,
                    reference_residual,
                    constraint_motion
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
        result.command_residual_norm = current.absolute_residual_norm;
        result.relative_residual_norm = current.relative_residual_norm;
        result.body_residual_norm = current.body_residual_norm;
        result.inequality_violation = current.inequality_violation;

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
    return result;
}

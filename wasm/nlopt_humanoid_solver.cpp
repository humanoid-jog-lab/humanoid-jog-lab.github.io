#include "nlopt_humanoid_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_HumanoidURDF.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>

#if NLOPT_AVAILABLE
#include <nlopt.hpp>
#endif

namespace {

using DQ_robotics::DQ;
using DQ_robotics::DQ_HumanoidURDF;
using DQ_robotics::DQ_Kinematics;
using DQ_robotics::conj;
using DQ_robotics::hamiplus8;
using DQ_robotics::rotation;
using DQ_robotics::translation;
using Eigen::MatrixXd;
using Eigen::SelfAdjointEigenSolver;
using Eigen::VectorXd;

constexpr double kBoundTolerance = 1e-12;
constexpr double kConstraintGradientStep = 1e-6;

bool isFiniteValue(double value) {
    return std::isfinite(value);
}

bool isFiniteVector(const std::vector<double>& values) {
    for (double value : values) {
        if (!isFiniteValue(value)) {
            return false;
        }
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

HumanoidAbsoluteNloptResult makeFailure(
    const HumanoidAbsoluteNloptRequest& request,
    const std::string& reason
) {
    HumanoidAbsoluteNloptResult result;
    result.reason = reason;
    result.q_next = request.q0;
    return result;
}

bool validateRequest(
    const DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteNloptRequest& request,
    std::string& reason
) {
    const std::size_t dof = request.q0.size();
    if (dof == 0) {
        reason = "q0 must not be empty";
        return false;
    }
    if (static_cast<std::size_t>(robot.get_dim_configuration_space()) != dof) {
        reason = "q0 size must match the humanoid robot configuration dimension";
        return false;
    }
    if (request.absolute_target.size() != 8) {
        reason = "absolute_target must be an 8-element dual quaternion";
        return false;
    }
    if (request.relative_reference.size() != 8) {
        reason = "relative_reference must be an 8-element dual quaternion";
        return false;
    }
    if (request.active_indices.empty()) {
        reason = "active_indices must not be empty";
        return false;
    }
    if (request.lower_bounds.size() != dof ||
        request.upper_bounds.size() != dof ||
        request.step_bounds.size() != dof) {
        reason = "lower_bounds, upper_bounds, and step_bounds must match q0 size";
        return false;
    }
    if (!isFiniteVector(request.q0) ||
        !isFiniteVector(request.absolute_target) ||
        !isFiniteVector(request.relative_reference) ||
        !isFiniteVector(request.lower_bounds) ||
        !isFiniteVector(request.upper_bounds) ||
        !isFiniteVector(request.step_bounds)) {
        reason = "all numeric request vectors must contain only finite values";
        return false;
    }
    if (!isFiniteValue(request.wvfi_min) ||
        !isFiniteValue(request.weight_absolute) ||
        !isFiniteValue(request.weight_delta) ||
        !isFiniteValue(request.weight_wvfi_reward) ||
        !isFiniteValue(request.relative_equality_tolerance) ||
        !isFiniteValue(request.weight_relative_penalty) ||
        !isFiniteValue(request.body_vertical_tolerance) ||
        !isFiniteValue(request.weight_body_vertical_penalty) ||
        !isFiniteValue(request.joint_limit_activation_threshold) ||
        !isFiniteValue(request.repair_damping) ||
        !isFiniteValue(request.xtol_rel) ||
        !isFiniteValue(request.ftol_rel)) {
        reason = "all scalar solver parameters must be finite";
        return false;
    }
    if (!(std::isfinite(request.waist_x_min) || std::isinf(request.waist_x_min)) ||
        !(std::isfinite(request.waist_x_max) || std::isinf(request.waist_x_max))) {
        reason = "waist_x_min and waist_x_max must be finite or infinite";
        return false;
    }
    if (request.wvfi_min < 0.0) {
        reason = "wvfi_min must be non-negative";
        return false;
    }
    if (request.weight_absolute < 0.0 ||
        request.weight_delta < 0.0 ||
        request.weight_wvfi_reward < 0.0 ||
        request.weight_relative_penalty < 0.0 ||
        request.weight_body_vertical_penalty < 0.0) {
        reason = "solver weights must be non-negative";
        return false;
    }
    if (request.relative_equality_tolerance < 0.0 || request.body_vertical_tolerance < 0.0) {
        reason = "constraint tolerances must be non-negative";
        return false;
    }
    if (request.enforce_waist_x_corridor && request.waist_x_min > request.waist_x_max) {
        reason = "waist_x_min must be less than or equal to waist_x_max";
        return false;
    }
    if (request.joint_limit_activation_threshold < 0.0) {
        reason = "joint_limit_activation_threshold must be non-negative";
        return false;
    }
    if (request.repair_max_iter < 0 || request.repair_damping < 0.0) {
        reason = "repair_max_iter and repair_damping must be non-negative";
        return false;
    }
    if (request.max_eval <= 0) {
        reason = "max_eval must be positive";
        return false;
    }
    if (request.xtol_rel < 0.0 || request.ftol_rel < 0.0) {
        reason = "xtol_rel and ftol_rel must be non-negative";
        return false;
    }

    std::unordered_set<int> seen_indices;
    for (int index : request.active_indices) {
        if (index < 0 || static_cast<std::size_t>(index) >= dof) {
            reason = "active_indices contains an out-of-range joint index";
            return false;
        }
        if (!seen_indices.insert(index).second) {
            reason = "active_indices must not contain duplicates";
            return false;
        }
    }

    for (std::size_t i = 0; i < dof; ++i) {
        if (request.lower_bounds[i] > request.upper_bounds[i]) {
            reason = "lower_bounds must be less than or equal to upper_bounds";
            return false;
        }
        if (request.step_bounds[i] < 0.0) {
            reason = "step_bounds must be non-negative";
            return false;
        }
    }

    return true;
}

std::vector<double> buildActiveBounds(
    const HumanoidAbsoluteNloptRequest& request,
    bool lower
) {
    std::vector<double> bounds;
    bounds.reserve(request.active_indices.size());
    for (int index : request.active_indices) {
        const std::size_t joint_index = static_cast<std::size_t>(index);
        const double q0 = request.q0[joint_index];
        const double step = request.step_bounds[joint_index];
        const double joint_limit = lower
            ? request.lower_bounds[joint_index]
            : request.upper_bounds[joint_index];
        const double step_limit = lower ? (q0 - step) : (q0 + step);
        bounds.push_back(lower
            ? std::max(joint_limit, step_limit)
            : std::min(joint_limit, step_limit));
    }
    return bounds;
}

std::vector<double> buildInitialActiveConfiguration(
    const HumanoidAbsoluteNloptRequest& request
) {
    std::vector<double> x0;
    x0.reserve(request.active_indices.size());
    for (int index : request.active_indices) {
        x0.push_back(request.q0[static_cast<std::size_t>(index)]);
    }
    return x0;
}

bool validateEffectiveBounds(
    const HumanoidAbsoluteNloptRequest& request,
    const std::vector<double>& lower_bounds,
    const std::vector<double>& upper_bounds,
    std::string& reason
) {
    for (std::size_t i = 0; i < lower_bounds.size(); ++i) {
        if (lower_bounds[i] > upper_bounds[i] + kBoundTolerance) {
            reason = "joint limits and step_bounds produce an empty active interval";
            return false;
        }
        const int active_index = request.active_indices[i];
        const double q0 = request.q0[static_cast<std::size_t>(active_index)];
        if (q0 < lower_bounds[i] - kBoundTolerance || q0 > upper_bounds[i] + kBoundTolerance) {
            reason = "q0 must lie inside the effective bounds for every active joint";
            return false;
        }
    }
    return true;
}

MatrixXd sliceActiveColumns(
    const MatrixXd& jacobian,
    const std::vector<int>& active_indices
) {
    MatrixXd sliced(jacobian.rows(), static_cast<Eigen::Index>(active_indices.size()));
    for (std::size_t i = 0; i < active_indices.size(); ++i) {
        sliced.col(static_cast<Eigen::Index>(i)) = jacobian.col(active_indices[i]);
    }
    return sliced;
}

VectorXd buildFullConfiguration(
    const HumanoidAbsoluteNloptRequest& request,
    const std::vector<double>& x
) {
    VectorXd q = toEigenVector(request.q0);
    for (std::size_t i = 0; i < request.active_indices.size(); ++i) {
        q(request.active_indices[i]) = x[i];
    }
    return q;
}

double signedAlignment(const VectorXd& target, const VectorXd& current) {
    return target.dot(current) < 0.0 ? -1.0 : 1.0;
}

VectorXd signedPoseError(
    const VectorXd& target,
    const VectorXd& current,
    double* sign_out = nullptr
) {
    const double sign = signedAlignment(target, current);
    if (sign_out != nullptr) {
        *sign_out = sign;
    }
    return target - sign * current;
}

MatrixXd computeTwistJacobian(
    const MatrixXd& pose_jacobian,
    const DQ& pose
) {
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

double computeWaistXPosition(const DQ_HumanoidURDF& robot, const VectorXd& q_full) {
    const DQ waist_pose = robot.fkm(q_full, 0);
    const DQ waist_translation = translation(waist_pose);
    return waist_translation.q(1);
}

double nonNegativeDeterminant(const MatrixXd& matrix) {
    if (matrix.rows() == 0 || matrix.cols() == 0) {
        return 0.0;
    }
    const MatrixXd symmetric = 0.5 * (matrix + matrix.transpose());
    SelfAdjointEigenSolver<MatrixXd> solver(symmetric);
    if (solver.info() != Eigen::Success) {
        return 0.0;
    }

    double determinant = 1.0;
    const auto eigenvalues = solver.eigenvalues();
    for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
        const double value = std::max(0.0, eigenvalues(i));
        determinant *= value;
        if (!std::isfinite(determinant)) {
            return 0.0;
        }
    }
    return determinant;
}

MatrixXd buildVFIAttenuationMatrix(
    const std::vector<double>& q_active,
    const std::vector<double>& joint_lower_bounds,
    const std::vector<double>& joint_upper_bounds,
    double activation_threshold
) {
    const Eigen::Index active_count = static_cast<Eigen::Index>(q_active.size());
    MatrixXd attenuation = MatrixXd::Identity(active_count, active_count);
    if (!(activation_threshold > 0.0)) {
        return attenuation;
    }

    for (Eigen::Index i = 0; i < active_count; ++i) {
        const double qi = q_active[static_cast<std::size_t>(i)];
        const double d_up = std::max(0.0, joint_upper_bounds[static_cast<std::size_t>(i)] - qi);
        const double d_lo = std::max(0.0, qi - joint_lower_bounds[static_cast<std::size_t>(i)]);
        const double d_min = std::min(d_up, d_lo);
        if (!std::isfinite(d_min) || d_min >= activation_threshold) {
            continue;
        }
        const double omega = d_min / activation_threshold;
        if (!(omega >= 0.0 && omega < 1.0)) {
            continue;
        }
        attenuation(i, i) -= (1.0 - omega);
    }

    return attenuation;
}

struct Evaluation {
    VectorXd q_full;
    VectorXd absolute_error;
    VectorXd relative_error;
    MatrixXd absolute_pose_jacobian_active;
    MatrixXd relative_pose_jacobian_active;
    VectorXd body_vertical_error;
    MatrixXd body_vertical_direction_jacobian_active;
    VectorXd body_vertical_gradient_active;
    VectorXd waist_x_gradient_active;
    double absolute_sign = 1.0;
    double relative_sign = 1.0;
    double absolute_residual = std::numeric_limits<double>::infinity();
    double relative_residual = std::numeric_limits<double>::infinity();
    double body_vertical_mismatch_squared = 0.0;
    double body_vertical_residual = 0.0;
    double waist_x_position = 0.0;
    double cost = std::numeric_limits<double>::infinity();
    double wvfi = 0.0;
};

struct ProblemContext {
    const DQ_HumanoidURDF& robot;
    const HumanoidAbsoluteNloptRequest& request;
    VectorXd q0_full;
    VectorXd absolute_target;
    VectorXd relative_reference;
    std::vector<double> x0_active;
    std::vector<double> effective_lower_bounds;
    std::vector<double> effective_upper_bounds;
    std::vector<double> joint_lower_bounds_active;
    std::vector<double> joint_upper_bounds_active;
};

Evaluation evaluateConfiguration(
    const ProblemContext& context,
    const std::vector<double>& x
) {
    Evaluation evaluation;
    evaluation.q_full = buildFullConfiguration(context.request, x);

    const DQ absolute_pose = context.robot.absolutePoseDualArm(evaluation.q_full);
    const DQ relative_pose = context.robot.relativePoseDualArm(evaluation.q_full);
    const VectorXd absolute_current = absolute_pose.q;
    const VectorXd relative_current = relative_pose.q;

    evaluation.absolute_error = signedPoseError(
        context.absolute_target,
        absolute_current,
        &evaluation.absolute_sign
    );
    evaluation.relative_error = signedPoseError(
        context.relative_reference,
        relative_current,
        &evaluation.relative_sign
    );

    evaluation.absolute_residual = evaluation.absolute_error.norm();
    evaluation.relative_residual = evaluation.relative_error.norm();

    const VectorXd delta = evaluation.q_full - context.q0_full;
    evaluation.cost =
        context.request.weight_absolute * evaluation.absolute_error.squaredNorm() +
        context.request.weight_delta * delta.squaredNorm();
    if (context.request.weight_wvfi_reward > 0.0) {
        evaluation.cost -= context.request.weight_wvfi_reward * evaluation.wvfi;
    }
    if (context.request.enforce_relative_pose_constraint) {
        evaluation.cost +=
            context.request.weight_relative_penalty * evaluation.relative_error.squaredNorm();
    }

    const MatrixXd absolute_pose_jacobian_full =
        context.robot.absolutePoseJacobianDualArm(evaluation.q_full);
    const MatrixXd relative_pose_jacobian_full =
        context.robot.relativePoseJacobianDualArm(evaluation.q_full);
    evaluation.absolute_pose_jacobian_active = sliceActiveColumns(
        absolute_pose_jacobian_full,
        context.request.active_indices
    );
    evaluation.relative_pose_jacobian_active = sliceActiveColumns(
        relative_pose_jacobian_full,
        context.request.active_indices
    );

    const MatrixXd absolute_twist_jacobian = computeTwistJacobian(
        evaluation.absolute_pose_jacobian_active,
        absolute_pose
    );
    const MatrixXd relative_twist_jacobian = computeTwistJacobian(
        evaluation.relative_pose_jacobian_active,
        relative_pose
    );
    MatrixXd full_twist_jacobian(
        absolute_twist_jacobian.rows() + relative_twist_jacobian.rows(),
        absolute_twist_jacobian.cols()
    );
    full_twist_jacobian << absolute_twist_jacobian, relative_twist_jacobian;

    const MatrixXd attenuation = buildVFIAttenuationMatrix(
        x,
        context.joint_lower_bounds_active,
        context.joint_upper_bounds_active,
        context.request.joint_limit_activation_threshold
    );
    const MatrixXd gram = full_twist_jacobian * attenuation * full_twist_jacobian.transpose();
    evaluation.wvfi = std::sqrt(std::max(0.0, nonNegativeDeterminant(gram)));

    evaluation.body_vertical_gradient_active = VectorXd::Zero(
        static_cast<Eigen::Index>(context.request.active_indices.size())
    );
    evaluation.waist_x_gradient_active = VectorXd::Zero(
        static_cast<Eigen::Index>(context.request.active_indices.size())
    );
    evaluation.body_vertical_error = VectorXd::Zero(4);
    evaluation.body_vertical_direction_jacobian_active = MatrixXd::Zero(
        4,
        static_cast<Eigen::Index>(context.request.active_indices.size())
    );
    if (context.request.keep_body_z_vertical) {
        const DQ line_direction(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        const DQ waist_pose = context.robot.fkm(evaluation.q_full, 0);
        const MatrixXd waist_pose_jacobian_full = context.robot.pose_jacobian(evaluation.q_full, 0);
        const MatrixXd line_jacobian_full =
            DQ_Kinematics::line_jacobian(waist_pose_jacobian_full, waist_pose, line_direction);
        evaluation.body_vertical_direction_jacobian_active = sliceActiveColumns(
            line_jacobian_full,
            context.request.active_indices
        );
        const MatrixXd rot_mat = computeRotationMatrixFromPose(waist_pose);
        const double diff0 = 0.0;
        const double diff1 = -rot_mat(0, 2);
        const double diff2 = -rot_mat(1, 2);
        const double diff3 = 1.0 - rot_mat(2, 2);
        evaluation.body_vertical_error << diff0, diff1, diff2, diff3;
        evaluation.body_vertical_mismatch_squared =
            diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
        evaluation.body_vertical_residual =
            std::sqrt(std::max(0.0, evaluation.body_vertical_mismatch_squared));
        evaluation.cost +=
            context.request.weight_body_vertical_penalty * evaluation.body_vertical_mismatch_squared;

        for (Eigen::Index j = 0; j < evaluation.body_vertical_direction_jacobian_active.cols(); ++j) {
            evaluation.body_vertical_gradient_active(j) =
                -2.0 * (
                    diff0 * evaluation.body_vertical_direction_jacobian_active(0, j) +
                    diff1 * evaluation.body_vertical_direction_jacobian_active(1, j) +
                    diff2 * evaluation.body_vertical_direction_jacobian_active(2, j) +
                    diff3 * evaluation.body_vertical_direction_jacobian_active(3, j)
                );
        }
    }

    if (context.request.enforce_waist_x_corridor) {
        evaluation.waist_x_position = computeWaistXPosition(context.robot, evaluation.q_full);
        for (std::size_t i = 0; i < x.size(); ++i) {
            const double xi = x[i];
            const double lower = context.effective_lower_bounds[i];
            const double upper = context.effective_upper_bounds[i];
            const double base_step = kConstraintGradientStep * std::max(1.0, std::abs(xi));
            const double backward_room = std::max(0.0, xi - lower);
            const double forward_room = std::max(0.0, upper - xi);
            const double backward_step = std::min(base_step, backward_room);
            const double forward_step = std::min(base_step, forward_room);
            const Eigen::Index active_index = static_cast<Eigen::Index>(i);

            if (forward_step > kBoundTolerance && backward_step > kBoundTolerance) {
                std::vector<double> x_plus = x;
                std::vector<double> x_minus = x;
                x_plus[i] += forward_step;
                x_minus[i] -= backward_step;
                const double value_plus = computeWaistXPosition(
                    context.robot,
                    buildFullConfiguration(context.request, x_plus)
                );
                const double value_minus = computeWaistXPosition(
                    context.robot,
                    buildFullConfiguration(context.request, x_minus)
                );
                evaluation.waist_x_gradient_active(active_index) =
                    (value_plus - value_minus) / (forward_step + backward_step);
            } else if (forward_step > kBoundTolerance) {
                std::vector<double> x_plus = x;
                x_plus[i] += forward_step;
                const double value_plus = computeWaistXPosition(
                    context.robot,
                    buildFullConfiguration(context.request, x_plus)
                );
                evaluation.waist_x_gradient_active(active_index) =
                    (value_plus - evaluation.waist_x_position) / forward_step;
            } else if (backward_step > kBoundTolerance) {
                std::vector<double> x_minus = x;
                x_minus[i] -= backward_step;
                const double value_minus = computeWaistXPosition(
                    context.robot,
                    buildFullConfiguration(context.request, x_minus)
                );
                evaluation.waist_x_gradient_active(active_index) =
                    (evaluation.waist_x_position - value_minus) / backward_step;
            }
        }
    }

    return evaluation;
}

void overwriteResultMetrics(
    const ProblemContext& context,
    const std::vector<double>& x,
    HumanoidAbsoluteNloptResult& result
) {
    const Evaluation evaluation = evaluateConfiguration(context, x);
    result.final_cost = evaluation.cost;
    result.final_absolute_residual = evaluation.absolute_residual;
    result.final_relative_residual = evaluation.relative_residual;
    result.final_body_vertical_residual = evaluation.body_vertical_residual;
    result.final_wvfi = evaluation.wvfi;
    result.q_next = toStdVector(evaluation.q_full);
}

double finiteDifferenceConstraintValue(
    const ProblemContext& context,
    const std::vector<double>& x
) {
    return context.request.wvfi_min - evaluateConfiguration(context, x).wvfi;
}

VectorXd computeWVFIGradient(
    const ProblemContext& context,
    const std::vector<double>& x,
    double current_wvfi
) {
    VectorXd gradient = VectorXd::Zero(static_cast<Eigen::Index>(x.size()));
    for (std::size_t i = 0; i < x.size(); ++i) {
        const double xi = x[i];
        const double lower = context.effective_lower_bounds[i];
        const double upper = context.effective_upper_bounds[i];
        const double base_step = kConstraintGradientStep * std::max(1.0, std::abs(xi));
        const double backward_room = std::max(0.0, xi - lower);
        const double forward_room = std::max(0.0, upper - xi);
        const double backward_step = std::min(base_step, backward_room);
        const double forward_step = std::min(base_step, forward_room);

        if (forward_step > kBoundTolerance && backward_step > kBoundTolerance) {
            std::vector<double> x_plus = x;
            std::vector<double> x_minus = x;
            x_plus[i] += forward_step;
            x_minus[i] -= backward_step;
            const double w_plus = evaluateConfiguration(context, x_plus).wvfi;
            const double w_minus = evaluateConfiguration(context, x_minus).wvfi;
            gradient(static_cast<Eigen::Index>(i)) = (w_plus - w_minus) / (forward_step + backward_step);
        } else if (forward_step > kBoundTolerance) {
            std::vector<double> x_plus = x;
            x_plus[i] += forward_step;
            const double w_plus = evaluateConfiguration(context, x_plus).wvfi;
            gradient(static_cast<Eigen::Index>(i)) = (w_plus - current_wvfi) / forward_step;
        } else if (backward_step > kBoundTolerance) {
            std::vector<double> x_minus = x;
            x_minus[i] -= backward_step;
            const double w_minus = evaluateConfiguration(context, x_minus).wvfi;
            gradient(static_cast<Eigen::Index>(i)) = (current_wvfi - w_minus) / backward_step;
        }
    }
    return gradient;
}

std::vector<double> clampToEffectiveBounds(
    const std::vector<double>& x,
    const ProblemContext& context
) {
    std::vector<double> clamped = x;
    for (std::size_t i = 0; i < clamped.size(); ++i) {
        clamped[i] = std::min(
            context.effective_upper_bounds[i],
            std::max(context.effective_lower_bounds[i], clamped[i])
        );
    }
    return clamped;
}

double constraintRepairMetric(
    const Evaluation& evaluation,
    const ProblemContext& context
) {
    double metric = 0.0;
    if (context.request.enforce_relative_pose_constraint) {
        metric += evaluation.relative_error.squaredNorm();
    }
    if (context.request.keep_body_z_vertical) {
        metric += evaluation.body_vertical_mismatch_squared;
    }
    return std::sqrt(std::max(0.0, metric));
}

bool isWaistXCorridorSatisfied(
    const Evaluation& evaluation,
    const ProblemContext& context,
    double tolerance = 1e-10
) {
    if (!context.request.enforce_waist_x_corridor) return true;
    if (std::isfinite(context.request.waist_x_min) &&
        evaluation.waist_x_position < context.request.waist_x_min - tolerance) {
        return false;
    }
    if (std::isfinite(context.request.waist_x_max) &&
        evaluation.waist_x_position > context.request.waist_x_max + tolerance) {
        return false;
    }
    return true;
}

struct RepairResult {
    std::vector<double> x;
    int iterations = 0;
    bool converged = false;
};

RepairResult repairConstraintProjection(
    const ProblemContext& context,
    const std::vector<double>& x_initial
) {
    RepairResult result;
    result.x = x_initial;

    if ((!context.request.enforce_relative_pose_constraint && !context.request.keep_body_z_vertical) ||
        context.request.repair_max_iter <= 0) {
        result.converged = true;
        return result;
    }

    const Eigen::Index active_count = static_cast<Eigen::Index>(context.request.active_indices.size());
    for (int iter = 0; iter < context.request.repair_max_iter; ++iter) {
        const Evaluation evaluation = evaluateConfiguration(context, result.x);
        const bool relative_ok =
            !context.request.enforce_relative_pose_constraint ||
            evaluation.relative_residual <= context.request.relative_equality_tolerance;
        const bool body_ok =
            !context.request.keep_body_z_vertical ||
            evaluation.body_vertical_residual <= context.request.body_vertical_tolerance;
        if (relative_ok && body_ok && isWaistXCorridorSatisfied(evaluation, context)) {
            result.converged = true;
            return result;
        }

        const Eigen::Index relative_rows =
            context.request.enforce_relative_pose_constraint
                ? evaluation.relative_error.size()
                : 0;
        const Eigen::Index body_rows =
            context.request.keep_body_z_vertical
                ? evaluation.body_vertical_error.size()
                : 0;
        const Eigen::Index total_rows = relative_rows + body_rows;
        if (total_rows <= 0) {
            result.converged = true;
            return result;
        }

        MatrixXd A = MatrixXd::Zero(total_rows, active_count);
        VectorXd e = VectorXd::Zero(total_rows);
        Eigen::Index row = 0;
        if (relative_rows > 0) {
            A.block(
                row,
                0,
                relative_rows,
                active_count
            ) = -evaluation.relative_sign * evaluation.relative_pose_jacobian_active;
            e.segment(row, relative_rows) = evaluation.relative_error;
            row += relative_rows;
        }
        if (body_rows > 0) {
            A.block(
                row,
                0,
                body_rows,
                active_count
            ) = -evaluation.body_vertical_direction_jacobian_active;
            e.segment(row, body_rows) = evaluation.body_vertical_error;
        }

        const MatrixXd normal =
            A.transpose() * A +
            context.request.repair_damping * MatrixXd::Identity(active_count, active_count);
        const VectorXd rhs = -A.transpose() * e;
        const VectorXd dx = normal.ldlt().solve(rhs);
        if (!dx.allFinite() || dx.norm() <= 1e-12) {
            result.iterations = iter + 1;
            return result;
        }

        const double current_metric = constraintRepairMetric(evaluation, context);
        bool accepted = false;
        std::vector<double> best_x = result.x;
        double best_metric = current_metric;

        for (double scale : {1.0, 0.5, 0.25, 0.125, 0.0625}) {
            std::vector<double> x_candidate = result.x;
            for (Eigen::Index j = 0; j < active_count; ++j) {
                x_candidate[static_cast<std::size_t>(j)] += scale * dx(j);
            }
            x_candidate = clampToEffectiveBounds(x_candidate, context);
            const Evaluation candidate = evaluateConfiguration(context, x_candidate);
            if (candidate.wvfi + 1e-10 < context.request.wvfi_min) {
                continue;
            }
            if (!isWaistXCorridorSatisfied(candidate, context)) {
                continue;
            }
            const double candidate_metric = constraintRepairMetric(candidate, context);
            if (candidate_metric + 1e-12 < best_metric) {
                best_metric = candidate_metric;
                best_x = std::move(x_candidate);
                accepted = true;
            }
        }

        result.iterations = iter + 1;
        if (!accepted) {
            return result;
        }
        result.x = std::move(best_x);
    }

    const Evaluation final_evaluation = evaluateConfiguration(context, result.x);
    result.converged =
        (!context.request.enforce_relative_pose_constraint ||
            final_evaluation.relative_residual <= context.request.relative_equality_tolerance) &&
        (!context.request.keep_body_z_vertical ||
            final_evaluation.body_vertical_residual <= context.request.body_vertical_tolerance) &&
        isWaistXCorridorSatisfied(final_evaluation, context);
    return result;
}

#if NLOPT_AVAILABLE
double objectiveCallback(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data
) {
    const auto* context = static_cast<const ProblemContext*>(data);
    const Evaluation evaluation = evaluateConfiguration(*context, x);

    if (!grad.empty()) {
        std::fill(grad.begin(), grad.end(), 0.0);
        const VectorXd q_active = toEigenVector(x);
        const VectorXd q0_active = toEigenVector(context->x0_active);
        VectorXd gradient =
            -2.0 * context->request.weight_absolute *
                evaluation.absolute_sign *
                evaluation.absolute_pose_jacobian_active.transpose() *
                evaluation.absolute_error
            +2.0 * context->request.weight_delta * (q_active - q0_active);
        if (context->request.enforce_relative_pose_constraint) {
            gradient +=
                -2.0 * context->request.weight_relative_penalty *
                evaluation.relative_sign *
                evaluation.relative_pose_jacobian_active.transpose() *
                evaluation.relative_error;
        }
        if (context->request.keep_body_z_vertical) {
            gradient +=
                context->request.weight_body_vertical_penalty *
                evaluation.body_vertical_gradient_active;
        }
        if (context->request.weight_wvfi_reward > 0.0) {
            gradient -= context->request.weight_wvfi_reward *
                computeWVFIGradient(*context, x, evaluation.wvfi);
        }
        for (std::size_t i = 0; i < grad.size(); ++i) {
            grad[i] = gradient(static_cast<Eigen::Index>(i));
        }
    }

    return evaluation.cost;
}

double wvfiConstraintCallback(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data
) {
    const auto* context = static_cast<const ProblemContext*>(data);
    const double value = finiteDifferenceConstraintValue(*context, x);

    if (!grad.empty()) {
        for (std::size_t i = 0; i < grad.size(); ++i) {
            const double xi = x[i];
            const double lower = context->effective_lower_bounds[i];
            const double upper = context->effective_upper_bounds[i];
            const double base_step = kConstraintGradientStep * std::max(1.0, std::abs(xi));
            const double backward_room = std::max(0.0, xi - lower);
            const double forward_room = std::max(0.0, upper - xi);
            const double backward_step = std::min(base_step, backward_room);
            const double forward_step = std::min(base_step, forward_room);

            if (forward_step > kBoundTolerance && backward_step > kBoundTolerance) {
                std::vector<double> x_plus = x;
                std::vector<double> x_minus = x;
                x_plus[i] += forward_step;
                x_minus[i] -= backward_step;
                const double value_plus = finiteDifferenceConstraintValue(*context, x_plus);
                const double value_minus = finiteDifferenceConstraintValue(*context, x_minus);
                grad[i] = (value_plus - value_minus) / (forward_step + backward_step);
            } else if (forward_step > kBoundTolerance) {
                std::vector<double> x_plus = x;
                x_plus[i] += forward_step;
                const double value_plus = finiteDifferenceConstraintValue(*context, x_plus);
                grad[i] = (value_plus - value) / forward_step;
            } else if (backward_step > kBoundTolerance) {
                std::vector<double> x_minus = x;
                x_minus[i] -= backward_step;
                const double value_minus = finiteDifferenceConstraintValue(*context, x_minus);
                grad[i] = (value - value_minus) / backward_step;
            } else {
                grad[i] = 0.0;
            }
        }
    }

    return value;
}

void relativePoseEqualityConstraintCallback(
    unsigned m,
    double* result,
    unsigned n,
    const double* x_raw,
    double* grad,
    void* data
) {
    const auto* context = static_cast<const ProblemContext*>(data);
    std::vector<double> x(x_raw, x_raw + n);
    const Evaluation evaluation = evaluateConfiguration(*context, x);

    for (unsigned i = 0; i < m; ++i) {
        result[i] = i < static_cast<unsigned>(evaluation.relative_error.size())
            ? evaluation.relative_error(static_cast<Eigen::Index>(i))
            : 0.0;
    }

    if (grad != nullptr) {
        for (unsigned i = 0; i < m; ++i) {
            for (unsigned j = 0; j < n; ++j) {
                const double value =
                    (i < static_cast<unsigned>(evaluation.relative_pose_jacobian_active.rows()) &&
                     j < static_cast<unsigned>(evaluation.relative_pose_jacobian_active.cols()))
                        ? (-evaluation.relative_sign *
                            evaluation.relative_pose_jacobian_active(
                                static_cast<Eigen::Index>(i),
                                static_cast<Eigen::Index>(j)
                            ))
                        : 0.0;
                grad[i * n + j] = value;
            }
        }
    }
}

double bodyVerticalConstraintCallback(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data
) {
    const auto* context = static_cast<const ProblemContext*>(data);
    const Evaluation evaluation = evaluateConfiguration(*context, x);
    if (!grad.empty()) {
        for (std::size_t i = 0; i < grad.size(); ++i) {
            grad[i] = i < static_cast<std::size_t>(evaluation.body_vertical_gradient_active.size())
                ? evaluation.body_vertical_gradient_active(static_cast<Eigen::Index>(i))
                : 0.0;
        }
    }
    return evaluation.body_vertical_mismatch_squared - context->request.body_vertical_tolerance;
}

double waistXUpperConstraintCallback(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data
) {
    const auto* context = static_cast<const ProblemContext*>(data);
    const Evaluation evaluation = evaluateConfiguration(*context, x);
    if (!grad.empty()) {
        for (std::size_t i = 0; i < grad.size(); ++i) {
            grad[i] = i < static_cast<std::size_t>(evaluation.waist_x_gradient_active.size())
                ? evaluation.waist_x_gradient_active(static_cast<Eigen::Index>(i))
                : 0.0;
        }
    }
    return evaluation.waist_x_position - context->request.waist_x_max;
}

double waistXLowerConstraintCallback(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data
) {
    const auto* context = static_cast<const ProblemContext*>(data);
    const Evaluation evaluation = evaluateConfiguration(*context, x);
    if (!grad.empty()) {
        for (std::size_t i = 0; i < grad.size(); ++i) {
            grad[i] = i < static_cast<std::size_t>(evaluation.waist_x_gradient_active.size())
                ? -evaluation.waist_x_gradient_active(static_cast<Eigen::Index>(i))
                : 0.0;
        }
    }
    return context->request.waist_x_min - evaluation.waist_x_position;
}

std::string resultToReason(nlopt::result result_code) {
    switch (result_code) {
        case nlopt::SUCCESS:
            return "success";
        case nlopt::STOPVAL_REACHED:
            return "stopval_reached";
        case nlopt::FTOL_REACHED:
            return "ftol_reached";
        case nlopt::XTOL_REACHED:
            return "xtol_reached";
        case nlopt::MAXEVAL_REACHED:
            return "maxeval_reached";
        case nlopt::MAXTIME_REACHED:
            return "maxtime_reached";
        default:
            return "terminated";
    }
}

bool isConvergedResult(nlopt::result result_code) {
    return result_code == nlopt::SUCCESS ||
        result_code == nlopt::STOPVAL_REACHED ||
        result_code == nlopt::FTOL_REACHED ||
        result_code == nlopt::XTOL_REACHED;
}
#endif

}  // namespace

HumanoidAbsoluteNloptResult solveHumanoidAbsoluteTargetNlopt(
    const DQ_HumanoidURDF& robot,
    const HumanoidAbsoluteNloptRequest& request
) {
    std::string reason;
    if (!validateRequest(robot, request, reason)) {
        return makeFailure(request, reason);
    }

#if !NLOPT_AVAILABLE
    return makeFailure(request, "NLopt support is not available in this WASM build");
#else
    const std::vector<double> lower_bounds = buildActiveBounds(request, true);
    const std::vector<double> upper_bounds = buildActiveBounds(request, false);
    if (!validateEffectiveBounds(request, lower_bounds, upper_bounds, reason)) {
        return makeFailure(request, reason);
    }

    ProblemContext context{
        robot,
        request,
        toEigenVector(request.q0),
        toEigenVector(request.absolute_target),
        toEigenVector(request.relative_reference),
        buildInitialActiveConfiguration(request),
        lower_bounds,
        upper_bounds,
        {},
        {},
    };
    context.joint_lower_bounds_active.reserve(request.active_indices.size());
    context.joint_upper_bounds_active.reserve(request.active_indices.size());
    for (int index : request.active_indices) {
        context.joint_lower_bounds_active.push_back(
            request.lower_bounds[static_cast<std::size_t>(index)]
        );
        context.joint_upper_bounds_active.push_back(
            request.upper_bounds[static_cast<std::size_t>(index)]
        );
    }

    HumanoidAbsoluteNloptResult result;
    result.q_next = request.q0;

    std::vector<double> x = context.x0_active;
    try {
        const auto active_dimension = static_cast<unsigned>(request.active_indices.size());
        nlopt::opt optimizer(nlopt::LD_SLSQP, active_dimension);
        optimizer.set_lower_bounds(context.effective_lower_bounds);
        optimizer.set_upper_bounds(context.effective_upper_bounds);
        optimizer.set_maxeval(request.max_eval);
        optimizer.set_xtol_rel(request.xtol_rel);
        optimizer.set_ftol_rel(request.ftol_rel);
        optimizer.set_min_objective(objectiveCallback, &context);
        if (request.wvfi_min > 0.0) {
            optimizer.add_inequality_constraint(
                wvfiConstraintCallback,
                &context,
                1e-8
            );
        }
        if (request.enforce_waist_x_corridor) {
            if (std::isfinite(request.waist_x_max)) {
                optimizer.add_inequality_constraint(
                    waistXUpperConstraintCallback,
                    &context,
                    1e-10
                );
            }
            if (std::isfinite(request.waist_x_min)) {
                optimizer.add_inequality_constraint(
                    waistXLowerConstraintCallback,
                    &context,
                    1e-10
                );
            }
        }

        double final_cost = std::numeric_limits<double>::quiet_NaN();
        const nlopt::result nlopt_result = optimizer.optimize(x, final_cost);

        result.iterations = optimizer.get_numevals();
        result.converged = isConvergedResult(nlopt_result);
        result.reason = resultToReason(nlopt_result);
        const RepairResult repair = repairConstraintProjection(context, x);
        x = repair.x;
        result.repair_iterations = repair.iterations;
        result.repair_converged = repair.converged;
        if ((request.enforce_relative_pose_constraint || request.keep_body_z_vertical) &&
            !repair.converged) {
            result.converged = false;
        }
        overwriteResultMetrics(context, x, result);
        return result;
    } catch (const std::exception& exception) {
        std::ostringstream stream;
        stream << "nlopt_exception: " << exception.what();
        result.reason = stream.str();
        result.iterations = 0;
        try {
            overwriteResultMetrics(context, x, result);
        } catch (...) {
            result.q_next = request.q0;
        }
        return result;
    }
#endif
}

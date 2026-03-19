#include "nlopt_humanoid_solver.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#if NLOPT_AVAILABLE
#include <nlopt.hpp>
#endif

namespace {

constexpr double kBoundTolerance = 1e-12;

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
    const HumanoidAbsoluteNloptRequest& request,
    std::string& reason
) {
    const std::size_t dof = request.q0.size();
    if (dof == 0) {
        reason = "q0 must not be empty";
        return false;
    }
    if (request.absolute_target.empty()) {
        reason = "absolute_target is required for the humanoid absolute-target solver";
        return false;
    }
    if (request.relative_reference.empty()) {
        reason = "relative_reference is required for the humanoid absolute-target solver";
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
        !isFiniteValue(request.weight_relative) ||
        !isFiniteValue(request.weight_delta) ||
        !isFiniteValue(request.xtol_rel) ||
        !isFiniteValue(request.ftol_rel)) {
        reason = "all scalar solver parameters must be finite";
        return false;
    }
    if (request.wvfi_min < 0.0) {
        reason = "wvfi_min must be non-negative";
        return false;
    }
    if (request.weight_absolute < 0.0 ||
        request.weight_relative < 0.0 ||
        request.weight_delta < 0.0) {
        reason = "solver weights must be non-negative";
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
        const double q0 = request.q0[static_cast<std::size_t>(index)];
        const double step = request.step_bounds[static_cast<std::size_t>(index)];
        const double joint_limit = lower
            ? request.lower_bounds[static_cast<std::size_t>(index)]
            : request.upper_bounds[static_cast<std::size_t>(index)];
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

std::vector<double> buildActiveStepBounds(
    const HumanoidAbsoluteNloptRequest& request
) {
    std::vector<double> step_bounds;
    step_bounds.reserve(request.active_indices.size());
    for (int index : request.active_indices) {
        step_bounds.push_back(request.step_bounds[static_cast<std::size_t>(index)]);
    }
    return step_bounds;
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

#if NLOPT_AVAILABLE
struct ObjectiveContext {
    std::vector<double> q0_active;
    double weight_delta = 0.0;
};

double objectiveCallback(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data
) {
    const auto* context = static_cast<const ObjectiveContext*>(data);
    double cost = 0.0;
    if (!grad.empty()) {
        std::fill(grad.begin(), grad.end(), 0.0);
    }

    for (std::size_t i = 0; i < x.size(); ++i) {
        const double delta = x[i] - context->q0_active[i];
        cost += context->weight_delta * delta * delta;
        if (!grad.empty()) {
            grad[i] = 2.0 * context->weight_delta * delta;
        }
    }

    return cost;
}

std::string resultToReason(nlopt::result result_code) {
    switch (result_code) {
        case nlopt::SUCCESS:
            return "NLopt shell solve succeeded";
        case nlopt::STOPVAL_REACHED:
            return "NLopt shell solve reached stopval";
        case nlopt::FTOL_REACHED:
            return "NLopt shell solve reached ftol_rel";
        case nlopt::XTOL_REACHED:
            return "NLopt shell solve reached xtol_rel";
        case nlopt::MAXEVAL_REACHED:
            return "NLopt shell solve reached max_eval";
        case nlopt::MAXTIME_REACHED:
            return "NLopt shell solve reached max_time";
        default:
            return "NLopt shell solve terminated without a success code";
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
    const HumanoidAbsoluteNloptRequest& request
) {
    std::string reason;
    if (!validateRequest(request, reason)) {
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

    HumanoidAbsoluteNloptResult result;
    result.q_next = request.q0;

    try {
        const auto active_dimension = static_cast<unsigned>(request.active_indices.size());
        nlopt::opt optimizer(nlopt::LD_SLSQP, active_dimension);
        optimizer.set_lower_bounds(lower_bounds);
        optimizer.set_upper_bounds(upper_bounds);
        optimizer.set_maxeval(request.max_eval);
        optimizer.set_xtol_rel(request.xtol_rel);
        optimizer.set_ftol_rel(request.ftol_rel);
        optimizer.set_initial_step(buildActiveStepBounds(request));

        ObjectiveContext objective_context{
            buildInitialActiveConfiguration(request),
            request.weight_delta,
        };
        optimizer.set_min_objective(objectiveCallback, &objective_context);

        std::vector<double> x = objective_context.q0_active;
        double final_cost = result.final_cost;
        const nlopt::result nlopt_result = optimizer.optimize(x, final_cost);

        for (std::size_t i = 0; i < request.active_indices.size(); ++i) {
            const int joint_index = request.active_indices[i];
            result.q_next[static_cast<std::size_t>(joint_index)] = x[i];
        }

        result.iterations = optimizer.get_numevals();
        result.final_cost = final_cost;
        result.converged = isConvergedResult(nlopt_result);
        result.reason =
            resultToReason(nlopt_result) +
            "; placeholder objective only penalizes active-joint delta";
        return result;
    } catch (const std::exception& exception) {
        std::ostringstream stream;
        stream << "NLopt shell solve failed: " << exception.what();
        result.reason = stream.str();
        return result;
    }
#endif
}

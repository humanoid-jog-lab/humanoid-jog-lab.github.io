#pragma once

#include <limits>
#include <string>
#include <vector>

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
    double weight_relative = 100.0;
    double weight_delta = 1e-3;
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
    double final_wvfi = 0.0;
    std::vector<double> q_next;
};

HumanoidAbsoluteNloptResult solveHumanoidAbsoluteTargetNlopt(
    const HumanoidAbsoluteNloptRequest& request
);

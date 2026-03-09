#include "urdf_humanoid_chain_loader.h"

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_HumanoidURDF.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>

#include <Eigen/Dense>
#include <tinyxml2.h>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using DQ_robotics::DQ;
using DQ_robotics::DQ_HumanoidURDF;
using DQ_robotics::HumanoidURDFLinkNames;
using DQ_robotics::URDFJointData;
using Eigen::LDLT;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
using AlignedVectors = std::vector<VectorXd, Eigen::aligned_allocator<VectorXd>>;

constexpr int kWaistCount = 4;
constexpr int kLeftArmCount = 7;
constexpr int kRightArmCount = 7;
constexpr int kHeadCount = 2;
constexpr int kTotalJointCount = kWaistCount + kLeftArmCount + kRightArmCount + kHeadCount;
constexpr double kPi = 3.14159265358979323846;

const char* const kJointNames[kTotalJointCount] = {
    "ankle_joint",
    "knee_joint",
    "hip_pitch_joint",
    "hip_yaw_joint",
    "ljoint1",
    "ljoint2",
    "ljoint3",
    "ljoint4",
    "ljoint5",
    "ljoint6",
    "ljoint7",
    "rjoint1",
    "rjoint2",
    "rjoint3",
    "rjoint4",
    "rjoint5",
    "rjoint6",
    "rjoint7",
    "head_yaw_joint",
    "head_pitch_joint",
};

struct JointLimit {
    double lower = -kPi;
    double upper = kPi;
};

struct SolveOptions {
    bool include_waist = false;
    bool keep_body_z_vertical = false;
    bool joint_limit_avoidance = true;
};

struct Outcome {
    std::string reason = "max_iter";
    bool converged = false;
    bool apply_result = false;
};

struct SolveResult {
    VectorXd q;
    std::string reason = "max_iter";
    bool converged = false;
    bool apply_result = false;
    double final_command_norm = std::numeric_limits<double>::infinity();
    double final_invariant_norm = std::numeric_limits<double>::infinity();
};

struct ParsedArgs {
    std::string mode;
    std::string urdf_path = SAILOR_SDK_DEFAULT_URDF_PATH;
    std::vector<double> q;
    std::vector<double> target;
    std::vector<std::vector<double>> waypoints;
    std::vector<double> max_vel;
    std::vector<double> max_acc;
    int gridpoints_hint = 0;
    int sample_count = 0;
    bool include_waist = false;
    bool keep_body_z_vertical = false;
    bool joint_limit_avoidance = true;
};

struct RetimeResult {
    double duration = 0.0;
    std::vector<double> waypoint_times;
    std::vector<double> sample_times;
    AlignedVectors positions;
    AlignedVectors velocities;
    AlignedVectors accelerations;
    VectorXd gridpoints;
    std::vector<double> grid_times;
    VectorXd path_velocity_squared;
};

struct BodyVerticalTaskState {
    bool valid = false;
    Vector2d error = Vector2d::Zero();
    MatrixXd jacobian_rows;
};

std::vector<std::string> split_csv(const std::string& text) {
    std::vector<std::string> tokens;
    std::stringstream stream(text);
    std::string token;
    while (std::getline(stream, token, ',')) {
        tokens.push_back(token);
    }
    return tokens;
}

std::vector<double> parse_csv_doubles(const std::string& text) {
    std::vector<double> values;
    for (const std::string& token : split_csv(text)) {
        if (!token.empty()) {
            values.push_back(std::stod(token));
        }
    }
    return values;
}

std::vector<std::vector<double>> parse_matrix_doubles(const std::string& text) {
    std::vector<std::vector<double>> rows;
    std::stringstream stream(text);
    std::string row;
    while (std::getline(stream, row, ';')) {
        if (!row.empty()) {
            rows.push_back(parse_csv_doubles(row));
        }
    }
    return rows;
}

bool parse_bool_text(const std::string& text) {
    std::string lower;
    lower.reserve(text.size());
    for (char ch : text) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    return lower == "1" || lower == "true" || lower == "yes" || lower == "on";
}

VectorXd vector_from_std(const std::vector<double>& values) {
    VectorXd out(static_cast<Eigen::Index>(values.size()));
    for (Eigen::Index i = 0; i < out.size(); ++i) {
        out(i) = values[static_cast<std::size_t>(i)];
    }
    return out;
}

std::string join_csv(const VectorXd& values) {
    std::ostringstream stream;
    stream << std::setprecision(17);
    for (Eigen::Index i = 0; i < values.size(); ++i) {
        if (i > 0) {
            stream << ',';
        }
        stream << values(i);
    }
    return stream.str();
}

std::string join_csv(const std::vector<double>& values) {
    std::ostringstream stream;
    stream << std::setprecision(17);
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            stream << ',';
        }
        stream << values[i];
    }
    return stream.str();
}

std::string join_matrix(const AlignedVectors& rows) {
    std::ostringstream stream;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        if (i > 0) {
            stream << ';';
        }
        stream << join_csv(rows[i]);
    }
    return stream.str();
}

VectorXd dq_to_vec8(const DQ& pose) {
    return pose.vec8();
}

VectorXd signed_pose_error(const VectorXd& target, const VectorXd& current) {
    const Eigen::Index n = std::min(target.size(), current.size());
    VectorXd error = VectorXd::Zero(n);
    if (n == 0) {
        return error;
    }
    const double sign = target.head(n).dot(current.head(n)) < 0.0 ? -1.0 : 1.0;
    error = target.head(n) - sign * current.head(n);
    return error;
}

std::vector<JointLimit> load_joint_limits(const std::string& urdf_path) {
    std::map<std::string, JointLimit> limit_map;

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Failed to load URDF file for joint limits: " + urdf_path);
    }

    tinyxml2::XMLElement* robot = doc.RootElement();
    if (robot == nullptr || std::string(robot->Name()) != "robot") {
        throw std::runtime_error("URDF root is not <robot>");
    }

    for (tinyxml2::XMLElement* joint = robot->FirstChildElement("joint");
         joint != nullptr;
         joint = joint->NextSiblingElement("joint")) {
        const char* joint_name = joint->Attribute("name");
        const char* joint_type = joint->Attribute("type");
        if (joint_name == nullptr || joint_type == nullptr) {
            continue;
        }

        JointLimit limit;
        if (std::string(joint_type) == "continuous") {
            limit.lower = -kPi;
            limit.upper = kPi;
        } else {
            tinyxml2::XMLElement* limit_el = joint->FirstChildElement("limit");
            if (limit_el != nullptr) {
                limit.lower = limit_el->DoubleAttribute("lower", -kPi);
                limit.upper = limit_el->DoubleAttribute("upper", kPi);
            }
        }
        limit_map[joint_name] = limit;
    }

    std::vector<JointLimit> ordered_limits;
    ordered_limits.reserve(kTotalJointCount);
    for (const char* joint_name : kJointNames) {
        auto iter = limit_map.find(joint_name);
        ordered_limits.push_back(iter != limit_map.end() ? iter->second : JointLimit{});
    }
    return ordered_limits;
}

std::unique_ptr<DQ_HumanoidURDF> load_humanoid_model(const std::string& urdf_path) {
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

    std::vector<URDFJointData> waist_chain;
    std::vector<URDFJointData> left_arm_chain;
    std::vector<URDFJointData> right_arm_chain;
    std::vector<URDFJointData> head_chain;
    std::string error_message;
    DQ waist_effector(1);
    DQ left_effector(1);
    DQ right_effector(1);
    DQ head_effector(1);

    if (!DQ_robotics::load_humanoid_chains_from_urdf(
            urdf_path,
            names,
            &waist_chain,
            &left_arm_chain,
            &right_arm_chain,
            &head_chain,
            &waist_effector,
            &left_effector,
            &right_effector,
            &head_effector,
            &error_message)) {
        throw std::runtime_error("Failed to load humanoid chains: " + error_message);
    }

    DQ base_frame(1, 0, 0, 0, 0, 0, 0, 0);
    return std::make_unique<DQ_HumanoidURDF>(
        base_frame,
        waist_chain,
        left_arm_chain,
        right_arm_chain,
        head_chain,
        waist_effector,
        left_effector,
        right_effector,
        head_effector
    );
}

MatrixXd weighted_damped_pseudoinverse(const MatrixXd& jacobian, const VectorXd& weights, double lambda) {
    if (jacobian.rows() == 0 || jacobian.cols() == 0) {
        return MatrixXd();
    }

    VectorXd winv = weights.unaryExpr([](double value) {
        return value > 1e-10 ? 1.0 / value : 0.0;
    });
    MatrixXd lhs = jacobian * winv.asDiagonal() * jacobian.transpose();
    lhs.diagonal().array() += lambda;

    LDLT<MatrixXd> ldlt(lhs);
    if (ldlt.info() != Eigen::Success) {
        return MatrixXd();
    }

    MatrixXd lhs_inverse = ldlt.solve(MatrixXd::Identity(lhs.rows(), lhs.cols()));
    if (ldlt.info() != Eigen::Success) {
        return MatrixXd();
    }
    return winv.asDiagonal() * jacobian.transpose() * lhs_inverse;
}

MatrixXd nullspace_projection(const MatrixXd& jacobian_pinv, const MatrixXd& jacobian) {
    return MatrixXd::Identity(jacobian_pinv.rows(), jacobian_pinv.rows()) - jacobian_pinv * jacobian;
}

std::vector<JointLimit> active_limits_from(const std::vector<JointLimit>& all_limits, int start, int count) {
    std::vector<JointLimit> active_limits;
    active_limits.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        active_limits.push_back(all_limits[static_cast<std::size_t>(start + i)]);
    }
    return active_limits;
}

BodyVerticalTaskState compute_body_vertical_task_state(
    const DQ_HumanoidURDF& model,
    const VectorXd& q,
    int arm_start,
    int active_count
) {
    BodyVerticalTaskState state;
    try {
        const DQ waist_pose = model.fkm(q, DQ_HumanoidURDF::EffectorWaist);
        const MatrixXd jacobian_waist_full = model.pose_jacobian(q, DQ_HumanoidURDF::EffectorWaist);
        const DQ body_z_line_direction(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        const MatrixXd jacobian_line_full = DQ_robotics::DQ_Kinematics::line_jacobian(
            jacobian_waist_full,
            waist_pose,
            body_z_line_direction
        );
        if (jacobian_line_full.rows() < 3 || jacobian_line_full.cols() < arm_start + active_count) {
            return state;
        }

        const DQ waist_rotation = DQ_robotics::rotation(waist_pose).normalize();
        const VectorXd quaternion = waist_rotation.vec4();
        const double qw = quaternion(0);
        const double qx = quaternion(1);
        const double qy = quaternion(2);
        const double qz = quaternion(3);
        const double r02 = 2.0 * (qx * qz + qw * qy);
        const double r12 = 2.0 * (qy * qz - qw * qx);

        state.error(0) = -r02;
        state.error(1) = -r12;
        state.jacobian_rows = MatrixXd::Zero(2, active_count);
        state.jacobian_rows.row(0) = jacobian_line_full.row(1).segment(arm_start, active_count);
        state.jacobian_rows.row(1) = jacobian_line_full.row(2).segment(arm_start, active_count);
        state.valid = true;
        return state;
    } catch (const std::exception&) {
        return state;
    }
}

VectorXd compute_joint_limit_gradient(const VectorXd& q, const std::vector<JointLimit>& limits) {
    VectorXd gradient = VectorXd::Zero(q.size());
    const double margin = 0.1;

    for (Eigen::Index i = 0; i < q.size() && i < static_cast<Eigen::Index>(limits.size()); ++i) {
        const JointLimit& limit = limits[static_cast<std::size_t>(i)];
        const double range = limit.upper - limit.lower;
        const double threshold = range * margin;
        if (threshold <= 1e-12) {
            continue;
        }
        if (q(i) < limit.lower + threshold) {
            gradient(i) = (limit.lower + threshold - q(i)) / threshold;
        } else if (q(i) > limit.upper - threshold) {
            gradient(i) = -(q(i) - (limit.upper - threshold)) / threshold;
        }
    }
    return gradient;
}

Outcome evaluate_outcome(
    const std::string& stop_reason,
    double initial_command_norm,
    double final_command_norm,
    double final_invariant_norm,
    double hard_command_tol,
    double hard_invariant_tol,
    double practical_command_tol,
    double practical_invariant_tol,
    double mismatch_tol,
    double mismatch_required_reduction
) {
    Outcome outcome;
    outcome.reason = stop_reason.empty() ? "max_iter" : stop_reason;

    const bool hard_satisfied =
        std::isfinite(final_command_norm) &&
        final_command_norm <= hard_command_tol &&
        (!std::isfinite(hard_invariant_tol) || (std::isfinite(final_invariant_norm) && final_invariant_norm <= hard_invariant_tol));

    if (outcome.reason == "max_iter" && hard_satisfied) {
        outcome.reason = "converged";
    }

    const bool practical_satisfied =
        outcome.reason != "converged" &&
        std::isfinite(final_command_norm) &&
        final_command_norm <= practical_command_tol &&
        (!std::isfinite(practical_invariant_tol) ||
         (std::isfinite(final_invariant_norm) && final_invariant_norm <= practical_invariant_tol));

    if (outcome.reason == "max_iter" && practical_satisfied) {
        outcome.reason = "practical_converged";
    }

    const bool reduction_satisfied =
        std::isfinite(initial_command_norm) &&
        initial_command_norm > 0.0 &&
        std::isfinite(final_command_norm) &&
        final_command_norm <= initial_command_norm * (1.0 - mismatch_required_reduction);

    const bool explicit_failure =
        outcome.reason == "fk_fail" ||
        outcome.reason == "jacobian_fail" ||
        outcome.reason == "body_task_fail" ||
        outcome.reason == "solver_fail";

    const bool reject_for_mismatch =
        !explicit_failure &&
        outcome.reason != "converged" &&
        outcome.reason != "practical_converged" &&
        outcome.reason != "single_step" &&
        std::isfinite(final_command_norm) &&
        final_command_norm > mismatch_tol &&
        !reduction_satisfied;

    if (reject_for_mismatch) {
        outcome.reason = "command_mismatch";
    }

    outcome.converged =
        outcome.reason == "converged" ||
        outcome.reason == "practical_converged" ||
        outcome.reason == "single_step";
    outcome.apply_result = !explicit_failure && outcome.reason != "command_mismatch";
    return outcome;
}

VectorXd parse_limit_vector(const std::vector<double>& raw, int dof, double fallback, const char* name) {
    VectorXd limit(dof);
    limit.setConstant(std::abs(fallback));

    if (raw.empty()) {
        return limit;
    }
    if (raw.size() == 1) {
        const double value = raw[0];
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::runtime_error(std::string(name) + " scalar must be positive");
        }
        limit.setConstant(std::abs(value));
        return limit;
    }
    if (raw.size() != static_cast<std::size_t>(dof)) {
        throw std::runtime_error(std::string(name) + " dimension must match waypoint dof");
    }
    for (int i = 0; i < dof; ++i) {
        const double value = raw[static_cast<std::size_t>(i)];
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::runtime_error(std::string(name) + " entries must all be positive");
        }
        limit(i) = std::abs(value);
    }
    return limit;
}

std::vector<double> compute_grid_times(const VectorXd& s_grid, const VectorXd& path_velocity_squared) {
    const VectorXd v_grid = path_velocity_squared.cwiseMax(0.0).array().sqrt();
    std::vector<double> t_grid(static_cast<std::size_t>(s_grid.size()), 0.0);
    for (Eigen::Index i = 0; i < s_grid.size() - 1; ++i) {
        const double ds = s_grid(i + 1) - s_grid(i);
        const double denom = v_grid(i) + v_grid(i + 1);
        double dt = 0.0;
        if (denom > 1e-9 && ds > 0.0) {
            dt = 2.0 * ds / denom;
        }
        if (!std::isfinite(dt) || dt < 0.0) {
            dt = 0.0;
        }
        t_grid[static_cast<std::size_t>(i + 1)] = t_grid[static_cast<std::size_t>(i)] + dt;
    }
    return t_grid;
}

double sample_time_from_s(const VectorXd& s_grid, const std::vector<double>& t_grid, double s_query) {
    if (s_query <= s_grid(0)) {
        return t_grid.front();
    }
    if (s_query >= s_grid(s_grid.size() - 1)) {
        return t_grid.back();
    }

    Eigen::Index lo = 0;
    Eigen::Index hi = s_grid.size() - 2;
    while (lo <= hi) {
        const Eigen::Index mid = lo + (hi - lo) / 2;
        if (s_grid(mid + 1) < s_query) {
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }

    const Eigen::Index max_idx = s_grid.size() - 2;
    const Eigen::Index idx = std::max<Eigen::Index>(0, std::min(max_idx, lo));
    const double s0 = s_grid(idx);
    const double s1 = s_grid(idx + 1);
    const double t0 = t_grid[static_cast<std::size_t>(idx)];
    const double t1 = t_grid[static_cast<std::size_t>(idx + 1)];
    if (s1 - s0 <= 1e-12) {
        return t0;
    }
    const double ratio = std::max(0.0, std::min(1.0, (s_query - s0) / (s1 - s0)));
    return t0 + (t1 - t0) * ratio;
}

RetimeResult retime_waypoints(
    const std::vector<std::vector<double>>& waypoint_rows,
    const std::vector<double>& max_vel_raw,
    const std::vector<double>& max_acc_raw,
    int gridpoints_hint,
    int sample_count
) {
    if (waypoint_rows.size() < 2) {
        throw std::runtime_error("waypoints require at least 2 rows");
    }

    const int dof = static_cast<int>(waypoint_rows.front().size());
    if (dof <= 0) {
        throw std::runtime_error("waypoint dof is invalid");
    }

    toppra::Vectors positions;
    positions.reserve(waypoint_rows.size());
    for (const auto& row : waypoint_rows) {
        if (static_cast<int>(row.size()) != dof) {
            throw std::runtime_error("waypoint row dimensions must be consistent");
        }
        toppra::Vector q(dof);
        for (int i = 0; i < dof; ++i) {
            const double value = row[static_cast<std::size_t>(i)];
            if (!std::isfinite(value)) {
                throw std::runtime_error("waypoints contain non-finite values");
            }
            q(i) = value;
        }
        positions.push_back(q);
    }

    const toppra::Vector vel_abs = parse_limit_vector(max_vel_raw, dof, 0.45, "max_vel");
    const toppra::Vector acc_abs = parse_limit_vector(max_acc_raw, dof, 0.90, "max_acc");
    const toppra::Vector vel_lower = -vel_abs;
    const toppra::Vector vel_upper = vel_abs;
    const toppra::Vector acc_lower = -acc_abs;
    const toppra::Vector acc_upper = acc_abs;

    const int waypoint_count = static_cast<int>(waypoint_rows.size());
    toppra::Vector knots = toppra::Vector::LinSpaced(waypoint_count, 0.0, static_cast<double>(waypoint_count - 1));
    toppra::BoundaryCondFull bc_type = {
        toppra::BoundaryCond("clamped"),
        toppra::BoundaryCond("clamped"),
    };
    toppra::PiecewisePolyPath path_poly = toppra::PiecewisePolyPath::CubicSpline(positions, knots, bc_type);
    toppra::GeometricPathPtr path = std::make_shared<toppra::PiecewisePolyPath>(path_poly);

    toppra::LinearConstraintPtr c_vel = std::make_shared<toppra::constraint::LinearJointVelocity>(vel_lower, vel_upper);
    toppra::LinearConstraintPtr c_acc = std::make_shared<toppra::constraint::LinearJointAcceleration>(acc_lower, acc_upper);
    c_acc->discretizationType(toppra::DiscretizationType::Interpolation);

    toppra::LinearConstraintPtrs constraints{c_vel, c_acc};
    auto algo = std::make_shared<toppra::algorithm::TOPPRA>(constraints, path);

    const int min_n = std::max(1, waypoint_count - 1);
    const int suggested_n = std::max(min_n, (waypoint_count - 1) * 5);
    const int final_n = gridpoints_hint > 0 ? std::max(gridpoints_hint, min_n) : suggested_n;
    algo->setN(final_n);

    const toppra::ReturnCode rc = algo->computePathParametrization(0.0, 0.0);
    if (rc != toppra::ReturnCode::OK) {
        throw std::runtime_error("TOPPRA failed: " + algo->getErrorMessage());
    }

    const auto data = algo->getParameterizationData();
    std::vector<double> waypoint_times(static_cast<std::size_t>(waypoint_count), 0.0);
    const std::vector<double> grid_times = compute_grid_times(data.gridpoints, data.parametrization);
    for (int i = 0; i < waypoint_count; ++i) {
        waypoint_times[static_cast<std::size_t>(i)] = sample_time_from_s(data.gridpoints, grid_times, knots(i));
        if (i > 0 && waypoint_times[static_cast<std::size_t>(i)] < waypoint_times[static_cast<std::size_t>(i - 1)]) {
            waypoint_times[static_cast<std::size_t>(i)] = waypoint_times[static_cast<std::size_t>(i - 1)];
        }
    }

    auto parametrizer = std::make_shared<toppra::parametrizer::ConstAccel>(path, data.gridpoints, data.parametrization);
    if (!parametrizer->validate()) {
        throw std::runtime_error("TOPPRA parametrizer validation failed");
    }

    const auto interval = parametrizer->pathInterval();
    const int final_sample_count = sample_count > 0 ? std::max(sample_count, 2) : std::max(2, (waypoint_count - 1) * 20 + 1);
    toppra::Vector sample_times = toppra::Vector::LinSpaced(final_sample_count, interval(0), interval(1));

    RetimeResult result;
    result.duration = waypoint_times.back();
    result.waypoint_times = waypoint_times;
    result.gridpoints = data.gridpoints;
    result.grid_times = grid_times;
    result.path_velocity_squared = data.parametrization;
    result.sample_times.resize(static_cast<std::size_t>(sample_times.size()));
    for (Eigen::Index i = 0; i < sample_times.size(); ++i) {
        result.sample_times[static_cast<std::size_t>(i)] = sample_times(i);
    }
    result.positions = parametrizer->eval(sample_times, 0);
    result.velocities = parametrizer->eval(sample_times, 1);
    result.accelerations = parametrizer->eval(sample_times, 2);
    return result;
}

SolveResult solve_absolute(
    const DQ_HumanoidURDF& model,
    const std::vector<JointLimit>& all_limits,
    const VectorXd& q_initial,
    const VectorXd& absolute_target_raw,
    const SolveOptions& options
) {
    SolveResult result;
    result.q = q_initial;

    VectorXd absolute_current = dq_to_vec8(model.absolutePoseDualArm(q_initial));
    VectorXd relative_current = dq_to_vec8(model.relativePoseDualArm(q_initial));

    VectorXd relative_constraint = relative_current;
    VectorXd absolute_target = absolute_target_raw;
    if (absolute_target.dot(absolute_current) < 0.0) {
        absolute_target = -absolute_target;
    }

    const int arm_start = options.include_waist ? 0 : kWaistCount;
    const int active_count = options.include_waist ? (kWaistCount + kLeftArmCount + kRightArmCount) : (kLeftArmCount + kRightArmCount);
    const bool enable_body_vertical_task = options.keep_body_z_vertical && options.include_waist;
    const std::vector<JointLimit> active_limits = active_limits_from(all_limits, arm_start, active_count);
    const VectorXd weights = VectorXd::Ones(active_count);
    const double kappa1 = 0.5;
    const double kappa2 = 0.5;
    const double kappa3 = 0.3;
    const double kappa_body_z = 0.8;
    const double hard_command_tol = 1e-6;
    const double hard_invariant_tol = 5e-5;
    const double practical_command_tol = 1e-4;
    const double practical_invariant_tol = 2.5e-4;
    const double mismatch_tol = 1e-3;
    const double mismatch_required_reduction = 0.8;
    const double damping = 0.001;
    const double max_step = 0.003;
    const double max_nullspace_step = max_step * 0.3;
    const int max_iterations = 200;
    const int kappa3_active_iters = 15;
    const double tolerance = 1e-16;
    const double body_z_tolerance = 1e-14;
    const double initial_command_norm = signed_pose_error(absolute_target, absolute_current).norm();

    VectorXd q = q_initial;
    std::string stop_reason = "max_iter";

    for (int iter = 0; iter < max_iterations; ++iter) {
        VectorXd relative_now = dq_to_vec8(model.relativePoseDualArm(q));
        VectorXd absolute_now = dq_to_vec8(model.absolutePoseDualArm(q));

        VectorXd invariant_error = signed_pose_error(relative_constraint, relative_now);
        VectorXd command_error = signed_pose_error(absolute_target, absolute_now);
        BodyVerticalTaskState body_state;
        double body_error_norm2 = 0.0;

        if (enable_body_vertical_task) {
            body_state = compute_body_vertical_task_state(model, q, arm_start, active_count);
            if (!body_state.valid) {
                stop_reason = "body_task_fail";
                break;
            }
            body_error_norm2 = body_state.error.squaredNorm();
        }

        if (
            invariant_error.squaredNorm() < tolerance &&
            command_error.squaredNorm() < tolerance &&
            (!enable_body_vertical_task || body_error_norm2 < body_z_tolerance)
        ) {
            stop_reason = "converged";
            break;
        }

        MatrixXd jacobian_relative_full = model.relativePoseJacobianDualArm(q);
        MatrixXd jacobian_absolute_full = model.absolutePoseJacobianDualArm(q);
        if (jacobian_relative_full.cols() < arm_start + active_count || jacobian_absolute_full.cols() < arm_start + active_count) {
            stop_reason = "jacobian_fail";
            break;
        }

        MatrixXd jacobian1 = jacobian_relative_full.block(0, arm_start, 8, active_count);
        VectorXd invariant_task_error = invariant_error;
        if (enable_body_vertical_task) {
            MatrixXd augmented_jacobian1 = MatrixXd::Zero(10, active_count);
            augmented_jacobian1.topRows(8) = jacobian1;
            augmented_jacobian1.bottomRows(2) = body_state.jacobian_rows;
            jacobian1 = augmented_jacobian1;

            invariant_task_error = VectorXd::Zero(10);
            invariant_task_error.head(8) = invariant_error;
            invariant_task_error(8) = body_state.error(0);
            invariant_task_error(9) = body_state.error(1);
        }
        MatrixXd jacobian2 = jacobian_absolute_full.block(0, arm_start, 8, active_count);

        MatrixXd jacobian1_pinv = weighted_damped_pseudoinverse(jacobian1, weights, damping);
        if (jacobian1_pinv.size() == 0) {
            stop_reason = "jacobian_fail";
            break;
        }

        MatrixXd projection1 = nullspace_projection(jacobian1_pinv, jacobian1);
        MatrixXd jacobian2_bar = jacobian2 * projection1;
        MatrixXd jacobian2_bar_pinv = weighted_damped_pseudoinverse(jacobian2_bar, weights, damping);
        if (jacobian2_bar_pinv.size() == 0) {
            stop_reason = "jacobian_fail";
            break;
        }

        MatrixXd projection12 = projection1 - jacobian2_bar_pinv * jacobian2_bar;
        VectorXd gradient_limit = compute_joint_limit_gradient(q.segment(arm_start, active_count), active_limits);

        VectorXd task1_error_scaled = invariant_task_error;
        task1_error_scaled.head(8) *= kappa1;
        if (enable_body_vertical_task) {
            task1_error_scaled.tail(2) *= kappa_body_z;
        }

        VectorXd dq = jacobian1_pinv * task1_error_scaled;
        dq += projection1 * (jacobian2_bar_pinv * (kappa2 * command_error));

        if (options.joint_limit_avoidance && iter < kappa3_active_iters) {
            const double primary_error = std::sqrt(invariant_task_error.squaredNorm() + command_error.squaredNorm());
            const double kappa3_eff = kappa3 / (1.0 + 20.0 * primary_error);
            VectorXd dq3 = projection12 * (kappa3_eff * gradient_limit);
            const double dq3_inf = dq3.cwiseAbs().maxCoeff();
            if (dq3_inf > max_nullspace_step) {
                dq3 *= (max_nullspace_step / dq3_inf);
            }
            dq += dq3;
        }

        for (int i = 0; i < active_count; ++i) {
            const double step = std::clamp(dq(i) * weights(i), -max_step, max_step);
            q(arm_start + i) += step;
        }

        for (int i = 0; i < active_count; ++i) {
            const JointLimit& limit = active_limits[static_cast<std::size_t>(i)];
            q(arm_start + i) = std::clamp(q(arm_start + i), limit.lower, limit.upper);
        }
    }

    VectorXd relative_end = dq_to_vec8(model.relativePoseDualArm(q));
    VectorXd absolute_end = dq_to_vec8(model.absolutePoseDualArm(q));
    result.final_command_norm = signed_pose_error(absolute_target, absolute_end).norm();
    VectorXd invariant_end = signed_pose_error(relative_constraint, relative_end);
    if (enable_body_vertical_task) {
        const BodyVerticalTaskState body_state_end = compute_body_vertical_task_state(model, q, arm_start, active_count);
        if (!body_state_end.valid) {
            stop_reason = "body_task_fail";
            result.final_invariant_norm = std::numeric_limits<double>::infinity();
        } else {
            VectorXd invariant_end_augmented = VectorXd::Zero(10);
            invariant_end_augmented.head(8) = invariant_end;
            invariant_end_augmented(8) = kappa_body_z * body_state_end.error(0);
            invariant_end_augmented(9) = kappa_body_z * body_state_end.error(1);
            result.final_invariant_norm = invariant_end_augmented.norm();
        }
    } else {
        result.final_invariant_norm = invariant_end.norm();
    }

    const Outcome outcome = evaluate_outcome(
        stop_reason,
        initial_command_norm,
        result.final_command_norm,
        result.final_invariant_norm,
        hard_command_tol,
        hard_invariant_tol,
        practical_command_tol,
        practical_invariant_tol,
        mismatch_tol,
        mismatch_required_reduction
    );

    result.reason = outcome.reason;
    result.converged = outcome.converged;
    result.apply_result = outcome.apply_result;
    result.q = outcome.apply_result ? q : q_initial;
    return result;
}

SolveResult solve_relative(
    const DQ_HumanoidURDF& model,
    const std::vector<JointLimit>& all_limits,
    const VectorXd& q_initial,
    const VectorXd& relative_target_raw,
    const SolveOptions& options
) {
    SolveResult result;
    result.q = q_initial;

    VectorXd absolute_current = dq_to_vec8(model.absolutePoseDualArm(q_initial));
    VectorXd relative_current = dq_to_vec8(model.relativePoseDualArm(q_initial));

    VectorXd relative_target = relative_target_raw;
    if (relative_target.dot(relative_current) < 0.0) {
        relative_target = -relative_target;
    }

    const int arm_start = options.include_waist ? 0 : kWaistCount;
    const int active_count = options.include_waist ? (kWaistCount + kLeftArmCount + kRightArmCount) : (kLeftArmCount + kRightArmCount);
    const bool enable_body_vertical_task = options.keep_body_z_vertical && options.include_waist;
    const std::vector<JointLimit> active_limits = active_limits_from(all_limits, arm_start, active_count);
    const VectorXd weights = VectorXd::Ones(active_count);
    const double kappa1 = 0.2;
    const double kappa2 = 0.4;
    const double kappa3 = 0.0;
    const double kappa_body_z = 0.8;
    const double hard_command_tol = 1e-6;
    const double hard_invariant_tol = 5e-5;
    const double practical_command_tol = 1e-4;
    const double practical_invariant_tol = 2.5e-4;
    const double mismatch_tol = 1e-3;
    const double mismatch_required_reduction = 0.8;
    const double damping = 1e-5;
    const double max_step = 0.0003;
    const int max_iterations = 200;
    const double tolerance = 1e-16;
    const double body_z_tolerance = 1e-14;
    const double initial_command_norm = signed_pose_error(relative_target, relative_current).norm();

    VectorXd q = q_initial;
    std::string stop_reason = "max_iter";

    for (int iter = 0; iter < max_iterations; ++iter) {
        VectorXd absolute_now = dq_to_vec8(model.absolutePoseDualArm(q));
        VectorXd relative_now = dq_to_vec8(model.relativePoseDualArm(q));

        VectorXd invariant_error = signed_pose_error(absolute_current, absolute_now);
        VectorXd command_error = signed_pose_error(relative_target, relative_now);
        BodyVerticalTaskState body_state;
        double body_error_norm2 = 0.0;

        if (enable_body_vertical_task) {
            body_state = compute_body_vertical_task_state(model, q, arm_start, active_count);
            if (!body_state.valid) {
                stop_reason = "body_task_fail";
                break;
            }
            body_error_norm2 = body_state.error.squaredNorm();
        }

        if (
            invariant_error.squaredNorm() < tolerance &&
            command_error.squaredNorm() < tolerance &&
            (!enable_body_vertical_task || body_error_norm2 < body_z_tolerance)
        ) {
            stop_reason = "converged";
            break;
        }

        MatrixXd jacobian_absolute_full = model.absolutePoseJacobianDualArm(q);
        MatrixXd jacobian_relative_full = model.relativePoseJacobianDualArm(q);
        if (jacobian_absolute_full.cols() < arm_start + active_count || jacobian_relative_full.cols() < arm_start + active_count) {
            stop_reason = "jacobian_fail";
            break;
        }

        MatrixXd jacobian1 = jacobian_absolute_full.block(0, arm_start, 8, active_count);
        VectorXd invariant_task_error = invariant_error;
        if (enable_body_vertical_task) {
            MatrixXd augmented_jacobian1 = MatrixXd::Zero(10, active_count);
            augmented_jacobian1.topRows(8) = jacobian1;
            augmented_jacobian1.bottomRows(2) = body_state.jacobian_rows;
            jacobian1 = augmented_jacobian1;

            invariant_task_error = VectorXd::Zero(10);
            invariant_task_error.head(8) = invariant_error;
            invariant_task_error(8) = body_state.error(0);
            invariant_task_error(9) = body_state.error(1);
        }
        MatrixXd jacobian2 = jacobian_relative_full.block(0, arm_start, 8, active_count);

        MatrixXd jacobian1_pinv = weighted_damped_pseudoinverse(jacobian1, weights, damping);
        if (jacobian1_pinv.size() == 0) {
            stop_reason = "jacobian_fail";
            break;
        }

        MatrixXd projection1 = nullspace_projection(jacobian1_pinv, jacobian1);
        MatrixXd jacobian2_bar = jacobian2 * projection1;
        MatrixXd jacobian2_bar_pinv = weighted_damped_pseudoinverse(jacobian2_bar, weights, damping);
        if (jacobian2_bar_pinv.size() == 0) {
            stop_reason = "jacobian_fail";
            break;
        }

        MatrixXd projection12 = projection1 - jacobian2_bar_pinv * jacobian2_bar;
        VectorXd gradient_limit = compute_joint_limit_gradient(q.segment(arm_start, active_count), active_limits);

        VectorXd task1_error_scaled = invariant_task_error;
        task1_error_scaled.head(8) *= kappa1;
        if (enable_body_vertical_task) {
            task1_error_scaled.tail(2) *= kappa_body_z;
        }

        VectorXd dq = jacobian1_pinv * task1_error_scaled;
        dq += projection1 * (jacobian2_bar_pinv * (kappa2 * command_error));
        if (options.joint_limit_avoidance && kappa3 > 0.0) {
            dq += projection12 * (kappa3 * gradient_limit);
        }

        for (int i = 0; i < active_count; ++i) {
            const double step = std::clamp(dq(i) * weights(i), -max_step, max_step);
            q(arm_start + i) += step;
        }

        for (int i = 0; i < active_count; ++i) {
            const JointLimit& limit = active_limits[static_cast<std::size_t>(i)];
            q(arm_start + i) = std::clamp(q(arm_start + i), limit.lower, limit.upper);
        }
    }

    VectorXd absolute_end = dq_to_vec8(model.absolutePoseDualArm(q));
    VectorXd relative_end = dq_to_vec8(model.relativePoseDualArm(q));
    result.final_command_norm = signed_pose_error(relative_target, relative_end).norm();
    VectorXd invariant_end = signed_pose_error(absolute_current, absolute_end);
    if (enable_body_vertical_task) {
        const BodyVerticalTaskState body_state_end = compute_body_vertical_task_state(model, q, arm_start, active_count);
        if (!body_state_end.valid) {
            stop_reason = "body_task_fail";
            result.final_invariant_norm = std::numeric_limits<double>::infinity();
        } else {
            VectorXd invariant_end_augmented = VectorXd::Zero(10);
            invariant_end_augmented.head(8) = invariant_end;
            invariant_end_augmented(8) = kappa_body_z * body_state_end.error(0);
            invariant_end_augmented(9) = kappa_body_z * body_state_end.error(1);
            result.final_invariant_norm = invariant_end_augmented.norm();
        }
    } else {
        result.final_invariant_norm = invariant_end.norm();
    }

    const Outcome outcome = evaluate_outcome(
        stop_reason,
        initial_command_norm,
        result.final_command_norm,
        result.final_invariant_norm,
        hard_command_tol,
        hard_invariant_tol,
        practical_command_tol,
        practical_invariant_tol,
        mismatch_tol,
        mismatch_required_reduction
    );

    result.reason = outcome.reason;
    result.converged = outcome.converged;
    result.apply_result = outcome.apply_result;
    result.q = outcome.apply_result ? q : q_initial;
    return result;
}

ParsedArgs parse_args(int argc, char** argv) {
    ParsedArgs args;
    std::map<std::string, std::string> values;

    for (int i = 1; i < argc; ++i) {
        const std::string key(argv[i]);
        if (key == "--help" || key == "-h") {
            throw std::runtime_error(
                "Usage: sailor_nullspace_solver --mode {state|absolute|relative|retime} "
                "[--q <csv>] [--target <csv>] [--waypoints <row1;row2;...>] "
                "[--max-vel <scalar-or-csv>] [--max-acc <scalar-or-csv>] "
                "[--gridpoints-hint <int>] [--sample-count <int>] "
                "[--include-waist 0|1] [--keep-body-z-vertical 0|1] "
                "[--joint-limit-avoidance 0|1] [--urdf-path <path>]"
            );
        }
        if (key.rfind("--", 0) != 0) {
            throw std::runtime_error("Unexpected positional argument: " + key);
        }
        if (i + 1 >= argc) {
            throw std::runtime_error("Missing value for argument: " + key);
        }
        values[key.substr(2)] = argv[++i];
    }

    auto mode_iter = values.find("mode");
    auto q_iter = values.find("q");
    if (mode_iter == values.end()) {
        throw std::runtime_error("Missing required argument: --mode");
    }
    args.mode = mode_iter->second;

    if (args.mode != "retime" && q_iter == values.end()) {
        throw std::runtime_error("Missing required argument: --q");
    }

    if (q_iter != values.end()) {
        args.q = parse_csv_doubles(q_iter->second);
        if (!args.q.empty() && args.q.size() != kTotalJointCount) {
            throw std::runtime_error("Expected 20 joint values in --q");
        }
    }

    auto target_iter = values.find("target");
    if (args.mode == "absolute" || args.mode == "relative") {
        if (target_iter == values.end()) {
            throw std::runtime_error("Missing required argument: --target");
        }
        args.target = parse_csv_doubles(target_iter->second);
        if (args.target.size() != 8) {
            throw std::runtime_error("Expected 8 dual-quaternion values in --target");
        }
    }

    auto include_waist_iter = values.find("include-waist");
    if (include_waist_iter != values.end()) {
        args.include_waist = parse_bool_text(include_waist_iter->second);
    }

    auto keep_body_z_vertical_iter = values.find("keep-body-z-vertical");
    if (keep_body_z_vertical_iter != values.end()) {
        args.keep_body_z_vertical = parse_bool_text(keep_body_z_vertical_iter->second);
    }

    auto joint_limit_iter = values.find("joint-limit-avoidance");
    if (joint_limit_iter != values.end()) {
        args.joint_limit_avoidance = parse_bool_text(joint_limit_iter->second);
    }

    auto urdf_iter = values.find("urdf-path");
    if (urdf_iter != values.end()) {
        args.urdf_path = urdf_iter->second;
    }

    auto waypoints_iter = values.find("waypoints");
    if (args.mode == "retime") {
        if (waypoints_iter == values.end()) {
            throw std::runtime_error("Missing required argument: --waypoints");
        }
        args.waypoints = parse_matrix_doubles(waypoints_iter->second);
        auto max_vel_iter = values.find("max-vel");
        if (max_vel_iter != values.end()) {
            args.max_vel = parse_csv_doubles(max_vel_iter->second);
        }
        auto max_acc_iter = values.find("max-acc");
        if (max_acc_iter != values.end()) {
            args.max_acc = parse_csv_doubles(max_acc_iter->second);
        }
        auto gridpoints_iter = values.find("gridpoints-hint");
        if (gridpoints_iter != values.end()) {
            args.gridpoints_hint = std::max(0, std::stoi(gridpoints_iter->second));
        }
        auto sample_count_iter = values.find("sample-count");
        if (sample_count_iter != values.end()) {
            args.sample_count = std::max(0, std::stoi(sample_count_iter->second));
        }
    }
    return args;
}

void print_ok(const std::vector<std::pair<std::string, std::string>>& fields) {
    std::cout << "status=ok\n";
    for (const auto& field : fields) {
        std::cout << field.first << '=' << field.second << '\n';
    }
}

void print_error(const std::string& message) {
    std::cout << "status=error\n";
    std::cout << "message=" << message << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const ParsedArgs args = parse_args(argc, argv);

        if (args.mode == "retime") {
            const RetimeResult result = retime_waypoints(
                args.waypoints,
                args.max_vel,
                args.max_acc,
                args.gridpoints_hint,
                args.sample_count
            );
            print_ok({
                {"duration", std::to_string(result.duration)},
                {"waypoint_times", join_csv(result.waypoint_times)},
                {"sample_times", join_csv(result.sample_times)},
                {"positions", join_matrix(result.positions)},
                {"velocities", join_matrix(result.velocities)},
                {"accelerations", join_matrix(result.accelerations)},
                {"gridpoints", join_csv(result.gridpoints)},
                {"grid_times", join_csv(result.grid_times)},
                {"path_velocity_squared", join_csv(result.path_velocity_squared)},
            });
            return EXIT_SUCCESS;
        }

        const std::vector<JointLimit> joint_limits = load_joint_limits(args.urdf_path);
        const std::unique_ptr<DQ_HumanoidURDF> model = load_humanoid_model(args.urdf_path);
        const VectorXd q = vector_from_std(args.q);

        if (args.mode == "state") {
            const VectorXd absolute_pose = dq_to_vec8(model->absolutePoseDualArm(q));
            const VectorXd relative_pose = dq_to_vec8(model->relativePoseDualArm(q));
            const VectorXd left_pose = dq_to_vec8(model->fkm(q, DQ_HumanoidURDF::EffectorLeftArm));
            const VectorXd right_pose = dq_to_vec8(model->fkm(q, DQ_HumanoidURDF::EffectorRightArm));

            print_ok({
                {"absolute_pose", join_csv(absolute_pose)},
                {"relative_pose", join_csv(relative_pose)},
                {"left_pose", join_csv(left_pose)},
                {"right_pose", join_csv(right_pose)},
            });
            return EXIT_SUCCESS;
        }

        const VectorXd target = vector_from_std(args.target);
        const SolveOptions options{args.include_waist, args.keep_body_z_vertical, args.joint_limit_avoidance};
        SolveResult result;

        if (args.mode == "absolute") {
            result = solve_absolute(*model, joint_limits, q, target, options);
        } else if (args.mode == "relative") {
            result = solve_relative(*model, joint_limits, q, target, options);
        } else {
            throw std::runtime_error("Unsupported mode: " + args.mode);
        }

        print_ok({
            {"q", join_csv(result.q)},
            {"reason", result.reason},
            {"converged", result.converged ? "true" : "false"},
            {"apply_result", result.apply_result ? "true" : "false"},
            {"final_command_norm", std::to_string(result.final_command_norm)},
            {"final_invariant_norm", std::to_string(result.final_invariant_norm)},
        });
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        print_error(error.what());
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
}

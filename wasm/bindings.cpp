/**
 * DQ Robotics WebAssembly Bindings
 * 
 * 这个文件使用 Embind 将 DQ Robotics C++ 类绑定到 JavaScript
 */

#include <emscripten/bind.h>
#include <dqrobotics/DQ.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorURDF.h>
#include <dqrobotics/robot_modeling/DQ_HumanoidURDF.h>
#include <dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h>
#include <dqrobotics/robot_modeling/DQ_JointType.h>
#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robots/KukaLw4Robot.h>
#include <Eigen/Dense>
#include "mj_restore_humanoid_solver.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#if TOPPRA_AVAILABLE
#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#endif

using namespace emscripten;
using namespace DQ_robotics;
using namespace Eigen;

// 辅助函数：将 JavaScript 数组转换为 VectorXd
VectorXd jsArrayToVectorXd(const val& jsArray) {
    int length = jsArray["length"].as<int>();
    VectorXd vec(length);
    for (int i = 0; i < length; i++) {
        vec(i) = jsArray[i].as<double>();
    }
    return vec;
}

// 辅助函数：将 VectorXd 转换为 JavaScript 数组
val vectorXdToJsArray(const VectorXd& vec) {
    val jsArray = val::array();
    for (int i = 0; i < vec.size(); i++) {
        jsArray.call<void>("push", vec(i));
    }
    return jsArray;
}

// 辅助函数：将 MatrixXd 转换为 JavaScript 二维数组
val matrixXdToJsArray(const MatrixXd& mat) {
    val jsArray = val::array();
    for (int i = 0; i < mat.rows(); i++) {
        val row = val::array();
        for (int j = 0; j < mat.cols(); j++) {
            row.call<void>("push", mat(i, j));
        }
        jsArray.call<void>("push", row);
    }
    return jsArray;
}

// 辅助函数：将 JavaScript 二维数组转换为 MatrixXd
MatrixXd jsArrayToMatrixXd(const val& jsArray2d) {
    int rows = jsArray2d["length"].as<int>();
    if (rows <= 0) {
        return MatrixXd(0, 0);
    }
    val first_row = jsArray2d[0];
    int cols = first_row["length"].as<int>();
    MatrixXd mat(rows, cols);
    for (int i = 0; i < rows; i++) {
        val row = jsArray2d[i];
        for (int j = 0; j < cols; j++) {
            mat(i, j) = row[j].as<double>();
        }
    }
    return mat;
}

val stdVectorToJsArray(const std::vector<double>& values) {
    val jsArray = val::array();
    for (double v : values) {
        jsArray.call<void>("push", v);
    }
    return jsArray;
}

std::vector<double> jsArrayToStdVectorDouble(const val& jsArray) {
    if (jsArray.isUndefined() || jsArray.isNull()) {
        return {};
    }
    const int length = jsArray["length"].as<int>();
    std::vector<double> values(static_cast<std::size_t>(length), 0.0);
    for (int i = 0; i < length; ++i) {
        values[static_cast<std::size_t>(i)] = jsArray[i].as<double>();
    }
    return values;
}

std::vector<int> jsArrayToStdVectorInt(const val& jsArray) {
    if (jsArray.isUndefined() || jsArray.isNull()) {
        return {};
    }
    const int length = jsArray["length"].as<int>();
    std::vector<int> values(static_cast<std::size_t>(length), 0);
    for (int i = 0; i < length; ++i) {
        values[static_cast<std::size_t>(i)] = jsArray[i].as<int>();
    }
    return values;
}

std::vector<std::vector<double>> jsArrayToNestedStdVectorDouble(const val& jsArray2d) {
    if (jsArray2d.isUndefined() || jsArray2d.isNull()) {
        return {};
    }
    const int rows = jsArray2d["length"].as<int>();
    std::vector<std::vector<double>> values(static_cast<std::size_t>(rows));
    for (int i = 0; i < rows; ++i) {
        values[static_cast<std::size_t>(i)] = jsArrayToStdVectorDouble(jsArray2d[i]);
    }
    return values;
}

double getOptionalNumberField(const val& object, const char* key, double fallback) {
    const val field = object[key];
    if (field.isUndefined() || field.isNull()) {
        return fallback;
    }
    return field.as<double>();
}

int getOptionalIntField(const val& object, const char* key, int fallback) {
    const val field = object[key];
    if (field.isUndefined() || field.isNull()) {
        return fallback;
    }
    return field.as<int>();
}

class TOPPRA_Wrapper {
public:
    static val retimeWaypoints(
        const val& waypoints_js,
        const val& max_vel_js,
        const val& max_acc_js,
        int gridpoints_hint = 0
    ) {
        val out = val::object();

#if TOPPRA_AVAILABLE
        try {
            const int num_waypoints = waypoints_js["length"].as<int>();
            if (num_waypoints < 2) {
                throw std::runtime_error("waypoints 至少需要 2 个点");
            }

            const val first = waypoints_js[0];
            const int dof = first["length"].as<int>();
            if (dof <= 0) {
                throw std::runtime_error("waypoints 维度无效");
            }

            toppra::Vectors positions;
            positions.reserve(static_cast<std::size_t>(num_waypoints));
            for (int i = 0; i < num_waypoints; ++i) {
                val row = waypoints_js[i];
                const int row_len = row["length"].as<int>();
                if (row_len != dof) {
                    throw std::runtime_error("waypoints 每个点维度必须一致");
                }
                toppra::Vector q(dof);
                for (int j = 0; j < dof; ++j) {
                    const double v = row[j].as<double>();
                    if (!std::isfinite(v)) {
                        throw std::runtime_error("waypoints 含非法数值");
                    }
                    q(j) = v;
                }
                positions.push_back(q);
            }

            auto parse_limit_vector = [&](const val& js, double fallback, const char* name) {
                toppra::Vector limit(dof);
                limit.setConstant(std::abs(fallback));

                if (js.isUndefined() || js.isNull()) {
                    return limit;
                }

                bool parsed_as_scalar = false;
                try {
                    std::string type = js.typeOf().as<std::string>();
                    if (type == "number") {
                        const double s = js.as<double>();
                        if (!std::isfinite(s) || s <= 0) {
                            throw std::runtime_error(std::string(name) + " 标量必须为正数");
                        }
                        limit.setConstant(std::abs(s));
                        parsed_as_scalar = true;
                    }
                } catch (...) {
                    parsed_as_scalar = false;
                }
                if (parsed_as_scalar) return limit;

                const int len = js["length"].as<int>();
                if (len != dof) {
                    throw std::runtime_error(std::string(name) + " 维度需与 waypoint 维度一致");
                }

                for (int i = 0; i < dof; ++i) {
                    const double v = js[i].as<double>();
                    if (!std::isfinite(v) || v <= 0) {
                        throw std::runtime_error(std::string(name) + " 每个维度都必须是正数");
                    }
                    limit(i) = std::abs(v);
                }
                return limit;
            };

            const toppra::Vector vel_abs = parse_limit_vector(max_vel_js, 0.45, "max_vel");
            const toppra::Vector acc_abs = parse_limit_vector(max_acc_js, 0.90, "max_acc");
            const toppra::Vector vel_lower = -vel_abs;
            const toppra::Vector vel_upper = vel_abs;
            const toppra::Vector acc_lower = -acc_abs;
            const toppra::Vector acc_upper = acc_abs;

            toppra::Vector knots = toppra::Vector::LinSpaced(
                num_waypoints,
                0.0,
                static_cast<double>(num_waypoints - 1)
            );
            toppra::BoundaryCondFull bc_type = {
                toppra::BoundaryCond("clamped"),
                toppra::BoundaryCond("clamped"),
            };
            toppra::PiecewisePolyPath path_poly = toppra::PiecewisePolyPath::CubicSpline(
                positions,
                knots,
                bc_type
            );
            toppra::GeometricPathPtr path = std::make_shared<toppra::PiecewisePolyPath>(path_poly);

            toppra::LinearConstraintPtr c_vel =
                std::make_shared<toppra::constraint::LinearJointVelocity>(vel_lower, vel_upper);
            toppra::LinearConstraintPtr c_acc =
                std::make_shared<toppra::constraint::LinearJointAcceleration>(acc_lower, acc_upper);
            c_acc->discretizationType(toppra::DiscretizationType::Interpolation);

            toppra::LinearConstraintPtrs constraints{c_vel, c_acc};
            auto algo = std::make_shared<toppra::algorithm::TOPPRA>(constraints, path);

            const int min_n = std::max(1, num_waypoints - 1);
            const int suggested_n = std::max(min_n, (num_waypoints - 1) * 5);
            const int final_n = gridpoints_hint > 0 ? std::max(gridpoints_hint, min_n) : suggested_n;
            algo->setN(final_n);

            const toppra::ReturnCode rc = algo->computePathParametrization(0.0, 0.0);
            if (rc != toppra::ReturnCode::OK) {
                throw std::runtime_error("TOPPRA 失败: " + algo->getErrorMessage());
            }

            const auto data = algo->getParameterizationData();
            const toppra::Vector s_grid = data.gridpoints;
            const toppra::Vector vsq = data.parametrization.cwiseMax(0.0);
            const toppra::Vector v_grid = vsq.array().sqrt();

            std::vector<double> t_grid(static_cast<std::size_t>(s_grid.size()), 0.0);
            for (int i = 0; i < s_grid.size() - 1; ++i) {
                const double ds = s_grid(i + 1) - s_grid(i);
                const double denom = v_grid(i) + v_grid(i + 1);
                double dt = 0.0;
                if (denom > 1e-9 && ds > 0) {
                    dt = 2.0 * ds / denom;
                }
                if (!std::isfinite(dt) || dt < 0) dt = 0.0;
                t_grid[static_cast<std::size_t>(i + 1)] = t_grid[static_cast<std::size_t>(i)] + dt;
            }

            auto sample_time_from_s = [&](double s_query) {
                if (s_query <= s_grid(0)) return t_grid.front();
                if (s_query >= s_grid(s_grid.size() - 1)) return t_grid.back();

                int lo = 0;
                int hi = s_grid.size() - 2;
                while (lo <= hi) {
                    int mid = lo + (hi - lo) / 2;
                    if (s_grid(mid + 1) < s_query) lo = mid + 1;
                    else hi = mid - 1;
                }
                const int max_idx = static_cast<int>(s_grid.size()) - 2;
                const int idx = std::max(0, std::min(max_idx, lo));
                const double s0 = s_grid(idx);
                const double s1 = s_grid(idx + 1);
                const double t0 = t_grid[static_cast<std::size_t>(idx)];
                const double t1 = t_grid[static_cast<std::size_t>(idx + 1)];
                if (s1 - s0 <= 1e-12) return t0;
                const double r = std::max(0.0, std::min(1.0, (s_query - s0) / (s1 - s0)));
                return t0 + (t1 - t0) * r;
            };

            std::vector<double> waypoint_times(static_cast<std::size_t>(num_waypoints), 0.0);
            for (int i = 0; i < num_waypoints; ++i) {
                waypoint_times[static_cast<std::size_t>(i)] = sample_time_from_s(knots(i));
                if (i > 0 && waypoint_times[static_cast<std::size_t>(i)] < waypoint_times[static_cast<std::size_t>(i - 1)]) {
                    waypoint_times[static_cast<std::size_t>(i)] = waypoint_times[static_cast<std::size_t>(i - 1)];
                }
            }

            out.set("ok", true);
            out.set("message", std::string("ok"));
            out.set("dof", dof);
            out.set("numWaypoints", num_waypoints);
            out.set("duration", waypoint_times.back());
            out.set("waypointTimes", stdVectorToJsArray(waypoint_times));
            out.set("gridpoints", vectorXdToJsArray(s_grid));
            out.set("gridTimes", stdVectorToJsArray(t_grid));
            out.set("pathVelocitySquared", vectorXdToJsArray(vsq));
            out.set("solverCode", static_cast<int>(data.ret_code));
            return out;
        } catch (const std::exception& e) {
            out.set("ok", false);
            out.set("message", std::string(e.what()));
            return out;
        }
#else
        (void)waypoints_js;
        (void)max_vel_js;
        (void)max_acc_js;
        (void)gridpoints_hint;
        out.set("ok", false);
        out.set("message", std::string("TOPPRA 不可用：WASM 构建未启用 TOPPRA"));
        return out;
#endif
    }
};

// DQ 类的包装器，提供更友好的 JavaScript 接口
class DQ_Wrapper {
public:
    static DQ create(double q0 = 0, double q1 = 0, double q2 = 0, double q3 = 0,
                    double q4 = 0, double q5 = 0, double q6 = 0, double q7 = 0) {
        return DQ(q0, q1, q2, q3, q4, q5, q6, q7);
    }
    
    static DQ createFromArray(const val& jsArray) {
        VectorXd vec = jsArrayToVectorXd(jsArray);
        return DQ(vec);
    }
    
    static val toArray(const DQ& dq) {
        return vectorXdToJsArray(dq.q);
    }
    
    static DQ multiply(const DQ& dq1, const DQ& dq2) {
        return dq1 * dq2;
    }
    
    static DQ add(const DQ& dq1, const DQ& dq2) {
        return dq1 + dq2;
    }
    
    static DQ conjugate(const DQ& dq) {
        return dq.conj();
    }
    
    static DQ normalize(const DQ& dq) {
        return dq.normalize();
    }
    
    static DQ inverse(const DQ& dq) {
        return dq.inv();
    }
    
    static DQ rotation(double angle, double x, double y, double z) {
        // 归一化旋转轴
        double norm = std::sqrt(x*x + y*y + z*z);
        if (norm > 1e-10) {
            x /= norm; y /= norm; z /= norm;
        }
        // 创建单位四元数表示旋转
        double half_angle = angle / 2.0;
        double s = std::sin(half_angle);
        double c = std::cos(half_angle);
        return DQ(c, s*x, s*y, s*z);
    }
    
    static DQ translation(double x, double y, double z) {
        return DQ(1) + 0.5 * DQ::E * DQ(0, x, y, z);
    }
    
    static DQ pose(double rx, double ry, double rz, double angle, 
                   double tx, double ty, double tz) {
        // 旋转
        double norm = std::sqrt(rx*rx + ry*ry + rz*rz);
        if (norm > 1e-10) {
            rx /= norm; ry /= norm; rz /= norm;
        }
        double half_angle = angle / 2.0;
        double s = std::sin(half_angle);
        double c = std::cos(half_angle);
        DQ r = DQ(c, s*rx, s*ry, s*rz);
        
        // 平移
        DQ t = DQ(1) + 0.5 * DQ::E * DQ(0, tx, ty, tz);
        return r * t;
    }
    
    static DQ fromQuaternionAndTranslation(double qw, double qx, double qy, double qz,
                                            double tx, double ty, double tz) {
        // 从四元数和平移创建对偶四元数，与 DQ 库约定一致：dq = r + ε·(1/2)·p·r
        // 这样 translation(dq) 才能正确得到 p，与 unitDQ / pose 一致
        double quatNorm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (quatNorm < 1e-10) {
            throw std::runtime_error("四元数不能为零");
        }
        qw /= quatNorm;
        qx /= quatNorm;
        qy /= quatNorm;
        qz /= quatNorm;
        
        DQ r = DQ(qw, qx, qy, qz);
        DQ p = DQ(0, tx, ty, tz);
        return r + DQ::E * 0.5 * p * r;
    }
    
    static val getTranslation(const DQ& dq) {
        DQ t = ::translation(dq);  // 使用全局函数
        val result = val::array();
        result.call<void>("push", 0.0);  // w 分量
        result.call<void>("push", t.q(1));  // x
        result.call<void>("push", t.q(2));  // y
        result.call<void>("push", t.q(3));  // z
        return result;
    }
    
    static val getRotation(const DQ& dq) {
        DQ r = ::rotation(dq);  // 使用全局函数
        return vectorXdToJsArray(r.q);
    }
    
    static val getRotationMatrix(const DQ& dq) {
        // 从旋转四元数计算旋转矩阵
        DQ r = ::rotation(dq);
        double w = r.q(0), x = r.q(1), y = r.q(2), z = r.q(3);
        
        MatrixXd mat(3, 3);
        mat(0,0) = 1 - 2*(y*y + z*z);
        mat(0,1) = 2*(x*y - w*z);
        mat(0,2) = 2*(x*z + w*y);
        mat(1,0) = 2*(x*y + w*z);
        mat(1,1) = 1 - 2*(x*x + z*z);
        mat(1,2) = 2*(y*z - w*x);
        mat(2,0) = 2*(x*z - w*y);
        mat(2,1) = 2*(y*z + w*x);
        mat(2,2) = 1 - 2*(x*x + y*y);
        
        return matrixXdToJsArray(mat);
    }
};

// 运动学工具包装器（静态方法）
class Kinematics_Wrapper {
public:
    /**
     * 距离雅可比（对应 DQ_Kinematics::distance_jacobian）
     * 给定 vec8(pose_dot)=J_pose*q_dot，返回满足 dot(d^2)=Jd*q_dot 的 Jd（1xN）
     *
     * @param pose_jacobian_js 8xN 的位姿雅可比（二维数组）
     * @param pose_dq_js 8 元素位姿双四元数（数组）
     * @return 1xN 的距离雅可比（二维数组，只有一行）
     */
    static val distance_jacobian(const val& pose_jacobian_js, const val& pose_dq_js) {
        MatrixXd J_pose = jsArrayToMatrixXd(pose_jacobian_js);
        VectorXd pose_vec = jsArrayToVectorXd(pose_dq_js);
        if (pose_vec.size() != 8) {
            throw std::runtime_error("pose_dq must be an 8-element array");
        }
        DQ pose(pose_vec);
        MatrixXd Jd = DQ_Kinematics::distance_jacobian(J_pose, pose);
        return matrixXdToJsArray(Jd);
    }

    /**
     * 线雅可比（对应 DQ_Kinematics::line_jacobian）
     * 给定 vec8(pose_dot)=J_pose*q_dot，返回满足 vec8(line_dot)=Jline*q_dot 的 Jline（8xN）
     *
     * @param pose_jacobian_js 8xN 的位姿雅可比（二维数组）
     * @param pose_dq_js 8 元素位姿双四元数（数组）
     * @param line_direction_js 8 元素线方向（数组）。例如 body z 轴使用 [0,0,0,1,0,0,0,0]
     * @return 8xN 的线雅可比（二维数组）
     */
    static val line_jacobian(const val& pose_jacobian_js, const val& pose_dq_js, const val& line_direction_js) {
        MatrixXd J_pose = jsArrayToMatrixXd(pose_jacobian_js);

        VectorXd pose_vec = jsArrayToVectorXd(pose_dq_js);
        if (pose_vec.size() != 8) {
            throw std::runtime_error("pose_dq must be an 8-element array");
        }
        DQ pose(pose_vec);

        VectorXd line_direction_vec = jsArrayToVectorXd(line_direction_js);
        if (line_direction_vec.size() != 8) {
            throw std::runtime_error("line_direction must be an 8-element array");
        }
        DQ line_direction(line_direction_vec);

        MatrixXd Jline = DQ_Kinematics::line_jacobian(J_pose, pose, line_direction);
        return matrixXdToJsArray(Jline);
    }
};

// 串联机械臂包装器（使用 Modified DH）
class SerialManipulator_Wrapper {
private:
    DQ_SerialManipulatorMDH robot;
    
    static MatrixXd extractDHMatrix(const val& dh_matrix_js) {
        // JavaScript 传入的格式: [[θ1,θ2,...], [d1,d2,...], [a1,a2,...], [α1,α2,...], [type1,type2,...]]
        // 即: 5行 x n列
        int num_rows = dh_matrix_js["length"].as<int>();
        if (num_rows != 5) {
            throw std::runtime_error("DH matrix should have 5 rows");
        }
        
        val first_row = dh_matrix_js[0];
        int num_joints = first_row["length"].as<int>();
        
        MatrixXd dh_matrix(5, num_joints);
        for (int row = 0; row < 5; row++) {
            val js_row = dh_matrix_js[row];
            for (int col = 0; col < num_joints; col++) {
                dh_matrix(row, col) = js_row[col].as<double>();
            }
        }
        return dh_matrix;
    }
    
public:
    SerialManipulator_Wrapper(const val& dh_matrix_js) 
        : robot(extractDHMatrix(dh_matrix_js)) {
    }
    
    val fkm(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        return vectorXdToJsArray(pose.q);
    }
    
    val pose_jacobian(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        MatrixXd J = robot.pose_jacobian(joint_angles);
        return matrixXdToJsArray(J);
    }
    
    int get_dim_configuration_space() const {
        return robot.get_dim_configuration_space();
    }
    
    // 设置基座偏移
    void set_base_frame(double x, double y, double z) {
        DQ base = 1 + E_ * 0.5 * DQ(0, x, y, z);
        base = base.normalize(); // 归一化（与Python逻辑一致）
        robot.set_base_frame(base);
        robot.set_reference_frame(base);
    }
    
    // 设置基座完整姿态（旋转 + 平移）
    void set_base_frame_pose(double qw, double qx, double qy, double qz,
                             double tx, double ty, double tz) {
        // 先归一化旋转四元数（只归一化旋转部分，不改变平移）
        double q_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (q_norm > 1e-10) {
            qw /= q_norm;
            qx /= q_norm;
            qy /= q_norm;
            qz /= q_norm;
        }
        // 直接构造双四元数：前4个是旋转四元数，后4个是1/2*平移*旋转四元数
        // DQ格式：[qw, qx, qy, qz, qw_dual, qx_dual, qy_dual, qz_dual]
        // 其中对偶部分 = 0.5 * [0, tx, ty, tz] * [qw, qx, qy, qz]
        // 对偶四元数乘法：(0, tx, ty, tz) * (qw, qx, qy, qz) = (0, tx, ty, tz) * 四元数乘法
        // 纯四元数 * 四元数：(0, tx, ty, tz) * (qw, qx, qy, qz)
        // = (0*qw - (tx,ty,tz)·(qx,qy,qz), 0*(qx,qy,qz) + qw*(tx,ty,tz) + (tx,ty,tz)×(qx,qy,qz))
        // = (-tx*qx - ty*qy - tz*qz, qw*tx + ty*qz - tz*qy, qw*ty + tz*qx - tx*qz, qw*tz + tx*qy - ty*qx)
        double dual_w = -0.5 * (tx*qx + ty*qy + tz*qz);
        double dual_x = 0.5 * (qw*tx + ty*qz - tz*qy);
        double dual_y = 0.5 * (qw*ty + tz*qx - tx*qz);
        double dual_z = 0.5 * (qw*tz + tx*qy - ty*qx);
        
        // 直接构造8元素双四元数
        DQ base(qw, qx, qy, qz, dual_w, dual_x, dual_y, dual_z);
        robot.set_base_frame(base);
        robot.set_reference_frame(base);
    }
    
    // 设置基座（直接使用双四元数，8元素数组）
    void set_base_frame_dq(const val& dq_array) {
        VectorXd dq_vec = jsArrayToVectorXd(dq_array);
        if (dq_vec.size() != 8) {
            throw std::runtime_error("双四元数必须是8元素数组");
        }
        DQ base(dq_vec);
        
        // 打印原始双四元数（用于调试）
        printf("原始双四元数: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               dq_vec(0), dq_vec(1), dq_vec(2), dq_vec(3),
               dq_vec(4), dq_vec(5), dq_vec(6), dq_vec(7));
        
        // 归一化（与Python代码保持一致）
        base = base.normalize();
        
        // 打印归一化后的双四元数（用于调试）
        printf("归一化后: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               base.q(0), base.q(1), base.q(2), base.q(3),
               base.q(4), base.q(5), base.q(6), base.q(7));
        
        robot.set_base_frame(base);
        robot.set_reference_frame(base);
    }
    
    // 设置末端偏移
    void set_effector(double x, double y, double z) {
        DQ eff = 1 + E_ * 0.5 * DQ(0, x, y, z);
        robot.set_effector(eff);
    }

    // 设置末端完整姿态（旋转 + 平移）
    void set_effector_pose(double qw, double qx, double qy, double qz,
                           double tx, double ty, double tz) {
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        DQ eff = r * t; // 先旋转再平移（工具坐标）
        robot.set_effector(eff);
    }
};

// URDF 串联机械臂包装器：从 JS 传入的关节链创建 (joint_type, axis, origin 每关节)
// chain_js: 数组，每项为 { joint_type: 0|1, axis: [x,y,z], origin: [q0..q7] }
static std::vector<URDFJointData> extractURDFChain(const val& chain_js) {
    int n = chain_js["length"].as<int>();
    if (n <= 0)
        throw std::runtime_error("URDF chain array cannot be empty");
    std::vector<URDFJointData> chain;
    chain.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; i++) {
        val joint_js = chain_js[i];
        int jtype = joint_js["joint_type"].as<int>();
        if (jtype != 0 && jtype != 1)
            throw std::runtime_error("joint_type must be 0 (REVOLUTE) or 1 (PRISMATIC)");
        URDFJointData j;
        j.joint_type = DQ_JointType(jtype);
        val axis_js = joint_js["axis"];
        if (axis_js["length"].as<int>() != 3)
            throw std::runtime_error("axis must be [x, y, z]");
        j.axis(0) = axis_js[0].as<double>();
        j.axis(1) = axis_js[1].as<double>();
        j.axis(2) = axis_js[2].as<double>();
        val origin_js = joint_js["origin"];
        if (origin_js["length"].as<int>() != 8)
            throw std::runtime_error("origin must be DQ array of 8 elements");
        j.origin = DQ(jsArrayToVectorXd(origin_js));
        chain.push_back(j);
    }
    return chain;
}

class SerialManipulatorURDF_Wrapper {
private:
    DQ_SerialManipulatorURDF robot;

public:
    SerialManipulatorURDF_Wrapper(const val& chain_js)
        : robot(extractURDFChain(chain_js)) {
    }

    val fkm(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        return vectorXdToJsArray(pose.q);
    }

    val pose_jacobian(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        MatrixXd J = robot.pose_jacobian(joint_angles);
        return matrixXdToJsArray(J);
    }

    int get_dim_configuration_space() const {
        return robot.get_dim_configuration_space();
    }

    void set_base_frame(double x, double y, double z) {
        DQ base = 1 + E_ * 0.5 * DQ(0, x, y, z);
        base = base.normalize();
        robot.set_base_frame(base);
        robot.set_reference_frame(base);
    }

    void set_base_frame_pose(double qw, double qx, double qy, double qz,
                             double tx, double ty, double tz) {
        double q_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (q_norm > 1e-10) {
            qw /= q_norm; qx /= q_norm; qy /= q_norm; qz /= q_norm;
        }
        double dual_w = -0.5 * (tx*qx + ty*qy + tz*qz);
        double dual_x = 0.5 * (qw*tx + ty*qz - tz*qy);
        double dual_y = 0.5 * (qw*ty + tz*qx - tx*qz);
        double dual_z = 0.5 * (qw*tz + tx*qy - ty*qx);
        DQ base(qw, qx, qy, qz, dual_w, dual_x, dual_y, dual_z);
        robot.set_base_frame(base);
        robot.set_reference_frame(base);
    }

    void set_base_frame_dq(const val& dq_array) {
        VectorXd dq_vec = jsArrayToVectorXd(dq_array);
        if (dq_vec.size() != 8)
            throw std::runtime_error("双四元数必须是8元素数组");
        DQ base(dq_vec);
        base = base.normalize();
        robot.set_base_frame(base);
        robot.set_reference_frame(base);
    }

    void set_effector(double x, double y, double z) {
        DQ eff = 1 + E_ * 0.5 * DQ(0, x, y, z);
        robot.set_effector(eff);
    }

    void set_effector_pose(double qw, double qx, double qy, double qz,
                          double tx, double ty, double tz) {
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        DQ eff = r * t;
        robot.set_effector(eff);
    }
};

// 人形 URDF 包装器：底盘 + 腰 + 左臂 + 右臂 + 头；链由 JS 从 URDF 解析后传入
class HumanoidURDF_Wrapper {
private:
    DQ_HumanoidURDF robot;

    static DQ extractDQOrIdentity(const val& dq_js) {
        if (dq_js.isUndefined() || dq_js.isNull()) return DQ(1);
        return DQ(jsArrayToVectorXd(dq_js));
    }

    static HumanoidAbsoluteMJRestoreRequest parseAbsoluteTargetMJRestoreRequest(const val& request_js) {
        HumanoidAbsoluteMJRestoreRequest request;
        request.solve_variant =
            request_js["solveVariant"].isUndefined()
                ? request.solve_variant
                : request_js["solveVariant"].as<std::string>();
        request.q_current = jsArrayToStdVectorDouble(request_js["qCurrent"]);
        request.active_start = getOptionalIntField(request_js, "activeStart", request.active_start);
        request.active_count = getOptionalIntField(request_js, "activeCount", request.active_count);
        request.absolute_target = jsArrayToStdVectorDouble(request_js["absoluteTarget"]);
        request.relative_target = jsArrayToStdVectorDouble(request_js["relativeTarget"]);
        request.left_target = jsArrayToStdVectorDouble(request_js["leftTarget"]);
        request.right_target = jsArrayToStdVectorDouble(request_js["rightTarget"]);
        request.keep_body_z_vertical =
            request_js["keepBodyZVertical"].isUndefined()
                ? request.keep_body_z_vertical
                : request_js["keepBodyZVertical"].as<bool>();
        request.body_equality_tolerance = getOptionalNumberField(
            request_js,
            "bodyEqualityTolerance",
            request.body_equality_tolerance
        );
        request.absolute_weight = getOptionalNumberField(
            request_js,
            "absoluteWeight",
            request.absolute_weight
        );
        request.relative_weight = getOptionalNumberField(
            request_js,
            "relativeWeight",
            request.relative_weight
        );
        request.body_weight = getOptionalNumberField(
            request_js,
            "bodyWeight",
            request.body_weight
        );
        request.lower_bounds = jsArrayToStdVectorDouble(request_js["lowerBounds"]);
        request.upper_bounds = jsArrayToStdVectorDouble(request_js["upperBounds"]);
        request.step_bounds = jsArrayToStdVectorDouble(request_js["stepBounds"]);
        request.regularization_weights = jsArrayToStdVectorDouble(request_js["regularizationWeights"]);
        request.wvfi_lower_bound = getOptionalNumberField(
            request_js,
            "wvfiLowerBound",
            request.wvfi_lower_bound
        );
        request.joint_limit_activation_threshold = getOptionalNumberField(
            request_js,
            "jointLimitActivationThreshold",
            request.joint_limit_activation_threshold
        );
        const val joint_limit_vfi_js = request_js["jointLimitVFI"];
        request.joint_limit_vfi.enabled =
            joint_limit_vfi_js.isUndefined()
                ? request.joint_limit_vfi.enabled
                : joint_limit_vfi_js["enabled"].as<bool>();
        request.joint_limit_vfi.rows = jsArrayToNestedStdVectorDouble(joint_limit_vfi_js["rows"]);
        request.joint_limit_vfi.rhs = jsArrayToStdVectorDouble(joint_limit_vfi_js["rhs"]);

        const val waist_corridor_js = request_js["waistXCorridor"];
        request.waist_x_corridor.enabled =
            waist_corridor_js.isUndefined()
                ? request.waist_x_corridor.enabled
                : waist_corridor_js["enabled"].as<bool>();
        request.waist_x_corridor.min = getOptionalNumberField(
            waist_corridor_js,
            "min",
            request.waist_x_corridor.min
        );
        request.waist_x_corridor.max = getOptionalNumberField(
            waist_corridor_js,
            "max",
            request.waist_x_corridor.max
        );
        request.waist_x_corridor.tolerance = getOptionalNumberField(
            waist_corridor_js,
            "tolerance",
            request.waist_x_corridor.tolerance
        );
        request.waist_x_corridor.activation_threshold = getOptionalNumberField(
            waist_corridor_js,
            "activationThreshold",
            request.waist_x_corridor.activation_threshold
        );
        request.waist_x_corridor.rows = jsArrayToNestedStdVectorDouble(waist_corridor_js["rows"]);
        request.waist_x_corridor.rhs = jsArrayToStdVectorDouble(waist_corridor_js["rhs"]);

        const val safety_push_js = request_js["safetyPush"];
        if (!safety_push_js.isUndefined()) {
            request.safety_push.enabled =
                safety_push_js["enabled"].isUndefined()
                    ? request.safety_push.enabled
                    : safety_push_js["enabled"].as<bool>();
            request.safety_push.weight = getOptionalNumberField(
                safety_push_js,
                "weight",
                request.safety_push.weight
            );
            request.safety_push.activation_residual = getOptionalNumberField(
                safety_push_js,
                "activationResidual",
                request.safety_push.activation_residual
            );
            request.safety_push.pose_hold_tolerance = getOptionalNumberField(
                safety_push_js,
                "poseHoldTolerance",
                request.safety_push.pose_hold_tolerance
            );
            request.safety_push.target_fraction = getOptionalNumberField(
                safety_push_js,
                "targetFraction",
                request.safety_push.target_fraction
            );
            request.safety_push.max_distance_step = getOptionalNumberField(
                safety_push_js,
                "maxDistanceStep",
                request.safety_push.max_distance_step
            );
            request.safety_push.include_waist_corridor =
                safety_push_js["includeWaistCorridor"].isUndefined()
                    ? request.safety_push.include_waist_corridor
                    : safety_push_js["includeWaistCorridor"].as<bool>();
            request.safety_push.waist_corridor_target_distance = getOptionalNumberField(
                safety_push_js,
                "waistCorridorTargetDistance",
                request.safety_push.waist_corridor_target_distance
            );
            request.safety_push.waist_corridor_max_distance_step = getOptionalNumberField(
                safety_push_js,
                "waistCorridorMaxDistanceStep",
                request.safety_push.waist_corridor_max_distance_step
            );
            request.safety_push.waist_corridor_weight_scale = getOptionalNumberField(
                safety_push_js,
                "waistCorridorWeightScale",
                request.safety_push.waist_corridor_weight_scale
            );
        }

        const val reference_model_js = request_js["referenceModel"];
        const val absolute_reference_js = reference_model_js["absolute"];
        const val relative_reference_js = reference_model_js["relative"];
        const val body_reference_js = reference_model_js["bodyZ"];
        request.absolute_reference.stiffness = getOptionalNumberField(
            absolute_reference_js,
            "stiffness",
            request.absolute_reference.stiffness
        );
        request.absolute_reference.damping = getOptionalNumberField(
            absolute_reference_js,
            "damping",
            request.absolute_reference.damping
        );
        request.relative_reference.stiffness = getOptionalNumberField(
            relative_reference_js,
            "stiffness",
            request.relative_reference.stiffness
        );
        request.relative_reference.damping = getOptionalNumberField(
            relative_reference_js,
            "damping",
            request.relative_reference.damping
        );
        request.body_reference.stiffness = getOptionalNumberField(
            body_reference_js,
            "stiffness",
            request.body_reference.stiffness
        );
        request.body_reference.damping = getOptionalNumberField(
            body_reference_js,
            "damping",
            request.body_reference.damping
        );

        request.max_iterations = getOptionalIntField(
            request_js,
            "maxIterations",
            request.max_iterations
        );
        request.damping = getOptionalNumberField(request_js, "damping", request.damping);
        request.max_damping_retries = getOptionalIntField(
            request_js,
            "maxDampingRetries",
            request.max_damping_retries
        );
        request.damping_growth_factor = getOptionalNumberField(
            request_js,
            "dampingGrowthFactor",
            request.damping_growth_factor
        );
        request.initial_scale = getOptionalNumberField(
            request_js,
            "initialScale",
            request.initial_scale
        );
        request.min_scale = getOptionalNumberField(
            request_js,
            "minScale",
            request.min_scale
        );
        request.reduction_factor = getOptionalNumberField(
            request_js,
            "reductionFactor",
            request.reduction_factor
        );
        request.initial_trust_radius = getOptionalNumberField(
            request_js,
            "initialTrustRadius",
            request.initial_trust_radius
        );
        request.min_trust_radius = getOptionalNumberField(
            request_js,
            "minTrustRadius",
            request.min_trust_radius
        );
        request.max_trust_radius = getOptionalNumberField(
            request_js,
            "maxTrustRadius",
            request.max_trust_radius
        );
        request.max_trust_region_retries = getOptionalIntField(
            request_js,
            "maxTrustRegionRetries",
            request.max_trust_region_retries
        );
        request.trust_region_shrink_factor = getOptionalNumberField(
            request_js,
            "trustRegionShrinkFactor",
            request.trust_region_shrink_factor
        );
        request.trust_region_expand_factor = getOptionalNumberField(
            request_js,
            "trustRegionExpandFactor",
            request.trust_region_expand_factor
        );
        request.rho_acceptance_threshold = getOptionalNumberField(
            request_js,
            "rhoAcceptanceThreshold",
            request.rho_acceptance_threshold
        );
        request.rho_shrink_threshold = getOptionalNumberField(
            request_js,
            "rhoShrinkThreshold",
            request.rho_shrink_threshold
        );
        request.rho_expand_threshold = getOptionalNumberField(
            request_js,
            "rhoExpandThreshold",
            request.rho_expand_threshold
        );
        request.equality_tolerance = getOptionalNumberField(
            request_js,
            "equalityTolerance",
            request.equality_tolerance
        );
        request.inequality_tolerance = getOptionalNumberField(
            request_js,
            "inequalityTolerance",
            request.inequality_tolerance
        );
        return request;
    }

    static val absoluteTargetMJRestoreResultToJs(const HumanoidAbsoluteMJRestoreResult& result) {
        val out = val::object();
        out.set("converged", result.converged);
        out.set("reason", result.reason);
        out.set("iterations", result.iterations);
        out.set("acceptedIterations", result.accepted_iterations);
        out.set("lastAcceptedScale", result.last_accepted_scale);
        out.set("lastAcceptedDamping", result.last_accepted_damping);
        out.set("commandResidualNorm", result.command_residual_norm);
        out.set("relativeResidualNorm", result.relative_residual_norm);
        out.set("bodyResidualNorm", result.body_residual_norm);
        out.set("inequalityViolation", result.inequality_violation);
        out.set("trustRadius", result.trust_radius);
        out.set("lastAcceptedRho", result.last_accepted_rho);
        out.set("lastPredictedReduction", result.last_predicted_reduction);
        out.set("lastActualReduction", result.last_actual_reduction);
        out.set("acceptedBy", result.accepted_by);
        out.set("lastRejectedRho", result.last_rejected_rho);
        out.set("lastRejectedPredictedReduction", result.last_rejected_predicted_reduction);
        out.set("lastRejectedActualReduction", result.last_rejected_actual_reduction);
        out.set("lastRejectedReason", result.last_rejected_reason);
        out.set("bestConstraintViolation", result.best_constraint_violation);
        out.set("filterSize", result.filter_size);
        out.set("safetyStatus", result.safety_status);
        out.set("safetyResidualBefore", result.safety_residual_before);
        out.set("safetyResidualAfter", result.safety_residual_after);
        out.set("safetyAcceptedIterations", result.safety_accepted_iterations);
        out.set("referenceResidual", stdVectorToJsArray(result.reference_residual));
        out.set("constraintMotion", stdVectorToJsArray(result.constraint_motion));
        out.set("qApplied", stdVectorToJsArray(result.q_applied));
        return out;
    }

public:
    // base_frame_dq: 8 元素；waist/left/right/head_chain_js: 每链为 { joint_type, axis, origin } 数组
    // effector frames: 可选的 8 元素 DQ 数组，用于末端固定关节的变换
    HumanoidURDF_Wrapper(const val& base_frame_dq,
                         const val& waist_chain_js,
                         const val& left_arm_chain_js,
                         const val& right_arm_chain_js,
                         const val& head_chain_js,
                         const val& waist_effector_js,
                         const val& left_effector_js,
                         const val& right_effector_js,
                         const val& head_effector_js)
        : robot(DQ(jsArrayToVectorXd(base_frame_dq)),
                extractURDFChain(waist_chain_js),
                extractURDFChain(left_arm_chain_js),
                extractURDFChain(right_arm_chain_js),
                extractURDFChain(head_chain_js),
                extractDQOrIdentity(waist_effector_js),
                extractDQOrIdentity(left_effector_js),
                extractDQOrIdentity(right_effector_js),
                extractDQOrIdentity(head_effector_js))
    {}

    val fkm(const val& q_js) {
        VectorXd q = jsArrayToVectorXd(q_js);
        DQ pose = robot.fkm(q);
        return vectorXdToJsArray(pose.q);
    }

    val fkmEffector(const val& q_js, int effector_index) {
        VectorXd q = jsArrayToVectorXd(q_js);
        DQ pose = robot.fkm(q, effector_index);
        return vectorXdToJsArray(pose.q);
    }

    val poseJacobian(const val& q_js) {
        VectorXd q = jsArrayToVectorXd(q_js);
        MatrixXd J = robot.pose_jacobian(q);
        return matrixXdToJsArray(J);
    }

    val poseJacobianEffector(const val& q_js, int effector_index) {
        VectorXd q = jsArrayToVectorXd(q_js);
        MatrixXd J = robot.pose_jacobian(q, effector_index);
        return matrixXdToJsArray(J);
    }

    int getDimConfigurationSpace() const {
        return robot.get_dim_configuration_space();
    }

    void setReferenceFrame(const val& dq_array) {
        VectorXd dq_vec = jsArrayToVectorXd(dq_array);
        if (dq_vec.size() != 8)
            throw std::runtime_error("双四元数必须是8元素数组");
        robot.set_reference_frame(DQ(dq_vec).normalize());
    }

    // 双臂协作空间：与 DQ_CooperativeDualTaskSpace 一致的相对/绝对位姿与雅可比
    val relativePoseDualArm(const val& q_js) {
        VectorXd q = jsArrayToVectorXd(q_js);
        DQ dq = robot.relativePoseDualArm(q);
        return vectorXdToJsArray(dq.q);
    }
    val absolutePoseDualArm(const val& q_js) {
        VectorXd q = jsArrayToVectorXd(q_js);
        DQ dq = robot.absolutePoseDualArm(q);
        return vectorXdToJsArray(dq.q);
    }
    val relativePoseJacobianDualArm(const val& q_js) {
        VectorXd q = jsArrayToVectorXd(q_js);
        MatrixXd J = robot.relativePoseJacobianDualArm(q);
        return matrixXdToJsArray(J);
    }
    val absolutePoseJacobianDualArm(const val& q_js) {
        VectorXd q = jsArrayToVectorXd(q_js);
        MatrixXd J = robot.absolutePoseJacobianDualArm(q);
        return matrixXdToJsArray(J);
    }

    val solveAbsoluteTargetMJRestore(const val& request_js) {
        const HumanoidAbsoluteMJRestoreRequest request =
            parseAbsoluteTargetMJRestoreRequest(request_js);
        const HumanoidAbsoluteMJRestoreResult result =
            solveHumanoidAbsoluteTargetMJRestore(robot, request);
        return absoluteTargetMJRestoreResultToJs(result);
    }

    val solveAbsoluteTargetMJRestoreTrustRegion(const val& request_js) {
        const HumanoidAbsoluteMJRestoreRequest request =
            parseAbsoluteTargetMJRestoreRequest(request_js);
        const HumanoidAbsoluteMJRestoreResult result =
            solveHumanoidAbsoluteTargetMJRestoreTrustRegion(robot, request);
        return absoluteTargetMJRestoreResultToJs(result);
    }

    val solveAbsoluteDQSetpointMJRestore(const val& request_js) {
        HumanoidAbsoluteMJRestoreRequest request =
            parseAbsoluteTargetMJRestoreRequest(request_js);
        request.solve_variant = "absolute_dq_setpoint";
        const HumanoidAbsoluteMJRestoreResult result =
            solveHumanoidAbsoluteDQSetpointMJRestore(robot, request);
        return absoluteTargetMJRestoreResultToJs(result);
    }

    val solveAbsoluteDQSetpointMJRestoreTrustRegion(const val& request_js) {
        HumanoidAbsoluteMJRestoreRequest request =
            parseAbsoluteTargetMJRestoreRequest(request_js);
        request.solve_variant = "absolute_dq_setpoint";
        const HumanoidAbsoluteMJRestoreResult result =
            solveHumanoidAbsoluteDQSetpointMJRestoreTrustRegion(robot, request);
        return absoluteTargetMJRestoreResultToJs(result);
    }

    val solveCooperativeDQSetpointMJRestore(const val& request_js) {
        HumanoidAbsoluteMJRestoreRequest request =
            parseAbsoluteTargetMJRestoreRequest(request_js);
        request.solve_variant = "cooperative_dq_setpoint";
        const HumanoidAbsoluteMJRestoreResult result =
            solveHumanoidCooperativeDQSetpointMJRestore(robot, request);
        return absoluteTargetMJRestoreResultToJs(result);
    }

    val solveCooperativeDQSetpointMJRestoreTrustRegion(const val& request_js) {
        HumanoidAbsoluteMJRestoreRequest request =
            parseAbsoluteTargetMJRestoreRequest(request_js);
        request.solve_variant = "cooperative_dq_setpoint";
        const HumanoidAbsoluteMJRestoreResult result =
            solveHumanoidCooperativeDQSetpointMJRestoreTrustRegion(robot, request);
        return absoluteTargetMJRestoreResultToJs(result);
    }
};

// 差速底盘包装器（配置量 q=[x, y, yaw]，控制量 u=[w_r, w_l]）
class DifferentialDriveRobot_Wrapper {
private:
    DQ_DifferentialDriveRobot robot;

public:
    DifferentialDriveRobot_Wrapper(double wheel_radius, double distance_between_wheels)
        : robot(wheel_radius, distance_between_wheels) {}

    val fkm(const val& q_js) {
        VectorXd q = jsArrayToVectorXd(q_js);
        if (q.size() != 3) {
            throw std::runtime_error("DifferentialDriveRobot::fkm expects q=[x,y,yaw]");
        }
        DQ pose = robot.fkm(q);
        return vectorXdToJsArray(pose.q);
    }

    val poseJacobian(const val& q_js) {
        VectorXd q = jsArrayToVectorXd(q_js);
        if (q.size() != 3) {
            throw std::runtime_error("DifferentialDriveRobot::poseJacobian expects q=[x,y,yaw]");
        }
        MatrixXd J = robot.pose_jacobian(q);
        return matrixXdToJsArray(J);
    }

    val constraintJacobian(double phi) {
        MatrixXd Jc = robot.constraint_jacobian(phi);
        return matrixXdToJsArray(Jc);
    }

    val constraintJacobianDerivative(double phi, double phi_dot) {
        MatrixXd Jcd = robot.constraint_jacobian_derivative(phi, phi_dot);
        return matrixXdToJsArray(Jcd);
    }
};

// 预定义机器人包装器 - Franka Panda (使用 MDH 参数)
class FrankaPanda_Wrapper {
private:
    DQ_SerialManipulatorMDH robot;
    
public:
    FrankaPanda_Wrapper() : robot(FrankaEmikaPandaRobot::kinematics()) {
    }
    
    val fkm(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        return vectorXdToJsArray(pose.q);
    }
    
    val pose_jacobian(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        MatrixXd J = robot.pose_jacobian(joint_angles);
        return matrixXdToJsArray(J);
    }
    
    val get_end_effector_pose_array(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        
        // 获取平移和旋转
        DQ trans = ::translation(pose);
        DQ rot = ::rotation(pose);
        
        val translation_array = val::array();
        translation_array.call<void>("push", 0.0);  // w 分量
        translation_array.call<void>("push", trans.q(1));  // x
        translation_array.call<void>("push", trans.q(2));  // y
        translation_array.call<void>("push", trans.q(3));  // z
        
        val result = val::object();
        result.set("translation", translation_array);
        result.set("rotation", vectorXdToJsArray(rot.q));
        
        return result;
    }
    
    int get_dim_configuration_space() const {
        return robot.get_dim_configuration_space();
    }
};

// Kuka LW4 机器人 (使用标准 DH 参数)
class KukaLw4_Wrapper {
private:
    DQ_SerialManipulatorDH robot;
    
public:
    KukaLw4_Wrapper() : robot(KukaLw4Robot::kinematics()) {
    }
    
    val fkm(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        return vectorXdToJsArray(pose.q);
    }
    
    val get_end_effector_pose_array(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        
        // 获取平移和旋转
        DQ trans = ::translation(pose);
        DQ rot = ::rotation(pose);
        
        val translation_array = val::array();
        translation_array.call<void>("push", 0.0);  // w 分量
        translation_array.call<void>("push", trans.q(1));  // x
        translation_array.call<void>("push", trans.q(2));  // y
        translation_array.call<void>("push", trans.q(3));  // z
        
        val result = val::object();
        result.set("translation", translation_array);
        result.set("rotation", vectorXdToJsArray(rot.q));
        
        return result;
    }
    
    int get_dim_configuration_space() const {
        return robot.get_dim_configuration_space();
    }
};

// 协作双臂系统包装器
class CooperativeDualTaskSpace_Wrapper {
private:
    DQ_CooperativeDualTaskSpace* dual_arm;
    DQ_SerialManipulatorMDH* robot1_ptr;
    DQ_SerialManipulatorMDH* robot2_ptr;
    bool owns_robots;
    
public:
    // 从两个已存在的机器人指针创建（不拥有所有权）
    CooperativeDualTaskSpace_Wrapper(SerialManipulator_Wrapper* r1, SerialManipulator_Wrapper* r2) 
        : owns_robots(false) {
        // 注意：这里需要访问内部robot，但embind限制，所以需要另一种方式
        // 为了简化，我们创建新的机器人实例
        throw std::runtime_error("Use createFromDHMatrices instead");
    }
    
    // 从DH矩阵创建（会创建新的机器人实例）
    CooperativeDualTaskSpace_Wrapper(const val& robot1_dh_js, const val& robot2_dh_js) 
        : owns_robots(true) {
        robot1_ptr = new DQ_SerialManipulatorMDH(extractDHMatrixStatic(robot1_dh_js));
        robot2_ptr = new DQ_SerialManipulatorMDH(extractDHMatrixStatic(robot2_dh_js));
        dual_arm = new DQ_CooperativeDualTaskSpace(robot1_ptr, robot2_ptr);
    }
    
    ~CooperativeDualTaskSpace_Wrapper() {
        if (owns_robots) {
            delete dual_arm;
            delete robot1_ptr;
            delete robot2_ptr;
        }
    }
    
    static MatrixXd extractDHMatrixStatic(const val& dh_matrix_js) {
        int num_rows = dh_matrix_js["length"].as<int>();
        if (num_rows != 5) {
            throw std::runtime_error("DH matrix should have 5 rows");
        }
        
        val first_row = dh_matrix_js[0];
        int num_joints = first_row["length"].as<int>();
        
        MatrixXd dh_matrix(5, num_joints);
        for (int row = 0; row < 5; row++) {
            val js_row = dh_matrix_js[row];
            for (int col = 0; col < num_joints; col++) {
                dh_matrix(row, col) = js_row[col].as<double>();
            }
        }
        return dh_matrix;
    }
    
    // 设置机器人1的基座和末端
    void setRobot1Base(double x, double y, double z) {
        DQ base = 1 + E_ * 0.5 * DQ(0, x, y, z);
        base = base.normalize(); // 归一化（与Python逻辑一致）
        robot1_ptr->set_base_frame(base);
        robot1_ptr->set_reference_frame(base);
    }
    
    void setRobot1BasePose(double qw, double qx, double qy, double qz,
                           double tx, double ty, double tz) {
        // 先归一化旋转四元数（只归一化旋转部分，不改变平移）
        double q_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (q_norm > 1e-10) {
            qw /= q_norm;
            qx /= q_norm;
            qy /= q_norm;
            qz /= q_norm;
        }
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        DQ base = r * t; // 先旋转再平移
        robot1_ptr->set_base_frame(base);
        robot1_ptr->set_reference_frame(base);
    }
    
    // 设置机器人1基座（直接使用双四元数，8元素数组）
    void setRobot1BaseDQ(const val& dq_array) {
        VectorXd dq_vec = jsArrayToVectorXd(dq_array);
        if (dq_vec.size() != 8) {
            throw std::runtime_error("双四元数必须是8元素数组");
        }
        DQ base(dq_vec);
        
        // 打印原始双四元数（用于调试）
        printf("机器人1原始双四元数: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               dq_vec(0), dq_vec(1), dq_vec(2), dq_vec(3),
               dq_vec(4), dq_vec(5), dq_vec(6), dq_vec(7));
        
        // 归一化（与Python代码保持一致）
        base = base.normalize();
        
        // 打印归一化后的双四元数（用于调试）
        printf("机器人1归一化后: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               base.q(0), base.q(1), base.q(2), base.q(3),
               base.q(4), base.q(5), base.q(6), base.q(7));
        
        robot1_ptr->set_base_frame(base);
        robot1_ptr->set_reference_frame(base);
    }
    
    void setRobot1Effector(double x, double y, double z) {
        DQ eff = 1 + E_ * 0.5 * DQ(0, x, y, z);
        robot1_ptr->set_effector(eff);
    }
    
    void setRobot1EffectorPose(double qw, double qx, double qy, double qz,
                                double tx, double ty, double tz) {
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        robot1_ptr->set_effector(r * t);
    }
    
    // 设置机器人2的基座和末端
    void setRobot2Base(double x, double y, double z) {
        DQ base = 1 + E_ * 0.5 * DQ(0, x, y, z);
        base = base.normalize(); // 归一化（与Python逻辑一致）
        robot2_ptr->set_base_frame(base);
        robot2_ptr->set_reference_frame(base);
    }
    
    void setRobot2BasePose(double qw, double qx, double qy, double qz,
                           double tx, double ty, double tz) {
        // 先归一化旋转四元数（只归一化旋转部分，不改变平移）
        double q_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (q_norm > 1e-10) {
            qw /= q_norm;
            qx /= q_norm;
            qy /= q_norm;
            qz /= q_norm;
        }
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        DQ base = r * t; // 先旋转再平移
        robot2_ptr->set_base_frame(base);
        robot2_ptr->set_reference_frame(base);
    }
    
    // 设置机器人2基座（直接使用双四元数，8元素数组）
    void setRobot2BaseDQ(const val& dq_array) {
        VectorXd dq_vec = jsArrayToVectorXd(dq_array);
        if (dq_vec.size() != 8) {
            throw std::runtime_error("双四元数必须是8元素数组");
        }
        DQ base(dq_vec);
        
        // 打印原始双四元数（用于调试）
        printf("机器人2原始双四元数: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               dq_vec(0), dq_vec(1), dq_vec(2), dq_vec(3),
               dq_vec(4), dq_vec(5), dq_vec(6), dq_vec(7));
        
        // 归一化（与Python代码保持一致）
        base = base.normalize();
        
        // 打印归一化后的双四元数（用于调试）
        printf("机器人2归一化后: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               base.q(0), base.q(1), base.q(2), base.q(3),
               base.q(4), base.q(5), base.q(6), base.q(7));
        
        robot2_ptr->set_base_frame(base);
        robot2_ptr->set_reference_frame(base);
    }
    
    void setRobot2Effector(double x, double y, double z) {
        DQ eff = 1 + E_ * 0.5 * DQ(0, x, y, z);
        robot2_ptr->set_effector(eff);
    }
    
    void setRobot2EffectorPose(double qw, double qx, double qy, double qz,
                                double tx, double ty, double tz) {
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        robot2_ptr->set_effector(r * t);
    }
    
    // 获取机器人1的位姿
    val pose1(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        DQ pose = dual_arm->pose1(theta);
        return vectorXdToJsArray(pose.q);
    }
    
    // 获取机器人2的位姿
    val pose2(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        DQ pose = dual_arm->pose2(theta);
        return vectorXdToJsArray(pose.q);
    }
    
    // 获取相对位姿（robot2相对于robot1）
    val relative_pose(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        DQ pose = dual_arm->relative_pose(theta);
        return vectorXdToJsArray(pose.q);
    }
    
    // 获取绝对位姿（两个末端的平均位置）
    val absolute_pose(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        DQ pose = dual_arm->absolute_pose(theta);
        return vectorXdToJsArray(pose.q);
    }
    
    // 获取相对位姿雅可比
    val relative_pose_jacobian(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        MatrixXd J = dual_arm->relative_pose_jacobian(theta);
        return matrixXdToJsArray(J);
    }
    
    // 获取绝对位姿雅可比
    val absolute_pose_jacobian(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        MatrixXd J = dual_arm->absolute_pose_jacobian(theta);
        return matrixXdToJsArray(J);
    }
};

// Embind 绑定
EMSCRIPTEN_BINDINGS(dqrobotics_module) {
    // DQ 类
    class_<DQ>("DQ")
        .constructor<>()
        .constructor<double, double, double, double, double, double, double, double>()
        .property("q", &DQ::q);
    
    // DQ 包装器 - 提供静态方法
    class_<DQ_Wrapper>("DQWrapper")
        .class_function("create", &DQ_Wrapper::create)
        .class_function("createFromArray", &DQ_Wrapper::createFromArray)
        .class_function("toArray", &DQ_Wrapper::toArray)
        .class_function("multiply", &DQ_Wrapper::multiply)
        .class_function("add", &DQ_Wrapper::add)
        .class_function("conjugate", &DQ_Wrapper::conjugate)
        .class_function("normalize", &DQ_Wrapper::normalize)
        .class_function("inverse", &DQ_Wrapper::inverse)
        .class_function("rotation", &DQ_Wrapper::rotation)
        .class_function("translation", &DQ_Wrapper::translation)
        .class_function("pose", &DQ_Wrapper::pose)
        .class_function("fromQuaternionAndTranslation", &DQ_Wrapper::fromQuaternionAndTranslation)
        .class_function("getTranslation", &DQ_Wrapper::getTranslation)
        .class_function("getRotation", &DQ_Wrapper::getRotation)
        .class_function("getRotationMatrix", &DQ_Wrapper::getRotationMatrix);

    // TOPPRA 路点时间参数化（waypoints -> timestamps）
    class_<TOPPRA_Wrapper>("TOPPRAWrapper")
        .class_function("retimeWaypoints", &TOPPRA_Wrapper::retimeWaypoints);

    // 运动学工具（静态函数）
    class_<Kinematics_Wrapper>("Kinematics")
        .class_function("distanceJacobian", &Kinematics_Wrapper::distance_jacobian)
        .class_function("lineJacobian", &Kinematics_Wrapper::line_jacobian);
    
    // 串联机械臂
    class_<SerialManipulator_Wrapper>("SerialManipulator")
        .constructor<const val&>()
        .function("fkm", &SerialManipulator_Wrapper::fkm)
        .function("poseJacobian", &SerialManipulator_Wrapper::pose_jacobian)
        .function("getDimConfigurationSpace", &SerialManipulator_Wrapper::get_dim_configuration_space)
        .function("setBaseFrame", &SerialManipulator_Wrapper::set_base_frame)
        .function("setBaseFramePose", &SerialManipulator_Wrapper::set_base_frame_pose)
        .function("setBaseFrameDQ", &SerialManipulator_Wrapper::set_base_frame_dq)
        .function("setEffector", &SerialManipulator_Wrapper::set_effector)
        .function("setEffectorPose", &SerialManipulator_Wrapper::set_effector_pose);

    // URDF 串联机械臂（由 JS 传入关节链：每关节 joint_type, axis, origin）
    class_<SerialManipulatorURDF_Wrapper>("SerialManipulatorURDF")
        .constructor<const val&>()
        .function("fkm", &SerialManipulatorURDF_Wrapper::fkm)
        .function("poseJacobian", &SerialManipulatorURDF_Wrapper::pose_jacobian)
        .function("getDimConfigurationSpace", &SerialManipulatorURDF_Wrapper::get_dim_configuration_space)
        .function("setBaseFrame", &SerialManipulatorURDF_Wrapper::set_base_frame)
        .function("setBaseFramePose", &SerialManipulatorURDF_Wrapper::set_base_frame_pose)
        .function("setBaseFrameDQ", &SerialManipulatorURDF_Wrapper::set_base_frame_dq)
        .function("setEffector", &SerialManipulatorURDF_Wrapper::set_effector)
        .function("setEffectorPose", &SerialManipulatorURDF_Wrapper::set_effector_pose);

    // 人形 URDF（底盘 + 腰 + 左臂 + 右臂 + 头）；链由 JS 从 URDF 解析后传入
    // effector_index: 0=腰, 1=左臂, 2=右臂, 3=头
    class_<HumanoidURDF_Wrapper>("HumanoidURDF")
        .constructor<const val&, const val&, const val&, const val&, const val&, const val&, const val&, const val&, const val&>()
        .function("fkm", &HumanoidURDF_Wrapper::fkm)
        .function("fkmEffector", &HumanoidURDF_Wrapper::fkmEffector)
        .function("poseJacobian", &HumanoidURDF_Wrapper::poseJacobian)
        .function("poseJacobianEffector", &HumanoidURDF_Wrapper::poseJacobianEffector)
        .function("getDimConfigurationSpace", &HumanoidURDF_Wrapper::getDimConfigurationSpace)
        .function("setReferenceFrame", &HumanoidURDF_Wrapper::setReferenceFrame)
        .function("relativePoseDualArm", &HumanoidURDF_Wrapper::relativePoseDualArm)
        .function("absolutePoseDualArm", &HumanoidURDF_Wrapper::absolutePoseDualArm)
        .function("relativePoseJacobianDualArm", &HumanoidURDF_Wrapper::relativePoseJacobianDualArm)
        .function("absolutePoseJacobianDualArm", &HumanoidURDF_Wrapper::absolutePoseJacobianDualArm)
        .function("solveAbsoluteTargetMJRestore", &HumanoidURDF_Wrapper::solveAbsoluteTargetMJRestore)
        .function("solveAbsoluteTargetMJRestoreTrustRegion", &HumanoidURDF_Wrapper::solveAbsoluteTargetMJRestoreTrustRegion)
        .function("solveAbsoluteDQSetpointMJRestore", &HumanoidURDF_Wrapper::solveAbsoluteDQSetpointMJRestore)
        .function("solveAbsoluteDQSetpointMJRestoreTrustRegion", &HumanoidURDF_Wrapper::solveAbsoluteDQSetpointMJRestoreTrustRegion)
        .function("solveCooperativeDQSetpointMJRestore", &HumanoidURDF_Wrapper::solveCooperativeDQSetpointMJRestore)
        .function("solveCooperativeDQSetpointMJRestoreTrustRegion", &HumanoidURDF_Wrapper::solveCooperativeDQSetpointMJRestoreTrustRegion);

    // 差速底盘
    class_<DifferentialDriveRobot_Wrapper>("DifferentialDriveRobot")
        .constructor<double, double>()
        .function("fkm", &DifferentialDriveRobot_Wrapper::fkm)
        .function("poseJacobian", &DifferentialDriveRobot_Wrapper::poseJacobian)
        .function("constraintJacobian", &DifferentialDriveRobot_Wrapper::constraintJacobian)
        .function("constraintJacobianDerivative", &DifferentialDriveRobot_Wrapper::constraintJacobianDerivative);
    
    // Franka Panda 机器人
    class_<FrankaPanda_Wrapper>("FrankaPanda")
        .constructor<>()
        .function("fkm", &FrankaPanda_Wrapper::fkm)
        .function("poseJacobian", &FrankaPanda_Wrapper::pose_jacobian)
        .function("getEndEffectorPose", &FrankaPanda_Wrapper::get_end_effector_pose_array)
        .function("getDimConfigurationSpace", &FrankaPanda_Wrapper::get_dim_configuration_space);
    
    // Kuka LW4 机器人
    class_<KukaLw4_Wrapper>("KukaLw4")
        .constructor<>()
        .function("fkm", &KukaLw4_Wrapper::fkm)
        .function("getEndEffectorPose", &KukaLw4_Wrapper::get_end_effector_pose_array)
        .function("getDimConfigurationSpace", &KukaLw4_Wrapper::get_dim_configuration_space);
    
    // 协作双臂系统
    class_<CooperativeDualTaskSpace_Wrapper>("CooperativeDualTaskSpace")
        .constructor<const val&, const val&>()
        .function("setRobot1Base", &CooperativeDualTaskSpace_Wrapper::setRobot1Base)
        .function("setRobot1BasePose", &CooperativeDualTaskSpace_Wrapper::setRobot1BasePose)
        .function("setRobot1BaseDQ", &CooperativeDualTaskSpace_Wrapper::setRobot1BaseDQ)
        .function("setRobot1Effector", &CooperativeDualTaskSpace_Wrapper::setRobot1Effector)
        .function("setRobot1EffectorPose", &CooperativeDualTaskSpace_Wrapper::setRobot1EffectorPose)
        .function("setRobot2Base", &CooperativeDualTaskSpace_Wrapper::setRobot2Base)
        .function("setRobot2BasePose", &CooperativeDualTaskSpace_Wrapper::setRobot2BasePose)
        .function("setRobot2BaseDQ", &CooperativeDualTaskSpace_Wrapper::setRobot2BaseDQ)
        .function("setRobot2Effector", &CooperativeDualTaskSpace_Wrapper::setRobot2Effector)
        .function("setRobot2EffectorPose", &CooperativeDualTaskSpace_Wrapper::setRobot2EffectorPose)
        .function("pose1", &CooperativeDualTaskSpace_Wrapper::pose1)
        .function("pose2", &CooperativeDualTaskSpace_Wrapper::pose2)
        .function("relativePose", &CooperativeDualTaskSpace_Wrapper::relative_pose)
        .function("absolutePose", &CooperativeDualTaskSpace_Wrapper::absolute_pose)
        .function("relativePoseJacobian", &CooperativeDualTaskSpace_Wrapper::relative_pose_jacobian)
        .function("absolutePoseJacobian", &CooperativeDualTaskSpace_Wrapper::absolute_pose_jacobian);
}

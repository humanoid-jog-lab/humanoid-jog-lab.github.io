/**
 * Load humanoid chains from URDF by link names.
 */
#include "urdf_humanoid_chain_loader.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_JointType.h>
#include <tinyxml2.h>
#include <cmath>
#include <map>
#include <vector>
#include <cstdio>

namespace DQ_robotics {

namespace {

static bool parse_xyz(const char* s, double& x, double& y, double& z) {
    if (!s) return false;
    return std::sscanf(s, "%lf %lf %lf", &x, &y, &z) == 3;
}

static DQ origin_xyz_rpy(double x, double y, double z, double roll, double pitch, double yaw) {
    double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;
    DQ r(qw, qx, qy, qz);
    DQ p(0, x, y, z);
    return r + DQ::E * 0.5 * p * r;
}

struct JointInfo {
    std::string parent_link;
    std::string type;
    double ox, oy, oz, oroll, opitch, oyaw;
    double ax, ay, az;
};

}  // namespace

std::vector<URDFJointData> extract_chain_from_urdf(
    const std::string& urdf_path,
    const std::string& start_link,
    const std::string& end_link,
    DQ* effector_frame,
    std::string* err_msg) {
    std::vector<URDFJointData> chain;
    if (effector_frame) *effector_frame = DQ(1);
    if (err_msg) err_msg->clear();

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
        if (err_msg) *err_msg = "Failed to load URDF file: " + urdf_path;
        return chain;
    }

    tinyxml2::XMLElement* robot = doc.RootElement();
    if (!robot || std::string(robot->Name()) != "robot") {
        if (err_msg) *err_msg = "URDF root is not <robot>";
        return chain;
    }

    // child_link -> (parent_link, joint info) for ALL joints (path uses all; chain only revolute/prismatic/continuous)
    std::map<std::string, std::pair<std::string, JointInfo>> child_to_parent_joint;

    for (tinyxml2::XMLElement* j = robot->FirstChildElement("joint"); j != nullptr;
         j = j->NextSiblingElement("joint")) {
        const char* type_str = j->Attribute("type");
        if (!type_str) continue;
        std::string type(type_str);

        tinyxml2::XMLElement* parent_el = j->FirstChildElement("parent");
        tinyxml2::XMLElement* child_el = j->FirstChildElement("child");
        if (!parent_el || !child_el) continue;
        const char* parent_link = parent_el->Attribute("link");
        const char* child_link = child_el->Attribute("link");
        if (!parent_link || !child_link) continue;

        JointInfo info;
        info.parent_link = parent_link;
        info.type = type;
        info.ox = info.oy = info.oz = 0;
        info.oroll = info.opitch = info.oyaw = 0;
        info.ax = 0; info.ay = 0; info.az = 1;

        tinyxml2::XMLElement* origin = j->FirstChildElement("origin");
        if (origin) {
            const char* xyz = origin->Attribute("xyz");
            const char* rpy = origin->Attribute("rpy");
            if (xyz) parse_xyz(xyz, info.ox, info.oy, info.oz);
            if (rpy) parse_xyz(rpy, info.oroll, info.opitch, info.oyaw);
        }
        tinyxml2::XMLElement* axis = j->FirstChildElement("axis");
        if (axis) {
            const char* ax = axis->Attribute("xyz");
            if (ax) parse_xyz(ax, info.ax, info.ay, info.az);
        }

        double n = std::sqrt(info.ax*info.ax + info.ay*info.ay + info.az*info.az);
        if (n < 1e-10) n = 1;
        info.ax /= n; info.ay /= n; info.az /= n;

        child_to_parent_joint[child_link] = std::make_pair(parent_link, info);
    }

    // Walk from end_link backward to start_link
    std::vector<JointInfo> path_infos;
    std::string current = end_link;
    while (current != start_link) {
        auto it = child_to_parent_joint.find(current);
        if (it == child_to_parent_joint.end()) {
            if (err_msg)
                *err_msg = "No path from " + start_link + " to " + end_link + " (stuck at " + current + ")";
            return chain;
        }
        path_infos.push_back(it->second.second);
        current = it->second.first;
    }
    // path_infos is [joint near end, ..., joint near start]; reverse for base->effector
    // Fixed joints' transforms are accumulated into the next active joint's origin
    // Also, the last fixed joint(s) before end_link need to be accumulated into an effector transform
    DQ accumulated_fixed = DQ(1);
    bool has_accumulated = false;
    
    for (auto it = path_infos.rbegin(); it != path_infos.rend(); ++it) {
        const JointInfo& info = *it;
        DQ this_origin = origin_xyz_rpy(info.ox, info.oy, info.oz, info.oroll, info.opitch, info.oyaw);
        
        if (info.type != "revolute" && info.type != "prismatic" && info.type != "continuous") {
            // Fixed joint: accumulate its transform
            accumulated_fixed = accumulated_fixed * this_origin;
            has_accumulated = true;
            continue;
        }
        
        URDFJointData j;
        j.joint_type = (info.type == "prismatic") ? DQ_JointType::PRISMATIC : DQ_JointType::REVOLUTE;
        j.axis(0) = info.ax; j.axis(1) = info.ay; j.axis(2) = info.az;
        
        // Pre-multiply accumulated fixed transforms into this joint's origin
        if (has_accumulated) {
            j.origin = accumulated_fixed * this_origin;
            accumulated_fixed = DQ(1);
            has_accumulated = false;
        } else {
            j.origin = this_origin;
        }
        chain.push_back(j);
    }
    
    // If there are trailing fixed joints (末端固定关节), store in effector_frame
    if (has_accumulated) {
        if (effector_frame) {
            *effector_frame = accumulated_fixed;
        }
    }
    return chain;
}

bool load_humanoid_chains_from_urdf(
    const std::string& urdf_path,
    const HumanoidURDFLinkNames& names,
    std::vector<URDFJointData>* waist_chain,
    std::vector<URDFJointData>* left_arm_chain,
    std::vector<URDFJointData>* right_arm_chain,
    std::vector<URDFJointData>* head_chain,
    DQ* waist_effector_frame,
    DQ* left_effector_frame,
    DQ* right_effector_frame,
    DQ* head_effector_frame,
    std::string* err_msg) {
    if (err_msg) err_msg->clear();
    std::string err;

    *waist_chain = extract_chain_from_urdf(urdf_path, names.waist_start, names.waist_end, waist_effector_frame, &err);
    if (waist_chain->empty()) {
        if (err_msg) *err_msg = "waist: " + err;
        return false;
    }
    *left_arm_chain = extract_chain_from_urdf(urdf_path, names.left_arm_start, names.left_arm_end, left_effector_frame, &err);
    if (left_arm_chain->empty()) {
        if (err_msg) *err_msg = "left_arm: " + err;
        return false;
    }
    *right_arm_chain = extract_chain_from_urdf(urdf_path, names.right_arm_start, names.right_arm_end, right_effector_frame, &err);
    if (right_arm_chain->empty()) {
        if (err_msg) *err_msg = "right_arm: " + err;
        return false;
    }
    *head_chain = extract_chain_from_urdf(urdf_path, names.head_start, names.head_end, head_effector_frame, &err);
    if (head_chain->empty()) {
        if (err_msg) *err_msg = "head: " + err;
        return false;
    }
    return true;
}

}  // namespace DQ_robotics

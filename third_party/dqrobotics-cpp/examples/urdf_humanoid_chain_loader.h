/**
 * Load humanoid chains from a URDF file given link names.
 * Requires: dqrobotics (DQ_HumanoidURDF, HumanoidURDFLinkNames, URDFJointData), tinyxml2.
 */
#pragma once

#include <dqrobotics/robot_modeling/DQ_HumanoidURDF.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorURDF.h>
#include <dqrobotics/DQ.h>
#include <string>
#include <vector>

namespace DQ_robotics {

/**
 * Extract one kinematic chain from URDF: path of revolute/prismatic joints from start_link to end_link.
 * Returns joint data in order from start to end (base to effector).
 * Trailing fixed joints are accumulated into effector_frame.
 * On failure (path not found, invalid joint) returns empty vector and err_msg is set.
 */
std::vector<URDFJointData> extract_chain_from_urdf(
    const std::string& urdf_path,
    const std::string& start_link,
    const std::string& end_link,
    DQ* effector_frame = nullptr,
    std::string* err_msg = nullptr);

/**
 * Build the four humanoid chains from URDF and link names.
 * Returns true on success; on failure err_msg is set and waist/left/right/head are unchanged.
 */
bool load_humanoid_chains_from_urdf(
    const std::string& urdf_path,
    const HumanoidURDFLinkNames& names,
    std::vector<URDFJointData>* waist_chain,
    std::vector<URDFJointData>* left_arm_chain,
    std::vector<URDFJointData>* right_arm_chain,
    std::vector<URDFJointData>* head_chain,
    DQ* waist_effector_frame = nullptr,
    DQ* left_effector_frame = nullptr,
    DQ* right_effector_frame = nullptr,
    DQ* head_effector_frame = nullptr,
    std::string* err_msg = nullptr);

}  // namespace DQ_robotics

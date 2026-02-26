// Test DQ_HumanoidURDF with Sailor R1 Pro: load chains from URDF by link names.
// Usage: sailor_r1_pro_test [urdf_path]
// Default urdf_path (relative to cwd when run from build/):
//   ../../public/robots/sailor_r1_pro_description/urdf/sailor_r1_pro_description.urdf
// Link names (HumanoidURDFLinkNames): base_link, waist base->body, left/right arm body->lee/ree, head body->head_pitch_link.
#include "urdf_humanoid_chain_loader.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_HumanoidURDF.h>
#include <iostream>
#include <string>
#include <vector>

using namespace DQ_robotics;
using namespace Eigen;

int main(int argc, char** argv) {
    std::string urdf_path = (argc > 1)
        ? argv[1]
        : "../../public/robots/sailor_r1_pro_description/urdf/sailor_r1_pro_description.urdf";

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

    std::vector<URDFJointData> waist_chain, left_arm_chain, right_arm_chain, head_chain;
    std::string err_msg;
    if (!load_humanoid_chains_from_urdf(urdf_path, names,
            &waist_chain, &left_arm_chain, &right_arm_chain, &head_chain, &err_msg)) {
        std::cerr << "Failed to load URDF chains: " << err_msg << "\n";
        return 1;
    }

    std::cout << "Sailor R1 Pro DQ_HumanoidURDF test (from URDF: " << urdf_path << ")\n";
    std::cout << "Chains: waist=" << waist_chain.size() << " left_arm=" << left_arm_chain.size()
              << " right_arm=" << right_arm_chain.size() << " head=" << head_chain.size() << "\n";

    DQ base_frame(1, 0, 0, 0, 0, 0, 0, 0);
    DQ_HumanoidURDF humanoid(base_frame, waist_chain, left_arm_chain, right_arm_chain, head_chain);

    int n = humanoid.get_dim_configuration_space();
    std::cout << "dim_configuration_space = " << n << "\n";

    VectorXd q = VectorXd::Zero(n);
    for (int eff = 0; eff <= 3; ++eff) {
        DQ pose = humanoid.fkm(q, eff);
        Vector4d t = pose.translation().vec4();
        std::cout << "eff " << eff << " (0=waist,1=Larm,2=Rarm,3=head) t: "
                  << t(1) << " " << t(2) << " " << t(3) << "\n";
    }
    std::cout << "Done.\n";
    return 0;
}

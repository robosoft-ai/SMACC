#include <moveit_z_client/client_behaviors/cb_move_joint_space_absolute.h>

namespace sm_moveit
{
namespace cl_movegroup
{
CbMoveJointSpaceAbsolute::CbMoveJointSpaceAbsolute()
{
    // name: [l_wheel_joint, r_wheel_joint, torso_lift_joint, bellows_joint, head_pan_joint, head_tilt_joint, shoulder_pan_joint, shoulder_lift_joint, upperarm_roll_joint, elbow_flex_joint, forearm_roll_joint, wrist_flex_joint, wrist_roll_joint, l_gripper_finger_joint, r_gripper_finger_joint]
    // position: [2.9275728245408317, 1.9922243979617988, 0.05362744887151194, 0.03275390517398992, -0.014734993538954022, 0.25970592566427175, -1.4380033270056636, -0.15894159804417818, -1.2742860019484459, -2.090400209914865, -0.8589412066693702, -1.9690302385065914, -1.7503763589178511, 0.029539108456176476, 0.02905822220431006]
}

void CbMoveJointSpaceAbsolute::onEntry()
{
    this->requiresClient(movegroupClient_);
    auto &moveGroupInterface = movegroupClient_->moveGroupClientInterface;
}

void CbMoveJointSpaceAbsolute::onExit()
{
}
} // namespace cl_movegroup
} // namespace sm_moveit
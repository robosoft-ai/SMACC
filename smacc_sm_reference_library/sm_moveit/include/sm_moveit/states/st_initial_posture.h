#include <smacc/smacc.h>
namespace sm_moveit
{
// STATE DECLARATION
struct StInitialPosture : smacc::SmaccState<StInitialPosture, SmMoveIt>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, SS1::SsPickObject>,
        Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        std::map<std::string, double> jointValues{
            {"l_wheel_joint", 0.2032989263112741},
            {"r_wheel_joint", -0.25798452342087597},
            {"torso_lift_joint", -1.2577603815717127e-05},
            {"bellows_joint", 0.006785193726937263},
            {"head_pan_joint", 0.0053190193943049024},
            {"head_tilt_joint", 0.0037173708013753526},
            {"shoulder_pan_joint", 1.3199986235290062},
            {"shoulder_lift_joint", 0.7000699976533715},
            {"upperarm_roll_joint", -7.398079828480064e-05},
            {"elbow_flex_joint", -1.99994794099468},
            {"forearm_roll_joint", 0.00021030168417901507},
            {"wrist_flex_joint", -0.5700976747076689},
            {"wrist_roll_joint", -0.00023467601597193521},
            {"l_gripper_finger_joint", 0.05001998415439018},
            {"r_gripper_finger_joint", 0.050039698895249535}

        };

        configure_orthogonal<OrNavigation, CbMoveJoints>(jointValues);
    }

    void runtimeConfigure()
    {
        // name: [l_wheel_joint, r_wheel_joint, torso_lift_joint, bellows_joint, head_pan_joint, head_tilt_joint, shoulder_pan_joint, shoulder_lift_joint, upperarm_roll_joint, elbow_flex_joint, forearm_roll_joint, wrist_flex_joint, wrist_roll_joint, l_gripper_finger_joint, r_gripper_finger_joint]
        // position: [2.9275728245408317, 1.9922243979617988, 0.05362744887151194, 0.03275390517398992, -0.014734993538954022, 0.25970592566427175, -1.4380033270056636, -0.15894159804417818, -1.2742860019484459, -2.090400209914865, -0.8589412066693702, -1.9690302385065914, -1.7503763589178511, 0.029539108456176476, 0.02905822220431006]

        //[l_wheel_joint, r_wheel_joint, torso_lift_joint, bellows_joint, head_pan_joint, head_tilt_joint, shoulder_pan_joint, shoulder_lift_joint, upperarm_roll_joint, elbow_flex_joint, forearm_roll_joint, wrist_flex_joint, wrist_roll_joint, l_gripper_finger_joint, r_gripper_finger_joint]
        //position: [0.2032989263112741, -0.25798452342087597, -1.2577603815717127e-05, 0.006785193726937263, 0.0053190193943049024, 0.0037173708013753526, 1.3199986235290062, 0.7000699976533715, -7.398079828480064e-05, -1.99994794099468, 0.00021030168417901507, -0.5700976747076689, -0.00023467601597193521, 0.05001998415439018, 0.050039698895249535]
    }
};
} // namespace sm_moveit
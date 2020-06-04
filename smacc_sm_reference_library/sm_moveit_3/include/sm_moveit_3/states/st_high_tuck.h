#include <smacc/smacc.h>
namespace sm_moveit_3
{
// STATE DECLARATION
struct StHighTuck : smacc::SmaccState<StHighTuck, SmMoveit3>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StHighLeftUpperCut> //,
        //Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        // std::map<std::string, double> jointValues{
        //     {"l_wheel_joint", 0.2032989263112741},
        //     {"r_wheel_joint", -0.25798452342087597},
        //     {"torso_lift_joint", -1.2577603815717127e-05},
        //     {"bellows_joint", 0.006785193726937263},
        //     {"head_pan_joint", 0.0053190193943049024},
        //     {"head_tilt_joint", 0.0037173708013753526},
        //     //{"shoulder_pan_joint", 0.7199986235290062},
        //     {"shoulder_pan_joint", 1.0},
        //     //{"shoulder_lift_joint", 0.7000699976533715},
        //     {"shoulder_lift_joint", 1.51},
        //     {"upperarm_roll_joint", -7.398079828480064e-05},
        //     //{"elbow_flex_joint", -1.99994794099468},
        //     {"elbow_flex_joint", -2.49994794099468},
        //     {"forearm_roll_joint", 0.00021030168417901507},
        //     {"wrist_flex_joint", -0.89},
        //     {"wrist_roll_joint", -0.00023467601597193521},
        //     {"l_gripper_finger_joint", 0.05001998415439018},
        //     {"r_gripper_finger_joint", 0.050039698895249535}};



     //   configure_orthogonal<OrNavigation, CbMoveJoints>(jointValues);
        configure_orthogonal<OrNavigation, CbMoveKnownState>("sm_moveit_3", "config/manipulation/known_states/high_left_under_tuck.yaml");
    }

    void runtimeConfigure()
    {
    }
};
} // namespace sm_moveit_3
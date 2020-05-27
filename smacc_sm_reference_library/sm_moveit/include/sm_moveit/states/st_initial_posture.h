#include <smacc/smacc.h>
namespace sm_moveit
{
// STATE DECLARATION
struct StInitialPosture : smacc::SmaccState<StInitialPosture, SmMoveIt>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StInitialForward>,
        Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
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


    std::map<std::string, double> jointValues
       {{"torso_lift_joint", 0.05},
        {"bellows_joint", 0.006636882979390101},
        {"head_pan_joint", -1.7973139598836951e-07},
        {"head_tilt_joint",0.0024005013786707607},
        {"shoulder_pan_joint", 1.3199500661528623},
        {"shoulder_lift_joint", 1.3999822887756963},
        {"upperarm_roll_joint", -0.19998775461277418},
        {"elbow_flex_joint",1.7199706352473747},
        {"forearm_roll_joint", 1.3521167669949818e-06},
        {"wrist_flex_joint", 1.6600028174761388},
        {"wrist_roll_joint", -2.1971452301983163e-07},
        {"l_gripper_finger_joint",0.05003185444046081},
        {"r_gripper_finger_joint", 0.050057917261335}};

        configure_orthogonal<OrNavigation, CbMoveJoints>(jointValues);
    }

    void runtimeConfigure()
    {
    }
};
} // namespace sm_moveit
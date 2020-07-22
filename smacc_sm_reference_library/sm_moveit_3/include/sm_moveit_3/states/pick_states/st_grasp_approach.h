
#pragma once
namespace sm_moveit_3
{
namespace pick_states
{
// STATE DECLARATION
struct StGraspApproach : smacc::SmaccState<StGraspApproach, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StCloseGripper, SUCCESS>,
        Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StGraspApproach, ABORT>/*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    { 
        configure_orthogonal<OrArm, CbMoveCartesianRelative>();
    }

    void runtimeConfigure()
    {
        auto moveCartesian = this->getOrthogonal<OrArm>()
                                ->getClientBehavior<CbMoveCartesianRelative>();

        geometry_msgs::Vector3 offset;
        offset.z = -0.12;

        moveit_msgs::JointConstraint jc;
        

        // moveit_msgs::Constraints path_constraints;

        // move_group_interface_client::ClMoveGroup* moveGroupSmaccClient_;
        // this->requiresClient(moveGroupSmaccClient_);

        // auto jointValues = moveGroupSmaccClient_->moveGroupClientInterface.getCurrentJointValues();
        // auto jointNames = moveGroupSmaccClient_->moveGroupClientInterface.getJointNames();
        
        // auto it = std::find(jointNames.begin(), jointNames.end(), "torso_lift_joint");
        // int index = std::distance(jointNames.begin(), it);
        // auto getCurrentTorsoValue = jointValues[index];

        // jc.joint_name = "torso_lift_joint";  
        // jc.position = getCurrentTorsoValue;
        // jc.tolerance_above = 0.05;
        // jc.tolerance_below = 0.05;
        // jc.weight = 1.0; 
        
        // path_constraints.joint_constraints.push_back(jc);

        // moveGroupSmaccClient_->moveGroupClientInterface.setPathConstraints(path_constraints);

        moveCartesian->offset_= offset;
        //moveCartesian->group_ = "arm";

    }
};
} // namespace pick_states
} // namespace sm_moveit_3
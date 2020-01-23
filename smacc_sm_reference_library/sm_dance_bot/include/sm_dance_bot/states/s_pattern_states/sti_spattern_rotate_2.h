namespace sm_dance_bot
{
namespace s_pattern_states
{
struct StiSPatternRotate2 : smacc::SmaccState<StiSPatternRotate2, SS>
{
    using SmaccState::SmaccState;

    typedef mpl::list<smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternForward2>,
                      smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternForward1>>
        reactions;

    static void staticConfigure()
    {
        float offset = 7;
        float angle = 0;
        if (SS::direction() == TDirection::LEFT)
            angle = 90 + offset ;
        else
            angle = -90 - offset;

        configure_orthogonal<OrNavigation, CbRotate>(angle);
        configure_orthogonal<OrLED, CbLEDOff>();
    }

    void runtimeConfiguration()
    {
        auto &superstate = this->context<SS>();
        ROS_INFO("[StiSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());
    }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot
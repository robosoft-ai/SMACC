namespace sm_dance_bot_3
{
namespace s_pattern_states
{
struct StiSPatternRotate3 : smacc::SmaccState<StiSPatternRotate3, SS>
{
    using SmaccState::SmaccState;

    typedef mpl::list<smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternForward3>,
                      smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternForward2>>
        reactions;

    static void staticConfigure()
    {
    }

    void runtimeConfiguration()
    {
        auto &superstate = this->context<SS>();
        ROS_INFO("[StiSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());

        float offset = 7;
        float angle = 0;
        if (superstate.direction() == TDirection::LEFT)
            angle = -90 - offset;
        else
            angle = +90 + offset;

        this->configure<OrNavigation, CbRotate>(angle);
        this->configure<OrLED, CbLEDOff>();
    }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot_3
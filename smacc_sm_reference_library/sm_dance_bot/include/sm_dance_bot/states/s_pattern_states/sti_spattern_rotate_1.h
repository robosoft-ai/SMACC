namespace sm_dance_bot
{
namespace s_pattern_states
{
struct StiSPatternRotate1 : smacc::SmaccState<StiSPatternRotate1, SS>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
        smacc::Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StiSPatternForward1>,
        smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StiSPatternLoopStart>>
        reactions;

    static void onDefinition()
    {
    }

    void onInitialize()
    {
        auto &superstate = this->context<SS>();
        ROS_INFO("[StiSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());

        if (superstate.iteration_count < SS::total_iterations())
        {
            float angle = 0;
            if (superstate.direction() == TDirection::LEFT)
                angle = 90;
            else
                angle = -90;

            this->configure<OrNavigation, CbAbsoluteRotate>(angle);
            this->configure<OrLED, CbLEDOff>();
        }
    }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot
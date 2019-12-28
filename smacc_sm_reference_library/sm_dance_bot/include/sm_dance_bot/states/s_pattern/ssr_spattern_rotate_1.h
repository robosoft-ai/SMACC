struct SsrSPatternRotate1 : smacc::SmaccState<SsrSPatternRotate1, SS>
{
    using SmaccState::SmaccState;

    typedef mpl::list<
                    smacc::transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SsrSPatternForward1>,
                    smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, SsrSPatternLoopStart>
                    >
                    reactions;

    static void onDefinition()
    {
    }

    void onInitialize()
    {
        auto &superstate = this->context<SS>();
        ROS_INFO("[SsrSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());

        if (superstate.iteration_count < SS::total_iterations())
        {
            float angle = 0;
            if (superstate.direction() == TDirection::LEFT)
                angle = 90;
            else
                angle = -90;

            this->configure<OrNavigation, CbRotate>(angle);
            this->configure<OrTool, CbToolStop>();
        }
    }
};
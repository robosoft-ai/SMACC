struct SsrSPatternRotate2 : smacc::SmaccState<SsrSPatternRotate2, SS>
{
    using SmaccState::SmaccState;

    typedef mpl::list<smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrSPatternForward2>,
                    smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, SsrSPatternForward1>
                    > reactions;

    static void onDefinition()
    {
        float angle = 0;
        if (SS::direction() == TDirection::LEFT)
            angle = -90;
        else
            angle = 90;

        static_configure<NavigationOrthogonal, SbRotate>(angle);
        static_configure<ToolOrthogonal, SbToolStop>();
    }

    void onInitialize()
    {
        auto &superstate = this->context<SS>();
        ROS_INFO("[SsrSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());
    }
};
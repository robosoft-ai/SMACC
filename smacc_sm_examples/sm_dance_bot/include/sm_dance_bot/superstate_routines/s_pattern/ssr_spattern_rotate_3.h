struct SsrSPatternRotate3 : smacc::SmaccState<SsrSPatternRotate3, SS>
{
    using SmaccState::SmaccState;

    typedef mpl::list<smacc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient>, SsrSPatternForward3>,
                    smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, SsrSPatternForward2>
                    > reactions;

    static void onDefinition()
    {
    }

    void onInitialize()
    {
        auto &superstate = this->context<SS>();
        ROS_INFO("[SsrSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());

        float angle = 0;
        if (superstate.direction() == TDirection::LEFT)
            angle = -90;
        else
            angle = 90;

        this->configure<NavigationOrthogonal>(std::make_shared<SbRotate>(angle));
        this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
    }
};
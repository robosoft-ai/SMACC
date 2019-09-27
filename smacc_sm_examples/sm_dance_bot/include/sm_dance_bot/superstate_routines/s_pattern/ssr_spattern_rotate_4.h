struct SsrSPatternRotate4 : smacc::SmaccState<SsrSPatternRotate4, SS>
{
    using SmaccState::SmaccState;

    typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, SsrSPatternForward4> reactions;

    void onInitialize()
    {
        auto &superstate = this->context<SS>();
        ROS_INFO("[SsrSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations);

        float angle = 0;
        if (superstate.direction == TDirection::LEFT)
            angle = 90;
        else
            angle = -90;

        this->configure<NavigationOrthogonal>(std::make_shared<SbRotate>(angle));
        this->configure<ToolOrthogonal>(std::make_shared<SbToolStop>());
    }
};
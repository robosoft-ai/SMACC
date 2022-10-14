namespace sm_calendar_week
{

// STATE DECLARATION
struct StMonday : smacc::SmaccState<StMonday, MsWorkweek>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StTuesday, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StTuesday, PREEMPT>

    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5); // EvTimer triggers once at 10 client ticks
        configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    }

    void runtimeConfigure()
    {
    }

    void onEntry()
    {
        ROS_INFO("On Entry!");
    }

    void onExit()
    {
        ROS_INFO("On Exit!");
    }

};
} // namespace sm_calendar_week

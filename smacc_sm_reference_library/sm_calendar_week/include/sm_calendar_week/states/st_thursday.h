namespace sm_calendar_week
{
// STATE DECLARATION
struct StThursday : smacc::SmaccState<StThursday, MsWorkweek>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StFriday, PREEMPT>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StFriday, SUCCESS>

    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer,  CbTimerCountdownOnce>(5);
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

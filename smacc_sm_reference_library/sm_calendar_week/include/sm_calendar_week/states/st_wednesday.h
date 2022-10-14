namespace sm_calendar_week
{
// STATE DECLARATION
struct StWednesday : smacc::SmaccState<StWednesday, MsWorkweek>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StThursday, PREEMPT>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StThursday, SUCCESS>

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

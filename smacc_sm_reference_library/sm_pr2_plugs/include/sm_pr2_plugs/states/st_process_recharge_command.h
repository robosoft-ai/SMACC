namespace sm_pr2_plugs{

// STATE DECLARATION
struct StProcessRechargeCommand : smacc::SmaccState<StProcessRechargeCommand, MsRecharge>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct TRUE : SUCCESS{};
    struct FALSE : ABORT{};

// TRANSITION TABLE
    typedef mpl::list<
   
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StNavigateToOutlet, TRUE>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StUnplug, FALSE>
    
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
} // namespace sm_pr2_plugs
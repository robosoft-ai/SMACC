namespace sm_pr2_plugs{
// STATE DECLARATION
struct StGoalIsLocal : smacc::SmaccState<StGoalIsLocal, MsRecharge>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct TRUE : SUCCESS{};
    struct FALSE : ABORT{};

// TRANSITION TABLE
    typedef mpl::list<
        
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StSafetyTuck, FALSE>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StUntuckAtOutlet, TRUE>
    
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
} // namespace sm_pr2_plugs
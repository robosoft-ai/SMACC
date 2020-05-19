namespace sm_pr2_plugs{
// STATE DECLARATION
struct StFailStillUnplugged : smacc::SmaccState<StFailStillUnplugged, MsRecharge>
{
    using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
    struct DONE : SUCCESS{};

// TRANSITION TABLE
    typedef mpl::list<
        
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StFetchPlug, PREEMPT>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StFailLowerSpine, SUCCESS>
    
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
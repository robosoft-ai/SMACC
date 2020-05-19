namespace sm_pr2_plugs{
// STATE DECLARATION
struct StPullBackFromWall : smacc::SmaccState<StPullBackFromWall, MsUnplug>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
        
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StWiggleOut, ABORT>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StStowPlug, SUCCESS>
    
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
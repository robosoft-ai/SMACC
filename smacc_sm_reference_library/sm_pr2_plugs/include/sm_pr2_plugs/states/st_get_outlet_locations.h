namespace sm_pr2_plugs{
// STATE DECLARATION
struct StGetOutletLocations : smacc::SmaccState<StGetOutletLocations, MsRecharge>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
        
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StNavigate, SUCCESS>
    
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
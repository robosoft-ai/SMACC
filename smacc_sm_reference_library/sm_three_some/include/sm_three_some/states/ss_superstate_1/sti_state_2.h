namespace sm_three_some
{
namespace ss1_states
{
// STATE DECLARATION
struct StiState2 : smacc::SmaccState<StiState2, SS>
{
  using SmaccState::SmaccState;

// DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : SUCCESS{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StiState3, TIMEOUT>,
  Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiState3, NEXT>,
  Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StiState1, PREVIOUS>
  
  >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10);
    configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
    configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
    // get reference to the client
    ClRosTimer *client;
    this->requiresClient(client);

    // subscribe to the timer client callback
    client->onTimerTick(&StiState2::onTimerClientTickCallback, this);

    // getting reference to the single countdown behavior
    auto *cbsingle = this->getOrthogonal<OrTimer>()
                         ->getClientBehavior<CbTimerCountdownOnce>();

    // subscribe to the single countdown behavior callback
    cbsingle->onTimerTick(&StiState2::onSingleBehaviorTickCallback, this);
  }
  
  void onEntry()
  {
    ROS_INFO("On Entry!");
  }

  void onExit()
  {
    ROS_INFO("On Exit!");
  }

  void onTimerClientTickCallback()
  {
    ROS_INFO("timer client tick!");
  }

  void onSingleBehaviorTickCallback()
  {
    ROS_INFO("single behavior tick!");
  }
};
}
}
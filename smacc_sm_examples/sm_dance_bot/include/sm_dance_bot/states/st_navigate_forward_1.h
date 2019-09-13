struct st_navigate_forward_1 : smacc::SmaccState<st_navigate_forward_1, sm_dance_bot>
{
  typedef mpl::list<sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv, st_rotate_degrees_2>,
                    sc::transition<smacc::KeyPressEvent<'n'>, st_rotate_degrees_2>
                    > reactions; 

  using SmaccState::SmaccState;

  // Key N -> next state
  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new sb_navigate_forward(1));
    this->configure<ToolOrthogonal>(new sb_tool_stop());
    this->configure<KeyboardOrthogonal>(new sb_keyboard());
  }
};
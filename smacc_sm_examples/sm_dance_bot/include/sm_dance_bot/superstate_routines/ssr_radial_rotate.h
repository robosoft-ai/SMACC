struct ssr_radial_rotate: smacc::SmaccState<ssr_radial_rotate,SS>
{
  using SmaccState::SmaccState;

  //typedef mpl::list<sc::transition<smacc::SmaccMoveBaseActionClient::SuccessEv,  ssr_radial_end_point>
  //                , sc::custom_reaction<smacc::SmaccMoveBaseActionClient::SuccessEv> > reactions; 

  typedef sc::custom_reaction<smacc::SmaccMoveBaseActionClient::SuccessEv> reactions;
  
  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new sb_rotate(10));
    this->configure<ToolOrthogonal>(new sb_tool_stop());
  }

   sc::result react(const smacc::SmaccMoveBaseActionClient::SuccessEv &ev)
   {
      ROS_INFO("CUSTOM REACTION MOVE BASE EVENT SUCCESS");
      return this->transit<SS1::ssr_radial_end_point>();
   }
  
};
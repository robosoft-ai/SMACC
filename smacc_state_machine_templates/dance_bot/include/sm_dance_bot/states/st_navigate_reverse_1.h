struct st_navigate_reverse_1: smacc::SmaccState<st_navigate_reverse_1,sm_dance_bot>
{
  using SmaccState::SmaccState;
  
   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(new sb_navigate_forward(-10));
      this->configure<ToolOrthogonal>(new sb_tool_stop());
   }

};

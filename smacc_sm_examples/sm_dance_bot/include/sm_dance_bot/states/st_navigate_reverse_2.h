struct st_navigate_reverse_2: smacc::SmaccState<st_navigate_reverse_2,sm_dance_bot>
{
  using SmaccState::SmaccState;

   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(new sb_navigate_backwards(1));
      this->configure<ToolOrthogonal>(new sb_tool_stop());
   }
};

#pragma once

namespace sm_moveit_4
{
namespace pick_states
{
// STATE DECLARATION
struct StCloseGripper : smacc::SmaccState<StCloseGripper, SS>
{
   using SmaccState::SmaccState;

   // TRANSITION TABLE
   typedef mpl::list<
       Transition<EvActionSucceeded<ClGripper, OrGripper>, StGraspRetreat>>
       reactions;

   // STATE FUNCTIONS
   static void staticConfigure()
   {
      configure_orthogonal<OrGripper, CbCloseGripper>();
   }

   void onEntry()
   {
      ros::Duration(1.0).sleep();
   }

   void onExit()
   {
      ros::Duration(1.0).sleep();
   }
};
} // namespace pick_states
} // namespace sm_moveit_4
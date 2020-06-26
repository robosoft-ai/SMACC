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

   void onExit()
   {
      ClMoveGroup *moveGroupClient;
      this->requiresClient(moveGroupClient);

      //moveGroupClient->planningSceneInterface.removeCollisionObjects({"cube_0"});
      ros::Duration(0.5).sleep();

      // moveGroupClient->moveGroupClientInterface.attachObject("cube_0");

      // moveGroupClient->moveGroupClientInterface.attachObject("cube_0");
      // moveGroupClient->planningSceneInterface.removeCollisionObjects({"cube_0"});

      /*
         std::vector<std::string> touch_links{"l_gripper_finger_link", "r_gripper_finger_link"};
         moveGroupClient->moveGroupClientInterface.attachObject("cube_0", "gripper_link", touch_links);
         moveGroupClient->moveGroupClientInterface.setSupportSurfaceName("table_0");
         */

      // planning_scene.world.collision_objects.clear();
      // planning_scene.world.collision_objects.push_back(remove_object);
      // planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
   }
};
} // namespace pick_states
} // namespace sm_moveit_4
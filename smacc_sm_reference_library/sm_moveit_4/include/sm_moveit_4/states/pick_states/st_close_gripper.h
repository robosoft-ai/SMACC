
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
            configure_orthogonal<OrArm, CbAttachObject>();
         }

         void runtimeConfigure()
         {
            ClPerceptionSystem *perceptionSystem;
            this->requiresClient(perceptionSystem);

            auto cubeinfo = perceptionSystem->getTargetCurrentCubeInfo();

            auto cbattach = this->getOrthogonal<OrArm>()->getClientBehavior<CbAttachObject>();
            cbattach->targetObjectName_ = cubeinfo->pose_->getFrameId();
         }

         void onEntry()
         {
            ros::Duration(1.0).sleep();
         }

         void onExit()
         {
            ClMoveGroup *moveGroupClient;
            this->requiresClient(moveGroupClient);


            //auto cubepos = cubeinfo->pose_->toPoseStampedMsg();
            // //std::vector<moveit_msgs::CollisionObject> collisionObjects;
            // moveit_msgs::CollisionObject cubeCollision;
            // auto time = ros::Time::now();

            // createCollisionBox(cubepos.pose.position.x, cubepos.pose.position.y, cubepos.pose.position.z, 0.06, 0.06, 0.06, "collisioncube", "map",  cubeCollision, time, moveit_msgs::CollisionObject::ADD);
            // planningSceneInterface.applyCollisionObject(cubeCollision);

            // // collisionObjects.push_back(cubeCollision);
            
            // std::vector<std::string> touch_links{"l_gripper_finger_link", "r_gripper_finger_link"};
            // moveGroupClient->moveGroupClientInterface.attachObject("collisioncube", "gripper_link", touch_links);
            // //moveGroupClient->moveGroupClientInterface.setSupportSurfaceName("table_1");
            
            moveGroupClient->getComponent<CpConstraintTableWorkspaces>()->disableTableCollisionVolume();

            ros::Duration(1.0).sleep();
         }
      };
   } // namespace pick_states
} // namespace sm_moveit_4
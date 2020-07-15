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
            ClMoveGroup *moveGroupClient;
            this->requiresClient(moveGroupClient);

            ClPerceptionSystem *perceptionSystem;
            this->requiresClient(perceptionSystem);

            auto &planningSceneInterface = moveGroupClient->planningSceneInterface;
            auto cubeinfo = perceptionSystem->getTargetCurrentCubeInfo();
            auto cubepos = cubeinfo->pose_->toPoseStampedMsg();

            //std::vector<moveit_msgs::CollisionObject> collisionObjects;
            moveit_msgs::CollisionObject cubeCollision;
            auto time = ros::Time::now();

            createCollisionBox(cubepos.pose.position.x, cubepos.pose.position.y, cubepos.pose.position.z, 0.06, 0.06, 0.06, "collisioncube", "map",  cubeCollision, time, moveit_msgs::CollisionObject::ADD);
            planningSceneInterface.applyCollisionObject(cubeCollision);

            // collisionObjects.push_back(cubeCollision);
            
            std::vector<std::string> touch_links{"l_gripper_finger_link", "r_gripper_finger_link"};
            moveGroupClient->moveGroupClientInterface.attachObject("collisioncube", "gripper_link", touch_links);
            //moveGroupClient->moveGroupClientInterface.setSupportSurfaceName("table_1");
            
            moveGroupClient->getComponent<CpConstraintTableWorkspaces>()->disableTableCollisionVolume();

            ros::Duration(1.0).sleep();
         }

         void createCollisionBox(float x, float y, float z, float xl, float yl, float zl, std::string id, std::string frameid, moveit_msgs::CollisionObject &collision, const ros::Time &time, int addOrRemove)
         {
            collision.operation = addOrRemove;
            collision.id = id;

            collision.primitives.resize(1);
            collision.primitives[0].type = collision.primitives[0].BOX;
            collision.primitives[0].dimensions.resize(3);

            collision.primitives[0].dimensions[0] = xl;
            collision.primitives[0].dimensions[1] = yl;
            collision.primitives[0].dimensions[2] = zl;

            collision.primitive_poses.resize(1);
            collision.primitive_poses[0].position.x = x;
            collision.primitive_poses[0].position.y = y;
            collision.primitive_poses[0].position.z = z;
            collision.primitive_poses[0].orientation.w = 1.0;

            collision.header.frame_id = frameid;
            collision.header.stamp = time;
         }
      };
   } // namespace pick_states
} // namespace sm_moveit_4
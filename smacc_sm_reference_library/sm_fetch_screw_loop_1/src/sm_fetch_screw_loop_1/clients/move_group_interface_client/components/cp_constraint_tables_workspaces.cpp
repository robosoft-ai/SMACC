#include <sm_fetch_screw_loop_1/clients/move_group_interface_client/components/cp_constraint_tables_workspaces.h>

namespace sm_fetch_screw_loop_1
{
    namespace cl_move_group_interface
    {
        void CpConstraintTableWorkspaces::setBigTableCollisionVolume()
        {
            safeTableHeightOffsetForCubeCollisions = 0.17;
            this->update();
        }

        void CpConstraintTableWorkspaces::setSmallTableCollisionVolume()
        {
            safeTableHeightOffsetForCubeCollisions = 0;
            this->update();
        }

        void CpConstraintTableWorkspaces::disableTableCollisionVolume()
        {
            safeTableHeightOffsetForCubeCollisions = -1;
            this->update();
        }

        void CpConstraintTableWorkspaces::onInitialize()
        {
            ClMoveGroup *movegroupclient;
            this->requiresClient(movegroupclient);
            planningSceneInterface_ = &movegroupclient->planningSceneInterface;

            ClPerceptionSystem *perceptionSystem;
            this->requiresClient(perceptionSystem);

            sceneState_ = perceptionSystem->sceneState_;
        }

        void CpConstraintTableWorkspaces::update()
        {
            std::vector<moveit_msgs::CollisionObject> collisionObjects(this->sceneState_->tablesInfo_.size());

            if (safeTableHeightOffsetForCubeCollisions != -1)
            {
                auto thickness = 0.12;
                int i = 0;
                for (auto &table : this->sceneState_->tablesInfo_)
                {
                    auto tablepose = table.pose_->toPoseStampedMsg();
                    std::string tablename = "table_" + std::to_string(i + 1);

                    //removeCollisionObjectNames[i] = tablename;
                    moveit_msgs::CollisionObject &collision = collisionObjects[i];
                    collision.operation = moveit_msgs::CollisionObject::ADD;
                    collision.id = tablename;
                    collision.primitives.resize(1);
                    collision.primitives[0].type = collision.primitives[0].BOX;
                    collision.primitives[0].dimensions.resize(3);
                    collision.primitives[0].dimensions[0] = 1.05;
                    collision.primitives[0].dimensions[1] = 1.4;
                    collision.primitives[0].dimensions[2] = 0.001 + thickness + safeTableHeightOffsetForCubeCollisions;

                    /* Define the pose of the table. */
                    collision.primitive_poses.resize(1);
                    collision.primitive_poses[0].position.x = tablepose.pose.position.x;
                    collision.primitive_poses[0].position.y = tablepose.pose.position.y;
                    collision.primitive_poses[0].position.z = 0.7 - thickness * 0.5;
                    collision.primitive_poses[0].orientation.w = 1.0;

                    collision.header.frame_id = tablepose.header.frame_id;
                    collision.header.stamp = tablepose.header.stamp;
                    i++;
                }

                this->planningSceneInterface_->applyCollisionObjects(collisionObjects);
            }
            else
            {
                int i = 0;
                std::vector<std::string> disableNames;
                for (auto &table : this->sceneState_->tablesInfo_)
                {
                    auto tablepose = table.pose_->toPoseStampedMsg();
                    std::string tablename = "table_" + std::to_string(i + 1);
                    disableNames.push_back(tablename);
                    i++;
                }

                this->planningSceneInterface_->removeCollisionObjects(disableNames);
            }
        }
    } // namespace cl_move_group_interface

} // namespace sm_fetch_screw_loop_1
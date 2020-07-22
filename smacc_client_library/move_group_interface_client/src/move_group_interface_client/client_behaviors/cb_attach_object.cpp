#include <move_group_interface_client/client_behaviors/cb_attach_object.h>
#include <move_group_interface_client/components/cp_grasping_objects.h>

namespace move_group_interface_client
{
    CbAttachObject::CbAttachObject(std::string targetObjectName)
        : targetObjectName_(targetObjectName)
    {
    }

    CbAttachObject::CbAttachObject()
    {
        
    }

    void CbAttachObject::onEntry()
    {
        move_group_interface_client::ClMoveGroup *moveGroup;
        this->requiresClient(moveGroup);

        move_group_interface_client::GraspingComponent *graspingComponent;
        this->requiresComponent(graspingComponent);

        // auto cubepos = cubeinfo->pose_->toPoseStampedMsg();

        moveit_msgs::CollisionObject targetCollisionObject;

        bool found = graspingComponent->getGraspingObject(targetObjectName_, targetCollisionObject);

        if (found)
        {
            targetCollisionObject.operation = moveit_msgs::CollisionObject::ADD;
            targetCollisionObject.header.stamp = ros::Time::now();

            moveGroup->planningSceneInterface.applyCollisionObject(targetCollisionObject);
            // collisionObjects.push_back(cubeCollision);

            graspingComponent->currentAttachedObjectName = targetObjectName_;
            moveGroup->moveGroupClientInterface.attachObject(targetObjectName_, "gripper_link", graspingComponent->fingerTipNames);
        }
    }

    void CbAttachObject::onExit()
    {
    }
} // namespace move_group_interface_client
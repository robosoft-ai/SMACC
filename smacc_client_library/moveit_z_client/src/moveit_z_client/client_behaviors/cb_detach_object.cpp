#include <moveit_z_client/client_behaviors/cb_detach_object.h>
#include <moveit_z_client/cl_movegroup.h>

namespace moveit_z_client
{
    void CbDetachObject::onEntry()
    {
        moveit_z_client::GraspingComponent *graspingComponent;
        this->requiresComponent(graspingComponent);

        moveit_z_client::ClMoveGroup *moveGroupClient;
        this->requiresClient(moveGroupClient);

        auto &planningSceneInterface = moveGroupClient->planningSceneInterface;

        moveGroupClient->moveGroupClientInterface.detachObject(*(graspingComponent->currentAttachedObjectName));
        planningSceneInterface.removeCollisionObjects({*(graspingComponent->currentAttachedObjectName)});
    }

    void CbDetachObject::onExit()
    {
    }
} // namespace moveit_z_client

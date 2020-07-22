#include <move_group_interface_client/client_behaviors/cb_detach_object.h>
#include <move_group_interface_client/cl_movegroup.h>

namespace move_group_interface_client
{
    void CbDetachObject::onEntry()
    {
        move_group_interface_client::GraspingComponent *graspingComponent;
        this->requiresComponent(graspingComponent);

        move_group_interface_client::ClMoveGroup *moveGroupClient;
        this->requiresClient(moveGroupClient);

        auto &planningSceneInterface = moveGroupClient->planningSceneInterface;

        moveGroupClient->moveGroupClientInterface.detachObject(*(graspingComponent->currentAttachedObjectName));
        planningSceneInterface.removeCollisionObjects({*(graspingComponent->currentAttachedObjectName)});
    }

    void CbDetachObject::onExit()
    {
    }
} // namespace move_group_interface_client

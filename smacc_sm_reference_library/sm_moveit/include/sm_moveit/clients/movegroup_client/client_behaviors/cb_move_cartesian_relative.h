#pragma once

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveCartesianRelative : public smacc::SmaccClientBehavior
{
public:
    CbMoveCartesianRelative(geometry_msgs::Vector3 offset, std::string frameId)
    {
    }

    virtual void onEntry() override
    {
        // ClMoveGroup *movegroupClient;

        // this->requiresClient(movegroupClient);

        // cl_move_base_z::Pose targetObjectPose("/cube_0", "/map");

        // moveToObjectGraspPose(movegroupClient->moveGroupClientInterface,
        //                      movegroupClient->planningSceneInterface,
        //                      targetObjectPose);
    }

    virtual void onExit() override
    {
    }

    void executeAsync()
    {
    }
};
} // namespace cl_movegroup
} // namespace sm_moveit
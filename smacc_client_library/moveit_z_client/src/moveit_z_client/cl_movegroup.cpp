#include <moveit_z_client/cl_movegroup.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

namespace sm_moveit
{
namespace cl_movegroup
{

ClMoveGroup::ClMoveGroup(std::string groupName)
    : moveGroupClientInterface(groupName)
{
    ros::WallDuration(10.0).sleep();
}

ClMoveGroup::~ClMoveGroup()
{
}

void ClMoveGroup::postEventMotionExecutionSucceded()
{
    ROS_INFO("[ClMoveGroup] Post Motion Success Event");
    postEventMotionExecutionSucceded_();
}

void ClMoveGroup::postEventMotionExecutionFailed()
{
    ROS_INFO("[ClMoveGroup] Post Motion Failure Event");
    postEventMotionExecutionFailed_();
}

} // namespace cl_movegroup
} // namespace sm_moveit
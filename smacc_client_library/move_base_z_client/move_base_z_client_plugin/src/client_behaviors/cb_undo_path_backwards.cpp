#include <move_base_z_client_plugin/client_behaviors/cb_undo_path_backwards.h>

namespace cl_move_base_z
{
using namespace ::cl_move_base_z::odom_tracker;

void CbUndoPathBackwards::onEntry()
{
    this->requiresClient(moveBaseClient_);
    auto *odomTracker = moveBaseClient_->getComponent<OdomTracker>();

    auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();

    nav_msgs::Path forwardpath = odomTracker->getPath();
    //ROS_INFO_STREAM("[UndoPathBackward] Current path backwards: " << forwardpath);

    odomTracker->setWorkingMode(WorkingMode::CLEAR_PATH_BACKWARD);

    ClMoveBaseZ::Goal goal;
    if (forwardpath.poses.size() > 0)
    {
        goal.target_pose = forwardpath.poses.front();
        plannerSwitcher->setBackwardPlanner();
        moveBaseClient_->sendGoal(goal);
    }
}

void CbUndoPathBackwards::onExit()
{
    this->requiresClient(moveBaseClient_);
    auto *odomTracker = moveBaseClient_->getComponent<OdomTracker>();
    odomTracker->popPath();
}

} // namespace cl_move_base_z
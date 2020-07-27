/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_end_effector_trajectory.h>
#include <moveit_msgs/GetPositionIK.h>

namespace cl_move_group_interface
{
    CbMoveEndEffectorTrajectory::CbMoveEndEffectorTrajectory(const std::vector<geometry_msgs::PoseStamped> &endEffectorTrajectory)
        : endEffectorTrajectory_(endEffectorTrajectory)

    {
    }

    void CbMoveEndEffectorTrajectory::onEntry()
    {
        moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;

        this->requiresClient(movegroupClient_);
        // get current robot state
        auto currentState = movegroupClient_->moveGroupClientInterface.getCurrentState();

        // get the IK client
        ros::NodeHandle nh;
        auto iksrv = nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
        auto groupname = movegroupClient_->moveGroupClientInterface.getName();
        auto currentjointnames = currentState->getJointModelGroup(groupname)->getActiveJointModelNames();

        std::vector<double> jointPositions;
        currentState->copyJointGroupPositions(groupname, jointPositions);

        std::vector<std::vector<double>> trajectory;
        trajectory.push_back(jointPositions);

        for (int k = 0; k < this->endEffectorTrajectory_.size(); k++)
        {
            auto &pose = this->endEffectorTrajectory_[k];
            moveit_msgs::GetPositionIKRequest req;
            req.ik_request.robot_state.joint_state.name = currentjointnames;
            req.ik_request.robot_state.joint_state.position = jointPositions;

            req.ik_request.group_name = groupname;
            req.ik_request.avoid_collisions = true;

            moveit_msgs::GetPositionIKResponse res;

            pose.header.stamp = ros::Time::now();
            req.ik_request.pose_stamped = pose;

            ROS_WARN_STREAM("IK request: " << req);
            if (iksrv.call(req, res))
            {
                auto &prevtrajpoint = trajectory.back();
                //jointPositions.clear();

                std::stringstream ss;
                for (int j = 0; j < res.solution.joint_state.position.size(); j++)
                {
                    auto &jointname = res.solution.joint_state.name[j];
                    auto it = std::find(currentjointnames.begin(), currentjointnames.end(), jointname);
                    if (it != currentjointnames.end())
                    {
                        int index = std::distance(currentjointnames.begin(), it);
                        jointPositions[index] = res.solution.joint_state.position[j];
                        ss << jointname << "(" << index << "): " << jointPositions[index] << std::endl;
                    }
                }

                // continuity check
                bool discontinuity = false;
                if(k!=0)
                {
                    for (int j = 0; j < jointPositions.size(); j++)
                    {
                        auto deltajoint = jointPositions[j] - prevtrajpoint[j];

                        if (fabs(deltajoint) > 0.07 /*2 deg*/)
                        {
                            discontinuity = true;
                            ROS_ERROR_STREAM("IK discontinuity " << currentjointnames[j] << " : " << deltajoint );
                            break;
                        }
                    }
                }

                if (discontinuity)
                {
                    k--;
                    ROS_ERROR("IK discontinuity detected, retrying");
                }
                else
                {
                    trajectory.push_back(jointPositions);
                    ROS_WARN_STREAM("IK solution: " << res.solution.joint_state);
                    ROS_WARN_STREAM("trajpoint: " << std::endl
                                                  << ss.str());
                }
            }
            else
            {
                ROS_ERROR("[CbMoveEndEffectorTrajectory] wrong IK call");
            }

            ROS_WARN_STREAM("-----");
        }

        // interpolate speeds?

        // interpolate accelerations?

        // get current robot state
        // fill plan message
        computedMotionPlan.start_state_.joint_state.name = currentjointnames;

        computedMotionPlan.start_state_.joint_state.position = trajectory.front();
        computedMotionPlan.trajectory_.joint_trajectory.joint_names = currentjointnames;

        double t = 0;
        for (auto &p : trajectory)
        {
            trajectory_msgs::JointTrajectoryPoint jp;
            jp.positions = p;
            jp.time_from_start = ros::Duration(t);
            t += 1.0;
            computedMotionPlan.trajectory_.joint_trajectory.points.push_back(jp);
        }

        // call execute
        this->movegroupClient_->moveGroupClientInterface.execute(computedMotionPlan.trajectory_);

        // handle finishing events
    }
} // namespace cl_move_group_interface
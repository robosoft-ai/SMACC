/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_end_effector_trajectory.h>
#include <moveit_msgs/GetPositionIK.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

namespace cl_move_group_interface
{
    CbMoveEndEffectorTrajectory::CbMoveEndEffectorTrajectory(std::string tipLink)
        : tipLink_(tipLink)
    {
        initializeROS();
    }

    CbMoveEndEffectorTrajectory::CbMoveEndEffectorTrajectory(const std::vector<geometry_msgs::PoseStamped> &endEffectorTrajectory, std::string tipLink)
        : endEffectorTrajectory_(endEffectorTrajectory), tipLink_(tipLink)

    {
        initializeROS();
    }

    void CbMoveEndEffectorTrajectory::initializeROS()
    {
        ros::NodeHandle nh;
        markersPub_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1);
        iksrv_ = nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
    }

    void CbMoveEndEffectorTrajectory::onEntry()
    {
        moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;

        this->requiresClient(movegroupClient_);

        this->generateTrajectory();

        if (this->endEffectorTrajectory_.size() == 0)
        {
            ROS_WARN_STREAM("[" << smacc::demangleSymbol(typeid(*this).name()) << "] No points in the trajectory. Skipping behavior.");
            return;
        }

        this->createMarkers();
        markersInitialized_ = true;

        // get current robot state
        auto currentState = movegroupClient_->moveGroupClientInterface.getCurrentState();

        // get the IK client
        auto groupname = movegroupClient_->moveGroupClientInterface.getName();
        auto currentjointnames = currentState->getJointModelGroup(groupname)->getActiveJointModelNames();

        if (!tipLink_ || *tipLink_ == "")
        {
            tipLink_ = movegroupClient_->moveGroupClientInterface.getEndEffectorLink();
        }

        std::vector<double> jointPositions;
        currentState->copyJointGroupPositions(groupname, jointPositions);

        std::vector<std::vector<double>> trajectory;
        std::vector<ros::Duration> trajectoryTimeStamps;

        trajectory.push_back(jointPositions);
        trajectoryTimeStamps.push_back(ros::Duration(0));

        auto &first = endEffectorTrajectory_.front();
        ros::Time referenceTime = first.header.stamp;

        for (int k = 0; k < this->endEffectorTrajectory_.size(); k++)
        {
            auto &pose = this->endEffectorTrajectory_[k];
            moveit_msgs::GetPositionIKRequest req;

            req.ik_request.ik_link_name = *tipLink_;
            req.ik_request.robot_state.joint_state.name = currentjointnames;
            req.ik_request.robot_state.joint_state.position = jointPositions;

            req.ik_request.group_name = groupname;
            req.ik_request.avoid_collisions = true;

            moveit_msgs::GetPositionIKResponse res;

            //pose.header.stamp = ros::Time::now();
            req.ik_request.pose_stamped = pose;

            ROS_WARN_STREAM("IK request: " << req);
            if (iksrv_.call(req, res))
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
                int jointindex = 0;
                double deltajoint;
                if (k != 0)
                {
                    for (jointindex = 0; jointindex < jointPositions.size(); jointindex++)
                    {
                        deltajoint = jointPositions[jointindex] - prevtrajpoint[jointindex];

                        discontinuity = fabs(deltajoint) > 0.07 /*2 deg*/;

                        if (discontinuity)
                        {
                            break;
                        }
                    }
                }

                if (discontinuity)
                {
                    std::stringstream ss;
                    ss << "Traj[" << k << "/" << endEffectorTrajectory_.size() << "] " << currentjointnames[jointindex] << " IK discontinuity : " << deltajoint << std::endl
                       << "prev joint value: " << prevtrajpoint[jointindex] << std::endl
                       << "current joint value: " << jointPositions[jointindex] << std::endl;

                    for (int kindex = 0; kindex < trajectory.size(); kindex++)
                    {
                        ss << "[" << kindex << "]: " << trajectory[kindex][jointindex] << std::endl;
                    }

                    ROS_ERROR_STREAM(ss.str());
                    k--;

                    movegroupClient_->postEventMotionExecutionFailed();
                    this->postFailureEvent();
                    return;
                }
                else
                {
                    trajectory.push_back(jointPositions);
                    ros::Duration durationFromStart = pose.header.stamp - referenceTime;
                    trajectoryTimeStamps.push_back(durationFromStart);

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

        int i = 0;
        for (auto &p : trajectory)
        {
            trajectory_msgs::JointTrajectoryPoint jp;
            jp.positions = p;
            jp.time_from_start = trajectoryTimeStamps[i]; //ros::Duration(t);
            computedMotionPlan.trajectory_.joint_trajectory.points.push_back(jp);
            i++;
        }

        // call execute
        auto executionResult = this->movegroupClient_->moveGroupClientInterface.execute(computedMotionPlan.trajectory_);

        if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_INFO_STREAM("[" << this->getName() << "] motion execution succedded");
            movegroupClient_->postEventMotionExecutionSucceded();
            this->postSuccessEvent();
        }
        else
        {
            ROS_INFO_STREAM("[" << this->getName() << "] motion execution failed");
            movegroupClient_->postEventMotionExecutionFailed();
            this->postFailureEvent();
        }

        // handle finishing events
    } // namespace cl_move_group_interface

    void CbMoveEndEffectorTrajectory::update()
    {
        if (markersInitialized_)
        {
            std::lock_guard<std::mutex> guard(m_mutex_);
            markersPub_.publish(beahiorMarkers_);
        }
    }

    void CbMoveEndEffectorTrajectory::onExit()
    {
        markersInitialized_= false;

        std::lock_guard<std::mutex> guard(m_mutex_);
        for(auto& marker: this->beahiorMarkers_.markers)
        {
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::DELETE;
        }

        markersPub_.publish(beahiorMarkers_);
    }

    void CbMoveEndEffectorTrajectory::createMarkers()
    {
        tf::Transform localdirection;
        localdirection.setIdentity();
        localdirection.setOrigin(tf::Vector3(0.05, 0, 0));
        auto frameid = this->endEffectorTrajectory_.front().header.frame_id;

        int i = 0;
        for (auto &pose : this->endEffectorTrajectory_)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frameid;
            marker.header.stamp = ros::Time::now();
            marker.ns = "trajectory";
            marker.id = i++;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.005;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color.a = 0.8;
            marker.color.r = 1.0;
            marker.color.g = 0;
            marker.color.b = 0;

            geometry_msgs::Point start, end;
            start.x = pose.pose.position.x;
            start.y = pose.pose.position.y;
            start.z = pose.pose.position.z;

            tf::Transform basetransform;
            tf::poseMsgToTF(pose.pose, basetransform);
            tf::Transform endarrow = localdirection * basetransform;

            end.x = endarrow.getOrigin().x();
            end.y = endarrow.getOrigin().y();
            end.z = endarrow.getOrigin().z();

            marker.pose.orientation.w = 1;
            marker.points.push_back(start);
            marker.points.push_back(end);

            beahiorMarkers_.markers.push_back(marker);
        }
    }

    void CbMoveEndEffectorTrajectory::generateTrajectory()
    {
        // bypass current trajectory, overriden in derived classes
        // this->endEffectorTrajectory_ = ...
    }
} // namespace cl_move_group_interface
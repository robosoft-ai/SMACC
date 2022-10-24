#pragma once

#include <move_group_interface_client/cl_movegroup.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

#include <smacc/smacc_client.h>
#include "components/cp_simulated_gazebo_perception.h"
#include "components/cp_scene_state.h"

#include <boost/algorithm/string.hpp>

namespace sm_fetch_two_table_whiskey_pour
{
    const int TABLE_COUNT = 6;
    const int CUBE_COUNT = 2;

    using namespace cl_move_base_z;

    namespace cl_perception_system
    {
        class ClPerceptionSystem : public smacc::ISmaccClient, public smacc::ISmaccUpdatable
        {

        public:
            int decissionsCount;
            int currentCube = 7;

            // subcomponents
            CpSceneState *sceneState_;
            CpSimulatedGazeboPerception *gazeboPerceptionSimulation_;

            ClPerceptionSystem()
            {
                decissionsCount = 0;
            }

            virtual ~ClPerceptionSystem()
            {
            }

            template <typename TOrthogonal, typename TSourceObject>
            void onOrthogonalAllocation()
            {
                // order matters, since gazeboPerception updates sceneState
                sceneState_ = this->createComponent<CpSceneState, TSourceObject, TOrthogonal>(CUBE_COUNT, TABLE_COUNT);
                gazeboPerceptionSimulation_ = this->createComponent<CpSimulatedGazeboPerception, TSourceObject, TOrthogonal>();
            }

            geometry_msgs::PoseStamped getMainTablePose()
            {
                for (auto &table : this->sceneState_->tablesInfo_)
                {
                    if (table.associatedCubeColor_ == "none")
                    {
                        return table.pose_->toPoseStampedMsg();
                    }
                }

                geometry_msgs::PoseStamped empty;
                return empty;
            }

            geometry_msgs::PoseStamped getTargetTablePose()
            {
                auto currentCube = getTargetCurrentCubeInfo();
                return currentCube->dstTableInfo_->pose_->toPoseStampedMsg();
            }

            void printCubesState()
            {
                std::stringstream ss;
                int i = 0;
                for (auto &c : this->sceneState_->cubeInfos_)
                {

                    ss << "- Cube " << i++ << " frame: " << c.pose_->toPoseStampedMsg().header.frame_id << " location: " << (c.location_ == CubeLocation::ORIGIN_TABLE ? " origin table" : " destination table") << std::endl;
                }

                ROS_INFO_STREAM(ss.str());
            }

            CubeInfo *nextObject()
            {
                auto currentCube = getTargetCurrentCubeInfo();
                currentCube->location_ = CubeLocation::DESTINY_TABLE;
                currentCube->dstTableInfo_->cubesCounter_++;
                printCubesState();

                return getTargetCurrentCubeInfo();
            }

            virtual void update() override
            {
            }

            CubeInfo *getTargetCurrentCubeInfo()
            {
                for (auto &c : this->sceneState_->cubeInfos_)
                {
                    if (c.location_ == CubeLocation::ORIGIN_TABLE)
                    {
                        return &c;
                    }
                }

                return nullptr;
            }

            bool decidePickCubePose(geometry_msgs::PoseStamped &out)
            {
                // we select here the first cube in the list that is located in the
                // origin table

                auto *c = this->getTargetCurrentCubeInfo();

                if (c != nullptr)
                {
                    out = c->pose_->toPoseStampedMsg();
                    return true;
                }
                else
                {
                    return false;
                }
            }

            bool decidePlacePose(geometry_msgs::PoseStamped &placePose)
            {
                auto *c = this->getTargetCurrentCubeInfo();
                ClMoveBaseZ *moveBaseClient;
                this->requiresClient(moveBaseClient);

                if (c != nullptr && moveBaseClient != nullptr)
                {
                    auto basepose = moveBaseClient->getComponent<Pose>()->toPoseMsg();

                    auto dstTablePose = c->dstTableInfo_->pose_->toPoseStampedMsg();

                    placePose.pose.position.z = 0.7;

                    if (basepose.position.x < dstTablePose.pose.position.x)
                    {
                        placePose.pose.position.x = dstTablePose.pose.position.x - 0.25;
                        placePose.pose.position.y = dstTablePose.pose.position.y + 0.1;
                    }
                    else
                    {
                        placePose.pose.position.x = dstTablePose.pose.position.x + 0.25;
                        placePose.pose.position.y = dstTablePose.pose.position.y + -0.1;
                    }

                    placePose.pose.position.y += -c->dstTableInfo_->cubesCounter_ * 0.1;
                    placePose.pose.orientation.w = 1;

                    placePose.header.frame_id = dstTablePose.header.frame_id;
                    placePose.header.stamp = dstTablePose.header.stamp;

                    return true;
                }

                return false;
            }

            void computePregraspPoseFromCubePose(geometry_msgs::PoseStamped &objectPose)
            {
                objectPose.pose.position.x -= 0.3;
                //objectPose.pose.position.y = 0;

                // ------ cube grasping orientation -------
                // grasp the object with the gripper using top-to-bottom direction
                auto cubeYawOnTable = tf::getYaw(objectPose.pose.orientation);

                const double degrees360 = 2 * M_PI;
                while (cubeYawOnTable > degrees360)
                {
                    cubeYawOnTable -= degrees360;
                }

                while (cubeYawOnTable < -degrees360)
                {
                    cubeYawOnTable += degrees360;
                }

                ROS_INFO("cube yaw: %lf", cubeYawOnTable);
                auto quat = tf::createQuaternionFromRPY(0, 0, cubeYawOnTable);
                tf::quaternionTFToMsg(quat, objectPose.pose.orientation);

                //objectPose.pose.orientation.w = 1;
            }

            bool decidePrePlacePose(geometry_msgs::PoseStamped &preplacePose)
            {
                if (this->decidePlacePose(preplacePose))
                {
                    ROS_INFO_STREAM("[ClPerceptionSystem] Decided pre place pose for cube: " << preplacePose);

                    preplacePose.pose.position.x -= 0;
                    preplacePose.pose.position.z += 0.1;
                    computePlaceCubeGraspingOrientation(preplacePose);
                    return true;
                }

                return false;
            }

            void computePlaceCubeGraspingOrientation(geometry_msgs::PoseStamped &objectPose)
            {
                // ------ cube grasping orientation -------
                // grasp the object with the gripper using top-to-bottom direction
                auto cubeYawOnTable = tf::getYaw(objectPose.pose.orientation);

                const double degrees90 = M_PI / 2;
                while (cubeYawOnTable > degrees90)
                {
                    cubeYawOnTable -= degrees90;
                }

                while (cubeYawOnTable < -degrees90)
                {
                    cubeYawOnTable += degrees90;
                }

                ROS_INFO("[ClPerceptionSystem] cube yaw: %lf", cubeYawOnTable);
                auto quat = tf::createQuaternionFromRPY(0, M_PI / 2, cubeYawOnTable);
                tf::quaternionTFToMsg(quat, objectPose.pose.orientation);
            }
        };
    } // namespace cl_perception_system
} // namespace sm_fetch_two_table_whiskey_pour

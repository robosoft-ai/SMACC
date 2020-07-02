#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

#include <smacc/smacc_client.h>
#include "components/cp_simulated_gazebo_perception.h"
#include "components/cp_scene_state.h"

#include <boost/algorithm/string.hpp>

namespace sm_moveit_4
{
    namespace cl_perception_system
    {
        enum class RobotProcessStatus : int
        {
            TABLE0 = 0,
            TABLE1 = 1,
            ONTHEWAY = 2
        };

        class ClPerceptionSystem : public smacc::ISmaccClient, public smacc::ISmaccUpdatable
        {

        public:
            int decissionsCount;
            int currentCube = 0;            

            CpSceneState* sceneState_;
            CpSimulatedGazeboPerception* gazeboPerceptionSimulation_;

            ClPerceptionSystem()
            {
                decissionsCount = 0;
            }

            virtual ~ClPerceptionSystem()
            {
            }

            template <typename TObjectTag, typename TDerived>
            void configureEventSourceTypes()
            {
                // order matters, since gazeboPerception udpates sceneState
                sceneState_ = this->createComponent<CpSceneState, TDerived, TObjectTag>();
                gazeboPerceptionSimulation_ = this->createComponent<CpSimulatedGazeboPerception, TDerived, TObjectTag>();
                
                int TABLE_COUNT = 6;
                for (int i = 0; i < TABLE_COUNT; i++)
                {
                    auto tablepose = this->createComponent<cl_move_base_z::Pose, TDerived, TObjectTag>("table_" + std::to_string(i));
                    sceneState_->tablePoses_.push_back(tablepose);
                }
            }

            geometry_msgs::PoseStamped getMainTablePose()
            {
                return this->sceneState_->tablePoses_[3]->toPoseStampedMsg();
            }

            void nextCube()
            {
                currentCube = (currentCube + 1) % 3;
                ROS_INFO_STREAM("[Perception system] next cube: " << currentCube);
            }

            void retryCubeAfterFail()
            {
                currentCube = (currentCube - 1) % 3;
                ROS_INFO_STREAM("[Perception system] next cube: " << currentCube);
            }

            virtual void update() override
            {
            }

            RobotProcessStatus getCurrentTable()
            {
                cl_move_base_z::ClMoveBaseZ *moveBaseclient;
                this->requiresClient(moveBaseclient);
                auto pose = moveBaseclient->getComponent<cl_move_base_z::Pose>();
                auto currentPose = pose->toPoseMsg();

                int tableIndex = 0;
                if (currentPose.position.x > -0.2)
                    return RobotProcessStatus::TABLE0;
                else if (currentPose.position.x < -0.8)
                    return RobotProcessStatus::TABLE1;
                else
                    return RobotProcessStatus::ONTHEWAY;
            }

            geometry_msgs::PoseStamped decidePickCubePose()
            {
                geometry_msgs::PoseStamped ret;
                return ret;
            }

            geometry_msgs::PoseStamped decidePlacePose()
            {
                cl_move_base_z::ClMoveBaseZ *moveBaseclient;
                this->requiresClient(moveBaseclient);
                auto pose = moveBaseclient->getComponent<cl_move_base_z::Pose>();
                auto currentPose = pose->toPoseMsg();

                geometry_msgs::PoseStamped ret;
                return ret;
            }
        };
    } // namespace cl_perception_system
} // namespace sm_moveit_4
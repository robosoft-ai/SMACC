#pragma once

#include <smacc/smacc_client.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <moveit_z_client/cl_movegroup.h>

#include <gazebo_msgs/LinkStates.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <boost/algorithm/string.hpp>

namespace sm_moveit_4
{
    namespace cl_perception_system
    {
        struct TableInfo
        {
            std::string targetColor_;
        };

        struct CubeInfo
        {
            std::string color;
            TableInfo *dstTableInfo_;
        };

        enum class RobotProcessStatus : int
        {
            TABLE0 = 0,
            TABLE1 = 1,
            ONTHEWAY = 2
        };

        class ClPerceptionSystem : public smacc::ISmaccClient, public smacc::ISmaccUpdatable
        {
        public:
            ros::Time startTime;
            ros::Duration readDelay;

            int decissionsCount;
            int currentCube = 0;

            tf::TransformListener tfListener_;
            tf::TransformBroadcaster tfBroadcaster_;
            ros::Subscriber gazeboStateSubscriber_;
            ros::Time lastUpdateStamp_;
            ros::Duration updatePeriod_;
            bool tableCollision_;
            bool cubeCollision_;
            moveit::planning_interface::PlanningSceneInterface *planningSceneInterface_;

            // demo state
            std::vector<CubeInfo> cubeInfos_;
            std::vector<TableInfo> tablesInfo_ = {{"yellow"}, {"white"}, {"purple"}, {"none"}, {"red"}, {"green"}};

            std::vector<cl_move_base_z::Pose *> tablePoses_;

            ClPerceptionSystem()
            {
                startTime = ros::Time::now();
                readDelay = ros::Duration(5);
                decissionsCount = 0;
            }

            virtual ~ClPerceptionSystem()
            {
            }

            template <typename TObjectTag, typename TDerived>
            void configureEventSourceTypes()
            {
                int TABLE_COUNT = 6;
                for (int i = 0; i < TABLE_COUNT; i++)
                {
                    auto tablepose = this->createComponent<cl_move_base_z::Pose, TDerived, TObjectTag>("table_" + std::to_string(i));
                    tablePoses_.push_back(tablepose);
                }
            }

            virtual void initialize() override
            {
                ros::NodeHandle nh;
                gazeboStateSubscriber_ =
                    nh.subscribe("/gazebo/link_states", 1, &ClPerceptionSystem::simulatedLinkStateCallback, this);

                moveit_z_client::ClMoveGroup *movegroupclient;
                requiresClient(movegroupclient);
                planningSceneInterface_ = &movegroupclient->planningSceneInterface;
                tableCollision_ = true;
                updatePeriod_ = ros::Duration(0.25);
            }

            void identifyCubeColorsByNames(const std::vector<std::string> &linknames)
            {
                if (cubeInfos_.size() == 0)
                {
                    std::vector<std::string> strs;
                    for (auto &linkname : linknames)
                    {
                        if (linkname.find("cube") != std::string::npos)
                        {
                            strs.clear();
                            boost::split(strs, linkname, boost::is_any_of("_:")); // demo_cube_green::link
                            CubeInfo cubeinfo;

                            cubeinfo.color = strs[2];

                            cubeinfo.dstTableInfo_ = &(*(std::find_if(this->tablesInfo_.begin(), this->tablesInfo_.end(),
                                                                      [&](auto &ti) { return ti.targetColor_ == cubeinfo.color; })));
                            cubeInfos_.push_back(cubeinfo);
                        }
                    }
                }
            }

            void simulatedLinkStateCallback(const gazebo_msgs::LinkStates &linksmsg)
            {
                identifyCubeColorsByNames(linksmsg.name);

                std::vector<tf::StampedTransform> cubeTransforms;
                this->propagateLinkStatesToTf(linksmsg, "cube", "cube_", "map", cubeTransforms);

                std::vector<tf::StampedTransform> tableTransforms;
                this->propagateLinkStatesToTf(linksmsg, "table", "table_", "map", tableTransforms);

                auto ellapsed = ros::Time::now() - this->lastUpdateStamp_;

                if (ellapsed > updatePeriod_)
                {
                    ROS_DEBUG("Updating planning scene perception");
                    auto attachedObjects = planningSceneInterface_->getAttachedObjects();

                    bool hasCubeAttached = false;
                    for (auto &k : attachedObjects)
                    {
                        if (k.first.find("cube") != std::string::npos)
                        {
                            hasCubeAttached = true;
                            break;
                        }
                    }

                    if (tableCollision_ && !hasCubeAttached)
                    {
                        std::vector<moveit_msgs::CollisionObject> collisionObjects(tableTransforms.size());
                        std::vector<std::string> removeCollisionObjectNames(tableTransforms.size());

                        auto thickness = 0.12;
                        for (int i = 0; i < tableTransforms.size(); i++)
                        {
                            auto &tableTransf = tableTransforms[i];
                            auto &pos = tableTransf.getOrigin();
                            std::string tablename = "table_" + std::to_string(i);
                            removeCollisionObjectNames[i] = tablename;
                            moveit_msgs::CollisionObject &collision = collisionObjects[i];
                            collision.operation = moveit_msgs::CollisionObject::ADD;
                            collision.id = tablename;
                            collision.primitives.resize(1);
                            collision.primitives[0].type = collision.primitives[0].BOX;
                            collision.primitives[0].dimensions.resize(3);
                            collision.primitives[0].dimensions[0] = 1.2;
                            collision.primitives[0].dimensions[1] = 1.3;
                            collision.primitives[0].dimensions[2] = 0.001 + thickness;

                            /* Define the pose of the table. */
                            collision.primitive_poses.resize(1);
                            collision.primitive_poses[0].position.x = pos[0];
                            collision.primitive_poses[0].position.y = pos[1];
                            collision.primitive_poses[0].position.z = 0.7 - thickness * 0.5;
                            collision.primitive_poses[0].orientation.w = 1.0;

                            collision.header.frame_id = tableTransf.frame_id_;
                            collision.header.stamp = tableTransf.stamp_;
                        }

                        //this->planningSceneInterface_->removeCollisionObjects(removeCollisionObjectNames);
                        this->planningSceneInterface_->addCollisionObjects(collisionObjects);
                    }

                    if (cubeCollision_ && !hasCubeAttached)
                    {
                        //         for i, cube_transf in enumerate(cube_transforms):
                        //             #self.planning_scene.removeCollisionObject("cube_" + str(i))
                        //             pos = cube_transf[0]
                        //             self.planning_scene.addCube(
                        //                 "cube_" + str(i), 0.06, pos[0],  pos[1],  pos[2])
                        //             #self.cube_collision = False
                    }
                }
            }

            void propagateLinkStatesToTf(const gazebo_msgs::LinkStates &linksmsg, std::string linkNameFilter,
                                         std::string objectPrefix, std::string globalFrame,
                                         std::vector<tf::StampedTransform> &transforms)
            {
                transforms.clear();
                int i = 0;
                std::vector<geometry_msgs::Pose> filteredLinkPoses;
                for (auto &b : linksmsg.pose)
                {
                    auto linkname = linksmsg.name[i];
                    if (linkname.find(linkNameFilter) != std::string::npos)
                        filteredLinkPoses.push_back(b);

                    i++;
                }

                i = 0;
                for (auto &cubepose : filteredLinkPoses)
                {
                    tf::Pose tfpose;
                    tf::poseMsgToTF(cubepose, tfpose);
                    tf::StampedTransform transform;
                    transform.setOrigin(tfpose.getOrigin());
                    transform.setRotation(tfpose.getRotation());
                    transform.frame_id_ = globalFrame;
                    transform.child_frame_id_ = objectPrefix + std::to_string(i);
                    transform.stamp_ = ros::Time::now();

                    tfBroadcaster_.sendTransform(transform);

                    i++;
                    transforms.push_back(transform);
                }
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
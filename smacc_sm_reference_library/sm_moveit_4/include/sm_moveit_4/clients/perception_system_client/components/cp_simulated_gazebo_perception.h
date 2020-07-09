#pragma once

#include <smacc/component.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <gazebo_msgs/LinkStates.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_z_client/cl_movegroup.h>

#include "cp_scene_state.h"

namespace sm_moveit_4
{
    namespace cl_perception_system
    {
        using namespace smacc;
        using namespace moveit_z_client;

        /*
        This component focuses in propagating gazebo links states (tables and cubes) to the tf system.
        It also creates the tables and cubes collision sets for the motion planning process.
        */
        class CpSimulatedGazeboPerception : public smacc::ISmaccComponent
        {
            ros::Time startTime;
            ros::Duration readDelay;

            ros::Time lastUpdateStamp_;
            ros::Duration updatePeriod_;
            bool tableCollision_;
            bool cubeCollision_;

            moveit::planning_interface::PlanningSceneInterface *planningSceneInterface_;
            ClMoveGroup *movegroupclient_;

        private:
            tf::TransformListener tfListener_;
            tf::TransformBroadcaster tfBroadcaster_;
            ros::Subscriber gazeboStateSubscriber_;

        public:
            double safeTableHeightOffsetForCubeCollisions = 0;


            void setSafeArmMotionToAvoidCubeCollisions()
            {
                 safeTableHeightOffsetForCubeCollisions = 0.17;
            }

            void unsetSafeArmMotionToAvoidCubeCollisions()
            {
                 safeTableHeightOffsetForCubeCollisions = 0;
            }

            virtual void onInitialize() override
            {
                this->requiresClient(movegroupclient_);
                planningSceneInterface_ = &movegroupclient_->planningSceneInterface;
                
                tableCollision_ = true;
                updatePeriod_ = ros::Duration(0.25);
                startTime = ros::Time::now();
                
                // order is important, to avoid "preemption" of the callback before it is initialized
                ros::NodeHandle nh;
                gazeboStateSubscriber_ =
                    nh.subscribe("/gazebo/link_states", 1, &CpSimulatedGazeboPerception::simulatedLinkStateCallback, this);
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
                        //std::vector<std::string> removeCollisionObjectNames(tableTransforms.size());
                        ros::Time time = ros::Time::now();
                        auto thickness = 0.12;
                        for (int i = 0; i < tableTransforms.size(); i++)
                        {
                            auto &tableTransf = tableTransforms[i];
                            auto &pos = tableTransf.getOrigin();
                            std::string tablename = "table_" + std::to_string(i + 1);
                            //removeCollisionObjectNames[i] = tablename;
                            moveit_msgs::CollisionObject &collision = collisionObjects[i];
                            collision.operation = moveit_msgs::CollisionObject::ADD;
                            collision.id = tablename;
                            collision.primitives.resize(1);
                            collision.primitives[0].type = collision.primitives[0].BOX;
                            collision.primitives[0].dimensions.resize(3);
                            collision.primitives[0].dimensions[0] = 1.05;
                            collision.primitives[0].dimensions[1] = 1.4;
                            collision.primitives[0].dimensions[2] = 0.001 + thickness + safeTableHeightOffsetForCubeCollisions;

                            /* Define the pose of the table. */
                            collision.primitive_poses.resize(1);
                            collision.primitive_poses[0].position.x = pos[0];
                            collision.primitive_poses[0].position.y = pos[1];
                            collision.primitive_poses[0].position.z = 0.7 - thickness * 0.5;
                            collision.primitive_poses[0].orientation.w = 1.0;

                            collision.header.frame_id = tableTransf.frame_id_;
                            collision.header.stamp = tableTransf.stamp_;
                            time = tableTransf.stamp_;
                        }

                        createVirtualFloorCollisionBox(collisionObjects, time);
                        lateralBox(collisionObjects, time);

                        this->planningSceneInterface_->applyCollisionObjects(collisionObjects);
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

            void additional(float x, float y , float z, float xl, float yl, float zl, std::string id, std::string frameid, moveit_msgs::CollisionObject& collision, ros::Time time)
            {
                collision.operation = moveit_msgs::CollisionObject::ADD;
                collision.id = id;

                collision.primitives.resize(1);
                collision.primitives[0].type = collision.primitives[0].BOX;
                collision.primitives[0].dimensions.resize(3);

                collision.primitives[0].dimensions[0] = xl;
                collision.primitives[0].dimensions[1] = yl;
                collision.primitives[0].dimensions[2] = zl ;

                collision.primitive_poses.resize(1);
                collision.primitive_poses[0].position.x = x;
                collision.primitive_poses[0].position.y = y;
                collision.primitive_poses[0].position.z = z;
                collision.primitive_poses[0].orientation.w = 1.0;

                collision.header.frame_id = frameid;
                collision.header.stamp = time;
            }

            void lateralBox(std::vector<moveit_msgs::CollisionObject>& collisions, ros::Time time)
            {
                moveit_msgs::CollisionObject box;
                additional(0, -0.28, 0.3, 1.0, 0.01, 0.6, "right", "base_link", box, time);
                collisions.push_back(box);

                additional(0, 0.28, 0.3, 1.0, 0.01, 0.6, "left", "base_link", box, time);
                collisions.push_back(box);
            }

            void createVirtualFloorCollisionBox(std::vector<moveit_msgs::CollisionObject>& collisions, ros::Time time)
            {
                moveit_msgs::CollisionObject floor;
                //additional(0, 0, -0.1, 8, 8, 0.05, "floor", "map", floor, time);
                //collisions.push_back(floor);
            }

            /*
            This method gets information about cube colors from gazebo link state messages.
            */
            void identifyCubeColorsByNames(const std::vector<std::string> &linknames)
            {
                CpSceneState *scene;
                this->requiresComponent(scene);

                if (scene->cubeInfos_.front().color == "")
                {
                    std::vector<std::string> strs;
                    int i=0;
                    for (auto &linkname : linknames)
                    {
                        if (linkname.find("cube") != std::string::npos)
                        {
                            strs.clear();
                            boost::split(strs, linkname, boost::is_any_of("_:")); // demo_cube_green::link
                            CubeInfo& cubeinfo = scene->cubeInfos_[i++];

                            cubeinfo.color = strs[2];

                            cubeinfo.dstTableInfo_ = &(*(std::find_if(scene->tablesInfo_.begin(), scene->tablesInfo_.end(),
                                                                      [&](auto &ti) { return ti.associatedCubeColor_ == cubeinfo.color; })));
                        }
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
                for (auto &linkpose : filteredLinkPoses)
                {
                    tf::Pose tfpose;
                    tf::poseMsgToTF(linkpose, tfpose);
                    tf::StampedTransform transform;
                    transform.setOrigin(tfpose.getOrigin());
                    transform.setRotation(tfpose.getRotation());
                    transform.frame_id_ = globalFrame;
                    transform.child_frame_id_ = objectPrefix + std::to_string(i + 1);
                    transform.stamp_ = ros::Time::now();

                    tfBroadcaster_.sendTransform(transform);

                    i++;
                    transforms.push_back(transform);
                }
            }
        };
    } // namespace cl_perception_system
} // namespace sm_moveit_4
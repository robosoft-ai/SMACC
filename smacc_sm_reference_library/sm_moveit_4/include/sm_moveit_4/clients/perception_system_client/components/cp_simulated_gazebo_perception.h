#pragma once

#include <smacc/component.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <gazebo_msgs/LinkStates.h>
#include <moveit_msgs/CollisionObject.h>
#include <move_group_interface_client/cl_movegroup.h>

#include "cp_scene_state.h"

namespace sm_moveit_4
{
    namespace cl_perception_system
    {
        using namespace smacc;
        using namespace move_group_interface_client;

        /*
        This component focuses in propagating gazebo links states (tables and cubes) to the tf system.
        It also creates the tables and cubes collision sets for the motion planning process.
        */
        class CpSimulatedGazeboPerception : public smacc::ISmaccComponent
        {        
        private:
            ros::Time startTime;
            ros::Duration readDelay;

            ros::Time lastUpdateStamp_;
            ros::Duration updatePeriod_;
            bool tableCollision_;
            bool cubeCollision_;

            tf::TransformListener tfListener_;
            tf::TransformBroadcaster tfBroadcaster_;
            ros::Subscriber gazeboStateSubscriber_;

        public:
            virtual void onInitialize() override
            {                
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
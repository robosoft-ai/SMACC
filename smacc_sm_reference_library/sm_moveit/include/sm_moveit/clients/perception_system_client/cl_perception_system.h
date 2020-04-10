#pragma once

#include <smacc/smacc_client.h>

namespace sm_moveit
{
namespace cl_perception_system
{
class ClPerceptionSystem : public smacc::ISmaccClient, public smacc::ISmaccUpdatable
{
public:
    cl_move_base_z::Pose *detectedCubePose0;
    cl_move_base_z::Pose *detectedCubePose1;

    boost::optional<geometry_msgs::PoseStamped> originalCube0Pose;
    boost::optional<geometry_msgs::PoseStamped> originalCube1Pose;

    boost::optional<geometry_msgs::PoseStamped> cube1DestinationInTable0;
    boost::optional<geometry_msgs::PoseStamped> cube0DestinationInTable1;

    ros::Time startTime;
    ros::Duration readDelay;

    int decissionsCount;

    ClPerceptionSystem()
    {
        startTime = ros::Time::now();
        readDelay = ros::Duration(5);
        decissionsCount = 0;
    }

    virtual ~ClPerceptionSystem()
    {
    }

    virtual void /*  */ update() override
    {
        auto ellapsed = ros::Time::now() - startTime;
        bool applyUpdate = ellapsed > readDelay;

        if (applyUpdate)
        {
            if (!originalCube0Pose && detectedCubePose0->isInitialized)
            {
                originalCube0Pose = detectedCubePose0->toPoseStampedMsg();

                cube1DestinationInTable0 = *originalCube0Pose;
                cube1DestinationInTable0->pose.position.y += 0.3;
                //cube1DestinationInTable0->pose.position.x -= 0.15;

                ROS_INFO_STREAM("[Perception system] Original Cube 0 Pose captured" << originalCube0Pose->pose);
            }

            if (!originalCube1Pose && detectedCubePose1->isInitialized)
            {
                originalCube1Pose = detectedCubePose1->toPoseStampedMsg();

                cube0DestinationInTable1 = *originalCube1Pose;
                cube0DestinationInTable1->pose.position.y += 0.3;
                //cube0DestinationInTable1->pose.position.x -= 0.15;

                ROS_INFO_STREAM("[Perception system] Original Cube 1 Pose captured" << originalCube1Pose->pose);
            }
        }
    }

    geometry_msgs::PoseStamped decidePlacePose(int cubeIndex, int tableIndex)
    {
        decissionsCount++;
        if (tableIndex == 0)
        {
            if (cubeIndex == 0)
            {
                if (decissionsCount % 2 == 0)
                    return *originalCube0Pose;
                else
                    return *cube1DestinationInTable0;
            }
            else
            {
                return *cube1DestinationInTable0;
            }
        }
        else
        {
            if (cubeIndex == 0)
            {
                return *cube0DestinationInTable1;
            }
            else
            {
                return *originalCube1Pose;
            }
        }
    }
};
} // namespace cl_perception_system
} // namespace sm_moveit
#pragma once

#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <smacc/smacc_client.h>
#include <move_group_interface_client/cl_movegroup.h>


#include <gazebo_msgs/LinkStates.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace sm_moveit_3
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
  cl_move_base_z::Pose *detectedCubePose0;
  cl_move_base_z::Pose *detectedCubePose1;
  cl_move_base_z::Pose *detectedCubePose2;

  boost::optional<geometry_msgs::PoseStamped> originalCube0Pose;
  boost::optional<geometry_msgs::PoseStamped> originalCube1Pose;
  boost::optional<geometry_msgs::PoseStamped> originalCube2Pose;

  boost::optional<geometry_msgs::PoseStamped> cube0DestinationInTable1;
  boost::optional<geometry_msgs::PoseStamped> cube1DestinationInTable0;
  boost::optional<geometry_msgs::PoseStamped> cube2DestinationInTable1;

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
  moveit::planning_interface::PlanningSceneInterface* planningInterface_;

  ClPerceptionSystem()
  {
    startTime = ros::Time::now();
    readDelay = ros::Duration(5);
    decissionsCount = 0;

    lastUpdateStamp_ = ros::Time::now();
    updatePeriod_ = ros::Duration(10);
    tableCollision_ = true;
    cubeCollision_ = false;
  }

  virtual ~ClPerceptionSystem()
  {
  }

  virtual void initialize() override
  {
    ros::NodeHandle nh;
    gazeboStateSubscriber_ =
        nh.subscribe("/gazebo/link_states", 1, &ClPerceptionSystem::simulatedLinkStateCallback, this);
    
    move_group_interface_client::ClMoveGroup* movegroupclient;
    requiresClient(movegroupclient);
    planningInterface_ = &movegroupclient->planningSceneInterface;
  }

  void simulatedLinkStateCallback(const gazebo_msgs::LinkStates &linksmsg)
  {
      std::vector<tf::StampedTransform> cubeTransforms;
      this->propagateLinkStatesToTf(linksmsg, "cube", "cube_", "map", cubeTransforms);

      std::vector<tf::StampedTransform> tableTransforms;
      this->propagateLinkStatesToTf(linksmsg, "table", "table_", "map", tableTransforms);

      auto ellapsed = ros::Time::now() - this->lastUpdateStamp_;

      if (ellapsed > updatePeriod_)
      {
          ROS_DEBUG("Updating planning scene perception");
          auto attachedObjects = planningInterface_->getAttachedObjects();

          bool hasCubeAttached = false;
          for(auto& k : attachedObjects)
          {
              if(k.first.find("cube")!= std::string::npos)
              {
                 hasCubeAttached = true;
                 break; 
              }
          }

        if(tableCollision_ && ! hasCubeAttached)
        {
        //         for i, table_transf in enumerate(table_transforms):
        //             #self.planning_scene.removeCollisionObject("table_" + str(i))
        //             thickness = 0.12
        //             pos = table_transf[0]
        //             self.planning_scene.addBox(
        //                 "table_" + str(i), 1.2, 1.3, 0.001 + thickness, pos[0],  pos[1],  0.7 -  thickness*0.5)  # 0.68
        }

        if(cubeCollision_ && ! hasCubeAttached)
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

  void tryCaptureInitialCubePoses()
  {
    auto ellapsed = ros::Time::now() - startTime;
    bool applyUpdate = ellapsed > readDelay;

    if (applyUpdate)
    {
      if (!originalCube0Pose && detectedCubePose0->isInitialized)
      {
        originalCube0Pose = detectedCubePose0->toPoseStampedMsg();

        cube0DestinationInTable1 = originalCube0Pose;
        (*cube0DestinationInTable1).pose.position.x = -originalCube0Pose->pose.position.x;

        ROS_INFO_STREAM("[Perception system] Original Cube 0 Pose captured" << originalCube0Pose->pose);
        ROS_INFO_STREAM("[Perception system] Cube 0 Pose table 1 destination" << (*cube0DestinationInTable1));
      }

      if (!originalCube1Pose && detectedCubePose1->isInitialized)
      {
        originalCube1Pose = detectedCubePose1->toPoseStampedMsg();

        cube1DestinationInTable0 = originalCube1Pose;
        (*cube1DestinationInTable0).pose.position.x = -originalCube1Pose->pose.position.x;

        ROS_INFO_STREAM("[Perception system] Original Cube 1 Pose captured" << originalCube1Pose->pose);
        ROS_INFO_STREAM("[Perception system] Cube 1 Pose table 0 destination" << (*cube1DestinationInTable0));
      }

      if (!originalCube2Pose && detectedCubePose2->isInitialized)
      {
        originalCube2Pose = detectedCubePose2->toPoseStampedMsg();

        cube2DestinationInTable1 = originalCube2Pose;
        (*cube2DestinationInTable1).pose.position.x = -originalCube2Pose->pose.position.x;

        ROS_INFO_STREAM("[Perception system] Original Cube 2 Pose captured" << originalCube2Pose->pose);
        ROS_INFO_STREAM("[Perception system] Cube 2 Pose table 1 destination" << (*cube2DestinationInTable1));
      }
    }
  }

  virtual void update() override
  {
    tryCaptureInitialCubePoses();
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
    cl_move_base_z::Pose *targetObjectPose;

    if (currentCube == 0)
    {
      targetObjectPose = detectedCubePose0;
    }
    else if (currentCube == 1)
    {
      targetObjectPose = detectedCubePose1;
    }
    else
    {
      targetObjectPose = detectedCubePose2;
    }

    targetObjectPose->waitTransformUpdate();
    return targetObjectPose->toPoseStampedMsg();
  }

  geometry_msgs::PoseStamped decidePlacePose()
  {
    cl_move_base_z::ClMoveBaseZ *moveBaseclient;
    this->requiresClient(moveBaseclient);
    auto pose = moveBaseclient->getComponent<cl_move_base_z::Pose>();
    auto currentPose = pose->toPoseMsg();

    int tableIndex = 0;
    if (currentPose.position.x > -0.2)  // negative side
    {
      ROS_INFO("[StOpenGripper] locate cube in table 0");
      tableIndex = 0;
    }
    else
    {
      ROS_INFO("[StOpenGripper] locate cube in table 1");
      tableIndex = 1;
    }

    decissionsCount++;
    if (tableIndex == 0)
    {
      ROS_INFO("[Perception System] deciding cube target in table 0");
      if (currentCube == 0)
      {
        ROS_INFO("[Perception System] deciding cube target 0 in table 0");
        return *originalCube0Pose;
      }
      else if (currentCube == 1)
      {
        ROS_INFO("[Perception System] deciding cube target 1 in table 0");
        return *cube1DestinationInTable0;
      }
      else
      {
        ROS_INFO("[Perception System] deciding cube target 2 in table 0");
        return *originalCube2Pose;
      }
    }
    else
    {
      if (currentCube == 0)
      {
        ROS_INFO("[Perception System] deciding cube target 0 in table 1");
        return *cube0DestinationInTable1;
      }
      else if (currentCube == 1)
      {
        ROS_INFO("[Perception System] deciding cube target 1 in table 1");
        return *originalCube1Pose;
      }
      else
      {
        ROS_INFO("[Perception System] deciding cube target 2 in table 1");
        return *cube2DestinationInTable1;
      }
    }
  }
};
}  // namespace cl_perception_system
}  // namespace sm_moveit_3
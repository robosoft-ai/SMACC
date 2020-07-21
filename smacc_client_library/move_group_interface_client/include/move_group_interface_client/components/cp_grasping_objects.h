#pragma once

#include <smacc/smacc.h>
#include <smacc/component.h>
#include <moveit_msgs/CollisionObject.h>
#include <map>

namespace move_group_interface_client
{
   class GraspingComponent : public smacc::ISmaccComponent
   {
   private:
      std::map<std::string, moveit_msgs::CollisionObject> graspingObjects;

   public:
      std::vector<std::string> fingerTipNames;

      boost::optional<std::string> currentAttachedObjectName;

      bool getGraspingObject(std::string name, moveit_msgs::CollisionObject &object);

      void createGraspableBox(std::string frameid, float x, float y, float z, float xl, float yl, float zl);
   };

} // namespace move_group_interface_client
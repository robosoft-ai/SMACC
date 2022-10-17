#pragma once

#include <smacc/client_bases/smacc_action_client_base.h>
#include <control_msgs/GripperCommandAction.h>

namespace sm_fetch_screw_loop_1
{
namespace cl_gripper
{
class ClGripper : public smacc::client_bases::SmaccActionClientBase<control_msgs::GripperCommandAction>
{

public:
  SMACC_ACTION_CLIENT_DEFINITION(control_msgs::GripperCommandAction)

  ClGripper(std::string actionServerName);

  ClGripper();

  virtual ~ClGripper();

  virtual std::string getName() const override;

  void executeSetGripperPositionValue(float value)
  {
  }

  void executeOpenGripper()
  {
    /*
  rostopic pub /gripper_controller/gripper_action/goal control_msgs/GripperCommandActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  command:
    position: 0.0
    max_effort: 0.0"
    */
  }
};
} // namespace cl_gripper
} // namespace sm_fetch_screw_loop_1

#include <non_rt_helper/DispenseModeAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/UInt8.h>
#include <reelrbtx_msgs/FollowControllerState.h>
#include <functional>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

typedef actionlib::SimpleActionServer<non_rt_helper::DispenseModeAction> Server;


class ToolActionServer
{
public:

  std::shared_ptr<Server> as_ ;

  
  ToolActionServer()
  {
    
  }


/**
******************************************************************************************************************
* execute()
******************************************************************************************************************
*/
void execute(const non_rt_helper::DispenseModeGoalConstPtr& goal)  // Note: "Action" is not appended to DoDishes here
{
   as_.setSucceeded();
};

/**
******************************************************************************************************************
* main()
******************************************************************************************************************
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "non_rt_helper_node");
  NonRTHelper nrthelper;
  nrthelper.run();

  return 0;
}


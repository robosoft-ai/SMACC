#include <actionlib/server/simple_action_server.h>
#include <std_msgs/UInt8.h>
#include <functional>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <smacc_tool_plugin_template/ToolControlAction.h>
#include <smacc_tool_plugin_template/ToolControlActionResult.h>
#include <smacc_tool_plugin_template/ToolControlResult.h>
#include <memory>
#include <visualization_msgs/MarkerArray.h>

typedef actionlib::SimpleActionServer<smacc_tool_plugin_template::ToolControlAction> Server;

// This class describes a preemptable-on/off tool action server to be used from smacc
// shows in rviz a sphere whoose color describe the current state (unknown, running, idle)
class ToolActionServer
{
public:
  std::shared_ptr<Server> as_ ;

  ros::Publisher stateMarkerPublisher_;

  smacc_tool_plugin_template::ToolControlGoal cmd;

  uint8_t currentState_;

/**
******************************************************************************************************************
* constructor()
******************************************************************************************************************
*/
  ToolActionServer()
  {
    currentState_ =  smacc_tool_plugin_template::ToolControlResult::STATE_UNKNOWN;
  }

/**
******************************************************************************************************************
* publishFeedback()
******************************************************************************************************************
*/
void publishFeedback()  // Note: "Action" is not appended to DoDishes here
{
    smacc_tool_plugin_template::ToolControlFeedback feedback_msg;
    
    as_->publishFeedback(feedback_msg);
}

/**
******************************************************************************************************************
* execute()
******************************************************************************************************************
*/
void execute(const smacc_tool_plugin_template::ToolControlGoalConstPtr& goal)  // Note: "Action" is not appended to DoDishes here
{
  ROS_INFO_STREAM("Tool action server request: "<< *goal);
  cmd = *goal;

  if(goal->command == smacc_tool_plugin_template::ToolControlGoal::CMD_START)
  {
    currentState_ =  smacc_tool_plugin_template::ToolControlResult::STATE_RUNNING;
  }
  else  if (goal->command == smacc_tool_plugin_template::ToolControlGoal::CMD_STOP)
  {
    currentState_ =  smacc_tool_plugin_template::ToolControlResult::STATE_IDLE;
  }

  // 10Hz internal loop
  ros::Rate rate(20);

  while(ros::ok())
  {
    if(as_->isPreemptRequested())
    {
       // a new request is being executed, we will stop this one
       ROS_WARN("NonRtHelper request preempted. Forgetting older request.");
       as_->setPreempted(); 
       return;
    }
    //publishFeedback();
    publishStateMarker();
    rate.sleep();
  }

   // never reach succeded because were are interested in keeping the feedback alive
   as_->setSucceeded();
}

/**
******************************************************************************************************************
* run()
******************************************************************************************************************
*/
void run()
{
  ros::NodeHandle n;
  ROS_INFO("Creating tool action server");

  as_ = std::make_shared<Server>(n, "tool_action_server", boost::bind(&ToolActionServer::execute, this,  _1), false);
  ROS_INFO("Starting Tool Action Server");

  stateMarkerPublisher_ = n.advertise<visualization_msgs::MarkerArray>("tool_markers", 1); 

  as_->start();

  ros::spin();
}

/**
******************************************************************************************************************
* publishStateMarker()
******************************************************************************************************************
*/
void publishStateMarker()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now ();

    marker.ns = "my_namespace2";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1;

    if(currentState_ == smacc_tool_plugin_template::ToolControlResult::STATE_RUNNING)
    {
      // show green ball
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
    }
    else if (currentState_ == smacc_tool_plugin_template::ToolControlResult::STATE_IDLE)
    {
      // show gray ball
      marker.color.r = 0.7;
      marker.color.g = 0.7;
      marker.color.b = 0.7;
    }
    else
    {
      // show black ball
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 0;
    }

    marker.pose.orientation.w=1;
    marker.pose.position.x=0;
    marker.pose.position.y=0;
    marker.pose.position.z=1;

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker);

    stateMarkerPublisher_.publish(ma);
}
};

/**
******************************************************************************************************************
* main()
******************************************************************************************************************
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tool_action_server_node");
  ToolActionServer toolActionServer;
  toolActionServer.run();

  return 0;
}


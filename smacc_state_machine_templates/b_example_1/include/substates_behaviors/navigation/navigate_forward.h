#include <radial_motion.h>
#include <thread>
#include <tf/transform_listener.h>
#include <tf/tf.h>

class NavigateForward : public smacc::SmaccStateBehavior
{
 public:
  tf::TransformListener listener;
    
  smacc::SmaccMoveBaseActionClient *moveBaseClient_;

  smacc_odom_tracker::OdomTracker* odomTracker_;

  smacc_planner_switcher::PlannerSwitcher* plannerSwitcher_;  

  virtual void onEntry() override
  {
    // straight motion distance
    double dist;
    this->currentState->param("straight_motion_distance", dist, 3.5);
    ROS_INFO_STREAM("Straight motion distance: " << dist);

    this->requiresComponent(moveBaseClient_ ,ros::NodeHandle("move_base"));
    this->requiresComponent(odomTracker_);
    this->requiresComponent(plannerSwitcher_ , ros::NodeHandle("move_base"));   

    //this should work better with a coroutine and await
    ros::Rate rate(10.0);
    tf::StampedTransform currentPose;

    while (ros::ok())
    {
        
        try{
        listener.lookupTransform("/base_link", "/odom",  
                                ros::Time(0), currentPose);
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }
    }   

    tf::Transform forwardDeltaTransform;
    forwardDeltaTransform.setOrigin(tf::Vector3(dist, 0, 0));

    tf::Transform targetPose = currentPose * forwardDeltaTransform;

    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose.header.frame_id = "/odom";
    goal.target_pose.header.stamp = ros::Time::now();
    tf::poseTFToMsg(targetPose, goal.target_pose.pose);

    this->odomTracker_->clearPath();
    
    geometry_msgs::PoseStamped currentPoseMsg;
    currentPoseMsg.header.frame_id = "/odom";
    currentPoseMsg.header.stamp = ros::Time::now();
    tf::poseTFToMsg(currentPose, currentPoseMsg.pose);
    this->odomTracker_->setStartPoint(currentPoseMsg);
    this->odomTracker_->setWorkingMode(smacc_odom_tracker::WorkingMode::RECORD_PATH_FORWARD);

    this->plannerSwitcher_->setForwardPlanner();

    moveBaseClient_->sendGoal(goal);
  }    
};

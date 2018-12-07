/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc_odom_tracker/odom_tracker.h>

namespace smacc_odom_tracker
{

OdomTracker::OdomTracker()
{
    workingMode_ = WorkingMode::RECORD_PATH_FORWARD;
    publishMessages = true;
    subscribeToOdometryTopic_ = true;
}
        
/**
******************************************************************************************************************
* init()
******************************************************************************************************************
*/
void OdomTracker::init(ros::NodeHandle& nh)
{
    ROS_WARN("Initializing Odometry Tracker");

    if(!nh.getParam("min_point_distance_forward_thresh",minPointDistanceForwardThresh_))
    {
        minPointDistanceForwardThresh_ = 0.005; // 1 mm
    }

    if(!nh.getParam("min_point_distance_backward_thresh",minPointDistanceRetractThresh_))
    {
        minPointDistanceRetractThresh_ = 0.1; // 1 mm
    }

    if(this->subscribeToOdometryTopic_)
    {
        odomSub_= nh.subscribe("odom", 1, &OdomTracker::processOdometryMessage, this);
    }

    robotBasePathPub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Path>>(nh, "odom_tracker_path", 1);
}


/**
******************************************************************************************************************
* setWorkingMode()
******************************************************************************************************************
*/
void OdomTracker::setWorkingMode(WorkingMode workingMode)
{
    //ROS_INFO("odom_tracker m_mutex acquire");
    std::lock_guard<std::mutex> lock(m_mutex_);
    workingMode_ = workingMode;
    //ROS_INFO("odom_tracker m_mutex release");
}

/**
******************************************************************************************************************
* setPublishMessages()
******************************************************************************************************************
*/
void OdomTracker::setPublishMessages(bool value)
{
    //ROS_INFO("odom_tracker m_mutex acquire");
    std::lock_guard<std::mutex> lock(m_mutex_);
    publishMessages = value;
    //ROS_INFO("odom_tracker m_mutex release");
}

void OdomTracker::pushPath()
{
    ROS_INFO("odom_tracker m_mutex acquire");
    std::lock_guard<std::mutex> lock(m_mutex_);
    pathStack_.push_back(baseTrajectory_);
    baseTrajectory_.poses.clear();
    ROS_INFO("odom_tracker m_mutex release");
}
        
void OdomTracker::popPath()
{
    ROS_INFO("odom_tracker m_mutex acquire");
    std::lock_guard<std::mutex> lock(m_mutex_);
    
    if(!pathStack_.empty())
    {
        baseTrajectory_.poses = pathStack_.back().poses;
        pathStack_.pop_back();
    }

    ROS_INFO("odom_tracker m_mutex release");
}

/**
******************************************************************************************************************
* rtPublishPaths()
******************************************************************************************************************
*/
void OdomTracker::rtPublishPaths(ros::Time timestamp)
{
    if(robotBasePathPub_->trylock())
    {
        nav_msgs::Path& msg = robotBasePathPub_->msg_;
        ///  Copy trajectory

        msg = baseTrajectory_;
        msg.header.stamp = timestamp;
        robotBasePathPub_->unlockAndPublish();
    }
}

/**
******************************************************************************************************************
* updateBackward()
******************************************************************************************************************
*/
bool OdomTracker::updateBackward(const nav_msgs::Odometry& odom)
{
    // we initially accept any message if the queue is empty   
    /// Track robot base pose
    geometry_msgs::PoseStamped base_pose;
    
    base_pose.pose = odom.pose.pose;
    base_pose.header = odom.header;
    baseTrajectory_.header = odom.header;

    bool acceptRetract = false;
    bool pullingerror = false;
    if(baseTrajectory_.poses.empty())
    {
        acceptRetract=false;
    }
    else
    {
        const geometry_msgs::Point& prevPoint = baseTrajectory_.poses.back().pose.position;
        const geometry_msgs::Point& currePoint = base_pose.pose.position;
        double lastpointdist = p2pDistance(prevPoint, currePoint);
        
        acceptRetract = !baseTrajectory_.poses.empty() 
                        && lastpointdist < minPointDistanceRetractThresh_;

        pullingerror = lastpointdist > 2 * minPointDistanceRetractThresh_;
    }

    //ROS_INFO("RETRACTING, last distance: %lf < %lf accept: %d", dist, minPointDistanceRetractionThresh_, acceptRetract);
    if (acceptRetract) 
    {
        baseTrajectory_.poses.pop_back();
    } 
    else if (pullingerror) {
        ROS_WARN("Incorrect retracting motion. The robot is pulling the cord.");
    } 
    else 
    {
        /// Not removing point because it is enough far from the last cord point
    }

    return acceptRetract;
}
/**
******************************************************************************************************************
* updateForward()
******************************************************************************************************************
*/
bool OdomTracker::updateForward(const nav_msgs::Odometry& odom)
{
    /// Track robot base pose
    geometry_msgs::PoseStamped base_pose;

    base_pose.pose = odom.pose.pose;
    base_pose.header = odom.header;
    baseTrajectory_.header = odom.header;

    bool enqueueOdomMessage = false;

    double dist = -1;
    if(baseTrajectory_.poses.empty())
    {
        enqueueOdomMessage = true;
    }
    else
    {
        const geometry_msgs::Point& prevPoint = baseTrajectory_.poses.back().pose.position;
        const geometry_msgs::Point& currePoint = base_pose.pose.position;
        dist = p2pDistance(prevPoint, currePoint);
        //ROS_WARN("dist %lf vs min %lf", dist, minPointDistanceForwardThresh_);

        if(dist > minPointDistanceForwardThresh_)
        {
            enqueueOdomMessage = true;
        }
        else
        {
            //ROS_WARN("skip odom, dist: %lf", dist);
            enqueueOdomMessage = false;
        }
    }

    if(enqueueOdomMessage)
    {
        baseTrajectory_.poses.push_back(base_pose);
    }

    return enqueueOdomMessage;
}

/**
******************************************************************************************************************
* processOdometryMessage()
******************************************************************************************************************
*/
void OdomTracker::processOdometryMessage(const nav_msgs::Odometry& odom)      
{
    //ROS_INFO("odom_tracker m_mutex acquire");
    std::lock_guard<std::mutex> lock(m_mutex_);

    if(workingMode_ == WorkingMode::RECORD_PATH_FORWARD)
    {
        updateForward(odom);
    }		
    else if (workingMode_ == WorkingMode::CLEAR_PATH_BACKWARD)
    {
        updateBackward(odom);
    }

    //ROS_WARN("odomTracker odometry callback");
    if(publishMessages)
    {
        rtPublishPaths(odom.header.stamp);
    }

    //ROS_INFO("odom_tracker m_mutex release");
}
}
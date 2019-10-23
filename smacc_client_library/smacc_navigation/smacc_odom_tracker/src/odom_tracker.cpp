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
void OdomTracker::initialize(std::string nodeName)
{
    ros::NodeHandle nh(nodeName);

    ROS_WARN("Initializing Odometry Tracker");

    if(!nh.getParam("min_point_distance_forward_thresh",minPointDistanceForwardThresh_))
    {
        minPointDistanceForwardThresh_ = 0.005; // 1 mm
    }

    if(!nh.getParam("min_point_distance_backward_thresh",minPointDistanceBackwardThresh_))
    {
        minPointDistanceBackwardThresh_ = 0.05; // 1 mm
    }

    if(!nh.getParam("min_point_angular_distance_forward_thresh",minPointAngularDistanceForwardThresh_))
    {
        minPointAngularDistanceForwardThresh_ = 0.15 ; // degree
    }

    if(!nh.getParam("min_point_angular_distance_backward_thresh",minPointAngularDistanceBackwardThresh_))
    {
        minPointAngularDistanceBackwardThresh_ = 0.15 ; // degree
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
    ROS_INFO("[OdomTracker] setting working mode to: %d", (uint8_t)workingMode);
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

void OdomTracker::clearPath()
{
    std::lock_guard<std::mutex> lock(m_mutex_);
    baseTrajectory_.poses.clear();

    rtPublishPaths(ros::Time::now());
}

void OdomTracker::setStartPoint(const geometry_msgs::PoseStamped& pose)
{
    std::lock_guard<std::mutex> lock(m_mutex_);
    if(baseTrajectory_.poses.size() >0)
    {
        baseTrajectory_.poses[0]=pose;
    }
    else
    {
        baseTrajectory_.poses.push_back(pose);
    }
}

nav_msgs::Path OdomTracker::getPath()
{
    std::lock_guard<std::mutex> lock(m_mutex_);
    return this->baseTrajectory_;
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

    bool acceptBackward = false;
    bool pullingerror = false;
    if(baseTrajectory_.poses.empty())
    {
        acceptBackward=false;
    }
    else
    {
        const geometry_msgs::Point& prevPoint = baseTrajectory_.poses.back().pose.position;
        const geometry_msgs::Point& currePoint = base_pose.pose.position;
        double lastpointdist = p2pDistance(prevPoint, currePoint);
        
        acceptBackward = !baseTrajectory_.poses.empty() 
                        && lastpointdist < minPointDistanceBackwardThresh_;

        pullingerror = lastpointdist > 2 * minPointDistanceBackwardThresh_;
    }

    //ROS_INFO("Backwards, last distance: %lf < %lf accept: %d", dist, minPointDistanceBackwardThresh_, acceptBackward);
    if (acceptBackward) 
    {
        baseTrajectory_.poses.pop_back();
    } 
    else if (pullingerror) {
        ROS_INFO_THROTTLE(2,"Incorrect backwards motion.");
    } 
    else 
    {
        /// Not removing point because it is enough far from the last cord point
    }

    return acceptBackward;
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
/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/common.h>
#include <smacc/component.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <realtime_tools/realtime_publisher.h>
#include <mutex>
#include <memory>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>


namespace smacc_odom_tracker
{

enum class WorkingMode : uint8_t {
    RECORD_PATH_FORWARD = 0,
    CLEAR_PATH_BACKWARD = 1,
    IDLE = 2
};

/// This class track the required distance of the cord based on the external localization system
class OdomTracker: public smacc::ISmaccComponent
{
    public:      
        OdomTracker();

        /// Must be called at the begining of the execution
        // by default, the component start in record_forward mode and publishing the
        // current path
        void initialize(std::string nodeName);

        
        // threadsafe
        /// odom callback: Updates the path - this must be called periodically for each odometry message. 
        // The odom parameters is the main input of this tracker
        virtual void processOdometryMessage(const nav_msgs::Odometry& odom);     

        // ------ CONTROL COMMANDS ---------------------
        // threadsafe
        void setWorkingMode(WorkingMode workingMode);

        // threadsafe
        void setPublishMessages(bool value);

        // threadsafe        
        void pushPath();
        
        // threadsafe
        void popPath();

        // threadsafe
        void clearPath();

        // threadsafe
        void setStartPoint(const geometry_msgs::PoseStamped& pose);
        
        // threadsafe
        nav_msgs::Path getPath();

    protected:

        virtual void rtPublishPaths(ros::Time timestamp); 

        // this is called when a new odom message is received in forward mode
        virtual bool updateForward(const nav_msgs::Odometry& odom);

        // this is called when a new odom message is received in backwards mode
        virtual bool updateBackward(const nav_msgs::Odometry& odom);
        
        // -------------- OUTPUTS ---------------------
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Path>> robotBasePathPub_;

        // --------------- INPUTS ------------------------
        // optional, this class can be used directly calling the odomProcessing method
        // without any subscriber
        ros::Subscriber odomSub_;

        // -------------- PARAMETERS ----------------------
        /// How much distance there is between two points of the path
        double minPointDistanceForwardThresh_;

        /// Meters
        double minPointDistanceBackwardThresh_;

        /// rads
        double minPointAngularDistanceForwardThresh_;

        /// rads
        double minPointAngularDistanceBackwardThresh_;


        // --------------- STATE ---------------
        // default true
        bool publishMessages;

        /// Processed path for the mouth of the reel
        nav_msgs::Path baseTrajectory_;

        WorkingMode workingMode_;

        std::vector<nav_msgs::Path> pathStack_;

        // subscribes to topic on init if true
        bool subscribeToOdometryTopic_;

        std::mutex m_mutex_;
};

/**
******************************************************************************************************************
* p2pDistance()
******************************************************************************************************************
*/
    inline double p2pDistance(const geometry_msgs::Point&p1, const geometry_msgs::Point&p2)
    {
        double dx = (p1.x - p2.x);
        double dy = (p1.y - p2.y);
        double dz = (p2.z - p2.z);
        double dist = sqrt(dx*dx + dy*dy+dz*dz);
        return dist;
    }
}

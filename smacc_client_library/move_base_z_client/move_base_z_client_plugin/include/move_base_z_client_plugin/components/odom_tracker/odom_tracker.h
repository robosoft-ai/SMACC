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

#include <dynamic_reconfigure/server.h>
#include <move_base_z_client_plugin/OdomTrackerConfig.h>
    
namespace cl_move_base_z
{
namespace odom_tracker
{

enum class WorkingMode : uint8_t
{
    RECORD_PATH = 0,
    CLEAR_PATH = 1,
    IDLE = 2
};

/// This class track the required distance of the cord based on the external localization system
class OdomTracker : public smacc::ISmaccComponent
{
public:
    // by default, the component start in record_path mode and publishing the
    // current path
    OdomTracker(std::string odomtopicName = "/odom", std::string odomFrame = "odom");

    // threadsafe
    /// odom callback: Updates the path - this must be called periodically for each odometry message.
    // The odom parameters is the main input of this tracker
    virtual void processOdometryMessage(const nav_msgs::Odometry &odom);

    // ------ CONTROL COMMANDS ---------------------
    // threadsafe
    void setWorkingMode(WorkingMode workingMode);

    // threadsafe
    void setPublishMessages(bool value);

    // threadsafe
    void pushPath();

    // threadsafe
    void popPath(int pathCount = 1, bool keepPreviousPath = false);

    // threadsafe
    void clearPath();

    // threadsafe
    void setStartPoint(const geometry_msgs::PoseStamped &pose);

    // threadsafe
    void setStartPoint(const geometry_msgs::Pose &pose);

    // threadsafe
    nav_msgs::Path getPath();

    void logStateString();

protected:
    dynamic_reconfigure::Server<move_base_z_client_plugin::OdomTrackerConfig> paramServer_;
    dynamic_reconfigure::Server<move_base_z_client_plugin::OdomTrackerConfig>::CallbackType f;

    void reconfigCB(move_base_z_client_plugin::OdomTrackerConfig &config, uint32_t level);

    virtual void rtPublishPaths(ros::Time timestamp);

    // this is called when a new odom message is received in record path mode
    virtual bool updateRecordPath(const nav_msgs::Odometry &odom);

    // this is called when a new odom message is received in clear path mode
    virtual bool updateClearPath(const nav_msgs::Odometry &odom);

    void updateAggregatedStackPath();

    // -------------- OUTPUTS ---------------------
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Path>> robotBasePathPub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Path>> robotBasePathStackedPub_;

    // --------------- INPUTS ------------------------
    // optional, this class can be used directly calling the odomProcessing method
    // without any subscriber
    ros::Subscriber odomSub_;

    // -------------- PARAMETERS ----------------------
    /// How much distance there is between two points of the path
    double recordPointDistanceThreshold_;

    /// Meters
    double recordAngularDistanceThreshold_;

    /// rads
    double clearPointDistanceThreshold_;

    /// rads
    double clearAngularDistanceThreshold_;

    std::string odomFrame_;

    // --------------- STATE ---------------
    // default true
    bool publishMessages;

    /// Processed path for the mouth of the reel
    nav_msgs::Path baseTrajectory_;

    WorkingMode workingMode_;

    std::vector<nav_msgs::Path> pathStack_;

    nav_msgs::Path aggregatedStackPathMsg_;

    // subscribes to topic on init if true
    bool subscribeToOdometryTopic_;

    std::mutex m_mutex_;
};

/**
******************************************************************************************************************
* p2pDistance()
******************************************************************************************************************
*/
inline double p2pDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    double dx = (p1.x - p2.x);
    double dy = (p1.y - p2.y);
    double dz = (p2.z - p2.z);
    double dist = sqrt(dx * dx + dy * dy + dz * dz);
    return dist;
}
} // namespace odom_tracker
} // namespace cl_move_base_z
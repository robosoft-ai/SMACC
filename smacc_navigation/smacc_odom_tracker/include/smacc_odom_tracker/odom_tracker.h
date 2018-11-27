#pragma once

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
    DISPENSING = 0,
    RETRACTING = 1,
    NONE = 2
};

/// This class track the required distance of the cord based on the external localization system
class OdomTracker
{
    public:       
        /// Must be called at the begining of the execution
        virtual void init(const tf::Transform& reel_mouth_transform, ros::NodeHandle& h);

        /// Updates the path - this must be called periodically for each odometry message. The odom parameters is the main input of this tracker
        virtual void processOdometryMessage(const nav_msgs::Odometry& odom, WorkingMode dispense_mode);     

    protected:
        virtual void rtPublishPaths(ros::Time timestamp); 

        void createMouthPoseFromBasePose(const std_msgs::Header& currentHeader, const geometry_msgs::PoseStamped& base_pose, geometry_msgs::PoseStamped& mouth_pose);
    
        /// Fixed and known transform from the base of the robot to the reel mouth
        tf::Transform reelMouthTransform_;
        
        /// Processed path for the mouth of the reel
        nav_msgs::Path reelMouthTrajectory_;

        /// Raw input
        std::vector<nav_msgs::Odometry> robotBaseOdometryHistory_;

        /// Processed path for the mouth of the reel
        nav_msgs::Path baseTrajectory_;

        void updateDispense(const geometry_msgs::PoseStamped& base_pose, const geometry_msgs::PoseStamped& mouth_pose);

        void updateRetracting(const geometry_msgs::PoseStamped& base_pose, const geometry_msgs::PoseStamped& mouth_pose);

        double computeDistance(geometry_msgs::Pose& a,geometry_msgs::Pose& b );
        
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Path>> robotBasePathPub_;

        /// How much distance there is between two points of the path
        double minPointDistanceDispenseThresh_;

        /// Meters
        double minPointDistanceRetractThresh_;

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
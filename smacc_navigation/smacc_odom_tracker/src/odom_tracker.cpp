#include <smacc_odom_tracker/odom_tracker.h>

namespace smacc_odom_tracker
{
/**
******************************************************************************************************************
* init()
******************************************************************************************************************
*/
    void OdomTracker::init(const tf::Transform& reel_mouth_transform, ros::NodeHandle& nh)
    {
        ROS_INFO("Initializing Odometry Tracker");
  
        robotBasePathPub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Path>>(nh, "reel_base_path", 1);

        reelMouthTransform_ = reel_mouth_transform;

        if(!nh.getParam("min_point_distance_dispense_thresh",minPointDistanceDispenseThresh_))
        {
            minPointDistanceDispenseThresh_ = 0.005; // 1 mm
        }

        if(!nh.getParam("min_point_distance_retract_thresh",minPointDistanceDispenseThresh_))
        {
            minPointDistanceRetractThresh_ = 0.1; // 1 mm
        }
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
* updateRetracting()
******************************************************************************************************************
*/
    void OdomTracker::updateRetracting(const geometry_msgs::PoseStamped& base_pose, const geometry_msgs::PoseStamped& mouth_pose)
    {
        const geometry_msgs::Point& prevPoint = reelMouthTrajectory_.poses.back().pose.position;
        const geometry_msgs::Point& currePoint = mouth_pose.pose.position;
        double lastpointdist = p2pDistance(prevPoint, currePoint);
        
        bool acceptRetract = !reelMouthTrajectory_.poses.empty()
                && lastpointdist < minPointDistanceRetractThresh_;

        //ROS_INFO("RETRACTING, last distance: %lf < %lf accept: %d", dist, minPointDistanceRetractionThresh_, acceptRetract);

        if (acceptRetract) {
            reelMouthTrajectory_.poses.pop_back();
            baseTrajectory_.poses.pop_back();
        } else if (lastpointdist > 2 * minPointDistanceRetractThresh_) {
            ROS_WARN("Incorrect retracting motion. The robot is pulling the cord.");
        } else {
            /// Not removing point because it is enough far from the last cord point
        }
    }
/**
******************************************************************************************************************
* updateDispense()
******************************************************************************************************************
*/
    void OdomTracker::updateDispense(const geometry_msgs::PoseStamped& base_pose, const geometry_msgs::PoseStamped& mouth_pose)
    {
        bool enqueueOdomMessage = false;

        double dist = -1;
        if(reelMouthTrajectory_.poses.size() == 0)
        {
            enqueueOdomMessage = true;
        }
        else
        {
            const geometry_msgs::Point& prevPoint = reelMouthTrajectory_.poses.back().pose.position;
            const geometry_msgs::Point& currePoint = mouth_pose.pose.position;
            dist = p2pDistance(prevPoint, currePoint);
            //ROS_WARN("dist %lf vs min %lf", dist, minPointDistanceDispenseThresh_);

            if(dist > minPointDistanceDispenseThresh_)
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
            //robotBaseOdometryHistory_.push_back(odom);
                       
            baseTrajectory_.poses.push_back(base_pose);
            reelMouthTrajectory_.poses.push_back(mouth_pose);
        }
    }

    void OdomTracker::createMouthPoseFromBasePose(const std_msgs::Header& currentHeader, const geometry_msgs::PoseStamped& base_pose, geometry_msgs::PoseStamped& mouth_pose)
    {
        /// Math for the mouth of the reel
        tf::Transform robot_base_transform;
        tf::poseMsgToTF(base_pose.pose,robot_base_transform);

        tf::Transform absoluteReelMoutTransform;
        absoluteReelMoutTransform.mult(robot_base_transform, reelMouthTransform_);

        tf::poseTFToMsg(absoluteReelMoutTransform, mouth_pose.pose);
        
		/// Copy Timestamp
        mouth_pose.header = currentHeader;
    }
/**
******************************************************************************************************************
* processOdometryMessage()
******************************************************************************************************************
*/
    void OdomTracker::processOdometryMessage(const nav_msgs::Odometry& odom, WorkingMode dispense_mode)      
    {
        std::lock_guard<std::mutex> lock(m_mutex_);

        //ROS_INFO("process odom callback queue");

        // we initially accept any message if the queue is empty
        
         /// Track robot base pose
        geometry_msgs::PoseStamped base_pose;
        base_pose.pose = odom.pose.pose;
        base_pose.header = odom.header;
        baseTrajectory_.header = odom.header;

        geometry_msgs::PoseStamped mouth_pose;
        createMouthPoseFromBasePose(odom.header, base_pose, mouth_pose);

        reelMouthTrajectory_.header = odom.header;
        
		if(dispense_mode == WorkingMode::DISPENSING)
        {
            updateDispense(base_pose, mouth_pose);
        }		
		else if (dispense_mode == WorkingMode::RETRACTING)
        {
            updateRetracting(base_pose, mouth_pose);
        }

        rtPublishPaths(odom.header.stamp);
    }
}
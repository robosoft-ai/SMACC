/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <forward_local_planner/forward_local_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/intrusive_ptr.hpp>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(forward_local_planner::ForwardLocalPlanner, nav_core::BaseLocalPlanner)

namespace forward_local_planner 
{
/**
******************************************************************************************************************
* ForwardLocalPlanner()
******************************************************************************************************************
*/
ForwardLocalPlanner::ForwardLocalPlanner()
{
}

/**
******************************************************************************************************************
* ForwardLocalPlanner()
******************************************************************************************************************
*/
ForwardLocalPlanner::~ForwardLocalPlanner()
{
}


void ForwardLocalPlanner::initialize()
{
    k_rho_ = 1.0;
    k_alpha_ = -0.2;
    k_betta_ = -1.0; // set to zero means that orientation is not important
    //k_betta_ = 1.0;
    //betta_offset_=0;
    
    goalReached_=false;
    carrot_distance_= 0.4;

    ros::NodeHandle private_nh("~");

    currentPoseIndex_ = 0;

    ros::NodeHandle nh("~/ForwardLocalPlanner");

    nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
    nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
    goalMarkerPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("goal_marker", 1);     
}

void ForwardLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) 
{
    costmapRos_ = costmap_ros;
    this->initialize();
}

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
void ForwardLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    costmapRos_ = costmap_ros;
    this->initialize();
}

/**
******************************************************************************************************************
* publishGoalMarker()
******************************************************************************************************************
*/
void ForwardLocalPlanner::publishGoalMarker(double x, double y, double phi)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now ();
    marker.ns = "my_namespace2";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x=0.1;
    marker.scale.y=0.3;
    marker.scale.z=0.1;
    marker.color.a= 1.0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1.0;

    geometry_msgs::Point start,end;
    start.x= x;
    start.y =y;

    end.x= x + 0.5 * cos(phi);
    end.y =y + 0.5 * sin(phi);

    marker.points.push_back(start);
    marker.points.push_back(end);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker);

    this->goalMarkerPublisher_.publish(ma);
}



template<class T>
auto optionalRobotPose(boost::intrusive_ptr<T>& obj, costmap_2d::Costmap2DROS* costmapRos)
 -> decltype(  obj->nh  )
{
    costmapRos->getRobotPose(*obj);
}

template<class T>
auto optionalRobotPose(T* obj, costmap_2d::Costmap2DROS* costmapRos) -> ros::NodeHandle
{
  tf::Stamped<tf::Pose> tfpose;
  costmapRos->getRobotPose(tfpose);

  tf::poseStampedTFToMsg(tfpose,*obj);
}


/**
******************************************************************************************************************
* computeVelocityCommands()
******************************************************************************************************************
*/
bool ForwardLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    goalReached_=false;
    //ROS_DEBUG("LOCAL PLANNER LOOP");

    geometry_msgs::PoseStamped paux;
    optionalRobotPose(&paux, costmapRos_);
    tf::Stamped<tf::Pose> tfpose;
    tf::poseStampedMsgToTF(paux, tfpose);

    tf::Quaternion q = tfpose.getRotation();

    bool ok = false;
    while (!ok) 
    {
        // iterate the point from the current position and ahead until reaching a new goal point in the path
        for (; !ok && currentPoseIndex_ < plan_.size(); currentPoseIndex_++) {
            auto& pose = plan_[currentPoseIndex_];
            const geometry_msgs::Point& p = pose.pose.position;
            tf::Quaternion q;
            tf::quaternionMsgToTF(pose.pose.orientation, q);

            // take error from the current position to the path point
            double dx = p.x - tfpose.getOrigin().x();
            double dy = p.y - tfpose.getOrigin().y();
            double dist = sqrt(dx * dx + dy * dy);

            double pangle = tf::getYaw(q);
            double angle = tf::getYaw(tfpose.getRotation());
            double angular_error = angles::shortest_angular_distance(pangle, angle);

            if (dist >= carrot_distance_|| angular_error >0.1)  
            {
                // the target pose is enough different to be defined as a target
                ok = true;
                //ROS_INFO("forward: %lf", 100.0 * currentPoseIndex_ / (double)plan_.size());
            }
        }

        if (currentPoseIndex_ >= plan_.size()) {
            // even the latest point is quite similar, then take the last since it is the final goal
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            //ROS_INFO("End Local planner");
            ok = true;
            currentPoseIndex_ = plan_.size() -1;
            //return true;
        }
    }

    //ROS_INFO("pose control algorithm");
    
    const geometry_msgs::PoseStamped& goalpose = plan_[currentPoseIndex_];
    const geometry_msgs::Point& goalposition = goalpose.pose.position;

    tf::Quaternion goalQ;
    tf::quaternionMsgToTF(goalpose.pose.orientation, goalQ);
    //ROS_INFO_STREAM("Plan goal quaternion at "<< goalpose.pose.orientation);

    //goal orientation (global frame)
    double betta = tf::getYaw(goalpose.pose.orientation)+ betta_offset_;

    double dx = goalposition.x - tfpose.getOrigin().x();
    double dy = goalposition.y - tfpose.getOrigin().y();

    //distance error to the targetpoint
    double rho_error = sqrt(dx * dx + dy * dy);

    //current angle
    double theta = tf::getYaw(q);
    double alpha = atan2(dy, dx);
    alpha = alpha + alpha_offset_;

    double alpha_error = angles::shortest_angular_distance(alpha, theta);
    double betta_error = angles::shortest_angular_distance(betta, theta);

    double vetta;// = k_rho_ * rho_error;
    double gamma ;//= k_alpha_ * alpha_error + k_betta_ * betta_error;

    if (rho_error > xy_goal_tolerance_)
    {
        vetta = k_rho_ * rho_error;
        gamma = k_alpha_ * alpha_error;
    }
    else if (fabs(betta_error) >= 0.01)
    {
        vetta = 0;
        gamma = k_betta_*betta_error;
    }
    else
    {
        vetta = 0;
        gamma = 0;
        goalReached_=true;
    }
    
    if (vetta > 0.4)
    {
        vetta = 0.4;
    }

    cmd_vel.linear.x = vetta;
    cmd_vel.angular.z = gamma;

    //ROS_INFO_STREAM("Local planner: "<< cmd_vel);

    publishGoalMarker(goalposition.x, goalposition.y, betta);
    
    ROS_DEBUG_STREAM("Forward local planner," << std::endl
                                        << " theta: " << theta << std::endl
                                        << " betta: " << betta << std::endl
                                        << " err_x: " << dx << std::endl
                                        << " err_y:" << dy << std::endl
                                        << " rho_error:" << rho_error << std::endl
                                        << " alpha_error:" << alpha_error << std::endl
                                        << " betta_error:" << betta_error << std::endl
                                        << " vetta:" << vetta << std::endl
                                        << " gamma:" << gamma << std::endl
                                        << "xy_goal_tolerance:" << xy_goal_tolerance_);
    
    //if(cmd_vel.linear.x==0 && cmd_vel.angular.z == 0 )
    //{  
    //}

    return true;
}

/**
******************************************************************************************************************
* isGoalReached()
******************************************************************************************************************
*/
bool ForwardLocalPlanner::isGoalReached()
{
    return goalReached_;
}

/**
******************************************************************************************************************
* setPlan()
******************************************************************************************************************
*/
bool ForwardLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    plan_ = plan;
    goalReached_=false;
    return true;
}
}

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <rrbtx_retracting_local_planner/retracting_local_planner.h>
#include <visualization_msgs/MarkerArray.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrbtx_retracting_local_planner::RetractingLocalPlanner, nav_core::BaseLocalPlanner)

namespace rrbtx_retracting_local_planner {
    /**
******************************************************************************************************************
* retractingLocalPlanner()
******************************************************************************************************************
*/
RetractingLocalPlanner::RetractingLocalPlanner()
    : paramServer_(ros::NodeHandle("~RetractingLocalPlanner"))
{
}

/**
******************************************************************************************************************
* retractingLocalPlanner()
******************************************************************************************************************
*/
RetractingLocalPlanner::~RetractingLocalPlanner()
{
}

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
void RetractingLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    costmapRos_ = costmap_ros;
    k_rho_ = 1.0;
    k_alpha_ = 0.5;
    k_betta_ = -1.0; // set to zero means that orientation is not important
    //k_betta_ = 1.0;

    f = boost::bind(&RetractingLocalPlanner::reconfigCB, this, _1, _2);
    paramServer_.setCallback(f);
    currentPoseIndex_ = 0;

    ros::NodeHandle nh("~/RetractingLocalPlanner");
    goalMarkerPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("goal_marker", 1); 
}

/**
******************************************************************************************************************
* publishGoalMarker()
******************************************************************************************************************
*/
void RetractingLocalPlanner::publishGoalMarker(double x, double y, double phi)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now ();

    marker.ns = "my_namespace2";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.scale.x = 0.05;
    marker.scale.y = 0.15;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;

    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;

    geometry_msgs::Point start,end;
    start.x = x;
    start.y = y;

    end.x = x + 0.5 * cos(phi);
    end.y = y + 0.5 * sin(phi);

    marker.points.push_back(start);
    marker.points.push_back(end);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker);

    goalMarkerPublisher_.publish(ma);
}

/**
******************************************************************************************************************
* computeVelocityCommands()
******************************************************************************************************************
*/
bool RetractingLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    ROS_DEBUG("LOCAL PLANNER LOOP");

    tf::Stamped<tf::Pose> tfpose;
    costmapRos_->getRobotPose(tfpose);
    tf::Quaternion q = tfpose.getRotation();

    bool ok = false;
    while (!ok) {
        // iterate the point from the current position and ahead until reaching a new goal point in the path
        for (; !ok && currentPoseIndex_ < backwardsPlanPath_.size(); currentPoseIndex_++) {
            auto& pose = backwardsPlanPath_[currentPoseIndex_];
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

            if (dist*2 >= carrot_distance_ || angular_error >= 0.01) {
                // target pose found
                ok = true;
                ROS_INFO("Retracting: %lf", 100.0 * currentPoseIndex_ / (double)backwardsPlanPath_.size());
            }
        }

        if (currentPoseIndex_ >= backwardsPlanPath_.size()) {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            ok = true;
            currentPoseIndex_ = backwardsPlanPath_.size() -1;
            //return true;
        }
    }

    //ROS_INFO("pose control algorithm");
    
    const geometry_msgs::PoseStamped& goalpose = backwardsPlanPath_[currentPoseIndex_++];
    const geometry_msgs::Point& goalposition = goalpose.pose.position;

    tf::Quaternion goalQ;
    tf::quaternionMsgToTF(goalpose.pose.orientation, goalQ);

    //goal orientation (global frame)
    double betta = tf::getYaw(goalQ);
    betta = betta + betta_offset_;

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

    double vetta = k_rho_ * rho_error;
    double gamma = k_alpha_ * alpha_error + k_betta_ * betta_error;
    
    if (rho_error > 0.05)
    {
        vetta = k_rho_ * rho_error;
        gamma = k_alpha_ * alpha_error;
    }
    else if (fabs(betta_error) >= 0.01)
    {
        vetta = 0;
        gamma = k_betta_*betta_error;
    }
    else if (currentPoseIndex_  >= backwardsPlanPath_.size() - 1)
    {
        vetta = 0;
        gamma = 0;
        goalReached_=true;
        ROS_INFO_STREAM("RETRACTION LOCAL PLANNER END: rhoerror: " << rho_error);
    }

    cmd_vel.linear.x = vetta;
    cmd_vel.angular.z = gamma;

    //ROS_INFO_STREAM("Local planner: "<< cmd_vel);

    publishGoalMarker(goalposition.x, goalposition.y, betta);
    
    /*
    ROS_INFO_STREAM("local planner," << std::endl
                                        << " theta: " << theta << std::endl
                                        << " betta: " << theta << std::endl
                                        << " err_x: " << dx << std::endl
                                        << " err_y:" << dy << std::endl
                                        << " rho_error:" << rho_error << std::endl
                                        << " alpha_error:" << alpha_error << std::endl
                                        << " betta_error:" << betta_error << std::endl
                                        << " vetta:" << vetta << std::endl
                                        << " gamma:" << gamma);*/

    //if(cmd_vel.linear.x==0 && cmd_vel.angular.z == 0 )
    //{  
    //}

    return true;
}

/**
******************************************************************************************************************
* reconfigCB()
******************************************************************************************************************
*/
void RetractingLocalPlanner::reconfigCB(rrbtx_retracting_local_planner::RetractingLocalPlannerConfig& config, uint32_t level)
{
    ROS_INFO("Reconfigure Request");
    k_alpha_ = config.k_alpha;
    k_betta_ = config.k_betta;
    k_rho_ = config.k_rho;

    alpha_offset_ = config.alpha_offset;
    betta_offset_ = config.betta_offset;

    carrot_distance_ = config.carrot_distance;
    carrot_angular_distance_ = config.carrot_angular_distance;
}

/**
******************************************************************************************************************
* isGoalReached()
******************************************************************************************************************
*/
bool RetractingLocalPlanner::isGoalReached()
{
    return goalReached_;
}

/**
******************************************************************************************************************
* setPlan()
******************************************************************************************************************
*/
bool RetractingLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    goalReached_=false;
    backwardsPlanPath_ = plan;
    return true;
}
}

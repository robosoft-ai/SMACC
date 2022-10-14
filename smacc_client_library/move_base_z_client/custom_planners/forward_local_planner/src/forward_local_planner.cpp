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
#include <angles/angles.h>

#include <base_local_planner/simple_trajectory_generator.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cl_move_base_z::forward_local_planner::ForwardLocalPlanner, nav_core::BaseLocalPlanner)

namespace cl_move_base_z
{
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
    k_alpha_ = -0.4;
    k_betta_ = -1.0; // set to zero means that orientation is not important
    //k_betta_ = 1.0;
    //betta_offset_=0;

    goalReached_ = false;
    carrot_distance_ = 0.4;

    ros::NodeHandle private_nh("~");

    currentPoseIndex_ = 0;

    ros::NodeHandle nh("~/ForwardLocalPlanner");

    nh.param("k_rho", k_rho_, k_rho_);
    nh.param("k_alpha", k_alpha_, k_alpha_);
    nh.param("k_betta", k_betta_, k_betta_);
    nh.param("carrot_distance", carrot_distance_, carrot_distance_);

    nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
    nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
    nh.param("max_linear_x_speed", max_linear_x_speed_, 1.0);
    nh.param("max_angular_z_speed", max_angular_z_speed_, 2.0);

    ROS_INFO("[ForwardLocalPlanner] max linear speed: %lf, max angular speed: %lf, k_rho: %lf, carrot_distance: %lf, ", max_linear_x_speed_, max_angular_z_speed_, k_rho_, carrot_distance_);
    goalMarkerPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("goal_marker", 1);

    waiting_ = false;
    waitingTimeout_ = ros::Duration(10);
    ROS_INFO("[ForwardLocalPlanner] initialized");
}

void ForwardLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
    costmapRos_ = costmap_ros;
    this->initialize();
}

void ForwardLocalPlanner::generateTrajectory(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, float maxdist, float maxanglediff, float maxtime, float dt, std::vector<Eigen::Vector3f> &outtraj)
{
    //simulate the trajectory and check for collisions, updating costs along the way
    bool end = false;
    float time = 0;
    Eigen::Vector3f currentpos = pos;
    int i = 0;
    while (!end)
    {
        //add the point to the trajectory so we can draw it later if we want
        //traj.addPoint(pos[0], pos[1], pos[2]);

        // if (continued_acceleration_) {
        //   //calculate velocities
        //   loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
        //   //ROS_WARN_NAMED("Generator", "Flag: %d, Loop_Vel %f, %f, %f", continued_acceleration_, loop_vel[0], loop_vel[1], loop_vel[2]);
        // }

        auto loop_vel = vel;
        //update the position of the robot using the velocities passed in
        auto newpos = computeNewPositions(currentpos, loop_vel, dt);

        auto dx = newpos[0] - currentpos[0];
        auto dy = newpos[1] - currentpos[1];
        float dist, angledist;

        //ROS_INFO("traj point %d", i);
        dist = sqrt(dx * dx + dy * dy);
        if (dist > maxdist)
        {
            end = true;
            //ROS_INFO("dist break: %f", dist);
        }
        else
        {
            // ouble from, double to
            angledist = fabs(angles::shortest_angular_distance(currentpos[2], newpos[2]));
            if (angledist > maxanglediff)
            {
                end = true;
                //ROS_INFO("angle dist break: %f", angledist);
            }
            else
            {
                outtraj.push_back(newpos);

                time += dt;
                if (time > maxtime)
                {
                    end = true;
                    //ROS_INFO("time break: %f", time);
                }

                //ROS_INFO("dist: %f, angledist: %f, time: %f", dist, angledist, time);
            }
        }

        currentpos = newpos;
        i++;
    } // end for simulation steps
}

Eigen::Vector3f ForwardLocalPlanner::computeNewPositions(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, double dt)
{
    Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
    new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
    new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
    new_pos[2] = pos[2] + vel[2] * dt;
    return new_pos;
}

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
void ForwardLocalPlanner::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
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

    marker.header.frame_id = this->costmapRos_->getGlobalFrameID();
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace2";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;

    marker.scale.x = 0.1;
    marker.scale.y = 0.3;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1.0;

    geometry_msgs::Point start, end;
    start.x = x;
    start.y = y;

    end.x = x + 0.5 * cos(phi);
    end.y = y + 0.5 * sin(phi);

    marker.points.push_back(start);
    marker.points.push_back(end);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker);

    this->goalMarkerPublisher_.publish(ma);
}

// MELODIC
#if ROS_VERSION_MINIMUM(1, 13, 0)
tf::Stamped<tf::Pose> optionalRobotPose(costmap_2d::Costmap2DROS *costmapRos)
{
    geometry_msgs::PoseStamped paux;
    costmapRos->getRobotPose(paux);
    tf::Stamped<tf::Pose> tfpose;
    tf::poseStampedMsgToTF(paux, tfpose);
    return tfpose;
}
#else
// INDIGO AND PREVIOUS
tf::Stamped<tf::Pose> optionalRobotPose(costmap_2d::Costmap2DROS *costmapRos)
{
    tf::Stamped<tf::Pose> tfpose;
    costmapRos->getRobotPose(tfpose);
    return tfpose;
}
#endif

void clamp(geometry_msgs::Twist &cmd_vel, double max_linear_x_speed_, double max_angular_z_speed_)
{
    if (max_angular_z_speed_ == 0 || max_linear_x_speed_ == 0)
        return;

    if (cmd_vel.angular.z == 0)
    {
        cmd_vel.linear.x = max_linear_x_speed_;
    }
    else
    {
        double kurvature = cmd_vel.linear.x / cmd_vel.angular.z;

        double linearAuthority = fabs(cmd_vel.linear.x / max_linear_x_speed_);
        double angularAuthority = fabs(cmd_vel.angular.z / max_angular_z_speed_);
        if (linearAuthority < angularAuthority)
        {
            // lets go to maximum linear speed
            cmd_vel.linear.x = max_linear_x_speed_;
            cmd_vel.angular.z = kurvature / max_linear_x_speed_;
            ROS_WARN_STREAM("k=" << kurvature << "lets go to maximum linear capacity: " << cmd_vel);
        }
        else
        {
            // lets go with maximum angular speed
            cmd_vel.angular.x = max_angular_z_speed_;
            cmd_vel.linear.x = kurvature * max_angular_z_speed_;
            ROS_WARN_STREAM("lets go to maximum angular capacity: " << cmd_vel);
        }
    }
}

/**
******************************************************************************************************************
* computeVelocityCommands()
******************************************************************************************************************
*/
bool ForwardLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    goalReached_ = false;
    ROS_DEBUG("[ForwardLocalPlanner] ----- COMPUTE VELOCITY COMMAND LOCAL PLANNER ---");

    tf::Stamped<tf::Pose> tfpose = optionalRobotPose(costmapRos_);

    geometry_msgs::PoseStamped currentPose;
    tf::poseStampedTFToMsg(tfpose,currentPose);
    ROS_DEBUG_STREAM("[ForwardLocalPlanner] current robot pose " << currentPose);


    bool ok = false;
    while (!ok)
    {
        // iterate the point from the current position and ahead until reaching a new goal point in the path
        while (!ok && currentPoseIndex_ < plan_.size())
        {
            auto &pose = plan_[currentPoseIndex_];
            const geometry_msgs::Point &p = pose.pose.position;
            tf::Quaternion q;
            tf::quaternionMsgToTF(pose.pose.orientation, q);

            // take error from the current position to the path point
            double dx = p.x - tfpose.getOrigin().x();
            double dy = p.y - tfpose.getOrigin().y();
            double dist = sqrt(dx * dx + dy * dy);

            double pangle = tf::getYaw(q);
            double angle = tf::getYaw(tfpose.getRotation());
            double angular_error = angles::shortest_angular_distance(pangle, angle);

            if (dist >= carrot_distance_ || fabs(angular_error) > 0.1)
            {
                // the target pose is enough different to be defined as a target
                ok = true;
                ROS_DEBUG("current index: %d, carrot goal percentaje: %lf, dist: %lf, maxdist: %lf, angle_error: %lf", currentPoseIndex_, 100.0 * currentPoseIndex_ / plan_.size(), dist, carrot_distance_, angular_error);
            }
            else
            {
                currentPoseIndex_++;
            }
        }

        ROS_DEBUG_STREAM("[ForwardLocalPlanner] selected carrot pose index " << currentPoseIndex_ << "/" << plan_.size());

        if (currentPoseIndex_ >= plan_.size())
        {
            // even the latest point is quite similar, then take the last since it is the final goal
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            //ROS_INFO("End Local planner");
            ok = true;
            currentPoseIndex_ = plan_.size() - 1;
            //return true;
        }
    }

    //ROS_INFO("pose control algorithm");

    const geometry_msgs::PoseStamped &finalgoalpose = plan_.back();
    const geometry_msgs::PoseStamped &carrot_goalpose = plan_[currentPoseIndex_];
    const geometry_msgs::Point &goalposition = carrot_goalpose.pose.position;

    tf::Quaternion carrotGoalQ;
    tf::quaternionMsgToTF(carrot_goalpose.pose.orientation, carrotGoalQ);
    //ROS_INFO_STREAM("Plan goal quaternion at "<< goalpose.pose.orientation);

    //goal orientation (global frame)
    double betta = tf::getYaw(carrot_goalpose.pose.orientation) + betta_offset_;

    double dx = goalposition.x - tfpose.getOrigin().x();
    double dy = goalposition.y - tfpose.getOrigin().y();

    //distance error to the targetpoint
    double rho_error = sqrt(dx * dx + dy * dy);

    //current angle
    tf::Quaternion currentOrientation = tfpose.getRotation();
    double theta = tf::getYaw(currentOrientation);
    double alpha = atan2(dy, dx);
    alpha = alpha + alpha_offset_;

    double alpha_error = angles::shortest_angular_distance(alpha, theta);
    double betta_error = angles::shortest_angular_distance(betta, theta);

    double vetta; // = k_rho_ * rho_error;
    double gamma; //= k_alpha_ * alpha_error + k_betta_ * betta_error;

    if (rho_error > xy_goal_tolerance_) // reguular control rule, be careful, rho error is with the carrot not with the final goal (this is something to improve like the backwards planner)
    {
        vetta = k_rho_ * rho_error;
        gamma = k_alpha_ * alpha_error;
    }
    else if (fabs(betta_error) >= yaw_goal_tolerance_) // pureSpining
    {
        vetta = 0;
        gamma = k_betta_ * betta_error;
    }
    else // goal reached
    {
        ROS_DEBUG("GOAL REACHED");
        vetta = 0;
        gamma = 0;
        goalReached_ = true;
    }

    // linear speed clamp
    if (vetta > max_linear_x_speed_)
    {
        vetta = max_linear_x_speed_;
    }
    else if (vetta < -max_linear_x_speed_)
    {
        vetta = -max_linear_x_speed_;
    }

    // angular speed clamp
    if (gamma > max_angular_z_speed_)
    {
        gamma = max_angular_z_speed_;
    }
    else if (gamma < -max_angular_z_speed_)
    {
        gamma = -max_angular_z_speed_;
    }

    cmd_vel.linear.x = vetta;
    cmd_vel.angular.z = gamma;

    //clamp(cmd_vel, max_linear_x_speed_, max_angular_z_speed_);

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
                                              << " xy_goal_tolerance:" << xy_goal_tolerance_ << std::endl
                                              << " yaw_goal_tolerance:" << yaw_goal_tolerance_ << std::endl);

    //if(cmd_vel.linear.x==0 && cmd_vel.angular.z == 0 )
    //{
    //}

    //integrate trajectory and check collision

    tf::Stamped<tf::Pose> global_pose = optionalRobotPose(costmapRos_);

    //->getRobotPose(global_pose);

    auto *costmap2d = costmapRos_->getCostmap();
    auto yaw = tf::getYaw(global_pose.getRotation());

    auto &pos = global_pose.getOrigin();

    Eigen::Vector3f currentpose(pos.x(), pos.y(), yaw);
    Eigen::Vector3f currentvel(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    std::vector<Eigen::Vector3f> trajectory;
    this->generateTrajectory(currentpose, currentvel, 0.8 /*meters*/, M_PI / 8 /*rads*/, 3.0 /*seconds*/, 0.05 /*seconds*/, trajectory);

    // check plan rejection
    bool aceptedplan = true;

    unsigned int mx, my;

    int i = 0;
    // ROS_INFO_STREAM("lplanner goal: " << finalgoalpose.pose.position);
    for (auto &p : trajectory)
    {
        float dx = p[0] - finalgoalpose.pose.position.x;
        float dy = p[1] - finalgoalpose.pose.position.y;

        float dst = sqrt(dx * dx + dy * dy);
        if (dst < xy_goal_tolerance_)
        {
            //  ROS_INFO("trajectory checking skipped, goal reached");
            break;
        }

        costmap2d->worldToMap(p[0], p[1], mx, my);
        unsigned int cost = costmap2d->getCost(mx, my);

        // ROS_INFO("checking cost pt %d [%lf, %lf] cell[%d,%d] = %d", i, p[0], p[1], mx, my, cost);
        // ROS_INFO_STREAM("cost: " << cost);

        // static const unsigned char NO_INFORMATION = 255;
        // static const unsigned char LETHAL_OBSTACLE = 254;
        // static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
        // static const unsigned char FREE_SPACE = 0;

        if (costmap2d->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            aceptedplan = false;
            // ROS_WARN("ABORTED LOCAL PLAN BECAUSE OBSTACLE DETEDTED");
            break;
        }
        i++;
    }

    if (aceptedplan)
    {
        waiting_ = false;
        return true;
    }
    else
    {
        // stop and wait
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;

        if (waiting_ == false)
        {
            waiting_ = true;
            waitingStamp_ = ros::Time::now();
        }
        else
        {
            auto waitingduration = ros::Time::now() - waitingStamp_;

            if (waitingduration > this->waitingTimeout_)
            {
                return false;
            }
        }

        return true;
    }
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
bool ForwardLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
    plan_ = plan;
    goalReached_ = false;
    return true;
}
} // namespace forward_local_planner
} // namespace cl_move_base_z

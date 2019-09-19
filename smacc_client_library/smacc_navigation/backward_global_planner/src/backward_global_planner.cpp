/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <boost/assign.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <backward_global_planner/backward_global_planner.h>
#include <fstream>
#include <streambuf>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <forward_global_planner/reel_path_tools.h>


//register this planner as a BaseGlobalPlanner plugin

PLUGINLIB_EXPORT_CLASS(backward_global_planner::BackwardGlobalPlanner, nav_core::BaseGlobalPlanner);


namespace backward_global_planner {
/**
******************************************************************************************************************
* Constructor()
******************************************************************************************************************
*/
BackwardGlobalPlanner::BackwardGlobalPlanner()
{
    skip_straight_motion_distance_=0.2;
}

BackwardGlobalPlanner::~BackwardGlobalPlanner()
{
    //clear
    nav_msgs::Path planMsg;
    planMsg.header.stamp = ros::Time::now();
    planPub_.publish(planMsg);
}

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    //ROS_INFO_NAMED("Backwards", "BackwardGlobalPlanner initialize");
    costmap_ros_ = costmap_ros;
    //ROS_WARN_NAMED("Backwards", "initializating global planner, costmap address: %ld", (long)costmap_ros);

    forwardPathSub_ = nh_.subscribe("odom_tracker_path", 2, &BackwardGlobalPlanner::onForwardTrailMsg, this);
    
    ros::NodeHandle nh;
    cmd_server_ = nh.advertiseService<backward_global_planner::command::Request , backward_global_planner::command::Response  >("cmd", boost::bind(&BackwardGlobalPlanner::commandServiceCall,this,_1,_2));
    planPub_ = nh.advertise<nav_msgs::Path>("backward_planner/global_plan", 1);
    markersPub_ = nh.advertise<visualization_msgs::MarkerArray>("backward_planner/markers", 1);
}

/**
******************************************************************************************************************
* onForwardTrailMsg()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::onForwardTrailMsg(const nav_msgs::Path::ConstPtr& trailMessage)
{
    lastForwardPathMsg_ = *trailMessage;
}

/**
******************************************************************************************************************
* publishGoalMarker()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::publishGoalMarker(const geometry_msgs::Pose& pose, double r, double g, double b)
{
    double phi = tf::getYaw(pose.orientation);
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
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    geometry_msgs::Point start,end;
    start.x= pose.position.x;
    start.y =pose.position.y;

    end.x = pose.position.x + 0.5 * cos(phi);
    end.y = pose.position.y + 0.5 * sin(phi);

    marker.points.push_back(start);
    marker.points.push_back(end);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker);

    markersPub_.publish(ma);
}

/**
******************************************************************************************************************
* createPureSpiningAndStragihtLineBackwardPath()
******************************************************************************************************************
*/
bool BackwardGlobalPlanner::createPureSpiningAndStragihtLineBackwardPath(const geometry_msgs::PoseStamped& start,
const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    //tf::Stamped<tf::Pose> tfpose;
    //ROS_WARN_NAMED("Backwards", "getting robot pose referencing costmap: %ld", (long)costmap_ros_);
    
    auto q = start.pose.orientation;

    double dx = start.pose.position.x - goal.pose.position.x ;
    double dy = start.pose.position.y - goal.pose.position.y ;

    double lenght = sqrt(dx*dx + dy*dy);

    geometry_msgs::PoseStamped prevState;
    if (lenght > skip_straight_motion_distance_) 
    {   
        // skip initial pure spinning and initial straight motion
        //ROS_INFO("1 - heading to goal position pure spinning");
        double heading_direction = atan2(dy, dx) ;
        double startyaw = tf::getYaw(q);
        double offset = angles::shortest_angular_distance(startyaw, heading_direction);
        heading_direction = startyaw+offset;
        
        prevState = reel_path_tools::makePureSpinningSubPlan(start, heading_direction, plan, puresSpinningRadStep_);
        //ROS_INFO("2 - going forward keep orientation pure straight");
        
        prevState = reel_path_tools::makePureStraightSubPlan(prevState, goal.pose.position,  lenght, plan);
    }
    else
    {
        prevState = start;
    }
}

/**
******************************************************************************************************************
* defaultBackwardPath()
******************************************************************************************************************
*/
bool BackwardGlobalPlanner::createDefaultBackwardPath(const geometry_msgs::PoseStamped& start,
const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    auto q = start.pose.orientation;

    geometry_msgs::PoseStamped pose;
    pose = start;

    plan.push_back(pose);

    //ROS_WARN_NAMED("Backwards", "Iterating in last forward cord path");
    int i=lastForwardPathMsg_.poses.size();
    double mindist =std::numeric_limits<double>::max();
    int mindistindex = -1;

    geometry_msgs::Pose goalProjected;

    for (auto& p : lastForwardPathMsg_.poses | boost::adaptors::reversed) 
    {
        pose = p;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();
        pose.header.stamp = ros::Time::now();

        double dx = pose.pose.position.x - goal.pose.position.x;
        double dy = pose.pose.position.y - goal.pose.position.y;
        
        double dist = sqrt(dx*dx + dy*dy);
        if(dist <= mindist)
        {
            mindistindex = i;
            mindist = dist;
            goalProjected = pose.pose;
        }

        i--;
    }

    if(mindistindex != -1)
    {
        for (int i = lastForwardPathMsg_.poses.size() -1 ; i>=mindistindex ;i--)
        {
            auto& pose = lastForwardPathMsg_.poses[i];
            plan.push_back(pose);
        }
    }
    else
    {
        ROS_WARN_STREAM("Creating the backwards plan, it is not found any close trajectory point. Last forward path plan message size: " << lastForwardPathMsg_.poses.size());
    }
}

/**
******************************************************************************************************************
* makePlan()
******************************************************************************************************************
*/
bool BackwardGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    //ROS_WARN_NAMED("Backwards", "Backwards global planner: Generating global plan ");
    //ROS_WARN_NAMED("Backwards", "Clearing...");

    plan.clear();

    this->createDefaultBackwardPath(start, goal, plan);
    //this->createPureSpiningAndStragihtLineBackwardPath(start, goal, plan);

    //ROS_INFO_STREAM(" start - " << start);
    //ROS_INFO_STREAM(" end - " << goal.pose.position);
     
    //ROS_INFO("3 - heading to goal orientation");
    //double goalOrientation = angles::normalize_angle(tf::getYaw(goal.pose.orientation));
    //reel_path_tools::makePureSpinningSubPlan(prevState,goalOrientation,plan);

    //ROS_WARN_STREAM( "MAKE PLAN INVOKED, plan size:"<< plan.size());
    publishGoalMarker(goal.pose,1.0,0,1.0);

    nav_msgs::Path planMsg;
    planMsg .poses = plan;
    planMsg.header.frame_id="/odom";
    planPub_.publish(planMsg);

    // this was previously set to size() <= 1, but a plan with a single point is also a valid plan (the goal)
    if (plan.size() <=1)
    {
        ROS_INFO("cannot create backward plan");
        return false;
    }
    else
    {
        return true;
    }
}

/**
******************************************************************************************************************
* makePlan()
******************************************************************************************************************
*/
bool BackwardGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
    double& cost)
{
    cost = 0;
    makePlan(start, goal, plan);
    return true;
}

/**
******************************************************************************************************************
* commandServiceCall()
******************************************************************************************************************
*/
bool BackwardGlobalPlanner::commandServiceCall(backward_global_planner::command::Request  &req, backward_global_planner::command::Response  &res)
{
    //ROS_INFO_NAMED("Backwards", "BackwardGlobalplanner SERVICE CALL");
    std::string msg = req.cmd.data.c_str();

    std::vector<std::string> fields;   // Create a vector of strings, called "fields"
    boost::split(fields, msg, boost::algorithm::is_any_of(" "));

    //ROS_INFO_NAMED("Backwards", "backward planner SERVICE REQUEST");

    if(fields.size() == 0 )
    {
        res.success.data = false;
        return false;
    }

    std::string cmd = fields[0];
    bool error = false;
    if(cmd == "savepath")
    {
        //ROS_INFO_NAMED("Backwards","SAVE PATH COMMAND");
        if(fields.size() >1 )
        {
            std::vector<std::string> tail (fields.begin()+1, fields.end());
            std::string filename = boost::algorithm::join(tail, " ");

            std::ofstream os;
            os.open(filename);
            os << lastForwardPathMsg_;
            os.close();

            //ROS_INFO_STREAM("serialized path: " << lastForwardPathMsg_);
        }
        else
        {
            error = true;
        }
    }
    else if(cmd== "loadpath")
    {
        //ROS_INFO_NAMED("Backwards", "LOAD PATH COMMAND");
        if(fields.size() >1 )
        {
            std::vector<std::string> tail (fields.begin()+1, fields.end());
            std::string filename = boost::algorithm::join(tail, " ");

            std::ifstream ifs(filename);
            std::string alltext((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

            nav_msgs::Path p;
            uint32_t serial_size = ros::serialization::serializationLength(p);
            boost::shared_array<uint8_t> buffer (new uint8_t[serial_size]);
            ros::serialization::IStream stream(buffer.get(),serial_size);
            ros::serialization::deserialize(stream, p);

            //ROS_INFO_STREAM_NAMED("Backwards", "serialized path: " << p);
            lastForwardPathMsg_ = p;
        }
        else
        {
            error = true;
        }
    }
    else
    {
        res.success.data = false;
        return false;
    }

    res.success.data = error;
    return true;
}
}
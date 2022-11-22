#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/components/waypoints_navigator/waypoints_navigator.h>
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_datatypes.h>

namespace cl_move_base_z
{
WaypointNavigator::WaypointNavigator()
    : currentWaypoint_(0),
      waypoints_(0)
{
}

void WaypointNavigator::onInitialize()
{
  client_ = dynamic_cast<ClMoveBaseZ *>(owner_);
}

void WaypointNavigator::onGoalReached(ClMoveBaseZ::ResultConstPtr &res)
{
  waypointsEventDispatcher.postWaypointEvent(currentWaypoint_);
  currentWaypoint_++;
  this->succeddedConnection_.disconnect();
}

void WaypointNavigator::sendNextGoal()
{
  if (currentWaypoint_ >= 0 && currentWaypoint_ < waypoints_.size())
  {
    auto &next = waypoints_[currentWaypoint_];

    auto odomTracker = client_->getComponent<cl_move_base_z::odom_tracker::OdomTracker>();
    auto p = client_->getComponent<cl_move_base_z::Pose>();
    auto pose = p->toPoseMsg();

    ClMoveBaseZ::Goal goal;
    goal.target_pose.header.frame_id = p->getReferenceFrame();
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = next;

    auto plannerSwitcher = client_->getComponent<PlannerSwitcher>();
    plannerSwitcher->setDefaultPlanners();

    ros::spinOnce();
    ros::Duration(5).sleep();

    if (odomTracker != nullptr)
    {
      odomTracker->pushPath("FreeNavigationToGoalWaypointPose");
      odomTracker->setStartPoint(pose);
      odomTracker->setWorkingMode(cl_move_base_z::odom_tracker::WorkingMode::RECORD_PATH);
    }

    this->succeddedConnection_ = client_->onSucceeded(&WaypointNavigator::onGoalReached, this);
    client_->sendGoal(goal);
  }
  else
  {
    ROS_WARN("[WaypointsNavigator] All waypoints were consumed. There is no more waypoints available.");
  }
}

void WaypointNavigator::insertWaypoint(int index, geometry_msgs::Pose &newpose)
{
  if (index >= 0 && index <= waypoints_.size())
  {
    waypoints_.insert(waypoints_.begin(), index, newpose);
  }
}

void WaypointNavigator::setWaypoints(const std::vector<geometry_msgs::Pose> &waypoints)
{
  this->waypoints_ = waypoints;
}

void WaypointNavigator::setWaypoints(const std::vector<Pose2D> &waypoints)
{
  this->waypoints_.clear();
  for (auto &p : waypoints)
  {
    geometry_msgs::Pose pose;
    pose.position.x = p.x_;
    pose.position.y = p.y_;
    pose.position.z = 0.0;
    pose.orientation = tf::createQuaternionMsgFromYaw(p.yaw_);

    this->waypoints_.push_back(pose);
  }
}

void WaypointNavigator::removeWaypoint(int index)
{
  if (index >= 0 && index < waypoints_.size())
  {
    waypoints_.erase(waypoints_.begin() + index);
  }
}

const std::vector<geometry_msgs::Pose> &WaypointNavigator::getWaypoints() const
{
  return waypoints_;
}

long WaypointNavigator::getCurrentWaypointIndex() const
{
  return currentWaypoint_;
}

#define HAVE_NEW_YAMLCPP
void WaypointNavigator::loadWayPointsFromFile(std::string filepath)
{
  this->waypoints_.clear();
  std::ifstream ifs(filepath.c_str(), std::ifstream::in);
  if (ifs.good() == false)
  {
    throw std::string("Waypoints file not found");
  }

  try
  {

#ifdef HAVE_NEW_YAMLCPP
    YAML::Node node = YAML::Load(ifs);
#else
    YAML::Parser parser(ifs);
    parser.GetNextDocument(node);
#endif

#ifdef HAVE_NEW_YAMLCPP
    const YAML::Node &wp_node_tmp = node["waypoints"];
    const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
#else
    const YAML::Node *wp_node = node.FindValue("waypoints");
#endif

    if (wp_node != NULL)
    {
      for (uint64_t i = 0; i < wp_node->size(); ++i)
      {
        // Parse waypoint entries on YAML
        geometry_msgs::Pose wp;

        try
        {
          // (*wp_node)[i]["name"] >> wp.name;
          // (*wp_node)[i]["frame_id"] >> wp.header.frame_id;
          wp.position.x = (*wp_node)[i]["position"]["x"].as<double>();
          wp.position.y = (*wp_node)[i]["position"]["y"].as<double>();
          wp.position.z = (*wp_node)[i]["position"]["z"].as<double>();
          wp.orientation.x = (*wp_node)[i]["orientation"]["x"].as<double>();
          wp.orientation.y = (*wp_node)[i]["orientation"]["y"].as<double>();
          wp.orientation.z = (*wp_node)[i]["orientation"]["z"].as<double>();
          wp.orientation.w = (*wp_node)[i]["orientation"]["w"].as<double>();

          this->waypoints_.push_back(wp);
        }
        catch (...)
        {
          ROS_ERROR("parsing waypoint file, syntax error in point %d", i);
        }
      }
      ROS_INFO_STREAM("Parsed " << this->waypoints_.size() << " waypoints.");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't find any waypoints in the provided yaml file.");
    }
  }
  catch (const YAML::ParserException &ex)
  {
    ROS_ERROR_STREAM("Error loading the Waypoints YAML file. Incorrect syntax: " << ex.what());
  }
}
} // namespace cl_move_base_z

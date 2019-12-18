#include <smacc_navigation_plugin/move_base_action_client.h>
#include <smacc_navigation_plugin/waypoints_navigator_component.h>

#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace smacc
{
WaypointNavigator::WaypointNavigator()
    : currentWaypoint(0)
{
}

void WaypointNavigator::onGoalReached(SmaccMoveBaseActionClient::ResultConstPtr &res)
{
  waypointsEventDispatcher.postWaypointEvent(currentWaypoint);
  currentWaypoint++;
  auto &next = waypoints[currentWaypoint];

  smacc::SmaccMoveBaseActionClient::Goal goal;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = next;

  client_->sendGoal(goal);
}

void WaypointNavigator::insertWaypoint(int index, geometry_msgs::Pose &newpose)
{
  if (index >= 0 && index <= waypoints.size())
  {
    waypoints.insert(waypoints.begin(), index, newpose);
  }
}

void WaypointNavigator::removeWaypoint(int index)
{
  if (index >= 0 && index < waypoints.size())
  {
    waypoints.erase(waypoints.begin() + index);
  }
}

void WaypointNavigator::loadWayPointsFromFile(std::string filepath)
{
//   std::ifstream ifs(filepath.c_str(), std::ifstream::in);
//   if (ifs.good() == false)
//   {
//     throw std::string("Waypoints file not found");
//   }

// #ifdef HAVE_NEW_YAMLCPP
//   node = YAML::Load(ifs);
// #else
//   YAML::Parser parser(ifs);
//   parser.GetNextDocument(node);
// #endif

// #ifdef HAVE_NEW_YAMLCPP
//   const YAML::Node &wp_node_tmp = node["waypoints"];
//   const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
// #else
//   const YAML::Node *wp_node = node.FindValue("waypoints");
// #endif

//   if (wp_node != NULL)
//   {
//     for (unsigned int i = 0; i < wp_node->size(); ++i)
//     {
//       // Parse waypoint entries on YAML
//       yocs_msgs::Waypoint wp;

//       (*wp_node)[i]["name"] >> wp.name;
//       (*wp_node)[i]["frame_id"] >> wp.header.frame_id;
//       (*wp_node)[i]["pose"]["position"]["x"] >> wp.pose.position.x;
//       (*wp_node)[i]["pose"]["position"]["y"] >> wp.pose.position.y;
//       (*wp_node)[i]["pose"]["position"]["z"] >> wp.pose.position.z;
//       (*wp_node)[i]["pose"]["orientation"]["x"] >> wp.pose.orientation.x;
//       (*wp_node)[i]["pose"]["orientation"]["y"] >> wp.pose.orientation.y;
//       (*wp_node)[i]["pose"]["orientation"]["z"] >> wp.pose.orientation.z;
//       (*wp_node)[i]["pose"]["orientation"]["w"] >> wp.pose.orientation.w;

//       wps.waypoints.push_back(wp);
//     }
//     ROS_INFO_STREAM("Parsed " << wps.waypoints.size() << " waypoints.");
//   }
//   else
//   {
//     ROS_WARN_STREAM("Couldn't find any waypoints in the provided yaml file.");
//   }
}
} // namespace smacc
/*

  void getYamlNode(const std::string& filename, YAML::Node& node)
  {
    std::ifstream ifs(filename.c_str(), std::ifstream::in);
    if (ifs.good() == false)
    {
      throw std::string("Waypoints file not found");
    }

    #ifdef HAVE_NEW_YAMLCPP
      node = YAML::Load(ifs);
    #else
      YAML::Parser parser(ifs);
      parser.GetNextDocument(node);
    #endif
  }

  void parseWaypoints(const YAML::Node& node, yocs_msgs::WaypointList& wps)
  {
    #ifdef HAVE_NEW_YAMLCPP
      const YAML::Node& wp_node_tmp = node["waypoints"];
      const YAML::Node* wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
    #else
      const YAML::Node* wp_node = node.FindValue("waypoints");
    #endif

    if(wp_node != NULL)
    {
      for(unsigned int i = 0; i < wp_node->size(); ++i)
      {
        // Parse waypoint entries on YAML
        yocs_msgs::Waypoint wp;

        (*wp_node)[i]["name"] >> wp.name;
        (*wp_node)[i]["frame_id"] >> wp.header.frame_id;
        (*wp_node)[i]["pose"]["position"]["x"] >> wp.pose.position.x;
        (*wp_node)[i]["pose"]["position"]["y"] >> wp.pose.position.y;
        (*wp_node)[i]["pose"]["position"]["z"] >> wp.pose.position.z;
        (*wp_node)[i]["pose"]["orientation"]["x"] >> wp.pose.orientation.x;
        (*wp_node)[i]["pose"]["orientation"]["y"] >> wp.pose.orientation.y;
        (*wp_node)[i]["pose"]["orientation"]["z"] >> wp.pose.orientation.z;
        (*wp_node)[i]["pose"]["orientation"]["w"] >> wp.pose.orientation.w;

        wps.waypoints.push_back(wp);
      }
      ROS_INFO_STREAM("Parsed " << wps.waypoints.size() << " waypoints.");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't find any waypoints in the provided yaml file.");
    }
  }
  */
//} // namespace smacc
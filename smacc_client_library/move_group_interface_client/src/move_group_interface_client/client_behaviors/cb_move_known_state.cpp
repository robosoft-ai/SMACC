/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_known_state.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/package.h>
#include <experimental/filesystem>

namespace move_group_interface_client
{
CbMoveKnownState::CbMoveKnownState(std::string pkg, std::string config_path)
  : CbMoveJoints(loadJointStatesFromFile(pkg, config_path))
{
}

#define HAVE_NEW_YAMLCPP
std::map<std::string, double> CbMoveKnownState::loadJointStatesFromFile(std::string pkg, std::string filepath)
{
  auto pkgpath = ros::package::getPath(pkg);
  std::map<std::string, double> jointStates;

  if(pkgpath == "")
  {
    ROS_ERROR_STREAM("[CbMoveKnownState] package not found for the known poses file: " << pkg << std::endl << " [IGNORING BEHAVIOR]");
    return jointStates;
  }

  filepath =  pkgpath +"/" + filepath;
  

  ROS_INFO("[CbMoveKnownState] Opening file with joint known state: %s",  filepath.c_str());


  if(std::experimental::filesystem::exists(filepath))
  {
    ROS_INFO_STREAM("[CbMoveKnownState] known state file exists: " << filepath);
  }
  else
  {
    ROS_ERROR_STREAM("[CbMoveKnownState] known state file does not exists: " << filepath);
  }

  std::ifstream ifs(filepath.c_str(), std::ifstream::in);
  if (ifs.good() == false)
  {
    ROS_ERROR("[CbMoveKnownState] Error opening file with joint known states: %s",  filepath.c_str());
    throw std::string("joint state files not found");
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
    const YAML::Node &wp_node_tmp = node["joint_states"];
    const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
#else
    const YAML::Node *wp_node = node.FindValue("waypoints");
#endif

    if (wp_node != NULL)
    {
      try
      {
        for(YAML::const_iterator it=wp_node->begin();it != wp_node->end();++it) 
        {
          std::string key = it->first.as<std::string>(); 
          double value = it->second.as<double>(); 
          ROS_DEBUG_STREAM(" joint - " << key << ": " << value);
          jointStates[key] = value;
        }

        return jointStates;
      }
      catch(std::exception& ex)
      {
        ROS_ERROR("trying to convert to map, failed, errormsg: %s", ex.what());
      }

      ROS_INFO_STREAM("Parsed " << jointStates.size() << " joint entries.");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't find any jointStates in the provided yaml file.");
    }
  }
  catch (const YAML::ParserException &ex)
  {
    ROS_ERROR_STREAM("Error loading the Waypoints YAML file. Incorrect syntax: " << ex.what());
  }
}
}  // namespace move_group_interface_client
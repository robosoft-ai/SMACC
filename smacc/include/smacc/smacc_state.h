/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include "smacc/common.h"
#include "smacc/string_type_walker.h"

namespace smacc
{
class ISmaccState
{
public:
  ros::NodeHandle nh;

  // Delegates to ROS param access with the current NodeHandle
  template <typename T>
  bool getParam(std::string param_name, T &param_storage);

  // Delegates to ROS param access with the current NodeHandle
  template <typename T>
  void setParam(std::string param_name, T param_val);

  //Delegates to ROS param access with the current NodeHandle
  template <typename T>
  bool param(std::string param_name, T &param_val, const T &default_val) const;

  virtual ISmaccStateMachine &getStateMachine() = 0;

  virtual std::string getClassName();

  template <typename TOrthogonal>
  void configure(std::shared_ptr<SmaccSubStateBehavior> smaccBehavior);

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *&storage);

  template <typename T>
  bool getGlobalSMData(std::string name, T &ret);

  // Store globally in this state machine. (By value parameter )
  template <typename T>
  void setGlobalSMData(std::string name, T value);

  template <typename TLUnit, typename TTriggerEvent, typename... TEvArgs>
  std::shared_ptr<TLUnit> createLogicUnit();

  template <typename EventType>
  void postEvent(const EventType &ev);

  template <typename TransitionType>
  void notifyTransition();

  void notifyTransitionFromTransitionTypeInfo(std::shared_ptr<smacc::TypeInfo>& transitionTypeInfo);
  
  std::vector<std::shared_ptr<LogicUnit>> logicUnits_;
};

//-------------------------------------------------------------------------------------------------------------------

} // namespace smacc

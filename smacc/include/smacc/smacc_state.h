/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include "smacc/smacc_state_machine.h"
#include "smacc/impl/smacc_state_machine_impl.h"
#include "smacc/orthogonal.h"
#include "smacc/smacc_substate_behavior.h"
#include "smacc/logic_units/logic_unit_base.h"

namespace smacc
{
class ISmaccState
{
public:
  ros::NodeHandle nh;

  // Delegates to ROS param access with the current NodeHandle
  template <typename T>
  bool getParam(std::string param_name, T &param_storage)
  {
    return nh.getParam(param_name, param_storage);
  }

  // Delegates to ROS param access with the current NodeHandle
  template <typename T>
  void setParam(std::string param_name, T param_val)
  {
    return nh.setParam(param_name, param_val);
  }

  //Delegates to ROS param access with the current NodeHandle
  template <typename T>
  bool param(std::string param_name, T &param_val, const T &default_val) const
  {
    return nh.param(param_name, param_val, default_val);
  }

  virtual ISmaccStateMachine &getStateMachine() = 0;

  virtual std::string getClassName()
  {
    return demangleSymbol(typeid(*this).name());
  }

  template <typename TOrthogonal>
  void configure(std::shared_ptr<SmaccSubStateBehavior> smaccBehavior)
  {
    std::string orthogonalkey = demangledTypeName<TOrthogonal>();
    ROS_INFO("Configuring orthogonal: %s", orthogonalkey.c_str());
    
    std::shared_ptr<TOrthogonal> orthogonal;
    if(this->getStateMachine().getOrthogonal<TOrthogonal>(orthogonal))
    {
      orthogonal->setStateBehavior(smaccBehavior);
    }
    else
    {
      ROS_ERROR("Skipping substate behavior creation. Orthogonal did not exist.");
    }
  }

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *&storage)
  {
    this->getStateMachine().requiresComponent(storage);
  }

  template <typename T>
  bool getGlobalSMData(std::string name, T &ret)
  {
    return this->getStateMachine().getGlobalSMData(name, ret);
  }

  // Store globally in this state machine. (By value parameter )
  template <typename T>
  void setGlobalSMData(std::string name, T value)
  {
    this->getStateMachine().setGlobalSMData(name, value);
  }

  template <typename TLUnit, typename TTriggerEvent, typename ...TEvArgs>
  std::shared_ptr<TLUnit> createLogicUnit()
  {
    auto lu = std::make_shared<TLUnit>();
    lu->initialize(this,typelist<TEvArgs...>());
    lu->declarePostEvent(typelist<TTriggerEvent>());
    logicUnits_.push_back(lu);

    return lu;
  }

  std::vector<std::shared_ptr<LogicUnit>> logicUnits_;
};

template<typename TEv>
void LogicUnit::declarePostEvent(typelist<TEv>)
{
    this->postEventFn = [this]()
    {
        ROS_INFO_STREAM("[Logic Unit Base] postingfn posting event: " << demangleSymbol<TEv>());
        auto* ev = new TEv();
        this->ownerState->getStateMachine().postEvent(ev);
    };
}

//-------------------------------------------------------------------------------------------------------------------

} // namespace smacc

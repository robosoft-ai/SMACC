/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <smacc/common.h>

namespace smacc
{
  class ISmaccState
  {
  public:
    virtual ISmaccStateMachine &getStateMachine() = 0;

    inline ISmaccState *getParentState() { return parentState_; }

    inline ros::NodeHandle &getROSNode() { return nh; }

    virtual std::string getClassName();

    template <typename TOrthogonal, typename TBehavior, typename... Args>
    std::shared_ptr<TBehavior> configure(Args &&... args);

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage);

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage);

    template <typename T>
    bool getGlobalSMData(std::string name, T &ret);

    // Store globally in this state machine. (By value parameter )
    template <typename T>
    void setGlobalSMData(std::string name, T value);

    template <typename TStateReactor, typename TTriggerEvent, typename TEventList, typename... TEvArgs>
    std::shared_ptr<TStateReactor> createStateReactor(TEvArgs... args);

    template <typename TStateReactor, typename... TEvArgs>
    std::shared_ptr<TStateReactor> createStateReactor(TEvArgs... args);

    template <typename TEventGenerator, typename... TEvArgs>
    std::shared_ptr<TEventGenerator> createEventGenerator(TEvArgs... args);

    template <typename EventType>
    void postEvent(const EventType &ev);

    template <typename EventType>
    void postEvent();

    // used for transition logging
    template <typename TransitionType>
    void notifyTransition();

    // used for transition logging
    void notifyTransitionFromTransitionTypeInfo(std::shared_ptr<smacc::introspection::TypeInfo> &transitionTypeInfo);

    inline std::vector<std::shared_ptr<StateReactor>> &getStateReactors() { return stateReactors_; }

    inline std::vector<std::shared_ptr<SmaccEventGenerator>> &getEventGenerators() { return eventGenerators_; }

    // Delegates to ROS param access with the current NodeHandle
    template <typename T>
    bool getParam(std::string param_name, T &param_storage);

    // Delegates to ROS param access with the current NodeHandle
    template <typename T>
    void setParam(std::string param_name, T param_val);

    //Delegates to ROS param access with the current NodeHandle
    template <typename T>
    bool param(std::string param_name, T &param_val, const T &default_val) const;

    template <typename TOrthogonal>
    TOrthogonal *getOrthogonal();

    template <typename TEventGenerator>
    TEventGenerator* getEventGenerator();

    template <typename TStateReactor>
    TStateReactor* getStateReactor();

  protected:
    std::vector<std::shared_ptr<StateReactor>> stateReactors_;
    std::vector<std::shared_ptr<smacc::SmaccEventGenerator>> eventGenerators_;

    ros::NodeHandle nh;

    ros::NodeHandle contextNh;

    ISmaccState *parentState_;

    const smacc::introspection::SmaccStateInfo *stateInfo_;
  };
} // namespace smacc

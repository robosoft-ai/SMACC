/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc_state_machine.h>

#include <smacc/smacc_client.h>
#include <smacc/smacc_orthogonal.h>
#include <smacc/smacc_state.h>

#include <smacc/introspection/introspection.h>
#include <smacc/smacc_signal_detector.h>
#include <smacc/smacc_state_reactor.h>
#include <smacc_msgs/SmaccStatus.h>
#include <sstream>

#include <boost/function_types/function_arity.hpp>
#include <boost/function_types/function_type.hpp>
#include <boost/function_types/parameter_types.hpp>

namespace smacc
{
  using namespace smacc::introspection;

  template <typename TOrthogonal>
  TOrthogonal *ISmaccStateMachine::getOrthogonal()
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);

    std::string orthogonalkey = demangledTypeName<TOrthogonal>();
    TOrthogonal *ret;

    auto it = orthogonals_.find(orthogonalkey);

    if (it != orthogonals_.end())
    {
      ROS_DEBUG("Orthogonal %s resource is being required from some state, client or component. Found resource in cache.", orthogonalkey.c_str());
      ret = dynamic_cast<TOrthogonal *>(it->second.get());
      return ret;
    }
    else
    {
      std::stringstream ss;
      ss << "Orthogonal not found " << orthogonalkey.c_str() << std::endl;
      ss << "The existing orthogonals are the following: " << std::endl;
      for (auto &orthogonal : orthogonals_)
      {
        ss << " - " << orthogonal.first << std::endl;
      }

      ROS_WARN_STREAM(ss.str());

      return nullptr;
    }
  }

  //-------------------------------------------------------------------------------------------------------
  template <typename TOrthogonal>
  void ISmaccStateMachine::createOrthogonal()
  {
    this->lockStateMachine("create orthogonal");
    std::string orthogonalkey = demangledTypeName<TOrthogonal>();

    if (orthogonals_.count(orthogonalkey) == 0)
    {
      auto ret = std::make_shared<TOrthogonal>();
      orthogonals_[orthogonalkey] = dynamic_pointer_cast<smacc::ISmaccOrthogonal>(ret);

      ret->setStateMachine(this);

      ROS_INFO("%s Orthogonal is created", orthogonalkey.c_str());
    }
    else
    {
      ROS_WARN_STREAM("There were already one existing orthogonal of type "
                      << orthogonalkey.c_str() << ". Skipping creation orthogonal request. ");
      std::stringstream ss;
      ss << "The existing orthogonals are the following: " << std::endl;
      for (auto &orthogonal : orthogonals_)
      {
        ss << " - " << orthogonal.first << std::endl;
      }
      ROS_WARN_STREAM(ss.str());
    }
    this->unlockStateMachine("create orthogonal");
  }

  //-------------------------------------------------------------------------------------------------------
  template <typename SmaccComponentType>
  void ISmaccStateMachine::requiresComponent(SmaccComponentType *&storage)
  {
    ROS_DEBUG("component %s is required", demangleSymbol(typeid(SmaccComponentType).name()).c_str());
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);

    for (auto ortho : this->orthogonals_)
    {
      for (auto &client : ortho.second->clients_)
      {

        storage = client->getComponent<SmaccComponentType>();
        if (storage != nullptr)
        {
          return;
        }
      }
    }

    ROS_WARN("component %s is required but it was not found in any orthogonal", demangleSymbol(typeid(SmaccComponentType).name()).c_str());

    // std::string componentkey = demangledTypeName<SmaccComponentType>();
    // SmaccComponentType *ret;

    // auto it = components_.find(componentkey);

    // if (it == components_.end())
    // {
    //     ROS_DEBUG("%s smacc component is required. Creating a new instance.", componentkey.c_str());

    //     ret = new SmaccComponentType();
    //     ret->setStateMachine(this);
    //     components_[componentkey] = static_cast<smacc::ISmaccComponent *>(ret);
    //     ROS_DEBUG("%s resource is required. Done.", componentkey.c_str());
    // }
    // else
    // {
    //     ROS_DEBUG("%s resource is required. Found resource in cache.", componentkey.c_str());
    //     ret = dynamic_cast<SmaccComponentType *>(it->second);
    // }

    // storage = ret;
  }
  //-------------------------------------------------------------------------------------------------------
  template <typename EventType>
  void ISmaccStateMachine::postEvent(EventType *ev, EventLifeTime evlifetime)
  {
    std::lock_guard<std::recursive_mutex> guard(eventQueueMutex_);
    if (evlifetime == EventLifeTime::CURRENT_STATE && (stateMachineCurrentAction == StateMachineInternalAction::STATE_EXITING ||
                                                       stateMachineCurrentAction == StateMachineInternalAction::TRANSITIONING))
    {
      ROS_WARN_STREAM("CURRENT STATE SCOPED EVENT SKIPPED, state is exiting/transitioning " << demangleSymbol<EventType>());
      return;
      // in this case we may lose/skip events, if this is not right for some cases we should create a queue
      // to lock the events during the transitions. This issues appeared when a client asyncbehavior was posting an event
      // meanwhile we were doing the transition, but the main thread was waiting for its correct finalization (with thread.join)
    }

    // when a postting event is requested by any component, client, or client behavior
    // we reach this place. Now, we propagate the events to all the state state reactors to generate
    // some more events

    ROS_DEBUG_STREAM("[PostEvent entry point] " << demangleSymbol<EventType>());

    for (auto currentState : currentState_)
    {
      propagateEventToStateReactors(currentState, ev);
    }

    this->signalDetector_->postEvent(ev);
  }

  template <typename EventType>
  void ISmaccStateMachine::postEvent(EventLifeTime evlifetime)
  {
    auto *ev = new EventType();
    this->postEvent(ev, evlifetime);
  }

  template <typename T>
  bool ISmaccStateMachine::getGlobalSMData(std::string name, T &ret)
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);
    // ROS_WARN("get SM Data lock acquire");
    bool success = false;

    if (!globalData_.count(name))
    {
      // ROS_WARN("get SM Data - data do not exist");
      success = false;
    }
    else
    {
      // ROS_WARN("get SM DAta -data exist. accessing");
      try
      {
        auto &v = globalData_[name];

        // ROS_WARN("get SM DAta -data exist. any cast");
        ret = boost::any_cast<T>(v.second);
        success = true;
        // ROS_WARN("get SM DAta -data exist. success");
      }
      catch (boost::bad_any_cast &ex)
      {
        ROS_ERROR("bad any cast: %s", ex.what());
        success = false;
      }
    }

    // ROS_WARN("get SM Data lock release");
    return success;
  }

  template <typename T>
  void ISmaccStateMachine::setGlobalSMData(std::string name, T value)
  {
    {
      std::lock_guard<std::recursive_mutex> lock(m_mutex_);
      // ROS_WARN("set SM Data lock acquire");

      globalData_[name] = {[this, name]() {
                             std::stringstream ss;
                             auto val = any_cast<T>(globalData_[name].second);
                             ss << val;
                             return ss.str();
                           },
                           value};
    }

    this->updateStatusMessage();
  }

  template <typename StateField, typename BehaviorType>
  void ISmaccStateMachine::mapBehavior()
  {
    std::string stateFieldName = demangleSymbol(typeid(StateField).name());
    std::string behaviorType = demangleSymbol(typeid(BehaviorType).name());
    ROS_INFO("Mapping state field '%s' to stateReactor '%s'", stateFieldName.c_str(), behaviorType.c_str());
    SmaccClientBehavior *globalreference;
    if (!this->getGlobalSMData(stateFieldName, globalreference))
    {
      // Using the requires component approach, we force a unique existence
      // of this component
      BehaviorType *behavior;
      this->requiresComponent(behavior);
      globalreference = dynamic_cast<ISmaccClientBehavior *>(behavior);

      this->setGlobalSMData(stateFieldName, globalreference);
    }
  }

  namespace utils
  {
    template <int arity>
    struct Bind
    {
      template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
      boost::signals2::connection bindaux(TSmaccSignal &signal, TMemberFunctionPrototype callback,
                                          TSmaccObjectType *object, std::shared_ptr<CallbackCounterSemaphore> callbackCounter);
    };

    template <>
    struct Bind<1>
    {
      template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
      boost::signals2::connection bindaux(TSmaccSignal &signal, TMemberFunctionPrototype callback, TSmaccObjectType *object, std::shared_ptr<CallbackCounterSemaphore> callbackCounter)
      {
        return signal.connect(

        [=]()
        {
          if(callbackCounter == nullptr)
          {
           (object->*callback)();
          }
          else if (callbackCounter->acquire())
          {
            (object->*callback)();
            callbackCounter->release();
        }});
      }
    };

    template <>
    struct Bind<2>
    {
      template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
      boost::signals2::connection bindaux(TSmaccSignal &signal, TMemberFunctionPrototype callback, TSmaccObjectType *object, std::shared_ptr<CallbackCounterSemaphore> callbackCounter)
      {
        return signal.connect([=](auto a1)
        {
            if(callbackCounter == nullptr)
          {
            (object->*callback)(a1);
          }
          else if (callbackCounter->acquire())
          {
            (object->*callback)(a1);
            callbackCounter->release();

          }
            });
      }
    };

    template <>
    struct Bind<3>
    {
      template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
      boost::signals2::connection bindaux(TSmaccSignal &signal, TMemberFunctionPrototype callback, TSmaccObjectType *object, std::shared_ptr<CallbackCounterSemaphore> callbackCounter)
      {
        return signal.connect([=](auto a1, auto a2) {
           if(callbackCounter == nullptr)
          {
            (object->*callback)(a1,a2);
          }
          else if (callbackCounter->acquire())
          {
            (object->*callback)(a1,a2);
            callbackCounter->release();

          }
          });
      }
    };

    template <>
    struct Bind<4>
    {
      template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
      boost::signals2::connection bindaux(TSmaccSignal &signal, TMemberFunctionPrototype callback, TSmaccObjectType *object, std::shared_ptr<CallbackCounterSemaphore> callbackCounter)
      {
        return signal.connect([=](auto a1, auto a2, auto a3) {
           if(callbackCounter == nullptr)
          {
            (object->*callback)(a1,a2,a3);
          }
          else if (callbackCounter->acquire())
          {
            (object->*callback)(a1,a2,a3);
            callbackCounter->release();

          }
          });

      }
    };
  } // namespace utils
  using namespace smacc::utils;

  template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
  boost::signals2::connection ISmaccStateMachine::createSignalConnection(TSmaccSignal &signal,
                                                                         TMemberFunctionPrototype callback,
                                                                         TSmaccObjectType *object)
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);

    static_assert(std::is_base_of<ISmaccState, TSmaccObjectType>::value ||
                      std::is_base_of<ISmaccClient, TSmaccObjectType>::value ||
                      std::is_base_of<ISmaccClientBehavior, TSmaccObjectType>::value ||
                      std::is_base_of<StateReactor, TSmaccObjectType>::value ||
                      std::is_base_of<ISmaccComponent, TSmaccObjectType>::value,
                  "Only are accepted smacc types as subscribers for smacc signals");

    typedef decltype(callback) ft;
    Bind<boost::function_types::function_arity<ft>::value> binder;

    boost::signals2::connection connection;

    // long life-time objects
    if (std::is_base_of<ISmaccComponent, TSmaccObjectType>::value ||
        std::is_base_of<ISmaccClient, TSmaccObjectType>::value ||
        std::is_base_of<ISmaccOrthogonal, TSmaccObjectType>::value ||
        std::is_base_of<ISmaccStateMachine, TSmaccObjectType>::value)
    {
      connection = binder.bindaux(signal, callback, object, nullptr);
    }
    else if (std::is_base_of<ISmaccState, TSmaccObjectType>::value ||
             std::is_base_of<StateReactor, TSmaccObjectType>::value ||
             std::is_base_of<ISmaccClientBehavior, TSmaccObjectType>::value)
    {
      ROS_INFO("[StateMachine] life-time constrained smacc signal subscription created. Subscriber is %s",
               demangledTypeName<TSmaccObjectType>().c_str());

      std::shared_ptr<CallbackCounterSemaphore> callbackCounterSemaphore;
      if(stateCallbackConnections.count(object))
      {
        callbackCounterSemaphore = stateCallbackConnections[object];
      }
      else
      {
          callbackCounterSemaphore =  std::make_shared<CallbackCounterSemaphore>(demangledTypeName<TSmaccObjectType>().c_str());
          stateCallbackConnections[object] = callbackCounterSemaphore;
      }

      connection = binder.bindaux(signal, callback, object, callbackCounterSemaphore);
      callbackCounterSemaphore->addConnection(connection);

    }
    else // state life-time objects
    {
      ROS_WARN("[StateMachine] connecting signal to an unknown object with life-time unknown behavior. It might provoke "
               "an exception if the object is destroyed during the execution.");

      connection = binder.bindaux(signal, callback, object, nullptr);
    }

    return connection;
  }

  template <typename T>
  bool ISmaccStateMachine::getParam(std::string param_name, T &param_storage)
  {
    return nh_.getParam(param_name, param_storage);
  }

  // Delegates to ROS param access with the current NodeHandle
  template <typename T>
  void ISmaccStateMachine::setParam(std::string param_name, T param_val)
  {
    return nh_.setParam(param_name, param_val);
  }

  // Delegates to ROS param access with the current NodeHandle
  template <typename T>
  bool ISmaccStateMachine::param(std::string param_name, T &param_val, const T &default_val) const
  {
    return nh_.param(param_name, param_val, default_val);
  }

  template <typename StateType>
  void ISmaccStateMachine::notifyOnStateEntryStart(StateType *state)
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);

    ROS_DEBUG("[State Machne] Initializating a new state '%s' and updating current state. Getting state meta-information. number of orthogonals: %ld", demangleSymbol(typeid(StateType).name()).c_str(), this->orthogonals_.size());

    stateSeqCounter_++;
    currentState_.push_back(state);
    currentStateInfo_ = stateMachineInfo_->getState<StateType>();
  }

  template <typename StateType>
  void ISmaccStateMachine::notifyOnStateEntryEnd(StateType *state)
  {
    ROS_INFO("[%s] State OnEntry code finished", demangleSymbol(typeid(StateType).name()).c_str());

    auto currentState = this->currentState_.back();
    for (auto pair : this->orthogonals_)
    {
      //ROS_INFO("ortho onentry: %s", pair.second->getName().c_str());
      auto &orthogonal = pair.second;
      try
      {
        orthogonal->onEntry();
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[Orthogonal %s] Exception on Entry - continuing with next orthogonal. Exception info: %s",
                  pair.second->getName().c_str(), e.what());
      }
    }

    for (auto &sr : currentState->getStateReactors())
    {
      auto srname = smacc::demangleSymbol(typeid(*sr).name()).c_str();
      ROS_INFO("state reactor onEntry: %s", srname);
      try
      {
        sr->onEntry();
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[State Reactor %s] Exception on Entry - continuing with next state reactor. Exception info: %s",
                  srname, e.what());
      }
    }

    for (auto &eg : currentState->getEventGenerators())
    {
      auto egname = smacc::demangleSymbol(typeid(*eg).name()).c_str();
      ROS_INFO("state reactor onEntry: %s", egname);
      try
      {
        eg->onEntry();
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[Event generator %s] Exception on Entry - continuing with next state reactor. Exception info: %s",
                  egname, e.what());
      }
    }

    {
      std::lock_guard<std::recursive_mutex> lock(m_mutex_);
      this->updateStatusMessage();
      stateMachineCurrentAction = StateMachineInternalAction::STATE_STEADY;
    }
  }

  template <typename StateType>
  void ISmaccStateMachine::notifyOnRuntimeConfigurationFinished(StateType *state)
  {
    for (auto pair : this->orthogonals_)
    {
      //ROS_INFO("ortho onruntime configure: %s", pair.second->getName().c_str());
      auto &orthogonal = pair.second;
      orthogonal->runtimeConfigure();
    }

    {
      std::lock_guard<std::recursive_mutex> lock(m_mutex_);
      this->updateStatusMessage();
      this->signalDetector_->notifyStateConfigured(this->currentState_.back());

      stateMachineCurrentAction = StateMachineInternalAction::STATE_ENTERING;
    }
  }

  template <typename StateType>
  void ISmaccStateMachine::notifyOnRuntimeConfigured(StateType *state)
  {
    stateMachineCurrentAction = StateMachineInternalAction::STATE_CONFIGURING;
  }

  template <typename StateType>
  void ISmaccStateMachine::notifyOnStateExitting(StateType *state)
  {
    stateMachineCurrentAction = StateMachineInternalAction::STATE_EXITING;

    auto fullname = demangleSymbol(typeid(StateType).name());
    ROS_WARN_STREAM("exiting state: " << fullname);
    //this->setParam("destroyed", true);

    ROS_INFO_STREAM("Notification State Exit: leaving state" << state);
    for (auto pair : this->orthogonals_)
    {
      auto &orthogonal = pair.second;
      try
      {
        orthogonal->onExit();
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[Orthogonal %s] Exception onExit - continuing with next orthogonal. Exception info: %s",
                  pair.second->getName().c_str(), e.what());
      }
    }

    for (auto &sr : state->getStateReactors())
    {
      auto srname = smacc::demangleSymbol(typeid(*sr).name()).c_str();
      ROS_INFO("state reactor OnExit: %s", srname);
      try
      {
        sr->onExit();
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[State Reactor %s] Exception on OnExit - continuing with next state reactor. Exception info: %s",
                  srname, e.what());
      }
    }

    for (auto &eg : state->getEventGenerators())
    {
      auto egname = smacc::demangleSymbol(typeid(*eg).name()).c_str();
      ROS_INFO("state reactor OnExit: %s", egname);
      try
      {
        eg->onExit();
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[State Reactor %s] Exception on OnExit - continuing with next state reactor. Exception info: %s",
                  egname, e.what());
      }
    }
  }

  template <typename StateType>
  void ISmaccStateMachine::notifyOnStateExited(StateType *state)
  {
    this->lockStateMachine("state exit");

    signalDetector_->notifyStateExited(state);

    auto fullname = demangleSymbol(typeid(StateType).name());
    ROS_WARN_STREAM("exiting state: " << fullname);

    ROS_INFO_STREAM("Notification State Disposing: leaving state" << state);
    for (auto pair : this->orthogonals_)
    {
      auto &orthogonal = pair.second;
      try
      {
        orthogonal->onDispose();
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[Orthogonal %s] Exception onDispose - continuing with next orthogonal. Exception info: %s",
                  pair.second->getName().c_str(), e.what());
      }
    }

    for (auto &sr : state->getStateReactors())
    {
      auto srname = smacc::demangleSymbol(typeid(*sr).name()).c_str();
      ROS_INFO("state reactor disposing: %s", srname);
      try
      {
        this->disconnectSmaccSignalObject(sr.get());
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[State Reactor %s] Exception on OnDispose - continuing with next state reactor. Exception info: %s",
                  srname, e.what());
      }
    }

    for (auto &eg : state->getEventGenerators())
    {
      auto egname = smacc::demangleSymbol(typeid(*eg).name()).c_str();
      ROS_INFO("state reactor disposing: %s", egname);
      try
      {
        this->disconnectSmaccSignalObject(eg.get());
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("[State Reactor %s] Exception on OnDispose - continuing with next state reactor. Exception info: %s",
                  egname, e.what());
      }
    }

    this->stateCallbackConnections.clear();
    currentState_.pop_back();

    // then call exit state
    ROS_WARN_STREAM("state exit: " << fullname);

    stateMachineCurrentAction = StateMachineInternalAction::TRANSITIONING;
    this->unlockStateMachine("state exit");
  }
  //-------------------------------------------------------------------------------------------------------
  template <typename EventType>
  void ISmaccStateMachine::propagateEventToStateReactors(ISmaccState *st, EventType *ev)
  {
    ROS_DEBUG("PROPAGATING EVENT [%s] TO LUs [%s]: ", demangleSymbol<EventType>().c_str(), st->getClassName().c_str());
    for (auto &sb : st->getStateReactors())
    {
      sb->notifyEvent(ev);
    }

    auto *pst = st->getParentState();
    if (pst != nullptr)
    {
      propagateEventToStateReactors(pst, ev);
    }
  }

  template <typename InitialStateType>
  void ISmaccStateMachine::buildStateMachineInfo()
  {
    this->stateMachineInfo_ = std::make_shared<SmaccStateMachineInfo>();
    this->stateMachineInfo_->buildStateMachineInfo<InitialStateType>();
    this->stateMachineInfo_->assembleSMStructureMessage(this);
    this->checkStateMachineConsistence();
  }

  uint64_t ISmaccStateMachine::getCurrentStateCounter() const
  {
    return this->stateSeqCounter_;
  }

  ISmaccState *ISmaccStateMachine::getCurrentState() const
  {
    return this->currentState_.back();
  }

  const smacc::introspection::SmaccStateMachineInfo &ISmaccStateMachine::getStateMachineInfo()
  {
    return *this->stateMachineInfo_;
  }

} // namespace smacc

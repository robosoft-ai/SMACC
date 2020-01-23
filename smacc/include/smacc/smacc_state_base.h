/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/smacc_state.h>
#include <smacc/smacc_state_reactor.h>

namespace smacc
{
using namespace smacc::introspection;
using namespace smacc::default_events;

template <class MostDerived,
          class Context,
          class InnerInitial = mpl::list<>,
          sc::history_mode historyMode = sc::has_deep_history>
class SmaccState : public sc::simple_state<
                       MostDerived, Context, InnerInitial, historyMode>,
                   public ISmaccState
{
  typedef sc::simple_state<MostDerived, Context, InnerInitial, historyMode>
      base_type;

public:
  typedef Context TContext;

  bool finishStateThrown;

  //////////////////////////////////////////////////////////////////////////
  struct my_context
  {
    my_context(typename base_type::context_ptr_type pContext) : pContext_(pContext)
    {
    }

    typename base_type::context_ptr_type pContext_;
  };

  SmaccState() = delete;

  virtual ISmaccState *getParentState()
  {
    //auto* ctx = dynamic_cast<ISmaccState*>(this->template context<Context *>());

    return parentState_;
  }

  // Constructor that initializes the state ros node handle
  SmaccState(my_context ctx)
  {
    ROS_WARN_STREAM("creatingState state: " << demangleSymbol(typeid(MostDerived).name()).c_str());
    this->set_context(ctx.pContext_);

    this->stateInfo_ = getStateInfo();

    // storing a reference to the parent state
    auto &ps = this->template context<Context>();
    parentState_ = dynamic_cast<ISmaccState *>(&ps);
    finishStateThrown = false;

    this->getStateMachine().notifyOnStateEntryStart(static_cast<MostDerived *>(this));

    ros::NodeHandle contextNh = optionalNodeHandle(ctx.pContext_);
    ROS_DEBUG("context node handle namespace: %s", contextNh.getNamespace().c_str());
    if (contextNh.getNamespace() == "/")
    {
      auto nhname = smacc::utils::cleanShortTypeName(typeid(Context));
      ROS_INFO("Creating ros NodeHandle for this state: %s", nhname.c_str());
      contextNh = ros::NodeHandle(nhname);
    }

    std::string classname = smacc::utils::cleanShortTypeName(typeid(MostDerived));

    // TODO: make this static to build the parameter tree at startup
    this->nh = ros::NodeHandle(contextNh.getNamespace() + std::string("/") + classname);

    ROS_DEBUG("nodehandle namespace: %s", nh.getNamespace().c_str());

    this->setParam("created", true);

    // before dynamic runtimeConfiguration, we execute the staticConfigure behavior configurations
    {
      ROS_INFO("-- STATIC STATE DESCRIPTION --");

      for (const auto &stateReactorsVector : SmaccStateInfo::staticBehaviorInfo)
      {
        ROS_DEBUG_STREAM(" - state info: " << demangleSymbol(stateReactorsVector.first->name()));
        for (auto &bhinfo : stateReactorsVector.second)
        {
          ROS_DEBUG_STREAM(" - client behavior: " << demangleSymbol(bhinfo.behaviorType->name()));
        }
      }

      const std::type_info *tindex = &(typeid(MostDerived));
      auto &staticDefinedBehaviors = SmaccStateInfo::staticBehaviorInfo[tindex];
      auto &staticDefinedStateReactors = SmaccStateInfo::stateReactorsInfo[tindex];

      for (auto &bhinfo : staticDefinedBehaviors)
      {
        ROS_INFO_STREAM("- Creating static client behavior: " << demangleSymbol(bhinfo.behaviorType->name()));
        bhinfo.factoryFunction(this);
      }

      for (auto &sb : staticDefinedStateReactors)
      {
        ROS_INFO_STREAM("- Creating static state reactor: " << demangleSymbol(sb->stateReactorType->name()));
        sb->factoryFunction(this);
      }

      ROS_INFO("---- END STATIC DESCRIPTION");
    }

    ROS_INFO("State runtime configuration");

    // first we runtime configure the state, where we create client behaviors
    static_cast<MostDerived *>(this)->runtimeConfiguration();

    // then the orthogonals are internally configured
    this->getStateMachine().notifyOnRuntimeConfigured(static_cast<MostDerived *>(this));

    //ROS_INFO("Not behavioral State");
    static_cast<MostDerived *>(this)->onEntry();

    // here orthogonals and client behaviors are entered OnEntry
    this->getStateMachine().notifyOnStateEntryEnd(static_cast<MostDerived *>(this));
  }

  typedef typename Context::inner_context_type context_type;
  typedef typename context_type::state_iterator state_iterator;

  InnerInitial *smacc_inner_type;

  const smacc::introspection::SmaccStateInfo *getStateInfo()
  {
    auto smInfo = this->getStateMachine().getStateMachineInfo();

    auto key = typeid(MostDerived).name();
    if (smInfo.states.count(key))
    {
      return smInfo.states[key].get();
    }
    else
    {
      return nullptr;
    }
  }

  std::string getFullName()
  {
    return demangleSymbol(typeid(MostDerived).name());
  }

  std::string getShortName()
  {
    return smacc::utils::cleanShortTypeName(typeid(MostDerived));
  }

  void exit()
  {
    // this function is called by boot statechart
    try
    {
      this->requestLockStateMachine("state exit");
      auto fullname = demangleSymbol(typeid(MostDerived).name());
      ROS_WARN_STREAM("exiting state: " << fullname);
      this->setParam("destroyed", true);

      // first process orthogonals onexits
      this->getStateMachine().notifyOnStateExit(static_cast<MostDerived *>(this));

      // then call exit state
      ROS_WARN_STREAM("state exit: " << fullname);
      static_cast<MostDerived *>(this)->onExit();
    }
    catch (...)
    {
    }
    this->requestUnlockStateMachine("state exit");
  }

  virtual ~SmaccState()
  {
  }

  void throwFinishEvent()
  {
    if (!finishStateThrown)
    {
      auto *finishEvent = new EvStateFinish<MostDerived>();
      finishEvent->state = static_cast<MostDerived *>(this);
      this->postEvent(finishEvent);
      finishStateThrown = true;
    }
  }

public:
  // This method is static-polymorphic because of the curiously recurring template pattern. It
  // calls to the most derived class onEntry method if declared on smacc state construction
  void runtimeConfiguration()
  {
  }

  // This method is static-polymorphic because of the curiously recurring template pattern. It
  // calls to the most derived class onEntry method if declared on smacc state construction
  void onEntry()
  {
  }

  // this method is static-polimorphic because of the curiously recurreing pattern. It
  // calls to the most derived class onExit method if declared on smacc state destruction
  void onExit()
  {
  }

  template <typename T>
  bool getGlobalSMData(std::string name, T &ret)
  {
    return base_type::outermost_context().getGlobalSMData(name, ret);
  }

  // Store globally in this state machine. (By value parameter )
  template <typename T>
  void setGlobalSMData(std::string name, T value)
  {
    base_type::outermost_context().setGlobalSMData(name, value);
  }

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *&storage)
  {
    base_type::outermost_context().requiresComponent(storage);
  }

  virtual ISmaccStateMachine &getStateMachine()
  {
    return base_type::outermost_context();
  }

  template <typename TOrthogonal, typename TBehavior, typename... Args>
  static void configure_orthogonal(Args &&... args)
  {
    auto strorthogonal = demangleSymbol(typeid(TOrthogonal).name());
    auto strbehavior = demangleSymbol(typeid(TBehavior).name());

    ROS_INFO_STREAM("Orthogonal " << strorthogonal << " -> " << strbehavior);

    ClientBehaviorInfoEntry bhinfo;
    bhinfo.factoryFunction = [=](ISmaccState *state) {
      //auto bh = std::make_shared<TBehavior>(args...);
      state->configure<TOrthogonal, TBehavior>(args...);
    };

    bhinfo.behaviorType = &(typeid(TBehavior));
    bhinfo.orthogonalType = &(typeid(TOrthogonal));

    const std::type_info *tindex = &(typeid(MostDerived));
    if (!SmaccStateInfo::staticBehaviorInfo.count(tindex))
      SmaccStateInfo::staticBehaviorInfo[tindex] = std::vector<ClientBehaviorInfoEntry>();

    SmaccStateInfo::staticBehaviorInfo[tindex].push_back(bhinfo);
  }

  template <typename TStateReactor, typename... TUArgs>
  static std::shared_ptr<smacc::introspection::StateReactorHandler> static_createStateReactor(TUArgs... args)
  {
    auto sbh = std::make_shared<smacc::introspection::StateReactorHandler>();

    auto sbinfo = std::make_shared<SmaccStateReactorInfo>();
    sbinfo->stateReactorType = &typeid(TStateReactor);
    sbinfo->sbh = sbh;
    sbh->sbInfo_ = sbinfo;

    const std::type_info *tindex = &(typeid(MostDerived)); // get identifier of the current state

    if (!SmaccStateInfo::stateReactorsInfo.count(tindex))
      SmaccStateInfo::stateReactorsInfo[tindex] = std::vector<std::shared_ptr<SmaccStateReactorInfo>>();

    sbinfo->factoryFunction = [&, sbh, args...](ISmaccState *state) {
      auto sb = state->createStateReactor<TStateReactor>(args...);
      sbh->configureStateReactor(sb);
      sb->initialize(state);
      return sb;
    };

    SmaccStateInfo::stateReactorsInfo[tindex].push_back(sbinfo);

    return sbh;
  }

  void checkWhileLoopConditionAndThrowEvent(bool (MostDerived::*conditionFn)())
  {
    auto *thisobject = static_cast<MostDerived *>(this);
    auto condition = boost::bind(conditionFn, thisobject);
    bool conditionResult = condition();
    //ROS_INFO("LOOP EVENT CONDITION: %d", conditionResult);
    if (conditionResult)
    {
      auto evloopcontinue = new EvLoopContinue<MostDerived>();
      this->postEvent(evloopcontinue);
    }
    else
    {
      auto evloopend = new EvLoopEnd<MostDerived>();
      this->postEvent(evloopend);
    }
    ROS_INFO("POST THROW CONDITION");
  }

  //////////////////////////////////////////////////////////////////////////
  // The following declarations should be private.
  // They are only public because many compilers lack template friends.
  //////////////////////////////////////////////////////////////////////////
  // See base class for documentation
  typedef typename base_type::outermost_context_base_type
      outermost_context_base_type;
  typedef typename base_type::inner_context_ptr_type inner_context_ptr_type;
  typedef typename base_type::context_ptr_type context_ptr_type;
  typedef typename base_type::inner_initial_list inner_initial_list;

  static void initial_deep_construct(
      outermost_context_base_type &outermostContextBase)
  {
    deep_construct(&outermostContextBase, outermostContextBase);
  }

  // See base class for documentation
  static void deep_construct(
      const context_ptr_type &pContext,
      outermost_context_base_type &outermostContextBase)
  {
    const inner_context_ptr_type pInnerContext(
        shallow_construct(pContext, outermostContextBase));
    base_type::template deep_construct_inner<inner_initial_list>(
        pInnerContext, outermostContextBase);
  }

  static inner_context_ptr_type shallow_construct(
      const context_ptr_type &pContext,
      outermost_context_base_type &outermostContextBase)
  {
    const inner_context_ptr_type pInnerContext(

        new MostDerived(
            SmaccState<MostDerived, Context, InnerInitial, historyMode>::

                my_context(pContext)));
    outermostContextBase.add(pInnerContext);
    return pInnerContext;
  }
};
} // namespace smacc